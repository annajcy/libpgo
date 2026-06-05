// Element-level finite-difference tests for the *parameter* Hessians of
// VolumetricDeformationModel when driven by the tricubic Hermite formulation.
//
// These three derivatives — d2E/da2, d2E/db2, d2E/dadb — live in
// VolumetricDeformationModel and are NOT assembled into any global matrix, so
// assembler/Energy FD nets cannot reach them. The tricubic Hermite variant
// exercises the 192-DOF kernel (HexTricubicHermiteBasis + GaussLegendreHexQuadrature4)
// and Hermite rest-field synthesis, which are different code paths from the
// trilinear-hex variant tested in volumetricElementParamHessianFD_gtest.cpp.
//
// Mechanism: the element reads a/b from the bound ParameterField during
// prepareData. We perturb parameters through DeformationModelState, re-run
// prepareData, and finite-difference the element's first parameter derivatives:
//   d2E/da2  == d(dE/da)/da
//   d2E/db2  == d(dE/db)/db
//   d2E/dadb == d(dE/da)/db   (np x ne)
//
// enforceSPD = 0 throughout. FD steps are magnitude-scaled.

#include "gtest/gtest.h"

#include "deformationModel.h"
#include "deformationModelManager.h"
#include "deformationModelState.h"
#include "simulationMesh.h"
#include "formulations/elements/volumetricDeformationModel.h"
#include "formulations/formulation.h"
#include "pgoLogging.h"
#include "EigenSupport.h"

#include <tbb/global_control.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace
{
namespace ES = pgo::EigenSupport;
using pgo::SolidDeformationModel::DeformationModelCacheData;
using pgo::SolidDeformationModel::DeformationModelElasticMaterial;
using pgo::SolidDeformationModel::DeformationModelManager;
using pgo::SolidDeformationModel::DeformationModelPlasticMaterial;
using pgo::SolidDeformationModel::DeformationModelState;
using pgo::SolidDeformationModel::ElasticFieldInit;
using pgo::SolidDeformationModel::PlasticFieldInit;
using pgo::SolidDeformationModel::SimulationMesh;
using pgo::SolidDeformationModel::SimulationMeshENuMaterial;
using pgo::SolidDeformationModel::SimulationMeshType;
using pgo::SolidDeformationModel::VolumetricDeformationModel;

constexpr double kFiniteDifferenceStep = 1e-6;
constexpr int kExactDerivativeEnforceSpd = 0;

double fdStep(double base) { return kFiniteDifferenceStep * std::max(1.0, std::abs(base)); }

class ScopedSerialTbb
{
public:
  ScopedSerialTbb(): control_(tbb::global_control::max_allowed_parallelism, 1) {}

private:
  tbb::global_control control_;
};

template<class Eval>
ES::VXd fivePointVector(Eval eval, double h)
{
  ES::VXd gp2 = eval(2.0 * h);
  ES::VXd gp1 = eval(h);
  ES::VXd gm1 = eval(-h);
  ES::VXd gm2 = eval(-2.0 * h);
  return (-gp2 + 8.0 * gp1 - 8.0 * gm1 + gm2) / (12.0 * h);
}

double relColumnError(const ES::VXd &fd, const ES::VXd &analytic)
{
  return (fd - analytic).norm() / std::max(1.0, analytic.norm());
}

struct ElementCase
{
  std::shared_ptr<const SimulationMesh> meshOwner;
  std::shared_ptr<DeformationModelState> state;
  std::unique_ptr<DeformationModelManager> manager;
  const VolumetricDeformationModel *fem = nullptr;
  std::unique_ptr<DeformationModelCacheData> cache;
  ES::VXd elementFiber, vertexFiber;
  ES::VXd positions;  // deformed, element-local (== global for a single element)
  ES::VXd aBase, bBase;
  int np = 0, ne = 0;
};

// A non-unit-cube single hex (stretched axes) so the rest F is well-conditioned.
std::unique_ptr<SimulationMesh> makeSingleHexMesh()
{
  const double vertices[] = {
    0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 1.5, 2.0, 0.0, 0.0, 2.0, 0.0,
    0.0, 0.0, 3.0, 1.5, 0.0, 3.0, 1.5, 2.0, 3.0, 0.0, 2.0, 3.0,
  };
  const int elementVertices[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
  const int elementMaterialIndices[] = { 0 };
  SimulationMeshENuMaterial baseMaterial(1000.0, 0.45);
  const pgo::SolidDeformationModel::SimulationMeshMaterial *materials[] = { &baseMaterial };
  return std::unique_ptr<SimulationMesh>(new SimulationMesh(
    8, vertices, 1, 8, elementVertices, elementMaterialIndices, 1, materials, SimulationMeshType::CUBIC));
}

// Builds the element-local case.
ElementCase makeCase(std::unique_ptr<SimulationMesh> meshMutable,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  bool withHill)
{
  pgo::Logging::init();

  ElementCase c;
  if (withHill) {
    static pgo::SolidDeformationModel::SimulationMeshHillMaterial hillMaterial(2500.0, 0.35, 1.0);
    meshMutable->appendMaterialToAllElements(&hillMaterial);
  }
  c.meshOwner = std::shared_ptr<const SimulationMesh>(std::move(meshMutable));

  const int nele = c.meshOwner->getNumElements();
  const int nvtx = c.meshOwner->getNumVertices();
  c.elementFiber = ES::VXd::Zero(nele * 3);
  c.vertexFiber = ES::VXd::Zero(nvtx * 3);
  for (int ei = 0; ei < nele; ei++)
    c.elementFiber.segment<3>(ei * 3) << 1.0, 0.0, 0.0;
  for (int vi = 0; vi < nvtx; vi++)
    c.vertexFiber.segment<3>(vi * 3) << 1.0, 0.0, 0.0;

  c.state = DeformationModelState::create(
    c.meshOwner, elastic, ElasticFieldInit{}, plastic, PlasticFieldInit{});

  const double *ef = withHill ? c.elementFiber.data() : nullptr;
  const double *vf = withHill ? c.vertexFiber.data() : nullptr;
  c.manager = std::make_unique<DeformationModelManager>(
    c.state, pgo::SolidDeformationModel::TricubicHermiteFormulation{}, kExactDerivativeEnforceSpd, ef, vf);

  c.fem = dynamic_cast<const VolumetricDeformationModel *>(c.manager->getDeformationModel(0));
  EXPECT_NE(c.fem, nullptr);
  c.cache = c.fem->allocateCacheData();
  c.np = c.manager->getNumPlasticParameters();
  c.ne = withHill ? c.manager->getNumElasticParameters() : 0;

  // Build the Hermite rest field by evaluating the formulation's buildGlobalRestDofs,
  // then add a smooth perturbation to all 192 DOFs.
  pgo::SolidDeformationModel::TricubicHermiteFormulation formulation;
  ES::VXd rest = formulation.buildGlobalRestDofs(*c.meshOwner);
  EXPECT_EQ(rest.size(), nvtx * 24);

  c.positions = rest;
  for (int i = 0; i < c.positions.size(); i++)
    c.positions[i] += 0.03 * std::sin(0.7 * i + 0.1) + 0.02 * std::cos(0.9 * i + 0.2);

  // Plastic base: identity stretch nudged off-rest so the derivatives are nonzero.
  c.aBase = ES::VXd::Zero(c.np);
  c.manager->getDeformationModel(0)->getPlasticModel()->defaultParams(c.aBase.data());
  for (int i = 0; i < c.np; i++)
    c.aBase[i] += 0.03 * std::sin(1.7 * i + 0.4);

  c.bBase = ES::VXd::Zero(c.ne);
  if (c.ne > 0)
    c.bBase.setConstant(0.75);

  return c;
}

// Refresh the cache after installing candidate parameters.
void prepareAt(ElementCase &c, const ES::VXd &a, const ES::VXd &b)
{
  c.state->setPlasticValues(a);
  if (c.ne > 0)
    c.state->setElasticValues(b);
  c.fem->prepareData(c.positions.data(), c.cache.get());
}

ES::VXd gradA(ElementCase &c, const ES::VXd &a, const ES::VXd &b)
{
  prepareAt(c, a, b);
  ES::VXd g = ES::VXd::Zero(c.np);
  c.fem->compute_dE_da(c.cache.get(), g.data());
  return g;
}

ES::VXd gradB(ElementCase &c, const ES::VXd &a, const ES::VXd &b)
{
  prepareAt(c, a, b);
  ES::VXd g = ES::VXd::Zero(c.ne);
  c.fem->compute_dE_db(c.cache.get(), g.data());
  return g;
}

// d2E/da2 == d(dE/da)/da, an np x np matrix.
void checkD2Eda2(ElementCase &c, double tol)
{
  ASSERT_GT(c.np, 0);
  prepareAt(c, c.aBase, c.bBase);
  ES::MXd analytic = ES::MXd::Zero(c.np, c.np);
  c.fem->compute_d2E_da2(c.cache.get(), analytic.data());

  ScopedSerialTbb serial;
  for (int j = 0; j < c.np; j++) {
    ES::VXd fd = fivePointVector([&](double delta) {
      ES::VXd a = c.aBase;
      a[j] += delta;
      return gradA(c, a, c.bBase);
    },
      fdStep(c.aBase[j]));
    EXPECT_LT(relColumnError(fd, analytic.col(j)), tol)
      << "d2E/da2 column " << j << " disagrees with FD of dE/da.";
  }
  EXPECT_GT(analytic.norm(), 0.0) << "d2E/da2 is identically zero; FD check is vacuous.";
}

// d2E/db2 == d(dE/db)/db, an ne x ne matrix.
// The only differentiable volumetric elastic parameter (Hill activation) enters
// linearly, so the true d2E/db2 is zero. We assert dE/db != 0 (parameter drives
// energy) and that the assembled d2E/db2 matches FD(dE/db).
void checkD2Edb2(ElementCase &c, double tol)
{
  ASSERT_GT(c.ne, 0);
  EXPECT_GT(gradB(c, c.aBase, c.bBase).norm(), 0.0)
    << "dE/db is zero; the elastic parameter does not affect the energy.";

  prepareAt(c, c.aBase, c.bBase);
  ES::MXd analytic = ES::MXd::Zero(c.ne, c.ne);
  c.fem->compute_d2E_db2(c.cache.get(), analytic.data());

  ScopedSerialTbb serial;
  for (int j = 0; j < c.ne; j++) {
    ES::VXd fd = fivePointVector([&](double delta) {
      ES::VXd b = c.bBase;
      b[j] += delta;
      return gradB(c, c.aBase, b);
    },
      fdStep(c.bBase[j]));
    EXPECT_LT(relColumnError(fd, analytic.col(j)), tol)
      << "d2E/db2 column " << j << " disagrees with FD of dE/db.";
  }
}

// d2E/dadb == d(dE/da)/db, an np x ne matrix (rows: plastic, cols: elastic).
void checkD2Edadb(ElementCase &c, double tol)
{
  ASSERT_GT(c.np, 0);
  ASSERT_GT(c.ne, 0);
  prepareAt(c, c.aBase, c.bBase);
  ES::MXd analytic = ES::MXd::Zero(c.np, c.ne);
  c.fem->compute_d2E_dadb(c.cache.get(), analytic.data());

  ScopedSerialTbb serial;
  for (int j = 0; j < c.ne; j++) {
    ES::VXd fd = fivePointVector([&](double delta) {
      ES::VXd b = c.bBase;
      b[j] += delta;
      return gradA(c, c.aBase, b);
    },
      fdStep(c.bBase[j]));
    EXPECT_LT(relColumnError(fd, analytic.col(j)), tol)
      << "d2E/dadb column (elastic " << j << ") disagrees with FD of dE/da.";
  }
  EXPECT_GT(analytic.norm(), 0.0) << "d2E/dadb is identically zero; FD check is vacuous.";
}
}  // namespace

TEST(TricubicHermiteElementParamHessianFDGTest, HermitePlasticHessianDOF6)
{
  auto c = makeCase(makeSingleHexMesh(),
    DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, false);
  checkD2Eda2(c, 1e-6);
}

TEST(TricubicHermiteElementParamHessianFDGTest, HermitePlasticHessianDOF3)
{
  auto c = makeCase(makeSingleHexMesh(),
    DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF3, false);
  checkD2Eda2(c, 1e-6);
}

TEST(TricubicHermiteElementParamHessianFDGTest, HermiteHillPlasticHessianDOF6)
{
  auto c = makeCase(makeSingleHexMesh(),
    DeformationModelElasticMaterial::HILL_STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, true);
  ASSERT_EQ(c.ne, 1);
  checkD2Eda2(c, 1e-6);
}

TEST(TricubicHermiteElementParamHessianFDGTest, HermiteHillElasticHessian)
{
  auto c = makeCase(makeSingleHexMesh(),
    DeformationModelElasticMaterial::HILL_STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, true);
  ASSERT_EQ(c.ne, 1);
  checkD2Edb2(c, 1e-6);
}

TEST(TricubicHermiteElementParamHessianFDGTest, HermiteHillMixedPlasticElasticHessian)
{
  auto c = makeCase(makeSingleHexMesh(),
    DeformationModelElasticMaterial::HILL_STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, true);
  ASSERT_EQ(c.ne, 1);
  checkD2Edadb(c, 1e-6);
}
