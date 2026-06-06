// Element-level finite-difference tests for the *parameter* Hessians of
// VolumetricDeformationModel: d2E/da2, d2E/db2, d2E/dadb.
//
// These three are implemented in the element (volumetricDeformationModel.cpp) but
// are NOT assembled into any global matrix, so the assembler / Energy FD nets
// cannot reach them. The only prior driver was a legacy FD harness (since removed)
// that was unregistered, log-only, and -- because the element sources its
// parameters from the bound ParameterField rather than from a packed x tail --
// no longer perturbed the parameters at all (so d2E/db2 and d2E/dadb were never
// actually exercised).
//
// Mechanism: the element reads a/b from the bound field during prepareData
// (plasticParams()->computeValue(ele, q, ...)). So we perturb parameters through
// the DeformationModelState, re-run prepareData, and finite-difference the
// element's first parameter derivatives:
//   d2E/da2  == d(dE/da)/da
//   d2E/db2  == d(dE/db)/db
//   d2E/dadb == d(dE/da)/db   (np x ne)
//
// enforceSPD = 0 throughout, so the assembled second derivatives are the true
// derivatives rather than the SPD-projected optimizer Hessian.

#include "gtest/gtest.h"

#include "deformation/deformationModel.h"
#include "deformation/deformationModelManager.h"
#include "deformation/deformationModelState.h"
#include "simulation/simulationMesh.h"
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
using pgo::SolidDeformationModel::ElasticMaterialFieldType;
using pgo::SolidDeformationModel::PlasticFieldInit;
using pgo::SolidDeformationModel::PlasticMaterialFieldType;
using pgo::SolidDeformationModel::SimulationMesh;
using pgo::SolidDeformationModel::SimulationMeshENuMaterial;
using pgo::SolidDeformationModel::SimulationMeshType;
using pgo::SolidDeformationModel::VolumetricDeformationModel;

constexpr double kFiniteDifferenceStep = 1e-6;
constexpr int kExactDerivativeEnforceSpd = 0;

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

// A single-element volumetric model, its supporting state/manager, and a deformed
// local position vector. Fiber arrays are owned here so they outlive the manager.
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

std::unique_ptr<SimulationMesh> makeSingleTetMesh()
{
  const double vertices[] = {
    0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 4.0,
  };
  const int elementVertices[] = { 0, 1, 2, 3 };
  const int elementMaterialIndices[] = { 0 };
  SimulationMeshENuMaterial baseMaterial(1000.0, 0.45);
  const pgo::SolidDeformationModel::SimulationMeshMaterial *materials[] = { &baseMaterial };
  return std::unique_ptr<SimulationMesh>(new SimulationMesh(
    4, vertices, 1, 4, elementVertices, elementMaterialIndices, 1, materials, SimulationMeshType::TET));
}

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

ES::VXd deformedPositions(const SimulationMesh &mesh)
{
  const int nvtx = mesh.getNumVertices();
  ES::VXd x(nvtx * 3);
  for (int vi = 0; vi < nvtx; vi++) {
    double p[3];
    mesh.getVertex(vi, p);
    x[vi * 3 + 0] = p[0] + 0.03 * std::sin(0.7 * vi + 0.1);
    x[vi * 3 + 1] = p[1] + 0.02 * std::cos(0.9 * vi + 0.2);
    x[vi * 3 + 2] = p[2] + 0.025 * std::sin(1.1 * vi + 0.3);
  }
  return x;
}

// Builds the element-local case. When `elastic` is HILL_STABLE_NEO an elastic
// parameter (activation) is present and fiber directions are supplied.
ElementCase makeCase(std::unique_ptr<SimulationMesh> meshMutable,
  SimulationMeshType type,
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
  if (type == SimulationMeshType::TET)
    c.manager = std::make_unique<DeformationModelManager>(
      c.state, pgo::SolidDeformationModel::P1TetFormulation{}, kExactDerivativeEnforceSpd, ef, vf);
  else
    c.manager = std::make_unique<DeformationModelManager>(
      c.state, pgo::SolidDeformationModel::LinearCubicFormulation{}, kExactDerivativeEnforceSpd, ef, vf);

  c.fem = dynamic_cast<const VolumetricDeformationModel *>(c.manager->getDeformationModel(0));
  EXPECT_NE(c.fem, nullptr);
  c.cache = c.fem->allocateCacheData();
  c.np = c.manager->getNumPlasticParameters();
  c.ne = withHill ? c.manager->getNumElasticParameters() : 0;

  c.positions = deformedPositions(*c.meshOwner);

  // Plastic base: identity stretch nudged off-rest so the derivatives are nonzero
  // but the plastic deformation gradient stays well-conditioned.
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

double fdStep(double base) { return kFiniteDifferenceStep * std::max(1.0, std::abs(base)); }

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
//
// Note: the only differentiable volumetric elastic parameter today is the Hill
// activation, which enters the energy *linearly* (compute_d2psi_dparam2 == 0), so
// the true d2E/db2 is zero. We therefore do NOT assert nonzero here; instead we
// assert (a) dE/db itself is nonzero -- the parameter genuinely drives the energy,
// so the zero second derivative is a real statement about linearity, not a dead
// parameter -- and (b) the assembled d2E/db2 matches FD(dE/db). A regression that
// returns a spurious nonzero, or that stubs a real nonzero to zero, fails (b).
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

TEST(VolumetricElementParamHessianFDGTest, TetPlasticHessianDOF6)
{
  auto c = makeCase(makeSingleTetMesh(), SimulationMeshType::TET,
    DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, false);
  checkD2Eda2(c, 1e-6);
}

TEST(VolumetricElementParamHessianFDGTest, HexPlasticHessianDOF6)
{
  auto c = makeCase(makeSingleHexMesh(), SimulationMeshType::CUBIC,
    DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, false);
  checkD2Eda2(c, 1e-6);
}

TEST(VolumetricElementParamHessianFDGTest, HexPlasticHessianDOF3)
{
  auto c = makeCase(makeSingleHexMesh(), SimulationMeshType::CUBIC,
    DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF3, false);
  checkD2Eda2(c, 1e-6);
}

// Hill material exposes one elastic (activation) parameter, so this case is the
// only place d2E/db2 and d2E/dadb get exercised at all.
TEST(VolumetricElementParamHessianFDGTest, HexHillPlasticHessianDOF6)
{
  auto c = makeCase(makeSingleHexMesh(), SimulationMeshType::CUBIC,
    DeformationModelElasticMaterial::HILL_STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, true);
  ASSERT_EQ(c.ne, 1);
  checkD2Eda2(c, 1e-6);
}

TEST(VolumetricElementParamHessianFDGTest, HexHillElasticHessian)
{
  auto c = makeCase(makeSingleHexMesh(), SimulationMeshType::CUBIC,
    DeformationModelElasticMaterial::HILL_STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, true);
  ASSERT_EQ(c.ne, 1);
  checkD2Edb2(c, 1e-6);
}

TEST(VolumetricElementParamHessianFDGTest, HexHillMixedPlasticElasticHessian)
{
  auto c = makeCase(makeSingleHexMesh(), SimulationMeshType::CUBIC,
    DeformationModelElasticMaterial::HILL_STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, true);
  ASSERT_EQ(c.ne, 1);
  checkD2Edadb(c, 1e-6);
}
