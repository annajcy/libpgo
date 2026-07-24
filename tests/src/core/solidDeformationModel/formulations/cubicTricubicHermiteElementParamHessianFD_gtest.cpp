// Element-level finite-difference tests for the *parameter* Hessians of
// VolumetricDeformationModel when driven by the tricubic Hermite formulation.
//
// These three derivatives — d2E/da2, d2E/db2, d2E/dadb — live in
// VolumetricDeformationModel and are NOT assembled into any global matrix, so
// assembler/Energy FD nets cannot reach them. The tricubic Hermite variant
// exercises the 192-DOF mapping (CubicTricubicHermiteShapeFunction + GaussLegendreHexQuadrature4)
// and Hermite rest-field synthesis, which are different code paths from the
// trilinear-hex variant tested in volumetricElementParamHessianFD_gtest.cpp.
//
// Mechanism: we perturb parameters through assembler-owned fields, compute
// material parameter values from its fields, pass those values into prepareData,
// and finite-difference the element's first parameter derivatives:
//   d2E/da2  == d(dE/da)/da
//   d2E/db2  == d(dE/db)/db
//   d2E/dadb == d(dE/da)/db   (np x ne)
//
// enforceSPD = 0 throughout. FD steps are magnitude-scaled.

#include "gtest/gtest.h"

#include "deformation/deformationModel.h"
#include "deformation/deformationModelAssembler.h"
#include "deformation/deformationModelManager.h"
#include "material/fields/materialParameterFieldInit.h"
#include "simulation/simulationMesh.h"
#include "deformation/volume/volumetricDeformationModel.h"
#include "formulations/formulation/formulations.h"
#include "material/fields/parameterField.h"
#include "pgoLogging.h"
#include "EigenSupport.h"

#include <tbb/global_control.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>

namespace
{
namespace ES = pgo::EigenSupport;
using pgo::SolidDeformationModel::DeformationModelCacheData;
using pgo::SolidDeformationModel::DeformationModelAssembler;
using pgo::SolidDeformationModel::DeformationModelElasticMaterial;
using pgo::SolidDeformationModel::DeformationModelManager;
using pgo::SolidDeformationModel::DeformationModelPlasticMaterial;
using pgo::SolidDeformationModel::ElasticFieldInit;
using pgo::SolidDeformationModel::PlasticFieldInit;
using pgo::SolidDeformationModel::SimulationMesh;
using pgo::SolidDeformationModel::SimulationMeshENuMaterial;
using pgo::SolidDeformationModel::SimulationMeshType;
using pgo::SolidDeformationModel::OptimizableField;
using pgo::SolidDeformationModel::VolumetricDeformationModel;
using pgo::SolidDeformationModel::createElasticParameterField;
using pgo::SolidDeformationModel::createPlasticParameterField;

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

ES::VXd materialValuesForElement(
  const OptimizableField *field, int ele, int numLocations, int numChannels)
{
  if (!field || numChannels == 0)
    return ES::VXd();

  ES::VXd values(numLocations * numChannels);
  for (int q = 0; q < numLocations; q++) {
    field->computeValue(ele, q, values.data() + static_cast<std::ptrdiff_t>(q) * numChannels);
  }
  return values;
}

struct ElementCase
{
  std::shared_ptr<const SimulationMesh> meshOwner;
  std::unique_ptr<DeformationModelAssembler> assembler;
  const VolumetricDeformationModel *fem = nullptr;
  std::unique_ptr<DeformationModelCacheData> cache;
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

  const int nvtx = c.meshOwner->getNumVertices();

  auto elasticField = createElasticParameterField(*c.meshOwner, elastic, ElasticFieldInit{});
  auto plasticField = createPlasticParameterField(*c.meshOwner, plastic, PlasticFieldInit{});

  pgo::SolidDeformationModel::CubicTricubicHermiteFormulation formulation;
  auto manager = std::make_shared<DeformationModelManager>(
    c.meshOwner, elastic, plastic, formulation, kExactDerivativeEnforceSpd);
  c.assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), formulation, std::move(elasticField), std::move(plasticField), nullptr);

  const auto &managerRef = c.assembler->getDeformationModelManager();
  c.fem = dynamic_cast<const VolumetricDeformationModel *>(managerRef.getDeformationModel(0));
  EXPECT_NE(c.fem, nullptr);
  c.cache = c.fem->allocateCacheData();
  c.np = managerRef.getNumPlasticParameters();
  c.ne = withHill ? managerRef.getNumElasticParameters() : 0;

  // Build the Hermite rest field by evaluating the formulation's buildGlobalRestDofs,
  // then add a smooth perturbation to all 192 DOFs.
  ES::VXd rest = formulation.buildGlobalRestDofs(*c.meshOwner);
  EXPECT_EQ(rest.size(), nvtx * 24);

  c.positions = rest;
  for (int i = 0; i < c.positions.size(); i++)
    c.positions[i] += 0.03 * std::sin(0.7 * i + 0.1) + 0.02 * std::cos(0.9 * i + 0.2);

  // Plastic base: identity stretch nudged off-rest so the derivatives are nonzero.
  c.aBase = ES::VXd::Zero(c.np);
  managerRef.getDeformationModel(0)->defaultPlasticParams(c.aBase.data());
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
  c.assembler->setPlasticValues(a);
  if (c.ne > 0)
    c.assembler->setElasticValues(b);
  const int numLocations = c.fem->getNumMaterialLocations();
  const ES::VXd plasticValues =
    materialValuesForElement(c.assembler->plasticParameterFieldPtr().get(), 0, numLocations, c.np);
  const ES::VXd elasticValues =
    materialValuesForElement(c.assembler->elasticParameterFieldPtr().get(), 0, numLocations, c.ne);
  c.fem->prepareData(c.positions.data(),
    elasticValues.size() ? elasticValues.data() : nullptr,
    plasticValues.size() ? plasticValues.data() : nullptr,
    c.cache.get());
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

TEST(CubicTricubicHermiteElementParamHessianFDGTest, HermitePlasticHessianDOF6)
{
  auto c = makeCase(makeSingleHexMesh(),
    DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, false);
  checkD2Eda2(c, 1e-6);
}

TEST(CubicTricubicHermiteElementParamHessianFDGTest, HermitePlasticHessianDOF3)
{
  auto c = makeCase(makeSingleHexMesh(),
    DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF3, false);
  checkD2Eda2(c, 1e-6);
}

TEST(CubicTricubicHermiteElementParamHessianFDGTest, HermiteHillPlasticHessianDOF6)
{
  auto c = makeCase(makeSingleHexMesh(),
    DeformationModelElasticMaterial::HILL_STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, true);
  ASSERT_EQ(c.ne, 1);
  checkD2Eda2(c, 1e-6);
}

TEST(CubicTricubicHermiteElementParamHessianFDGTest, HermiteHillElasticHessian)
{
  auto c = makeCase(makeSingleHexMesh(),
    DeformationModelElasticMaterial::HILL_STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, true);
  ASSERT_EQ(c.ne, 1);
  checkD2Edb2(c, 1e-6);
}

TEST(CubicTricubicHermiteElementParamHessianFDGTest, HermiteHillMixedPlasticElasticHessian)
{
  auto c = makeCase(makeSingleHexMesh(),
    DeformationModelElasticMaterial::HILL_STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, true);
  ASSERT_EQ(c.ne, 1);
  checkD2Edadb(c, 1e-6);
}
