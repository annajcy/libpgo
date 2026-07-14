#include <gtest/gtest.h>

#include "deformation/deformationModelAssembler.h"
#include "deformation/deformationModel.h"
#include "deformation/deformationModelManager.h"
#include "formulations/formulation/formulations.h"
#include "material/fields/materialParameterFieldInit.h"
#include "pgoLogging.h"
#include "simulation/simulationMesh.h"
#include "material/elastic/elasticModelFactory.h"
#include "material/plastic/plasticModelFactory.h"
#include "material/plastic/plasticModel3DDeformationGradient.h"
#include "cubicMesh.h"
#include "tetMesh.h"
#include "triMeshGeo.h"

#include <tbb/global_control.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <memory>

namespace
{
namespace ES = pgo::EigenSupport;
using pgo::SolidDeformationModel::DeformationModelAssembler;
using pgo::SolidDeformationModel::DeformationModelElasticMaterial;
using pgo::SolidDeformationModel::ParameterField;
using pgo::SolidDeformationModel::DeformationModelManager;
using pgo::SolidDeformationModel::DeformationModelPlasticMaterial;
using pgo::SolidDeformationModel::ElasticModelFactory;
using pgo::SolidDeformationModel::OptimizableField;
using pgo::SolidDeformationModel::PlasticModel3DDeformationGradient;
using pgo::SolidDeformationModel::PlasticModelFactory;
using pgo::SolidDeformationModel::SimulationMesh;
using pgo::SolidDeformationModel::ElasticFieldInit;
using pgo::SolidDeformationModel::PlasticFieldInit;
using pgo::SolidDeformationModel::createElasticParameterField;
using pgo::SolidDeformationModel::createPlasticParameterField;
using pgo::SolidDeformationModel::SimulationMeshENuhMaterial;
using pgo::SolidDeformationModel::SimulationMeshENuMaterial;
using pgo::SolidDeformationModel::SimulationMeshType;
using pgo::SolidDeformationModel::DeformationModel;
using pgo::SolidDeformationModel::DeformationModelAssemblerCacheData;
using pgo::SolidDeformationModel::DeformationModelCacheData;
using pgo::SolidDeformationModel::DofGroup;
using pgo::SolidDeformationModel::HessianBlockOffset;
using pgo::SolidDeformationModel::buildHessianBlockOffsetsForGroups;

constexpr const char *kTorusVegPath = LIBPGO_TEST_TORUS_VEG;
constexpr const char *kShellObjPath = LIBPGO_TEST_SHELL_OBJ;
constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;
constexpr double kFiniteDifferenceStep = 1e-6;
constexpr int kExactDerivativeEnforceSpd = 0;

void setFieldDataIfPresent(OptimizableField &field, const ES::VXd &values)
{
  if (values.size() > 0)
    field.setGlobalData(values.data());
}

ES::VXd makePerturbedRestPositions(const SimulationMesh &mesh)
{
  ES::VXd x(mesh.getNumVertices() * 3);
  for (int vi = 0; vi < mesh.getNumVertices(); vi++) {
    double p[3];
    mesh.getVertex(vi, p);
    x[vi * 3 + 0] = p[0] + 1e-3 * ((vi % 3) - 1);
    x[vi * 3 + 1] = p[1] + 5e-4 * ((vi % 5) - 2);
    x[vi * 3 + 2] = p[2] + 7.5e-4 * ((vi % 7) - 3);
  }
  return x;
}

ES::VXd makeRestPositions(const SimulationMesh &mesh)
{
  ES::VXd x(mesh.getNumVertices() * 3);
  for (int vi = 0; vi < mesh.getNumVertices(); vi++) {
    double p[3];
    mesh.getVertex(vi, p);
    x.segment<3>(vi * 3) = ES::V3d(p[0], p[1], p[2]);
  }
  return x;
}

void expectAllFinite(const ES::VXd &v)
{
  for (Eigen::Index i = 0; i < v.size(); i++) {
    EXPECT_TRUE(std::isfinite(v[i])) << "Non-finite vector entry at " << i;
  }
}

void expectAllFinite(const ES::SpMatD &m)
{
  for (Eigen::Index i = 0; i < m.nonZeros(); i++) {
    EXPECT_TRUE(std::isfinite(m.valuePtr()[i])) << "Non-finite sparse entry at " << i;
  }
}

std::unique_ptr<SimulationMesh> makeSingleElementCubicSimulationMesh()
{
  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    1.0, 1.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
    1.0, 0.0, 1.0,
    1.0, 1.0, 1.0,
    0.0, 1.0, 1.0,
  };
  const int elementVertices[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
  const int elementMaterialIndices[] = { 0 };

  SimulationMeshENuMaterial baseMaterial(1200.0, 0.45);
  const pgo::SolidDeformationModel::SimulationMeshMaterial *materials[] = { &baseMaterial };

  return std::unique_ptr<SimulationMesh>(new SimulationMesh(
    8, vertices,
    1, 8, elementVertices,
    elementMaterialIndices, 1, materials,
    SimulationMeshType::CUBIC));
}

std::shared_ptr<const SimulationMesh> makeTwoElementCubicSimulationMesh()
{
  static const double vertices[] = {
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0,
    2.0, 0.0, 0.0, 2.0, 1.0, 0.0, 2.0, 0.0, 1.0, 2.0, 1.0, 1.0
  };
  static const int elementVertices[] = {
    0, 1, 2, 3, 4, 5, 6, 7,
    1, 8, 9, 2, 5, 10, 11, 6
  };
  static const int elementMaterialIndices[] = { 0, 0 };
  static SimulationMeshENuMaterial baseMaterial(1200.0, 0.45);
  static const pgo::SolidDeformationModel::SimulationMeshMaterial *materials[] = { &baseMaterial };
  return std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    12, vertices,
    2, 8, elementVertices,
    elementMaterialIndices, 1, materials,
    SimulationMeshType::CUBIC));
}

struct HessianBlockKeyForTest
{
  std::vector<int> rows;
  std::vector<int> cols;

  bool operator<(const HessianBlockKeyForTest &other) const
  {
    if (rows != other.rows)
      return rows < other.rows;
    return cols < other.cols;
  }
};

ES::SpMatD buildTripletHessianTopologyReference(const pgo::SolidDeformationModel::DofLayout &layout,
  int numElements, int numDofs)
{
  std::set<HessianBlockKeyForTest> blocks;
  std::vector<DofGroup> groups;
  for (int ele = 0; ele < numElements; ele++) {
    layout.getDofGroups(ele, groups);
    for (const DofGroup &rowGroup : groups)
      for (const DofGroup &colGroup : groups)
        blocks.insert({ globalDofs(rowGroup), globalDofs(colGroup) });
  }

  std::vector<ES::TripletD> entries;
  for (const HessianBlockKeyForTest &block : blocks)
    for (int row : block.rows)
      for (int col : block.cols)
        entries.emplace_back(row, col, 1.0);

  ES::SpMatD reference(numDofs, numDofs);
  reference.setFromTriplets(entries.begin(), entries.end());
  return reference;
}

void expectSameSparseTopology(const ES::SpMatD &actual, const ES::SpMatD &reference)
{
  ASSERT_EQ(actual.rows(), reference.rows());
  ASSERT_EQ(actual.cols(), reference.cols());
  ASSERT_EQ(actual.nonZeros(), reference.nonZeros());
  for (int row = 0; row <= actual.outerSize(); row++)
    ASSERT_EQ(actual.outerIndexPtr()[row], reference.outerIndexPtr()[row]) << "outer index " << row;
  for (Eigen::Index offset = 0; offset < actual.nonZeros(); offset++)
    ASSERT_EQ(actual.innerIndexPtr()[offset], reference.innerIndexPtr()[offset]) << "inner index " << offset;
}

struct FieldBackedManager
{
  std::shared_ptr<DeformationModelManager> manager;
  std::shared_ptr<OptimizableField> elasticField;
  std::shared_ptr<OptimizableField> plasticField;
};

template<class FormulationT>
FieldBackedManager makeFieldBackedManager(
  std::shared_ptr<const SimulationMesh> mesh,
  DeformationModelPlasticMaterial plastic,
  DeformationModelElasticMaterial elastic,
  const FormulationT &formulation,
  int enforceSPD = 1,
  const double *elementFiberDirections = nullptr,
  const double *vertexFiberDirections = nullptr)
{
  auto elasticField = createElasticParameterField(*mesh, elastic, ElasticFieldInit{});
  auto plasticField = createPlasticParameterField(*mesh, plastic, PlasticFieldInit{});
  auto manager = std::make_shared<DeformationModelManager>(
    mesh,
    elastic,
    plastic,
    formulation,
    enforceSPD,
    elementFiberDirections,
    vertexFiberDirections);
  return { std::move(manager), std::move(elasticField), std::move(plasticField) };
}

class ScopedSerialTbb
{
public:
  ScopedSerialTbb(): control_(tbb::global_control::max_allowed_parallelism, 1) {}

private:
  tbb::global_control control_;
};

template<class EvalGradientAtDelta>
ES::VXd fivePointFiniteDifference(EvalGradientAtDelta eval, double h = kFiniteDifferenceStep)
{
  ES::VXd gp2 = eval(2.0 * h);
  ES::VXd gp1 = eval(h);
  ES::VXd gm1 = eval(-h);
  ES::VXd gm2 = eval(-2.0 * h);
  return (-gp2 + 8.0 * gp1 - 8.0 * gm1 + gm2) / (12.0 * h);
}

double relativeColumnError(const ES::VXd &fd, const ES::VXd &analytic)
{
  return (fd - analytic).norm() / std::max(1.0, analytic.norm());
}

struct FakeCacheData : public DeformationModelCacheData
{
  explicit FakeCacheData(int layout_): layout(layout_) {}
  int layout = 0;
};

class FakeCacheModel : public DeformationModel
{
public:
  explicit FakeCacheModel(int layout_): layout(layout_) {}

  std::unique_ptr<CacheData> allocateCacheData() const override
  {
    return std::make_unique<FakeCacheData>(layout);
  }

  bool isCacheDataCompatible(const CacheData &cache) const override
  {
    const auto *typedCache = dynamic_cast<const FakeCacheData *>(&cache);
    return typedCache != nullptr && typedCache->layout == layout;
  }

  void prepareData(const double *, const double *, const double *, CacheData *) const override {}
  double computeEnergy(const CacheData *) const override { return 0.0; }
  void compute_dE_dx(const CacheData *, double *) const override {}
  void compute_d2E_dx2(const CacheData *, double *) const override {}
  void compute_d2E_dxda(const CacheData *, double *) const override {}
  void compute_d2E_dxdb(const CacheData *, double *) const override {}
  int getNumElasticParameters() const override { return 0; }
  int getNumPlasticParameters() const override { return 0; }
  int getNumVertices() const override { return 0; }
  int getNumDOFs() const override { return 0; }

private:
  int layout = 0;
};

// An assembler built with enforceSPD = 0 so finite differences measure the true
// derivative of computeGradient. The assembler owns the parameter fields so callers
// can mutate them between gradient evaluations.
struct ExactStateAssembler
{
  std::unique_ptr<DeformationModelAssembler> assembler;
};

template<class FormulationT>
ExactStateAssembler buildExactAssembler(
  std::shared_ptr<const SimulationMesh> mesh,
  DeformationModelElasticMaterial elastic, ElasticFieldInit elasticInit,
  DeformationModelPlasticMaterial plastic, PlasticFieldInit plasticInit,
  const FormulationT &formulation,
  const double *elementFiberDirections = nullptr,
  const double *vertexFiberDirections = nullptr)
{
  auto elasticField = createElasticParameterField(*mesh, elastic, std::move(elasticInit));
  auto plasticField = createPlasticParameterField(*mesh, plastic, std::move(plasticInit));
  auto manager = std::make_shared<DeformationModelManager>(
    mesh, elastic, plastic, formulation, kExactDerivativeEnforceSpd, elementFiberDirections, vertexFiberDirections);
  auto assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), formulation, std::move(elasticField), std::move(plasticField), nullptr);
  return { std::move(assembler) };
}

template<class FormulationT>
std::unique_ptr<DeformationModelAssembler> buildTopologyTestAssembler(
  std::shared_ptr<const SimulationMesh> mesh, const FormulationT &formulation)
{
  auto managerFields = makeFieldBackedManager(
    mesh, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, DeformationModelElasticMaterial::STABLE_NEO, formulation);
  return std::make_unique<DeformationModelAssembler>(
    std::move(managerFields.manager), formulation,
    std::move(managerFields.elasticField), std::move(managerFields.plasticField), nullptr);
}

void expectAssemblerTopologyMatchesTripletReference(const DeformationModelAssembler &assembler)
{
  const SimulationMesh *mesh = assembler.getDeformationModelManager().getMesh();
  ASSERT_NE(mesh, nullptr);
  ES::SpMatD reference = buildTripletHessianTopologyReference(
    assembler.getDofLayout(), mesh->getNumElements(), assembler.getNumDOFs());
  expectSameSparseTopology(assembler.getHessianTemplate(), reference);
}

// Central five-point finite-difference of the assembled gradient with respect to
// a single scalar entry of a parameter vector. setParams installs a candidate
// parameter vector into the live field the assembler reads. The step is scaled by
// the parameter magnitude so that large-valued channels (e.g. a Young's modulus
// of 2e4) and tiny ones (e.g. a 1e-3 thickness) are both perturbed meaningfully.
template<class SetParams>
ES::VXd fdGradientColumn(DeformationModelAssembler &assembler, SetParams setParams,
  const ES::VXd &base, int idx, const ES::VXd &x)
{
  const double h = kFiniteDifferenceStep * std::max(1.0, std::abs(base[idx]));
  return fivePointFiniteDifference([&](double delta) {
    ES::VXd b = base;
    b[idx] += delta;
    setParams(b);
    ES::VXd g = ES::VXd::Zero(assembler.getNumDOFs());
    assembler.computeGradient(x.data(), g.data());
    return g;
  },
    h);
}
}  // namespace

TEST(DeformationModelAssemblerGTest, ThreadScratchReusesCompatibleCacheData)
{
  DeformationModelAssemblerCacheData::ThreadScratch scratch(3, 1, 0, 0);
  FakeCacheModel layoutA(7);
  FakeCacheModel compatibleA(7);
  FakeCacheModel layoutB(11);

  DeformationModel::CacheData *first = scratch.cacheFor(layoutA);
  first->markPrepared();
  DeformationModel::CacheData *second = scratch.cacheFor(compatibleA);
  DeformationModel::CacheData *third = scratch.cacheFor(layoutB);

  EXPECT_EQ(first, second);
  EXPECT_FALSE(second->isPrepared());
  EXPECT_NE(first, third);
  EXPECT_EQ(scratch.numReusableCaches(), 2);
}

TEST(DeformationModelAssemblerGTest, TetLinearHessianTemplateTopologyMatchesTripletReference)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  std::shared_ptr<const SimulationMesh> mesh(pgo::SolidDeformationModel::loadTetMesh(&tetMesh).release());
  ASSERT_NE(mesh, nullptr);

  const pgo::SolidDeformationModel::TetLinearFormulation formulation{};
  auto assembler = buildTopologyTestAssembler(mesh, formulation);
  expectAssemblerTopologyMatchesTripletReference(*assembler);
}

TEST(DeformationModelAssemblerGTest, CubicLinearHessianTemplateTopologyMatchesTripletReference)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> mesh(pgo::SolidDeformationModel::loadCubicMesh(&cubicMesh).release());
  ASSERT_NE(mesh, nullptr);

  const pgo::SolidDeformationModel::CubicLinearFormulation formulation{};
  auto assembler = buildTopologyTestAssembler(mesh, formulation);
  expectAssemblerTopologyMatchesTripletReference(*assembler);
}

TEST(DeformationModelAssemblerGTest, TricubicHermiteHessianTemplateTopologyMatchesTripletReference)
{
  pgo::Logging::init();

  std::shared_ptr<const SimulationMesh> mesh = makeTwoElementCubicSimulationMesh();
  ASSERT_NE(mesh, nullptr);

  const pgo::SolidDeformationModel::CubicTricubicHermiteFormulation formulation{};
  auto assembler = buildTopologyTestAssembler(mesh, formulation);
  expectAssemblerTopologyMatchesTripletReference(*assembler);
}

template<class FormulationT>
void expectExplicitNestedTbbPoliciesAgree(const FormulationT &formulation)
{
  auto mesh = makeTwoElementCubicSimulationMesh();
  auto assembler = buildTopologyTestAssembler(mesh, formulation);
  ES::VXd x = assembler->getRestPosition();
  for (Eigen::Index i = 0; i < x.size(); ++i)
    x[i] += 1e-4 * std::sin(0.37 * static_cast<double>(i) + 0.2);

  const double boundedOneEnergy = pgo::parallel::withSingleThreadedTbb(
    [&] { return assembler->computeEnergy(x.data()); });
  const double multiEnergy = assembler->computeEnergy(x.data());
  EXPECT_NEAR(boundedOneEnergy, multiEnergy,
    1e-10 * std::max({ 1.0, std::abs(boundedOneEnergy), std::abs(multiEnergy) }));

  ES::VXd boundedOneGradient = ES::VXd::Zero(assembler->getNumDOFs());
  ES::VXd multiGradient = ES::VXd::Zero(assembler->getNumDOFs());
  pgo::parallel::withSingleThreadedTbb(
    [&] { assembler->computeGradient(x.data(), boundedOneGradient.data()); });
  assembler->computeGradient(x.data(), multiGradient.data());
  EXPECT_LE((boundedOneGradient - multiGradient).norm(),
    1e-10 * std::max(1.0, boundedOneGradient.norm()));

  ES::SpMatD boundedOneHessian = assembler->getHessianTemplate();
  ES::SpMatD multiHessian = assembler->getHessianTemplate();
  pgo::parallel::withSingleThreadedTbb(
    [&] { assembler->computeHessian(x.data(), boundedOneHessian); });
  assembler->computeHessian(x.data(), multiHessian);
  EXPECT_LE((boundedOneHessian - multiHessian).norm(),
    1e-10 * std::max(1.0, boundedOneHessian.norm()));
}

TEST(DeformationModelAssemblerGTest, ExplicitNestedTbbPoliciesAgreeForCubicLinear)
{
  expectExplicitNestedTbbPoliciesAgree(pgo::SolidDeformationModel::CubicLinearFormulation{});
}

TEST(DeformationModelAssemblerGTest, ExplicitNestedTbbPoliciesAgreeForTricubicHermite)
{
  expectExplicitNestedTbbPoliciesAgree(
    pgo::SolidDeformationModel::CubicTricubicHermiteFormulation{});
}

TEST(DeformationModelAssemblerGTest, HessianBlockOffsetsRejectMissingContiguousColumns)
{
  std::vector<DofGroup> groups = {
    DofGroup{ 0, 0, 3 },
  };

  std::vector<ES::TripletD> entries = { { 0, 0, 1.0 }, { 0, 2, 1.0 } };
  ES::SpMatD tmpl(3, 3);
  tmpl.setFromTriplets(entries.begin(), entries.end());

  std::vector<HessianBlockOffset> blocks;
  EXPECT_THROW(buildHessianBlockOffsetsForGroups(tmpl, 3, 3, groups, blocks), std::runtime_error);
}

TEST(DeformationModelAssemblerGTest, TetAssemblerRegression)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  std::shared_ptr<const SimulationMesh> mesh(pgo::SolidDeformationModel::loadTetMesh(&tetMesh).release());
  ASSERT_NE(mesh, nullptr);

  const int nele = mesh->getNumElements();

  const pgo::SolidDeformationModel::TetLinearFormulation formulation{};
  auto managerFields = makeFieldBackedManager(
    mesh, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, DeformationModelElasticMaterial::STABLE_NEO, formulation);

  const int numPlasticParams = managerFields.manager->getNumPlasticParameters();
  const int numElasticParams = managerFields.manager->getNumElasticParameters();

  auto assembler = std::make_unique<DeformationModelAssembler>(
    std::move(managerFields.manager), formulation,
    managerFields.elasticField, managerFields.plasticField, nullptr);

  ES::VXd x = makePerturbedRestPositions(*assembler->getDeformationModelManager().getMesh());
  ES::VXd plasticParams(numPlasticParams * nele);
  ES::VXd elasticParams(numElasticParams * nele);
  elasticParams.setZero();

  for (int ei = 0; ei < nele; ei++) {
    assembler->getDeformationModelManager().getDeformationModel(ei)->defaultPlasticParams(
      plasticParams.data() + ei * numPlasticParams);
  }
  setFieldDataIfPresent(*managerFields.elasticField, elasticParams);
  setFieldDataIfPresent(*managerFields.plasticField, plasticParams);

  ES::VXd grad = ES::VXd::Zero(assembler->getNumDOFs());
  assembler->computeGradient(x.data(), grad.data());
  EXPECT_EQ(grad.size(), assembler->getNumDOFs());
  expectAllFinite(grad);

  ES::SpMatD hess = assembler->getHessianTemplate();
  assembler->computeHessian(x.data(), hess);
  EXPECT_EQ(hess.rows(), assembler->getNumDOFs());
  EXPECT_EQ(hess.cols(), assembler->getNumDOFs());
  expectAllFinite(hess);

  ES::SpMatD dfda = assembler->get_dfda_Template();
  assembler->compute_df_da(x.data(), dfda);
  EXPECT_EQ(dfda.rows(), assembler->getNumDOFs());
  EXPECT_EQ(dfda.cols(), nele * numPlasticParams);
  expectAllFinite(dfda);
}

TEST(DeformationModelAssemblerGTest, ConstantFieldSharesParamColumnsAcrossElements)
{
  using pgo::SolidDeformationModel::ElasticMaterialFieldType;
  using pgo::SolidDeformationModel::PlasticMaterialFieldType;

  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  std::shared_ptr<const SimulationMesh> mesh(pgo::SolidDeformationModel::loadTetMesh(&tetMesh).release());
  ASSERT_NE(mesh, nullptr);

  const int nele = mesh->getNumElements();
  ASSERT_GT(nele, 1);

  const pgo::SolidDeformationModel::TetLinearFormulation formulation{};
  const auto elastic = DeformationModelElasticMaterial::STABLE_NEO;
  const auto plastic = DeformationModelPlasticMaterial::VOLUMETRIC_DOF6;
  const int numPlasticParams = 6;

  // A single shared plastic set (a mild stretch so the derivatives are nonzero).
  ES::VXd plasticShared(numPlasticParams);
  plasticShared << 1.1, 0.0, 0.0, 1.0, 0.0, 1.0;

  auto constantElasticField = createElasticParameterField(*mesh, elastic, ElasticFieldInit{});
  auto constantPlasticField = createPlasticParameterField(
    *mesh, plastic, PlasticFieldInit{ PlasticMaterialFieldType::CONSTANT, plasticShared });

  ES::VXd plasticEw(static_cast<Eigen::Index>(nele) * numPlasticParams);
  for (int ei = 0; ei < nele; ei++)
    plasticEw.segment(ei * numPlasticParams, numPlasticParams) = plasticShared;
  auto elementwiseElasticField = createElasticParameterField(*mesh, elastic, ElasticFieldInit{});
  auto elementwisePlasticField = createPlasticParameterField(
    *mesh, plastic, PlasticFieldInit{ PlasticMaterialFieldType::ELEMENTWISE, plasticEw });

  auto makeAssembler = [&](std::shared_ptr<OptimizableField> elasticField,
                         std::shared_ptr<OptimizableField> plasticField) {
    auto manager = std::make_shared<DeformationModelManager>(mesh, elastic, plastic, formulation, 1, nullptr, nullptr);
    return std::make_unique<DeformationModelAssembler>(
      std::move(manager), formulation, std::move(elasticField), std::move(plasticField), nullptr);
  };
  auto constantAssembler = makeAssembler(std::move(constantElasticField), std::move(constantPlasticField));
  auto elementwiseAssembler = makeAssembler(std::move(elementwiseElasticField), std::move(elementwisePlasticField));
  ASSERT_EQ(constantAssembler->getNumPlasticParams(), numPlasticParams);

  ES::VXd x = makePerturbedRestPositions(*constantAssembler->getDeformationModelManager().getMesh());
  const int numDOFs = constantAssembler->getNumDOFs();
  ASSERT_EQ(numDOFs, elementwiseAssembler->getNumDOFs());

  // Constant field: numPlasticParams shared columns (NOT nele * numPlasticParams).
  ES::SpMatD dfdaConst = constantAssembler->get_dfda_Template();
  constantAssembler->compute_df_da(x.data(), dfdaConst);
  EXPECT_EQ(dfdaConst.rows(), numDOFs);
  EXPECT_EQ(dfdaConst.cols(), numPlasticParams);
  expectAllFinite(dfdaConst);
  EXPECT_GT(dfdaConst.norm(), 0.0);

  // Elementwise field: per-element columns.
  ES::SpMatD dfdaEw = elementwiseAssembler->get_dfda_Template();
  elementwiseAssembler->compute_df_da(x.data(), dfdaEw);
  ASSERT_EQ(dfdaEw.cols(), nele * numPlasticParams);

  // A shared parameter's derivative column equals the sum over all elements of the
  // corresponding per-element columns: that is exactly what the constant-field
  // column folding produces in the assembler.
  ES::MXd accum = ES::MXd::Zero(numDOFs, numPlasticParams);
  for (int ei = 0; ei < nele; ei++) {
    for (int j = 0; j < numPlasticParams; j++) {
      accum.col(j) += dfdaEw.col(ei * numPlasticParams + j);
    }
  }
  ES::MXd constDense = ES::MXd(dfdaConst);
  EXPECT_TRUE(accum.isApprox(constDense, 1e-9))
    << "Constant dfda columns must equal the sum of the elementwise per-element columns.";
}

TEST(DeformationModelAssemblerGTest, PlasticParamJacobianMatchesFiniteDifference)
{
  using pgo::SolidDeformationModel::PlasticMaterialFieldType;

  pgo::Logging::init();

  // Use the regular-hex cubic box: well-conditioned elements (no slivers) so the
  // SVD-based plastic gradient is smooth and central differences are reliable. The
  // torus tet mesh, by contrast, has sliver elements where the gradient has kinks
  // that wreck finite-difference accuracy.
  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> mesh(pgo::SolidDeformationModel::loadCubicMesh(&cubicMesh).release());
  ASSERT_NE(mesh, nullptr);

  const int nele = mesh->getNumElements();
  ASSERT_GT(nele, 2);

  const pgo::SolidDeformationModel::CubicLinearFormulation formulation{};
  const auto elastic = DeformationModelElasticMaterial::STABLE_NEO;
  const auto plastic = DeformationModelPlasticMaterial::VOLUMETRIC_DOF6;
  const int numPlasticParams = 6;

  // A single shared plastic set (a mild stretch so the derivatives are nonzero).
  ES::VXd plasticShared(numPlasticParams);
  plasticShared << 1.01, 0.004, -0.003, 0.994, 0.005, 1.008;

  // Tolerance for the finite-difference cross-check. A correctly-placed column with a
  // sign/factor/index bug would be off by O(1); exact column-folding correctness is
  // covered separately by ConstantFieldSharesParamColumnsAcrossElements.
  const double fdTol = 1e-6;

  struct StateAssembler
  {
    std::unique_ptr<DeformationModelAssembler> assembler;
  };
  auto build = [&](PlasticFieldInit plasticInit) {
    // Finite differences measure the true derivative of computeGradient. The SPD
    // projected Hessian is an optimizer stabilization, not the exact dP/dF.
    auto elasticField = createElasticParameterField(*mesh, elastic, ElasticFieldInit{});
    auto plasticField = createPlasticParameterField(*mesh, plastic, std::move(plasticInit));
    auto manager = std::make_shared<DeformationModelManager>(
      mesh, elastic, plastic, formulation, kExactDerivativeEnforceSpd, nullptr, nullptr);
    auto assembler = std::make_unique<DeformationModelAssembler>(
      std::move(manager), formulation, std::move(elasticField), std::move(plasticField), nullptr);
    return StateAssembler{ std::move(assembler) };
  };

  // gradient(x) as a function of the plastic parameter vector.
  auto gradientAtPlastic = [&](StateAssembler &sa, const ES::VXd &params, const ES::VXd &x) {
    sa.assembler->setPlasticValues(params);
    ES::VXd g = ES::VXd::Zero(sa.assembler->getNumDOFs());
    sa.assembler->computeGradient(x.data(), g.data());
    return g;
  };

  // Serialize gradient assembly while constructing the FD reference: otherwise
  // atomic accumulation order can add tiny run-to-run noise that FD divides by h.
  ScopedSerialTbb fdSerialTbb;
  auto fdColumn = [&](StateAssembler &sa, const ES::VXd &base, int idx, const ES::VXd &x) {
    return fivePointFiniteDifference([&](double delta) {
      ES::VXd b = base;
      b[idx] += delta;
      return gradientAtPlastic(sa, b, x);
    });
  };

  // ---- Constant field: every shared column is checked (only numPlasticParams of them). ----
  auto constantSA = build(PlasticFieldInit{ PlasticMaterialFieldType::CONSTANT, plasticShared });

  ES::VXd x = makePerturbedRestPositions(*constantSA.assembler->getDeformationModelManager().getMesh());

  ES::SpMatD dfdaConstSp = constantSA.assembler->get_dfda_Template();
  constantSA.assembler->setPlasticValues(plasticShared);
  constantSA.assembler->compute_df_da(x.data(), dfdaConstSp);
  ES::MXd dfdaConst(dfdaConstSp);
  ASSERT_EQ(dfdaConst.cols(), numPlasticParams);

  for (int j = 0; j < numPlasticParams; j++) {
    ES::VXd fd = fdColumn(constantSA, plasticShared, j, x);
    EXPECT_LT(relativeColumnError(fd, dfdaConst.col(j)), fdTol)
      << "Constant dfda column " << j << " disagrees with the finite-difference gradient.";
  }
  // Restore the field state after the perturbations.
  constantSA.assembler->setPlasticValues(plasticShared);

  // ---- Elementwise field: sample a few per-element columns (cheap) to confirm placement. ----
  ES::VXd plasticEw(static_cast<Eigen::Index>(nele) * numPlasticParams);
  for (int ei = 0; ei < nele; ei++)
    plasticEw.segment(ei * numPlasticParams, numPlasticParams) = plasticShared;
  auto elementwiseSA = build(PlasticFieldInit{ PlasticMaterialFieldType::ELEMENTWISE, plasticEw });

  ES::SpMatD dfdaEwSp = elementwiseSA.assembler->get_dfda_Template();
  elementwiseSA.assembler->setPlasticValues(plasticEw);
  elementwiseSA.assembler->compute_df_da(x.data(), dfdaEwSp);
  ES::MXd dfdaEw(dfdaEwSp);
  ASSERT_EQ(dfdaEw.cols(), nele * numPlasticParams);

  // Spot-check a few elements to confirm each element's derivative lands in its own
  // column block. Cross-element placement is additionally pinned by
  // ConstantFieldSharesParamColumnsAcrossElements.
  const int sampleEles[] = { 0, 1, 2 };
  for (int ele : sampleEles) {
    for (int j = 0; j < numPlasticParams; j++) {
      const int col = ele * numPlasticParams + j;
      ES::VXd fd = fdColumn(elementwiseSA, plasticEw, col, x);
      EXPECT_LT(relativeColumnError(fd, dfdaEw.col(col)), fdTol)
        << "Elementwise dfda column for element " << ele << ", param " << j
        << " disagrees with the finite-difference gradient.";
    }
  }
}

TEST(DeformationModelAssemblerGTest, PlasticEnergyGradientMatchesFiniteDifference)
{
  using pgo::SolidDeformationModel::PlasticMaterialFieldType;

  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> mesh(pgo::SolidDeformationModel::loadCubicMesh(&cubicMesh).release());
  ASSERT_NE(mesh, nullptr);

  const int nele = mesh->getNumElements();
  ASSERT_GT(nele, 2);
  const int numPlasticParams = 6;
  const pgo::SolidDeformationModel::CubicLinearFormulation formulation{};
  const auto elastic = DeformationModelElasticMaterial::STABLE_NEO;
  const auto plastic = DeformationModelPlasticMaterial::VOLUMETRIC_DOF6;

  ES::VXd plasticBase(static_cast<Eigen::Index>(nele) * numPlasticParams);
  for (int ei = 0; ei < nele; ei++)
    plasticBase.segment(ei * numPlasticParams, numPlasticParams) << 1.01, 0.004, -0.003, 0.994, 0.005, 1.008;

  auto sa = buildExactAssembler(
    mesh,
    elastic,
    ElasticFieldInit{},
    plastic,
    PlasticFieldInit{ PlasticMaterialFieldType::ELEMENTWISE, plasticBase },
    formulation);

  ES::VXd x = makePerturbedRestPositions(*sa.assembler->getDeformationModelManager().getMesh());

  ES::VXd analytic = ES::VXd::Zero(sa.assembler->getNumPlasticGlobalParams());
  sa.assembler->setPlasticValues(plasticBase);
  sa.assembler->computePlasticGradient(x.data(), analytic.data());
  ASSERT_EQ(analytic.size(), nele * numPlasticParams);
  EXPECT_GT(analytic.norm(), 0.0);

  ScopedSerialTbb serial;
  for (int col : { 0, numPlasticParams, 2 * numPlasticParams + 3 }) {
    auto energyAt = [&](double delta) {
      ES::VXd p = plasticBase;
      p[col] += delta;
      sa.assembler->setPlasticValues(p);
      return sa.assembler->computeEnergy(x.data());
    };
    const double h = kFiniteDifferenceStep;
    const double fd =
      (-energyAt(2.0 * h) + 8.0 * energyAt(h) - 8.0 * energyAt(-h) + energyAt(-2.0 * h)) / (12.0 * h);
    EXPECT_LT(std::abs(fd - analytic[col]) / std::max(1.0, std::abs(fd)), 1e-6)
      << "Plastic energy gradient column " << col << " disagrees with FD.";
  }
  sa.assembler->setPlasticValues(plasticBase);
}

TEST(DeformationModelAssemblerGTest, ElasticEnergyGradientMatchesFiniteDifference)
{
  using pgo::SolidDeformationModel::ElasticMaterialFieldType;
  using pgo::SolidDeformationModel::PlasticMaterialFieldType;

  pgo::Logging::init();

  pgo::Mesh::TriMeshGeo surfaceMesh;
  ASSERT_TRUE(surfaceMesh.load(kShellObjPath));

  SimulationMeshENuhMaterial mat(1000.0, 0.45, 1e-3);
  std::shared_ptr<const SimulationMesh> mesh(pgo::SolidDeformationModel::loadShellMesh(surfaceMesh, &mat).release());
  ASSERT_NE(mesh, nullptr);

  const int nele = mesh->getNumElements();
  const int numElasticParams = 5;
  const int numPlasticParams = 1;
  ES::VXd elasticBase(numElasticParams);
  elasticBase << 20000.0, 0.45, 10000.0, 0.3, 1e-3;
  ES::VXd plasticBase = ES::VXd::Constant(static_cast<Eigen::Index>(nele) * numPlasticParams, 1.2);

  auto sa = buildExactAssembler(
    mesh,
    DeformationModelElasticMaterial::KOITER_STVK,
    ElasticFieldInit{ ElasticMaterialFieldType::CONSTANT, elasticBase },
    DeformationModelPlasticMaterial::SHELL_FF_DOF1,
    PlasticFieldInit{ PlasticMaterialFieldType::ELEMENTWISE, plasticBase },
    pgo::SolidDeformationModel::KoiterShellFormulation{});

  ES::VXd x = makePerturbedRestPositions(*sa.assembler->getDeformationModelManager().getMesh());

  ES::VXd analytic = ES::VXd::Zero(sa.assembler->getNumElasticGlobalParams());
  sa.assembler->setElasticValues(elasticBase);
  sa.assembler->computeElasticGradient(x.data(), analytic.data());
  ASSERT_EQ(analytic.size(), numElasticParams);
  EXPECT_GT(analytic.norm(), 0.0);

  ScopedSerialTbb serial;
  for (int col = 0; col < numElasticParams; col++) {
    auto energyAt = [&](double delta) {
      ES::VXd p = elasticBase;
      p[col] += delta;
      sa.assembler->setElasticValues(p);
      return sa.assembler->computeEnergy(x.data());
    };
    const double h = kFiniteDifferenceStep * std::max(1.0, std::abs(elasticBase[col]));
    const double fd =
      (-energyAt(2.0 * h) + 8.0 * energyAt(h) - 8.0 * energyAt(-h) + energyAt(-2.0 * h)) / (12.0 * h);
    EXPECT_LT(std::abs(fd - analytic[col]) / std::max(1.0, std::abs(fd)), 1e-5)
      << "Elastic energy gradient column " << col << " disagrees with FD.";
  }
  sa.assembler->setElasticValues(elasticBase);
}

TEST(DeformationModelAssemblerGTest, ConstantPlasticEnergyGradientAccumulatesElementwiseGradient)
{
  using pgo::SolidDeformationModel::PlasticMaterialFieldType;

  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> mesh(pgo::SolidDeformationModel::loadCubicMesh(&cubicMesh).release());
  ASSERT_NE(mesh, nullptr);

  const int nele = mesh->getNumElements();
  ASSERT_GT(nele, 1);
  const int numPlasticParams = 6;
  const pgo::SolidDeformationModel::CubicLinearFormulation formulation{};
  const auto elastic = DeformationModelElasticMaterial::STABLE_NEO;
  const auto plastic = DeformationModelPlasticMaterial::VOLUMETRIC_DOF6;

  ES::VXd shared(numPlasticParams);
  shared << 1.01, 0.004, -0.003, 0.994, 0.005, 1.008;

  auto makeAssembler = [&](PlasticFieldInit plasticInit) {
    auto elasticField = createElasticParameterField(*mesh, elastic, ElasticFieldInit{});
    auto plasticField = createPlasticParameterField(*mesh, plastic, std::move(plasticInit));
    auto manager = std::make_shared<DeformationModelManager>(
      mesh, elastic, plastic, formulation, kExactDerivativeEnforceSpd, nullptr, nullptr);
    return std::make_unique<DeformationModelAssembler>(
      std::move(manager), formulation, std::move(elasticField), std::move(plasticField), nullptr);
  };

  auto constantAssembler = makeAssembler(PlasticFieldInit{ PlasticMaterialFieldType::CONSTANT, shared });

  ES::VXd elementwiseValues(static_cast<Eigen::Index>(nele) * numPlasticParams);
  for (int ei = 0; ei < nele; ei++)
    elementwiseValues.segment(ei * numPlasticParams, numPlasticParams) = shared;
  auto elementwiseAssembler = makeAssembler(PlasticFieldInit{ PlasticMaterialFieldType::ELEMENTWISE, elementwiseValues });

  ES::VXd x = makePerturbedRestPositions(*constantAssembler->getDeformationModelManager().getMesh());
  ES::VXd gConst = ES::VXd::Zero(constantAssembler->getNumPlasticGlobalParams());
  ES::VXd gEw = ES::VXd::Zero(elementwiseAssembler->getNumPlasticGlobalParams());

  constantAssembler->computePlasticGradient(x.data(), gConst.data());
  elementwiseAssembler->computePlasticGradient(x.data(), gEw.data());

  ASSERT_EQ(gConst.size(), numPlasticParams);
  ASSERT_EQ(gEw.size(), nele * numPlasticParams);
  ES::VXd accum = ES::VXd::Zero(numPlasticParams);
  for (int ei = 0; ei < nele; ei++)
    accum += gEw.segment(ei * numPlasticParams, numPlasticParams);

  EXPECT_TRUE(accum.isApprox(gConst, 1e-9));
}

TEST(DeformationModelAssemblerGTest, PlasticEnergyHessianMatchesFiniteDifferenceOfGradient)
{
  using pgo::SolidDeformationModel::PlasticMaterialFieldType;

  pgo::Logging::init();

  auto meshMutable = makeSingleElementCubicSimulationMesh();
  ASSERT_NE(meshMutable, nullptr);
  std::shared_ptr<const SimulationMesh> mesh(std::move(meshMutable));

  const int numPlasticParams = 6;
  ES::VXd plasticBase(numPlasticParams);
  plasticBase << 1.01, 0.004, -0.003, 0.994, 0.005, 1.008;

  auto sa = buildExactAssembler(
    mesh,
    DeformationModelElasticMaterial::STABLE_NEO,
    ElasticFieldInit{},
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    PlasticFieldInit{ PlasticMaterialFieldType::ELEMENTWISE, plasticBase },
    pgo::SolidDeformationModel::CubicLinearFormulation{});

  ES::VXd x = makePerturbedRestPositions(*sa.assembler->getDeformationModelManager().getMesh());

  ES::SpMatD hess = sa.assembler->getPlasticHessianTemplate();
  sa.assembler->setPlasticValues(plasticBase);
  sa.assembler->computePlasticHessian(x.data(), hess);
  ES::MXd analytic(hess);
  ASSERT_EQ(analytic.rows(), numPlasticParams);
  ASSERT_EQ(analytic.cols(), numPlasticParams);

  ScopedSerialTbb serial;
  ES::MXd fd(numPlasticParams, numPlasticParams);
  for (int col = 0; col < numPlasticParams; col++) {
    fd.col(col) = fivePointFiniteDifference([&](double delta) {
      ES::VXd p = plasticBase;
      p[col] += delta;
      sa.assembler->setPlasticValues(p);
      ES::VXd g = ES::VXd::Zero(numPlasticParams);
      sa.assembler->computePlasticGradient(x.data(), g.data());
      return g;
    });
  }
  EXPECT_LT((fd - analytic).norm() / std::max(1.0, analytic.norm()), 1e-6);
  sa.assembler->setPlasticValues(plasticBase);
}

TEST(DeformationModelAssemblerGTest, ElasticEnergyHessianMatchesFiniteDifferenceOfGradient)
{
  using pgo::SolidDeformationModel::ElasticMaterialFieldType;
  using pgo::SolidDeformationModel::PlasticMaterialFieldType;

  pgo::Logging::init();

  pgo::Mesh::TriMeshGeo surfaceMesh;
  ASSERT_TRUE(surfaceMesh.load(kShellObjPath));

  SimulationMeshENuhMaterial mat(1000.0, 0.45, 1e-3);
  std::shared_ptr<const SimulationMesh> mesh(pgo::SolidDeformationModel::loadShellMesh(surfaceMesh, &mat).release());
  ASSERT_NE(mesh, nullptr);

  const int nele = mesh->getNumElements();
  const int numElasticParams = 5;
  const int numPlasticParams = 1;
  ES::VXd elasticBase(numElasticParams);
  elasticBase << 20000.0, 0.45, 10000.0, 0.3, 1e-3;
  ES::VXd plasticBase = ES::VXd::Constant(static_cast<Eigen::Index>(nele) * numPlasticParams, 1.2);

  auto sa = buildExactAssembler(
    mesh,
    DeformationModelElasticMaterial::KOITER_STVK,
    ElasticFieldInit{ ElasticMaterialFieldType::CONSTANT, elasticBase },
    DeformationModelPlasticMaterial::SHELL_FF_DOF1,
    PlasticFieldInit{ PlasticMaterialFieldType::ELEMENTWISE, plasticBase },
    pgo::SolidDeformationModel::KoiterShellFormulation{});

  ES::VXd x = makePerturbedRestPositions(*sa.assembler->getDeformationModelManager().getMesh());

  ES::SpMatD hess = sa.assembler->getElasticHessianTemplate();
  sa.assembler->setElasticValues(elasticBase);
  sa.assembler->computeElasticHessian(x.data(), hess);
  ES::MXd analytic(hess);
  ASSERT_EQ(analytic.rows(), numElasticParams);
  ASSERT_EQ(analytic.cols(), numElasticParams);

  ScopedSerialTbb serial;
  ES::MXd fd(numElasticParams, numElasticParams);
  for (int col = 0; col < numElasticParams; col++) {
    const double h = kFiniteDifferenceStep * std::max(1.0, std::abs(elasticBase[col]));
    fd.col(col) = fivePointFiniteDifference([&](double delta) {
      ES::VXd p = elasticBase;
      p[col] += delta;
      sa.assembler->setElasticValues(p);
      ES::VXd g = ES::VXd::Zero(numElasticParams);
      sa.assembler->computeElasticGradient(x.data(), g.data());
      return g;
    },
      h);
  }
  EXPECT_LT((fd - analytic).norm() / std::max(1.0, analytic.norm()), 1e-5);
  sa.assembler->setElasticValues(elasticBase);
}

TEST(DeformationModelAssemblerGTest, PlasticElasticEnergyHessianMatchesFiniteDifferenceOfPlasticGradient)
{
  using pgo::SolidDeformationModel::ElasticMaterialFieldType;
  using pgo::SolidDeformationModel::PlasticMaterialFieldType;

  pgo::Logging::init();

  pgo::Mesh::TriMeshGeo surfaceMesh;
  ASSERT_TRUE(surfaceMesh.load(kShellObjPath));

  SimulationMeshENuhMaterial mat(1000.0, 0.45, 1e-3);
  std::shared_ptr<const SimulationMesh> mesh(pgo::SolidDeformationModel::loadShellMesh(surfaceMesh, &mat).release());
  ASSERT_NE(mesh, nullptr);

  const int nele = mesh->getNumElements();
  const int numElasticParams = 5;
  const int numPlasticParams = 1;
  ES::VXd elasticBase(numElasticParams);
  elasticBase << 20000.0, 0.45, 10000.0, 0.3, 1e-3;
  ES::VXd plasticBase = ES::VXd::Constant(static_cast<Eigen::Index>(nele) * numPlasticParams, 1.2);

  auto sa = buildExactAssembler(
    mesh,
    DeformationModelElasticMaterial::KOITER_STVK,
    ElasticFieldInit{ ElasticMaterialFieldType::CONSTANT, elasticBase },
    DeformationModelPlasticMaterial::SHELL_FF_DOF1,
    PlasticFieldInit{ PlasticMaterialFieldType::ELEMENTWISE, plasticBase },
    pgo::SolidDeformationModel::KoiterShellFormulation{});

  ES::VXd x = makePerturbedRestPositions(*sa.assembler->getDeformationModelManager().getMesh());

  sa.assembler->setElasticValues(elasticBase);
  sa.assembler->setPlasticValues(plasticBase);
  ES::SpMatD hess = sa.assembler->getPlasticElasticHessianTemplate();
  sa.assembler->computePlasticElasticHessian(x.data(), hess);
  ES::MXd analytic(hess);
  ASSERT_EQ(analytic.rows(), nele * numPlasticParams);
  ASSERT_EQ(analytic.cols(), numElasticParams);

  ScopedSerialTbb serial;
  ES::MXd fd(nele * numPlasticParams, numElasticParams);
  for (int col = 0; col < numElasticParams; col++) {
    const double h = kFiniteDifferenceStep * std::max(1.0, std::abs(elasticBase[col]));
    fd.col(col) = fivePointFiniteDifference([&](double delta) {
      ES::VXd p = elasticBase;
      p[col] += delta;
      sa.assembler->setElasticValues(p);
      ES::VXd g = ES::VXd::Zero(nele * numPlasticParams);
      sa.assembler->computePlasticGradient(x.data(), g.data());
      return g;
    },
      h);
  }
  EXPECT_LT((fd - analytic).norm() / std::max(1.0, analytic.norm()), 2e-5);
  sa.assembler->setElasticValues(elasticBase);
  sa.assembler->setPlasticValues(plasticBase);
}

TEST(DeformationModelAssemblerGTest, ConstantPlasticEnergyHessianAccumulatesElementwiseHessian)
{
  using pgo::SolidDeformationModel::PlasticMaterialFieldType;

  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> mesh(pgo::SolidDeformationModel::loadCubicMesh(&cubicMesh).release());
  ASSERT_NE(mesh, nullptr);

  const int nele = mesh->getNumElements();
  ASSERT_GT(nele, 1);
  const int numPlasticParams = 6;
  const pgo::SolidDeformationModel::CubicLinearFormulation formulation{};
  const auto elastic = DeformationModelElasticMaterial::STABLE_NEO;
  const auto plastic = DeformationModelPlasticMaterial::VOLUMETRIC_DOF6;

  ES::VXd shared(numPlasticParams);
  shared << 1.01, 0.004, -0.003, 0.994, 0.005, 1.008;

  auto makeAssembler = [&](PlasticFieldInit plasticInit) {
    auto elasticField = createElasticParameterField(*mesh, elastic, ElasticFieldInit{});
    auto plasticField = createPlasticParameterField(*mesh, plastic, std::move(plasticInit));
    auto manager = std::make_shared<DeformationModelManager>(
      mesh, elastic, plastic, formulation, kExactDerivativeEnforceSpd, nullptr, nullptr);
    return std::make_unique<DeformationModelAssembler>(
      std::move(manager), formulation, std::move(elasticField), std::move(plasticField), nullptr);
  };

  auto constantAssembler = makeAssembler(PlasticFieldInit{ PlasticMaterialFieldType::CONSTANT, shared });

  ES::VXd elementwiseValues(static_cast<Eigen::Index>(nele) * numPlasticParams);
  for (int ei = 0; ei < nele; ei++)
    elementwiseValues.segment(ei * numPlasticParams, numPlasticParams) = shared;
  auto elementwiseAssembler = makeAssembler(PlasticFieldInit{ PlasticMaterialFieldType::ELEMENTWISE, elementwiseValues });

  ES::VXd x = makePerturbedRestPositions(*constantAssembler->getDeformationModelManager().getMesh());
  ES::SpMatD hConstSp = constantAssembler->getPlasticHessianTemplate();
  ES::SpMatD hEwSp = elementwiseAssembler->getPlasticHessianTemplate();
  constantAssembler->computePlasticHessian(x.data(), hConstSp);
  elementwiseAssembler->computePlasticHessian(x.data(), hEwSp);

  ES::MXd hConst(hConstSp);
  ES::MXd hEw(hEwSp);
  ES::MXd accum = ES::MXd::Zero(numPlasticParams, numPlasticParams);
  for (int ei = 0; ei < nele; ei++) {
    const int row0 = ei * numPlasticParams;
    accum += hEw.block(row0, row0, numPlasticParams, numPlasticParams);
  }

  EXPECT_TRUE(accum.isApprox(hConst, 1e-9));
}

TEST(DeformationModelAssemblerGTest, TetVonMisesStressIsZeroAtRestAndNonzeroUnderStretch)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  std::shared_ptr<const SimulationMesh> mesh(pgo::SolidDeformationModel::loadTetMesh(&tetMesh).release());
  ASSERT_NE(mesh, nullptr);

  const int nele = mesh->getNumElements();
  const int nvtx = mesh->getNumVertices();

  pgo::SolidDeformationModel::TetLinearFormulation formulation;
  auto managerFields = makeFieldBackedManager(
    mesh, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, DeformationModelElasticMaterial::STABLE_NEO, formulation);

  const int numPlasticParams = managerFields.manager->getNumPlasticParameters();
  const int numElasticParams = managerFields.manager->getNumElasticParameters();

  auto assembler = std::make_unique<DeformationModelAssembler>(
    managerFields.manager, formulation, managerFields.elasticField, managerFields.plasticField, nullptr);

  ES::VXd plasticParams(numPlasticParams * nele);
  ES::VXd elasticParams(numElasticParams * nele);
  elasticParams.setZero();

  for (int ei = 0; ei < nele; ei++) {
    assembler->getDeformationModelManager().getDeformationModel(ei)->defaultPlasticParams(
      plasticParams.data() + ei * numPlasticParams);
  }

  setFieldDataIfPresent(*managerFields.elasticField, elasticParams);
  setFieldDataIfPresent(*managerFields.plasticField, plasticParams);

  const auto &meshPtr = *assembler->getDeformationModelManager().getMesh();
  ES::VXd rest = makeRestPositions(meshPtr);
  ES::VXd stresses = ES::VXd::Constant(nele, -1.0);
  assembler->computeVonMisesStresses(rest.data(), stresses.data());
  expectAllFinite(stresses);
  EXPECT_LE(stresses.cwiseAbs().maxCoeff(), 1e-8);

  ES::VXd stretched = rest;
  for (int vi = 0; vi < nvtx; vi++) {
    stretched[vi * 3] *= 1.01;
  }
  stresses.setConstant(-1.0);
  assembler->computeVonMisesStresses(stretched.data(), stresses.data());
  expectAllFinite(stresses);
  EXPECT_GE(stresses.minCoeff(), 0.0);
  EXPECT_GT(stresses.maxCoeff(), 1e-8);
}

TEST(DeformationModelAssemblerGTest, ShellAssemblerRegression)
{
  pgo::Logging::init();

  pgo::Mesh::TriMeshGeo surfaceMesh;
  ASSERT_TRUE(surfaceMesh.load(kShellObjPath));

  SimulationMeshENuhMaterial mat(1000.0, 0.45, 1e-3);
  std::shared_ptr<const SimulationMesh> mesh(pgo::SolidDeformationModel::loadShellMesh(surfaceMesh, &mat).release());
  ASSERT_NE(mesh, nullptr);

  const int nele = mesh->getNumElements();

  pgo::SolidDeformationModel::KoiterShellFormulation formulation;
  auto managerFields = makeFieldBackedManager(
    mesh, DeformationModelPlasticMaterial::SHELL_FF_DOF1, DeformationModelElasticMaterial::KOITER_STVK, formulation);

  const int numPlasticParams = managerFields.manager->getNumPlasticParameters();
  const int numElasticParams = managerFields.manager->getNumElasticParameters();

  auto assembler = std::make_unique<DeformationModelAssembler>(
    managerFields.manager, formulation, managerFields.elasticField, managerFields.plasticField, nullptr);

  const auto &meshPtr = *assembler->getDeformationModelManager().getMesh();
  ES::VXd x = makePerturbedRestPositions(meshPtr);
  ES::VXd plasticParams = ES::VXd::Constant(numPlasticParams * nele, 1.2);
  ES::VXd elasticParams(numElasticParams * nele);

  ASSERT_EQ(numElasticParams, 5);
  for (int ei = 0; ei < nele; ei++) {
    elasticParams.segment<5>(ei * 5) << 20000.0, 0.45, 10000.0, 0.3, 1e-3;
  }
  setFieldDataIfPresent(*managerFields.elasticField, elasticParams);
  setFieldDataIfPresent(*managerFields.plasticField, plasticParams);

  ES::VXd grad = ES::VXd::Zero(assembler->getNumDOFs());
  assembler->computeGradient(x.data(), grad.data());
  EXPECT_EQ(grad.size(), assembler->getNumDOFs());
  expectAllFinite(grad);

  ES::SpMatD hess = assembler->getHessianTemplate();
  assembler->computeHessian(x.data(), hess);
  EXPECT_EQ(hess.rows(), assembler->getNumDOFs());
  EXPECT_EQ(hess.cols(), assembler->getNumDOFs());
  expectAllFinite(hess);

  ES::SpMatD dfda = assembler->get_dfda_Template();
  assembler->compute_df_da(x.data(), dfda);
  EXPECT_EQ(dfda.rows(), assembler->getNumDOFs());
  EXPECT_EQ(dfda.cols(), nele * numPlasticParams);
  expectAllFinite(dfda);

  if (numElasticParams > 0) {
    ES::SpMatD dfdb = assembler->get_dfdb_Template();
    assembler->compute_df_db(x.data(), dfdb);
    EXPECT_EQ(dfdb.rows(), assembler->getNumDOFs());
    EXPECT_EQ(dfdb.cols(), nele * numElasticParams);
    expectAllFinite(dfdb);
  }
}

TEST(DeformationModelAssemblerGTest, CubicAssemblerSmokeRegression)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> mesh(pgo::SolidDeformationModel::loadCubicMesh(&cubicMesh).release());
  ASSERT_NE(mesh, nullptr);

  const int nele = mesh->getNumElements();

  pgo::SolidDeformationModel::CubicLinearFormulation formulation;
  auto managerFields = makeFieldBackedManager(
    mesh, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, DeformationModelElasticMaterial::STABLE_NEO, formulation);

  const int numPlasticParams = managerFields.manager->getNumPlasticParameters();
  const int numElasticParams = managerFields.manager->getNumElasticParameters();

  auto assembler = std::make_unique<DeformationModelAssembler>(
    managerFields.manager, formulation, managerFields.elasticField, managerFields.plasticField, nullptr);

  const auto &meshPtr = *assembler->getDeformationModelManager().getMesh();
  ES::VXd x = makePerturbedRestPositions(meshPtr);
  ES::VXd plasticParams(numPlasticParams * nele);
  ES::VXd elasticParams(numElasticParams * nele);
  elasticParams.setZero();

  for (int ei = 0; ei < nele; ei++) {
    assembler->getDeformationModelManager().getDeformationModel(ei)->defaultPlasticParams(
      plasticParams.data() + ei * numPlasticParams);
  }
  setFieldDataIfPresent(*managerFields.elasticField, elasticParams);
  setFieldDataIfPresent(*managerFields.plasticField, plasticParams);

  ES::VXd grad = ES::VXd::Zero(assembler->getNumDOFs());
  assembler->computeGradient(x.data(), grad.data());
  EXPECT_EQ(grad.size(), assembler->getNumDOFs());
  expectAllFinite(grad);

  ES::SpMatD hess = assembler->getHessianTemplate();
  assembler->computeHessian(x.data(), hess);
  EXPECT_EQ(hess.rows(), assembler->getNumDOFs());
  EXPECT_EQ(hess.cols(), assembler->getNumDOFs());
  expectAllFinite(hess);

  ES::SpMatD dfda = assembler->get_dfda_Template();
  assembler->compute_df_da(x.data(), dfda);
  EXPECT_EQ(dfda.rows(), assembler->getNumDOFs());
  EXPECT_EQ(dfda.cols(), nele * numPlasticParams);
  expectAllFinite(dfda);
}

TEST(DeformationModelAssemblerGTest, CubicAssemblerMaterialParamRegression)
{
  pgo::Logging::init();

  auto meshMutable = makeSingleElementCubicSimulationMesh();
  ASSERT_NE(meshMutable, nullptr);

  pgo::SolidDeformationModel::SimulationMeshHillMaterial hillMaterial(2500.0, 0.35, 1.0);
  meshMutable->appendMaterialToAllElements(&hillMaterial);
  std::shared_ptr<const SimulationMesh> mesh(std::move(meshMutable));

  const int nele = mesh->getNumElements();
  const int nvtx = mesh->getNumVertices();

  ES::VXd elementFiberDirections = ES::VXd::Zero(nele * 3);
  for (int ei = 0; ei < nele; ei++) {
    elementFiberDirections.segment<3>(ei * 3) << 1.0, 0.0, 0.0;
  }

  ES::VXd vertexFiberDirections = ES::VXd::Zero(nvtx * 3);
  for (int vi = 0; vi < nvtx; vi++) {
    vertexFiberDirections.segment<3>(vi * 3) << 1.0, 0.0, 0.0;
  }

  pgo::SolidDeformationModel::CubicLinearFormulation formulation;
  auto managerFields = makeFieldBackedManager(
    mesh,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    DeformationModelElasticMaterial::HILL_STABLE_NEO,
    formulation,
    1,
    elementFiberDirections.data(),
    vertexFiberDirections.data());

  const int numPlasticParams = managerFields.manager->getNumPlasticParameters();
  const int numElasticParams = managerFields.manager->getNumElasticParameters();
  ASSERT_EQ(numElasticParams, 1);

  auto assembler = std::make_unique<DeformationModelAssembler>(
    managerFields.manager, formulation, managerFields.elasticField, managerFields.plasticField, nullptr);

  const auto &meshPtr = *assembler->getDeformationModelManager().getMesh();
  ES::VXd x = makePerturbedRestPositions(meshPtr);
  ES::VXd plasticParams(numPlasticParams * nele);
  ES::VXd elasticParams = ES::VXd::Constant(numElasticParams * nele, 0.75);

  for (int ei = 0; ei < nele; ei++) {
    assembler->getDeformationModelManager().getDeformationModel(ei)->defaultPlasticParams(
      plasticParams.data() + ei * numPlasticParams);
  }
  setFieldDataIfPresent(*managerFields.elasticField, elasticParams);
  setFieldDataIfPresent(*managerFields.plasticField, plasticParams);

  ES::SpMatD dfdb = assembler->get_dfdb_Template();
  assembler->compute_df_db(x.data(), dfdb);
  EXPECT_EQ(dfdb.rows(), assembler->getNumDOFs());
  EXPECT_EQ(dfdb.cols(), nele * numElasticParams);
  expectAllFinite(dfdb);
  EXPECT_GT(dfdb.norm(), 0.0);
}

// dfdb == d(gradient)/d(elastic parameter): cross-check the assembled elastic-parameter
// Jacobian against a finite difference of the gradient. STABLE_NEO exposes zero
// differentiable elastic parameters, so we use a Hill-type material (one activation
// parameter) on a single well-conditioned hex.
TEST(DeformationModelAssemblerGTest, CubicElasticParamJacobianMatchesFiniteDifference)
{
  using pgo::SolidDeformationModel::ElasticMaterialFieldType;

  pgo::Logging::init();

  auto meshMutable = makeSingleElementCubicSimulationMesh();
  ASSERT_NE(meshMutable, nullptr);

  pgo::SolidDeformationModel::SimulationMeshHillMaterial hillMaterial(2500.0, 0.35, 1.0);
  meshMutable->appendMaterialToAllElements(&hillMaterial);
  std::shared_ptr<const SimulationMesh> mesh(std::move(meshMutable));

  const int nele = mesh->getNumElements();
  const int nvtx = mesh->getNumVertices();

  ES::VXd elementFiberDirections = ES::VXd::Zero(nele * 3);
  for (int ei = 0; ei < nele; ei++)
    elementFiberDirections.segment<3>(ei * 3) << 1.0, 0.0, 0.0;
  ES::VXd vertexFiberDirections = ES::VXd::Zero(nvtx * 3);
  for (int vi = 0; vi < nvtx; vi++)
    vertexFiberDirections.segment<3>(vi * 3) << 1.0, 0.0, 0.0;

  // Activation level slightly off rest so the derivative is nonzero.
  const int numElasticParams = 1;
  ES::VXd elasticBase = ES::VXd::Constant(nele * numElasticParams, 0.75);

  auto sa = buildExactAssembler(
    mesh,
    DeformationModelElasticMaterial::HILL_STABLE_NEO,
    ElasticFieldInit{ ElasticMaterialFieldType::ELEMENTWISE, elasticBase },
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    PlasticFieldInit{},
    pgo::SolidDeformationModel::CubicLinearFormulation{},
    elementFiberDirections.data(),
    vertexFiberDirections.data());
  ASSERT_EQ(sa.assembler->getNumElasticParams(), numElasticParams);

  // Set plastic to identity so the Hill material sits at a smooth operating point.
  const int numPlasticParams = sa.assembler->getNumPlasticParams();
  ES::VXd plasticParams(numPlasticParams * nele);
  for (int ei = 0; ei < nele; ei++)
    sa.assembler->getDeformationModelManager().getDeformationModel(ei)->defaultPlasticParams(
      plasticParams.data() + ei * numPlasticParams);
  sa.assembler->setPlasticValues(plasticParams);

  ES::VXd x = makePerturbedRestPositions(*sa.assembler->getDeformationModelManager().getMesh());

  sa.assembler->setElasticValues(elasticBase);
  ES::SpMatD dfdbSp = sa.assembler->get_dfdb_Template();
  sa.assembler->compute_df_db(x.data(), dfdbSp);
  ES::MXd dfdb(dfdbSp);
  ASSERT_EQ(dfdb.cols(), nele * numElasticParams);

  ScopedSerialTbb fdSerialTbb;
  auto setElastic = [&](const ES::VXd &p) {
    sa.assembler->setElasticValues(p);
  };
  for (int j = 0; j < dfdb.cols(); j++) {
    ES::VXd fd = fdGradientColumn(*sa.assembler, setElastic, elasticBase, j, x);
    EXPECT_LT(relativeColumnError(fd, dfdb.col(j)), 1e-6)
      << "Cubic dfdb column " << j << " disagrees with the finite-difference gradient.";
  }
  sa.assembler->setElasticValues(elasticBase);
}

// Shell dfda == d(gradient)/d(plastic parameter): the shell formulation has its own
// element mapping and (for the Hessian) an eigenvalue clamp, so it needs an
// independent FD net. The clamp does not touch the gradient, so FD of the gradient
// is a valid reference for the plastic Jacobian.
TEST(DeformationModelAssemblerGTest, ShellPlasticParamJacobianMatchesFiniteDifference)
{
  using pgo::SolidDeformationModel::ElasticMaterialFieldType;
  using pgo::SolidDeformationModel::PlasticMaterialFieldType;

  pgo::Logging::init();

  pgo::Mesh::TriMeshGeo surfaceMesh;
  ASSERT_TRUE(surfaceMesh.load(kShellObjPath));

  SimulationMeshENuhMaterial mat(1000.0, 0.45, 1e-3);
  std::shared_ptr<const SimulationMesh> mesh(pgo::SolidDeformationModel::loadShellMesh(surfaceMesh, &mat).release());
  ASSERT_NE(mesh, nullptr);

  const int nele = mesh->getNumElements();
  const int numPlasticParams = 1;  // SHELL_FF_DOF1

  ES::VXd plasticBase = ES::VXd::Constant(static_cast<Eigen::Index>(nele) * numPlasticParams, 1.2);

  auto sa = buildExactAssembler(
    mesh,
    DeformationModelElasticMaterial::KOITER_STVK,
    ElasticFieldInit{},
    DeformationModelPlasticMaterial::SHELL_FF_DOF1,
    PlasticFieldInit{ PlasticMaterialFieldType::ELEMENTWISE, plasticBase },
    pgo::SolidDeformationModel::KoiterShellFormulation{});
  ASSERT_EQ(sa.assembler->getNumPlasticParams(), numPlasticParams);

  const int numElasticParams = sa.assembler->getNumElasticParams();
  ASSERT_EQ(numElasticParams, 5);
  ES::VXd elasticBase(static_cast<Eigen::Index>(nele) * numElasticParams);
  for (int ei = 0; ei < nele; ei++)
    elasticBase.segment<5>(ei * 5) << 20000.0, 0.45, 10000.0, 0.3, 1e-3;
  sa.assembler->setElasticValues(elasticBase);

  ES::VXd x = makePerturbedRestPositions(*sa.assembler->getDeformationModelManager().getMesh());

  sa.assembler->setPlasticValues(plasticBase);
  ES::SpMatD dfdaSp = sa.assembler->get_dfda_Template();
  sa.assembler->compute_df_da(x.data(), dfdaSp);
  ES::MXd dfda(dfdaSp);
  ASSERT_EQ(dfda.cols(), nele * numPlasticParams);

  ScopedSerialTbb fdSerialTbb;
  auto setPlastic = [&](const ES::VXd &p) {
    sa.assembler->setPlasticValues(p);
  };
  // Spot-check a few elements (each column is a full-gradient FD, so keep it cheap).
  const int sampleEles[] = { 0, 1, nele / 2 };
  for (int ele : sampleEles) {
    const int col = ele * numPlasticParams;  // single plastic param per element
    ES::VXd fd = fdGradientColumn(*sa.assembler, setPlastic, plasticBase, col, x);
    EXPECT_LT(relativeColumnError(fd, dfda.col(col)), 1e-5)
      << "Shell dfda column for element " << ele << " disagrees with the finite-difference gradient.";
  }
  sa.assembler->setPlasticValues(plasticBase);
}

// Shell dfdb == d(gradient)/d(elastic parameter), exercising all five KOITER_STVK
// elastic channels [stretchE, stretchNu, bendE, bendNu, thickness]. This pins both
// the per-channel derivatives and the column placement of the elastic Jacobian.
TEST(DeformationModelAssemblerGTest, ShellElasticParamJacobianMatchesFiniteDifference)
{
  using pgo::SolidDeformationModel::ElasticMaterialFieldType;
  using pgo::SolidDeformationModel::PlasticMaterialFieldType;

  pgo::Logging::init();

  pgo::Mesh::TriMeshGeo surfaceMesh;
  ASSERT_TRUE(surfaceMesh.load(kShellObjPath));

  SimulationMeshENuhMaterial mat(1000.0, 0.45, 1e-3);
  std::shared_ptr<const SimulationMesh> mesh(pgo::SolidDeformationModel::loadShellMesh(surfaceMesh, &mat).release());
  ASSERT_NE(mesh, nullptr);

  const int nele = mesh->getNumElements();
  const int numElasticParams = 5;  // KOITER_STVK
  const int numPlasticParams = 1;  // SHELL_FF_DOF1

  ES::VXd elasticBase(static_cast<Eigen::Index>(nele) * numElasticParams);
  for (int ei = 0; ei < nele; ei++)
    elasticBase.segment<5>(ei * 5) << 20000.0, 0.45, 10000.0, 0.3, 1e-3;

  ES::VXd plasticBase = ES::VXd::Constant(static_cast<Eigen::Index>(nele) * numPlasticParams, 1.2);

  auto sa = buildExactAssembler(
    mesh,
    DeformationModelElasticMaterial::KOITER_STVK,
    ElasticFieldInit{ ElasticMaterialFieldType::ELEMENTWISE, elasticBase },
    DeformationModelPlasticMaterial::SHELL_FF_DOF1,
    PlasticFieldInit{ PlasticMaterialFieldType::ELEMENTWISE, plasticBase },
    pgo::SolidDeformationModel::KoiterShellFormulation{});
  ASSERT_EQ(sa.assembler->getNumElasticParams(), numElasticParams);

  ES::VXd x = makePerturbedRestPositions(*sa.assembler->getDeformationModelManager().getMesh());

  sa.assembler->setElasticValues(elasticBase);
  ES::SpMatD dfdbSp = sa.assembler->get_dfdb_Template();
  sa.assembler->compute_df_db(x.data(), dfdbSp);
  ES::MXd dfdb(dfdbSp);
  ASSERT_EQ(dfdb.cols(), nele * numElasticParams);

  ScopedSerialTbb fdSerialTbb;
  auto setElastic = [&](const ES::VXd &p) {
    sa.assembler->setElasticValues(p);
  };
  // Check every elastic channel on a couple of elements.
  const int sampleEles[] = { 0, nele / 2 };
  for (int ele : sampleEles) {
    for (int c = 0; c < numElasticParams; c++) {
      const int col = ele * numElasticParams + c;
      ES::VXd fd = fdGradientColumn(*sa.assembler, setElastic, elasticBase, col, x);
      EXPECT_LT(relativeColumnError(fd, dfdb.col(col)), 1e-5)
        << "Shell dfdb column for element " << ele << ", channel " << c
        << " disagrees with the finite-difference gradient.";
    }
  }
  sa.assembler->setElasticValues(elasticBase);
}

// dfdb for the anisotropic Koiter fabric across all 12 elastic channels
// [mu0, k1_warp, k2_warp, k1_weft, k2_weft, ks, alpha, kappa11, kappa22, kappa12,
//  I8_0, thickness]. This exercises compute_dP_dparam for every channel of the
// fabric model (whose membrane energy uses exponential fiber laws).
TEST(DeformationModelAssemblerGTest, ShellFabricElasticParamJacobianMatchesFiniteDifference)
{
  using pgo::SolidDeformationModel::ElasticMaterialFieldType;
  using pgo::SolidDeformationModel::PlasticMaterialFieldType;

  pgo::Logging::init();

  pgo::Mesh::TriMeshGeo surfaceMesh;
  ASSERT_TRUE(surfaceMesh.load(kShellObjPath));

  SimulationMeshENuhMaterial mat(1000.0, 0.45, 1e-3);
  std::shared_ptr<const SimulationMesh> mesh(pgo::SolidDeformationModel::loadShellMesh(surfaceMesh, &mat).release());
  ASSERT_NE(mesh, nullptr);

  const int nele = mesh->getNumElements();
  const int numElasticParams = 12;  // KOITER_FABRIC
  const int numPlasticParams = 1;   // SHELL_FF_DOF1

  ES::VXd channelTemplate(numElasticParams);
  channelTemplate << 1, 1, 1, 1, 1, 1, 1, 1000, 1000, 1000, 1, 1e-3;
  ES::VXd elasticBase(static_cast<Eigen::Index>(nele) * numElasticParams);
  for (int ei = 0; ei < nele; ei++)
    elasticBase.segment<12>(ei * 12) = channelTemplate;

  ES::VXd plasticBase = ES::VXd::Constant(static_cast<Eigen::Index>(nele) * numPlasticParams, 1.2);

  auto sa = buildExactAssembler(
    mesh,
    DeformationModelElasticMaterial::KOITER_FABRIC,
    ElasticFieldInit{ ElasticMaterialFieldType::ELEMENTWISE, elasticBase },
    DeformationModelPlasticMaterial::SHELL_FF_DOF1,
    PlasticFieldInit{ PlasticMaterialFieldType::ELEMENTWISE, plasticBase },
    pgo::SolidDeformationModel::KoiterShellFormulation{});
  ASSERT_EQ(sa.assembler->getNumElasticParams(), numElasticParams);

  ES::VXd x = makePerturbedRestPositions(*sa.assembler->getDeformationModelManager().getMesh());

  sa.assembler->setElasticValues(elasticBase);
  ES::SpMatD dfdbSp = sa.assembler->get_dfdb_Template();
  sa.assembler->compute_df_db(x.data(), dfdbSp);
  ES::MXd dfdb(dfdbSp);
  ASSERT_EQ(dfdb.cols(), nele * numElasticParams);

  ScopedSerialTbb fdSerialTbb;
  auto setElastic = [&](const ES::VXd &p) {
    sa.assembler->setElasticValues(p);
  };

  const int sampleEles[] = { 0, nele / 2 };
  for (int ele : sampleEles) {
    for (int c = 0; c < numElasticParams; c++) {
      const int col = ele * numElasticParams + c;
      ES::VXd fd = fdGradientColumn(*sa.assembler, setElastic, elasticBase, col, x);
      const double err = relativeColumnError(fd, dfdb.col(col));
      EXPECT_LT(err, 1e-5)
        << "Fabric dfdb column for element " << ele << ", channel " << c
        << " disagrees with the finite-difference gradient.";
    }
  }
  sa.assembler->setElasticValues(elasticBase);
}
