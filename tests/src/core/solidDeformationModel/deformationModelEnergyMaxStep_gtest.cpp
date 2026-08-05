#include <gtest/gtest.h>
#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModel2DFundamentalFormsSTVK.h"
#include "material/plastic/plasticModel3D3DOF.h"
#include "material/plastic/plasticModel3D6DOF.h"
#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"

#include "EigenSupport.h"
#include "deformation/deformationModelAssembler.h"
#include "energy/deformationEnergyOperator.h"
#include "deformation/deformationModelManager.h"
#include "backwardEuler/backwardEulerStepper.h"
#include "trbdf2/trbdf2Stepper.h"
#include "dynamicStepper.h"
#include "energy/energySet.h"
#include "formulations/formulation/formulations.h"
#include "pgoLogging.h"
#include "energy/potentialEnergy.h"
#include "simulation/simulationMesh.h"
#include "formulations/shapeFunction/cubicLinearShapeFunction.h"
#include "formulations/shapeFunction/tetLinearShapeFunction.h"
#include "formulations/quadrature/gaussLegendreHexQuadrature.h"
#include "deformation/volume/volumetricElementMapping.h"
#include "deformation/volume/volumetricDeformationModel.h"
#include "triMeshGeo.h"
#include "materialTestUtils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <span>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;
using pgo::NonlinearOptimization::PotentialEnergy;
using pgo::SolidDeformationModel::DeformationModelAssembler;
using pgo::SolidDeformationModel::ElasticModelDefinition;
using pgo::SolidDeformationModel::DeformationEnergyOperator;
using pgo::SolidDeformationModel::DeformationPotentialEnergy;
using pgo::SolidDeformationModel::DeformationModelManager;
using pgo::SolidDeformationModel::PlasticModelDefinition;
using pgo::SolidDeformationModel::SimulationMesh;
using pgo::SolidDeformationModel::SimulationMeshType;
using pgo::SolidDeformationModel::tetLinearComputeDs;
using CubicFEM = pgo::SolidDeformationModel::VolumetricDeformationModel;
using pgo::NonlinearOptimization::SolveDiagnostics;
using pgo::NonlinearOptimization::StepSource;
using pgo::NonlinearOptimization::StepConstraint;
constexpr int src(StepSource s) { return static_cast<int>(s); }

constexpr const char *kShellObjPath = LIBPGO_TEST_SHELL_OBJ;

void initializeLogging()
{
  static const bool initialized = []() {
    pgo::Logging::init();
    return true;
  }();
  (void)initialized;
}

struct EnergyFixture
{
  std::shared_ptr<const SimulationImportResult> asset;
  std::shared_ptr<const SimulationMesh> meshOwner;
  std::shared_ptr<DeformationPotentialEnergy> energy;
  ES::VXd restPositions;

  // Borrow accessors through the unique_ptr spine (energy -> assembler -> manager -> mesh).
  const SimulationMesh &mesh() const { return energy->assembler().getDeformationModelManager().getMesh(); }
  const DeformationModelManager &manager() const { return energy->assembler().getDeformationModelManager(); }
  const DeformationModelAssembler &assembler() const { return energy->assembler(); }
};

ES::VXd gatherRestPositions(const SimulationMesh &mesh)
{
  ES::VXd rest(mesh.getNumVertices() * 3);
  for (int vi = 0; vi < mesh.getNumVertices(); vi++) {
    rest.segment<3>(vi * 3) = mesh.getVertex(vi);
  }
  return rest;
}

EnergyFixture makeTetFixture(
  const std::vector<double> &vertices, const std::vector<int> &elementVertices, int offset = 0)
{
  initializeLogging();

  EnergyFixture fixture;
  fixture.meshOwner = std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    static_cast<int>(vertices.size() / 3), vertices,
    static_cast<int>(elementVertices.size() / 4), 4, elementVertices,
    SimulationMeshType::TET));
  fixture.asset = TestUtils::makeENuAsset(
    fixture.meshOwner, 1200.0, 0.45);

  fixture.restPositions = gatherRestPositions(*fixture.meshOwner);

  pgo::SolidDeformationModel::TetLinearFormulation formulation;
  auto parameters = TestUtils::makeDefaultMaterialState(
    *fixture.asset, *std::make_shared<StableNeoDefinition>(),
    *std::make_shared<VolumetricPlasticity6Definition>());
  auto material = TestUtils::makeMaterialBinding(
    fixture.asset, std::make_shared<StableNeoDefinition>(),
    std::make_shared<VolumetricPlasticity6Definition>(), parameters);
  DeformationModelOptions options;
  options.dofOffset = offset;
  auto energyOperator = std::make_shared<DeformationEnergyOperator>(
    fixture.asset->mesh(), material.binding, formulation, options);
  fixture.energy = std::make_shared<DeformationPotentialEnergy>(
    std::move(energyOperator), *material.state);
  return fixture;
}

EnergyFixture makeSingleTetFixture(int offset = 0)
{
  const std::vector<double> vertices = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
  const std::vector<int> elementVertices = { 0, 1, 2, 3 };
  return makeTetFixture(vertices, elementVertices, offset);
}

EnergyFixture makeCubicFixture(const std::vector<double> &vertices, const std::vector<int> &elementVertices)
{
  initializeLogging();

  EnergyFixture fixture;
  fixture.meshOwner = std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    static_cast<int>(vertices.size() / 3), vertices,
    static_cast<int>(elementVertices.size() / 8), 8, elementVertices,
    SimulationMeshType::CUBIC));
  fixture.asset = TestUtils::makeENuAsset(
    fixture.meshOwner, 1200.0, 0.45);

  fixture.restPositions = gatherRestPositions(*fixture.meshOwner);

  pgo::SolidDeformationModel::CubicLinearFormulation formulation;
  auto parameters = TestUtils::makeDefaultMaterialState(
    *fixture.asset, *std::make_shared<StableNeoDefinition>(),
    *std::make_shared<VolumetricPlasticity6Definition>());
  auto material = TestUtils::makeMaterialBinding(
    fixture.asset, std::make_shared<StableNeoDefinition>(),
    std::make_shared<VolumetricPlasticity6Definition>(), parameters);
  auto energyOperator = std::make_shared<DeformationEnergyOperator>(
    fixture.asset->mesh(), material.binding, formulation);
  fixture.energy = std::make_shared<DeformationPotentialEnergy>(
    std::move(energyOperator), *material.state);
  return fixture;
}

EnergyFixture makeSingleCubicFixture()
{
  const std::vector<double> vertices = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    1.0, 1.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
    1.0, 0.0, 1.0,
    1.0, 1.0, 1.0,
    0.0, 1.0, 1.0,
  };
  const std::vector<int> elementVertices = { 0, 1, 2, 3, 4, 5, 6, 7 };
  return makeCubicFixture(vertices, elementVertices);
}

EnergyFixture makeShellFixture()
{
  initializeLogging();

  pgo::Mesh::TriMeshGeo surfaceMesh;
  if (!surfaceMesh.load(kShellObjPath))
    throw std::runtime_error("Failed to load shell regression mesh.");

  EnergyFixture fixture;
  fixture.asset = TestUtils::shareAsset(
    pgo::SolidDeformationModel::loadShellMesh(surfaceMesh),
    TestUtils::uniformImportedMaterialCatalog(
      surfaceMesh.numTriangles(), {"E", "nu", "h", "J"},
      {1000.0, 0.45, 1e-3, 10000.0}, "shell"));
  fixture.meshOwner = fixture.asset->mesh();

  fixture.restPositions = gatherRestPositions(*fixture.meshOwner);

  pgo::SolidDeformationModel::KoiterShellFormulation formulation;
  auto parameters = TestUtils::makeDefaultMaterialState(
    *fixture.asset, *std::make_shared<KoiterStVKDefinition>(),
    *std::make_shared<ShellPlasticity1Definition>());
  auto material = TestUtils::makeMaterialBinding(
    fixture.asset, std::make_shared<KoiterStVKDefinition>(),
    std::make_shared<ShellPlasticity1Definition>(), parameters);
  auto energyOperator = std::make_shared<DeformationEnergyOperator>(
    fixture.asset->mesh(), material.binding, formulation);
  fixture.energy = std::make_shared<DeformationPotentialEnergy>(
    std::move(energyOperator), *material.state);
  return fixture;
}

ES::VXd makeTetFlipDirection(int numVertices, int topVertex, double dz)
{
  ES::VXd dx = ES::VXd::Zero(numVertices * 3);
  dx[topVertex * 3 + 2] = dz;
  return dx;
}

void applyUniformTranslation(ES::VXd &dx, double tx, double ty, double tz)
{
  for (Eigen::Index vi = 0; vi < dx.size() / 3; vi++) {
    dx[vi * 3 + 0] = tx;
    dx[vi * 3 + 1] = ty;
    dx[vi * 3 + 2] = tz;
  }
}

ES::VXd makeCubicTopFaceDirection(int numVertices, int baseVertex, double dz)
{
  ES::VXd dx = ES::VXd::Zero(numVertices * 3);
  for (int localVertex = 4; localVertex < 8; localVertex++) {
    dx[(baseVertex + localVertex) * 3 + 2] = dz;
  }
  return dx;
}

double tetDeterminant(const SimulationMesh &mesh, int ele, const ES::VXd &absolutePositions)
{
  ES::V12d localPositions = ES::V12d::Zero();
  for (int j = 0; j < 4; j++) {
    const int vi = mesh.getVertexIndex(ele, j);
    localPositions.segment<3>(j * 3) = absolutePositions.segment<3>(vi * 3);
  }

  return tetLinearComputeDs(localPositions).determinant();
}

double minCubicDeterminant(const SimulationMesh &mesh, const DeformationModelManager &manager,
  int ele, const ES::VXd &absolutePositions)
{
  const auto *model = dynamic_cast<const CubicFEM *>(&manager.getDeformationModel(ele));
  if (model == nullptr)
    return -std::numeric_limits<double>::infinity();

  std::array<double, 24> localPositions{};
  for (int j = 0; j < 8; j++) {
    const int vi = mesh.getVertexIndex(ele, j);
    localPositions[j * 3 + 0] = absolutePositions[vi * 3 + 0];
    localPositions[j * 3 + 1] = absolutePositions[vi * 3 + 1];
    localPositions[j * 3 + 2] = absolutePositions[vi * 3 + 2];
  }

  double minDet = std::numeric_limits<double>::infinity();
  for (int q = 0; q < model->getNumMaterialLocations(); q++) {
    const ES::M3d F = model->compute_F(
      std::span<const double>(localPositions.data(), localPositions.size()), q);
    minDet = std::min(minDet, F.determinant());
  }

  return minDet;
}

std::size_t countOccurrences(const std::string &haystack, const std::string &needle)
{
  std::size_t count = 0;
  std::size_t pos = 0;
  while ((pos = haystack.find(needle, pos)) != std::string::npos) {
    count++;
    pos += needle.size();
  }
  return count;
}

class FixedMaxStepEnergy : public PotentialEnergy
{
public:
  FixedMaxStepEnergy(int numDOFs, double maxStep):
    numDOFs_(numDOFs), maxStep_(maxStep)
  {
    dofs_.resize(numDOFs_);
    std::iota(dofs_.begin(), dofs_.end(), 0);
  }

  double func(ES::ConstRefVecXd) const override { return 0.0; }
  void gradient(ES::ConstRefVecXd, ES::RefVecXd grad) const override { grad.setZero(); }
  void hessianInPlace(ES::ConstRefVecXd, ES::SpMatD &) const override {}
  void hessianAlloc(ES::SpMatD &hess) const override { hess = ES::SpMatD(numDOFs_, numDOFs_); }
  void getDOFs(std::vector<int> &dofs) const override { dofs = dofs_; }
  int getNumDOFs() const override { return numDOFs_; }
  StepConstraint computeMaxStepLimit(ES::ConstRefVecXd, ES::ConstRefVecXd, pgo::NonlinearOptimization::StepConstraintSink *sink = nullptr) const override { StepConstraint c{StepSource::Material, maxStep_}; if (sink) sink->report(c); return c; }

private:
  int numDOFs_;
  double maxStep_;
  std::vector<int> dofs_;
};

}  // namespace

TEST(DeformationEnergyOperatorMaxStepGTest, ZeroDirectionReturnsOneAndDoesNotClamp)
{
  EnergyFixture fixture = makeSingleTetFixture();
  const ES::VXd x = ES::VXd::Zero(fixture.restPositions.size());
  const ES::VXd dx = ES::VXd::Zero(fixture.restPositions.size());

  const StepConstraint result = fixture.energy->computeMaxStepLimit(x, dx);
  EXPECT_DOUBLE_EQ(result.alpha, 1.0);
  EXPECT_DOUBLE_EQ(result.alpha, 1.0);
  EXPECT_FALSE(result.clamped());
}

TEST(DeformationEnergyOperatorMaxStepGTest, TetPureTranslationReturnsOne)
{
  EnergyFixture fixture = makeSingleTetFixture();
  const ES::VXd x = ES::VXd::Zero(fixture.restPositions.size());
  ES::VXd dx = ES::VXd::Zero(fixture.restPositions.size());
  applyUniformTranslation(dx, 1.0, 2.0, 3.0);

  const StepConstraint result = fixture.energy->computeMaxStepLimit(x, dx);
  EXPECT_DOUBLE_EQ(result.alpha, 1.0);
  EXPECT_DOUBLE_EQ(result.alpha, 1.0);
  EXPECT_FALSE(result.clamped());
}

TEST(DeformationEnergyOperatorMaxStepGTest, TetShrinksBeforeInversion)
{
  EnergyFixture fixture = makeSingleTetFixture();
  const ES::VXd x = ES::VXd::Zero(fixture.restPositions.size());
  const ES::VXd dx = makeTetFlipDirection(fixture.mesh().getNumVertices(), 3, -2.0);

  const StepConstraint result = fixture.energy->computeMaxStepLimit(x, dx);
  const double alpha = result.alpha;
  ASSERT_LT(alpha, 1.0);
  ASSERT_GT(alpha, 0.0);
  EXPECT_DOUBLE_EQ(result.alpha, alpha);
  EXPECT_TRUE(result.clamped());

  const ES::VXd updatedPositions = fixture.restPositions + alpha * dx;
  EXPECT_LT(tetDeterminant(fixture.mesh(), 0, fixture.restPositions + dx), 0.0);
  EXPECT_GT(tetDeterminant(fixture.mesh(), 0, updatedPositions), 0.0);
}

TEST(DeformationEnergyOperatorMaxStepGTest, NonzeroOffsetEnergySetMapsLocalStateAndDirection)
{
  constexpr int kOffset = 6;
  EnergyFixture fixture = makeSingleTetFixture(kOffset);
  const int numLocalDOFs = fixture.energy->getNumDOFs();
  const int numGlobalDOFs = kOffset + numLocalDOFs;

  std::vector<int> dofs;
  fixture.energy->getDOFs(dofs);
  ASSERT_EQ(dofs.size(), static_cast<std::size_t>(numLocalDOFs));
  EXPECT_EQ(dofs.front(), kOffset);
  EXPECT_EQ(dofs.back(), numGlobalDOFs - 1);

  pgo::NonlinearOptimization::EnergySet energySet(
    numGlobalDOFs, { { fixture.energy, 1.0 } });

  ES::VXd localX = ES::VXd::Zero(numLocalDOFs);
  localX[0] = 1e-3;
  ES::VXd globalX = ES::VXd::Constant(numGlobalDOFs, 17.0);
  globalX.segment(kOffset, numLocalDOFs) = localX;

  const double localValue = fixture.energy->func(localX);
  EXPECT_DOUBLE_EQ(energySet.func(globalX), localValue);

  ES::VXd localGradient(numLocalDOFs);
  fixture.energy->gradient(localX, localGradient);
  ES::VXd globalGradient(numGlobalDOFs);
  energySet.gradient(globalX, globalGradient);
  EXPECT_DOUBLE_EQ(globalGradient.head(kOffset).squaredNorm(), 0.0);
  EXPECT_NEAR(
    (globalGradient.segment(kOffset, numLocalDOFs) - localGradient).norm(), 0.0, 1e-12);

  ES::SpMatD localHessian;
  fixture.energy->hessian(localX, localHessian);
  ES::SpMatD globalHessian;
  energySet.hessian(globalX, globalHessian);
  const ES::MXd globalHessianDense(globalHessian);
  EXPECT_DOUBLE_EQ(globalHessianDense.topRows(kOffset).squaredNorm(), 0.0);
  EXPECT_DOUBLE_EQ(globalHessianDense.leftCols(kOffset).squaredNorm(), 0.0);
  EXPECT_NEAR(
    (globalHessianDense.block(kOffset, kOffset, numLocalDOFs, numLocalDOFs) -
      ES::MXd(localHessian))
      .norm(),
    0.0, 1e-12);

  const ES::VXd localDx =
    makeTetFlipDirection(fixture.mesh().getNumVertices(), 3, -2.0);
  ES::VXd globalDx = ES::VXd::Constant(numGlobalDOFs, -23.0);
  globalDx.segment(kOffset, numLocalDOFs) = localDx;
  const StepConstraint localConstraint =
    fixture.energy->computeMaxStepLimit(localX, localDx);
  const StepConstraint globalConstraint =
    energySet.computeMaxStepLimit(globalX, globalDx);
  EXPECT_DOUBLE_EQ(globalConstraint.alpha, localConstraint.alpha);
  EXPECT_EQ(globalConstraint.source, localConstraint.source);

  EXPECT_THROW(fixture.energy->func(globalX), std::invalid_argument);
  EXPECT_THROW(
    fixture.energy->computeMaxStepLimit(globalX, globalDx), std::invalid_argument);
}

TEST(DeformationEnergyOperatorMaxStepGTest, DisabledMaterialMaxStepSkipsTetClamp)
{
  EnergyFixture fixture = makeSingleTetFixture();
  fixture.energy->setEnableMaterialMaxStep(false);

  const ES::VXd x = ES::VXd::Zero(fixture.restPositions.size());
  const ES::VXd dx = makeTetFlipDirection(fixture.mesh().getNumVertices(), 3, -2.0);

  const StepConstraint result = fixture.energy->computeMaxStepLimit(x, dx);
  EXPECT_DOUBLE_EQ(result.alpha, 1.0);
  EXPECT_DOUBLE_EQ(result.alpha, 1.0);
  EXPECT_FALSE(result.clamped());
}

TEST(DeformationEnergyOperatorMaxStepGTest, TetIllegalInitialStateWarnsEachCallAndClamps)
{
  EnergyFixture fixture = makeSingleTetFixture();
  ES::VXd x = ES::VXd::Zero(fixture.restPositions.size());
  ES::VXd dx = ES::VXd::Zero(fixture.restPositions.size());
  x[3 * 3 + 2] = -2.2;
  dx[0] = 0.1;

  testing::internal::CaptureStdout();
  const StepConstraint result1 = fixture.energy->computeMaxStepLimit(x, dx);
  const StepConstraint result2 = fixture.energy->computeMaxStepLimit(x, dx);
  const std::string logOutput = testing::internal::GetCapturedStdout();

  const double alpha1 = result1.alpha;
  const double alpha2 = result2.alpha;
  EXPECT_GT(alpha1, 0.0);
  EXPECT_LT(alpha1, 1e-9);
  EXPECT_DOUBLE_EQ(alpha1, alpha2);
  EXPECT_TRUE(result1.clamped());
  EXPECT_TRUE(result2.clamped());
  EXPECT_EQ(countOccurrences(logOutput, "material max step encountered illegal initial state"), 2u);
}

TEST(DeformationEnergyOperatorMaxStepGTest, TetSmallAlphaWarnsAndCanBeRecordedInDiagnostics)
{
  EnergyFixture fixture = makeSingleTetFixture();
  const ES::VXd x = ES::VXd::Zero(fixture.restPositions.size());
  const ES::VXd dx = makeTetFlipDirection(fixture.mesh().getNumVertices(), 3, -200.0);

  testing::internal::CaptureStdout();
  const StepConstraint result = fixture.energy->computeMaxStepLimit(x, dx);
  const std::string logOutput = testing::internal::GetCapturedStdout();

  const double alpha = result.alpha;
  EXPECT_GT(alpha, 0.0);
  EXPECT_LT(alpha, 0.01);
  EXPECT_TRUE(result.clamped());
  EXPECT_NE(logOutput.find("materialFeasibleAlpha"), std::string::npos);

  SolveDiagnostics diagnostics;
  diagnostics.report(result);
  EXPECT_EQ(diagnostics.clampCounts[src(StepSource::Material)], 1);
  EXPECT_DOUBLE_EQ(diagnostics.minSourceFeasibleAlpha[src(StepSource::Material)], alpha);
}

TEST(DeformationEnergyOperatorMaxStepGTest, SolveDiagnosticsResetClearsMaterialCountAndMinimumAlpha)
{
  EnergyFixture fixture = makeSingleTetFixture();
  const ES::VXd x = ES::VXd::Zero(fixture.restPositions.size());
  const ES::VXd dx = makeTetFlipDirection(fixture.mesh().getNumVertices(), 3, -2.0);

  const StepConstraint result = fixture.energy->computeMaxStepLimit(x, dx);
  ASSERT_LT(result.alpha, 1.0);
  ASSERT_TRUE(result.clamped());

  SolveDiagnostics diagnostics;
  diagnostics.report(result);
  ASSERT_EQ(diagnostics.clampCounts[src(StepSource::Material)], 1);
  ASSERT_DOUBLE_EQ(diagnostics.minSourceFeasibleAlpha[src(StepSource::Material)], result.alpha);

  diagnostics.reset();
  EXPECT_EQ(diagnostics.clampCounts[src(StepSource::Material)], 0);
  EXPECT_DOUBLE_EQ(diagnostics.minSourceFeasibleAlpha[src(StepSource::Material)], 1.0);
}

TEST(DeformationEnergyOperatorMaxStepGTest, TetMultipleElementsReturnEarliestClamp)
{
  const std::vector<double> vertices = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
    3.0, 0.0, 0.0,
    4.0, 0.0, 0.0,
    3.0, 1.0, 0.0,
    3.0, 0.0, 1.0,
  };
  const std::vector<int> elementVertices = {
    0, 1, 2, 3,
    4, 5, 6, 7,
  };

  EnergyFixture multiFixture = makeTetFixture(vertices, elementVertices);
  const ES::VXd x = ES::VXd::Zero(multiFixture.restPositions.size());
  ES::VXd dx = ES::VXd::Zero(multiFixture.restPositions.size());
  dx[3 * 3 + 2] = -2.0;
  dx[7 * 3 + 2] = -1.2;

  const StepConstraint result = multiFixture.energy->computeMaxStepLimit(x, dx);
  const double alpha = result.alpha;

  EnergyFixture singleFixture = makeSingleTetFixture();
  const ES::VXd singleX = ES::VXd::Zero(singleFixture.restPositions.size());
  const double alphaA = singleFixture.energy->computeMaxStepLimit(singleX, makeTetFlipDirection(singleFixture.mesh().getNumVertices(), 3, -2.0)).alpha;
  const double alphaB = singleFixture.energy->computeMaxStepLimit(singleX, makeTetFlipDirection(singleFixture.mesh().getNumVertices(), 3, -1.2)).alpha;

  EXPECT_NEAR(alpha, std::min(alphaA, alphaB), 1e-12);
  EXPECT_TRUE(result.clamped());
}

TEST(DeformationEnergyOperatorMaxStepGTest, CubicShrinksBeforeInversion)
{
  EnergyFixture fixture = makeSingleCubicFixture();
  const ES::VXd x = ES::VXd::Zero(fixture.restPositions.size());
  const ES::VXd dx = makeCubicTopFaceDirection(fixture.mesh().getNumVertices(), 0, -2.0);

  const StepConstraint result = fixture.energy->computeMaxStepLimit(x, dx);
  const double alpha = result.alpha;
  ASSERT_LT(alpha, 1.0);
  ASSERT_GT(alpha, 0.0);
  EXPECT_DOUBLE_EQ(result.alpha, alpha);
  EXPECT_TRUE(result.clamped());

  const ES::VXd updatedPositions = fixture.restPositions + alpha * dx;
  EXPECT_LT(minCubicDeterminant(fixture.mesh(), fixture.manager(), 0, fixture.restPositions + dx), 0.0);
  EXPECT_GT(minCubicDeterminant(fixture.mesh(), fixture.manager(), 0, updatedPositions), 0.0);
}

TEST(DeformationEnergyOperatorMaxStepGTest, CubicFeasibleDirectionReturnsOne)
{
  EnergyFixture fixture = makeSingleCubicFixture();
  const ES::VXd x = ES::VXd::Zero(fixture.restPositions.size());
  const ES::VXd dx = makeCubicTopFaceDirection(fixture.mesh().getNumVertices(), 0, -0.2);

  const StepConstraint result = fixture.energy->computeMaxStepLimit(x, dx);
  EXPECT_DOUBLE_EQ(result.alpha, 1.0);
  EXPECT_DOUBLE_EQ(result.alpha, 1.0);
  EXPECT_FALSE(result.clamped());
}

TEST(DeformationEnergyOperatorMaxStepGTest, CubicMultipleElementsReturnEarliestClamp)
{
  const std::vector<double> vertices = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    1.0, 1.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
    1.0, 0.0, 1.0,
    1.0, 1.0, 1.0,
    0.0, 1.0, 1.0,
    3.0, 0.0, 0.0,
    4.0, 0.0, 0.0,
    4.0, 1.0, 0.0,
    3.0, 1.0, 0.0,
    3.0, 0.0, 1.0,
    4.0, 0.0, 1.0,
    4.0, 1.0, 1.0,
    3.0, 1.0, 1.0,
  };
  const std::vector<int> elementVertices = {
    0, 1, 2, 3, 4, 5, 6, 7,
    8, 9, 10, 11, 12, 13, 14, 15,
  };

  EnergyFixture multiFixture = makeCubicFixture(vertices, elementVertices);
  const ES::VXd x = ES::VXd::Zero(multiFixture.restPositions.size());
  ES::VXd dx = ES::VXd::Zero(multiFixture.restPositions.size());
  dx += makeCubicTopFaceDirection(multiFixture.mesh().getNumVertices(), 0, -2.0);
  dx += makeCubicTopFaceDirection(multiFixture.mesh().getNumVertices(), 8, -1.2);

  const StepConstraint result = multiFixture.energy->computeMaxStepLimit(x, dx);
  const double alpha = result.alpha;

  EnergyFixture singleFixture = makeSingleCubicFixture();
  const ES::VXd singleX = ES::VXd::Zero(singleFixture.restPositions.size());
  const double alphaA = singleFixture.energy->computeMaxStepLimit(singleX, makeCubicTopFaceDirection(singleFixture.mesh().getNumVertices(), 0, -2.0)).alpha;
  const double alphaB = singleFixture.energy->computeMaxStepLimit(singleX, makeCubicTopFaceDirection(singleFixture.mesh().getNumVertices(), 0, -1.2)).alpha;

  EXPECT_NEAR(alpha, std::min(alphaA, alphaB), 1e-12);
  EXPECT_TRUE(result.clamped());
}

TEST(DeformationEnergyOperatorMaxStepGTest, ShellKeepsUnitStep)
{
  EnergyFixture fixture = makeShellFixture();
  const ES::VXd x = ES::VXd::Zero(fixture.restPositions.size());
  ES::VXd dx = ES::VXd::Zero(fixture.restPositions.size());
  applyUniformTranslation(dx, 0.1, -0.05, 0.2);

  const StepConstraint result = fixture.energy->computeMaxStepLimit(x, dx);
  EXPECT_DOUBLE_EQ(result.alpha, 1.0);
  EXPECT_DOUBLE_EQ(result.alpha, 1.0);
  EXPECT_FALSE(result.clamped());
}

TEST(DeformationEnergyOperatorMaxStepGTest, BackwardEulerTakesMinWithOtherEnergy)
{
  EnergyFixture fixture = makeSingleTetFixture();
  const ES::VXd x = ES::VXd::Zero(fixture.restPositions.size());
  const ES::VXd dx = makeTetFlipDirection(fixture.mesh().getNumVertices(), 3, -2.0);
  const double materialAlpha = fixture.energy->computeMaxStepLimit(x, dx).alpha;
  ASSERT_LT(materialAlpha, 0.95);

  ES::SpMatD mass(fixture.restPositions.size(), fixture.restPositions.size());
  mass.setIdentity();

  pgo::Simulation::DynamicProblem prob;
  prob.mass = mass;
  prob.timestep = 0.01;
  prob.persistentTerms = {{fixture.energy, 0.0, 0.0},
    {std::make_shared<FixedMaxStepEnergy>(fixture.restPositions.size(), 0.95), 0.0, 0.0}};
  pgo::Simulation::BackwardEulerStepper stepper(std::move(prob));

  StepConstraint merged = stepper.getStageEnergy()->computeMaxStepLimit(x, dx);
  EXPECT_NEAR(merged.alpha, materialAlpha, 1e-12);
  EXPECT_NEAR(merged.alpha, materialAlpha, 1e-12);
  EXPECT_TRUE(merged.clamped());

  // Rebuild with a tighter max-step constraint.
  pgo::Simulation::DynamicProblem prob2;
  prob2.mass = mass;
  prob2.timestep = 0.01;
  prob2.persistentTerms = {{fixture.energy, 0.0, 0.0},
    {std::make_shared<FixedMaxStepEnergy>(fixture.restPositions.size(), 0.25), 0.0, 0.0}};
  pgo::Simulation::BackwardEulerStepper stepper2(std::move(prob2));

  merged = stepper2.getStageEnergy()->computeMaxStepLimit(x, dx);
  EXPECT_DOUBLE_EQ(merged.alpha, 0.25);
  EXPECT_DOUBLE_EQ(merged.alpha, 0.25);
  EXPECT_TRUE(merged.clamped());
}

TEST(DeformationEnergyOperatorMaxStepGTest, TRBDF2TakesMinWithOtherEnergy)
{
  EnergyFixture fixture = makeSingleTetFixture();
  const ES::VXd x = ES::VXd::Zero(fixture.restPositions.size());
  const ES::VXd dx = makeTetFlipDirection(fixture.mesh().getNumVertices(), 3, -2.0);
  const double materialAlpha = fixture.energy->computeMaxStepLimit(x, dx).alpha;
  ASSERT_LT(materialAlpha, 0.9);

  ES::SpMatD mass(fixture.restPositions.size(), fixture.restPositions.size());
  mass.setIdentity();

  pgo::Simulation::DynamicProblem prob;
  prob.mass = mass;
  prob.timestep = 0.01;
  prob.persistentTerms = {{fixture.energy, 0.0, 0.0},
    {std::make_shared<FixedMaxStepEnergy>(fixture.restPositions.size(), 0.9), 0.0, 0.0}};
  pgo::Simulation::TRBDF2Stepper stepper(std::move(prob), 0.5);

  StepConstraint merged = stepper.getStage1Energy()->computeMaxStepLimit(x, dx);
  EXPECT_NEAR(merged.alpha, materialAlpha, 1e-12);
  EXPECT_NEAR(merged.alpha, materialAlpha, 1e-12);
  EXPECT_TRUE(merged.clamped());

  // Rebuild with a tighter constraint.
  pgo::Simulation::DynamicProblem prob2;
  prob2.mass = mass;
  prob2.timestep = 0.01;
  prob2.persistentTerms = {{fixture.energy, 0.0, 0.0},
    {std::make_shared<FixedMaxStepEnergy>(fixture.restPositions.size(), 0.2), 0.0, 0.0}};
  pgo::Simulation::TRBDF2Stepper stepper2(std::move(prob2), 0.5);

  merged = stepper2.getStage1Energy()->computeMaxStepLimit(x, dx);
  EXPECT_DOUBLE_EQ(merged.alpha, 0.2);
  EXPECT_DOUBLE_EQ(merged.alpha, 0.2);
  EXPECT_TRUE(merged.clamped());
}
