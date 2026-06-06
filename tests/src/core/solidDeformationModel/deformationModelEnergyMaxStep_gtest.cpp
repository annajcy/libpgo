#include <gtest/gtest.h>

#include "EigenSupport.h"
#include "deformation/deformationModelAssembler.h"
#include "energy/deformationModelEnergy.h"
#include "deformation/deformationModelManager.h"
#include "deformation/deformationModelState.h"
#include "dynamicStepper.h"
#include "energySet.h"
#include "pgoLogging.h"
#include "potentialEnergy.h"
#include "simulation/simulationMesh.h"
#include "formulations/geometry/tetP1Geometry.h"
#include "formulations/basis/hexTrilinearBasis.h"
#include "formulations/quadrature/gaussLegendreHexQuadrature.h"
#include "formulations/kernels/volumetricKernel.h"
#include "formulations/elements/volumetricDeformationModel.h"
#include "triMeshGeo.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
using pgo::NonlinearOptimization::PotentialEnergy;
using pgo::SolidDeformationModel::DeformationModelAssembler;
using pgo::SolidDeformationModel::DeformationModelElasticMaterial;
using pgo::SolidDeformationModel::ParameterField;
using pgo::SolidDeformationModel::DeformationModelEnergy;
using pgo::SolidDeformationModel::DeformationModelManager;
using pgo::SolidDeformationModel::DeformationModelPlasticMaterial;
using pgo::SolidDeformationModel::SimulationMesh;
using pgo::SolidDeformationModel::DeformationModelState;
using pgo::SolidDeformationModel::ElasticFieldInit;
using pgo::SolidDeformationModel::PlasticFieldInit;
using pgo::SolidDeformationModel::SimulationMeshENuMaterial;
using pgo::SolidDeformationModel::SimulationMeshENuhMaterial;
using pgo::SolidDeformationModel::SimulationMeshMaterial;
using pgo::SolidDeformationModel::SimulationMeshType;
using pgo::SolidDeformationModel::tetP1ComputeDs;
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
  std::shared_ptr<const SimulationMesh> meshOwner;
  std::shared_ptr<DeformationModelEnergy> energy;
  ES::VXd restPositions;

  // Borrow accessors through the unique_ptr spine (energy -> assembler -> manager -> mesh).
  const SimulationMesh &mesh() const { return *energy->assembler().getDeformationModelManager().getMesh(); }
  const DeformationModelManager &manager() const { return energy->assembler().getDeformationModelManager(); }
  const DeformationModelAssembler &assembler() const { return energy->assembler(); }
};

ES::VXd gatherRestPositions(const SimulationMesh &mesh)
{
  ES::VXd rest(mesh.getNumVertices() * 3);
  for (int vi = 0; vi < mesh.getNumVertices(); vi++) {
    double p[3];
    mesh.getVertex(vi, p);
    rest.segment<3>(vi * 3) << p[0], p[1], p[2];
  }
  return rest;
}

EnergyFixture makeTetFixture(const std::vector<double> &vertices, const std::vector<int> &elementVertices)
{
  initializeLogging();

  std::vector<int> elementMaterialIndices(elementVertices.size() / 4, 0);
  SimulationMeshENuMaterial baseMaterial(1200.0, 0.45);
  const SimulationMeshMaterial *materials[] = { &baseMaterial };

  EnergyFixture fixture;
  fixture.meshOwner = std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    static_cast<int>(vertices.size() / 3), vertices.data(),
    static_cast<int>(elementVertices.size() / 4), 4, elementVertices.data(),
    elementMaterialIndices.data(), 1, materials,
    SimulationMeshType::TET));

  fixture.restPositions = gatherRestPositions(*fixture.meshOwner);

  auto state = DeformationModelState::create(
    fixture.meshOwner,
    DeformationModelElasticMaterial::STABLE_NEO,
    ElasticFieldInit{},
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    PlasticFieldInit{});
  auto manager = std::make_unique<DeformationModelManager>(state, pgo::SolidDeformationModel::P1TetFormulation{}, 1, nullptr, nullptr);

  auto assembler = std::make_unique<DeformationModelAssembler>(std::move(manager), nullptr);
  fixture.energy = std::make_shared<DeformationModelEnergy>(std::move(assembler), 0);
  return fixture;
}

EnergyFixture makeSingleTetFixture()
{
  const std::vector<double> vertices = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
  const std::vector<int> elementVertices = { 0, 1, 2, 3 };
  return makeTetFixture(vertices, elementVertices);
}

EnergyFixture makeCubicFixture(const std::vector<double> &vertices, const std::vector<int> &elementVertices)
{
  initializeLogging();

  std::vector<int> elementMaterialIndices(elementVertices.size() / 8, 0);
  SimulationMeshENuMaterial baseMaterial(1200.0, 0.45);
  const SimulationMeshMaterial *materials[] = { &baseMaterial };

  EnergyFixture fixture;
  fixture.meshOwner = std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    static_cast<int>(vertices.size() / 3), vertices.data(),
    static_cast<int>(elementVertices.size() / 8), 8, elementVertices.data(),
    elementMaterialIndices.data(), 1, materials,
    SimulationMeshType::CUBIC));

  fixture.restPositions = gatherRestPositions(*fixture.meshOwner);

  auto state = DeformationModelState::create(
    fixture.meshOwner,
    DeformationModelElasticMaterial::STABLE_NEO,
    ElasticFieldInit{},
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    PlasticFieldInit{});
  auto manager = std::make_unique<DeformationModelManager>(state, pgo::SolidDeformationModel::LinearCubicFormulation{}, 1, nullptr, nullptr);

  auto assembler = std::make_unique<DeformationModelAssembler>(std::move(manager), nullptr);
  fixture.energy = std::make_shared<DeformationModelEnergy>(std::move(assembler), 0);
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

  SimulationMeshENuhMaterial shellMaterial(1000.0, 0.45, 1e-3);

  EnergyFixture fixture;
  fixture.meshOwner = std::shared_ptr<const SimulationMesh>(
    pgo::SolidDeformationModel::loadShellMesh(surfaceMesh, &shellMaterial).release());

  fixture.restPositions = gatherRestPositions(*fixture.meshOwner);

  auto state = DeformationModelState::create(
    fixture.meshOwner,
    DeformationModelElasticMaterial::KOITER_STVK,
    ElasticFieldInit{},
    DeformationModelPlasticMaterial::SHELL_FF_DOF1,
    PlasticFieldInit{});
  auto manager = std::make_unique<DeformationModelManager>(state, pgo::SolidDeformationModel::KoiterShellFormulation{}, 1, nullptr, nullptr);

  auto assembler = std::make_unique<DeformationModelAssembler>(std::move(manager), nullptr);
  fixture.energy = std::make_shared<DeformationModelEnergy>(std::move(assembler), 0);
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
  std::array<double, 12> localPositions{};
  for (int j = 0; j < 4; j++) {
    const int vi = mesh.getVertexIndex(ele, j);
    localPositions[j * 3 + 0] = absolutePositions[vi * 3 + 0];
    localPositions[j * 3 + 1] = absolutePositions[vi * 3 + 1];
    localPositions[j * 3 + 2] = absolutePositions[vi * 3 + 2];
  }

  std::array<double, 9> Ds{};
  tetP1ComputeDs(localPositions.data(), Ds.data());
  return Eigen::Map<const ES::M3d>(Ds.data()).determinant();
}

double minCubicDeterminant(const SimulationMesh &mesh, const DeformationModelManager &manager,
  int ele, const ES::VXd &absolutePositions)
{
  const auto *model = dynamic_cast<const CubicFEM *>(manager.getDeformationModel(ele));
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
    std::array<double, 9> F{};
    model->computeF(localPositions.data(), q, F.data());
    minDet = std::min(minDet, Eigen::Map<const ES::M3d>(F.data()).determinant());
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

TEST(DeformationModelEnergyMaxStepGTest, ZeroDirectionReturnsOneAndDoesNotClamp)
{
  EnergyFixture fixture = makeSingleTetFixture();
  const ES::VXd x = ES::VXd::Zero(fixture.restPositions.size());
  const ES::VXd dx = ES::VXd::Zero(fixture.restPositions.size());

  const StepConstraint result = fixture.energy->computeMaxStepLimit(x, dx);
  EXPECT_DOUBLE_EQ(result.alpha, 1.0);
  EXPECT_DOUBLE_EQ(result.alpha, 1.0);
  EXPECT_FALSE(result.clamped());
}

TEST(DeformationModelEnergyMaxStepGTest, TetPureTranslationReturnsOne)
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

TEST(DeformationModelEnergyMaxStepGTest, TetShrinksBeforeInversion)
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

TEST(DeformationModelEnergyMaxStepGTest, DisabledMaterialMaxStepSkipsTetClamp)
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

TEST(DeformationModelEnergyMaxStepGTest, TetIllegalInitialStateWarnsEachCallAndClamps)
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

TEST(DeformationModelEnergyMaxStepGTest, TetSmallAlphaWarnsAndCanBeRecordedInDiagnostics)
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

TEST(DeformationModelEnergyMaxStepGTest, SolveDiagnosticsResetClearsMaterialCountAndMinimumAlpha)
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

TEST(DeformationModelEnergyMaxStepGTest, TetMultipleElementsReturnEarliestClamp)
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

TEST(DeformationModelEnergyMaxStepGTest, CubicShrinksBeforeInversion)
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

TEST(DeformationModelEnergyMaxStepGTest, CubicFeasibleDirectionReturnsOne)
{
  EnergyFixture fixture = makeSingleCubicFixture();
  const ES::VXd x = ES::VXd::Zero(fixture.restPositions.size());
  const ES::VXd dx = makeCubicTopFaceDirection(fixture.mesh().getNumVertices(), 0, -0.2);

  const StepConstraint result = fixture.energy->computeMaxStepLimit(x, dx);
  EXPECT_DOUBLE_EQ(result.alpha, 1.0);
  EXPECT_DOUBLE_EQ(result.alpha, 1.0);
  EXPECT_FALSE(result.clamped());
}

TEST(DeformationModelEnergyMaxStepGTest, CubicMultipleElementsReturnEarliestClamp)
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

TEST(DeformationModelEnergyMaxStepGTest, ShellKeepsUnitStep)
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

TEST(DeformationModelEnergyMaxStepGTest, ImplicitBackwardEulerTakesMinWithOtherEnergy)
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
  pgo::Simulation::ImplicitEulerStepper stepper(std::move(prob));

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
  pgo::Simulation::ImplicitEulerStepper stepper2(std::move(prob2));

  merged = stepper2.getStageEnergy()->computeMaxStepLimit(x, dx);
  EXPECT_DOUBLE_EQ(merged.alpha, 0.25);
  EXPECT_DOUBLE_EQ(merged.alpha, 0.25);
  EXPECT_TRUE(merged.clamped());
}

TEST(DeformationModelEnergyMaxStepGTest, TRBDF2TakesMinWithOtherEnergy)
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
