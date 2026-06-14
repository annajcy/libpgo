#include <gtest/gtest.h>

#include "ipc/ipcPairGenerator.h"
#include "ipc/broadPhase/spatialHashGrid.h"
#include "ipc/core/surfaceIPCMaxStep.h"
#include "ipc/external/obstacleSurface.h"
#include "testCIPCHelpers.h"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
namespace IPC = pgo::Contact::IPC;
namespace NO = pgo::NonlinearOptimization;

constexpr int src(NO::StepSource source) { return static_cast<int>(source); }

ES::MXd makeObstacleTriangle(double z)
{
  ES::MXd V(3, 3);
  V << 0.0, 0.0, z,
       1.0, 0.0, z,
       0.0, 1.0, z;
  return V;
}

ES::MXi makeObstacleFace()
{
  ES::MXi F(1, 3);
  F << 0, 1, 2;
  return F;
}

std::vector<std::unique_ptr<IPC::ObstacleSurface>> makeMixedObstacles()
{
  std::vector<std::unique_ptr<IPC::ObstacleSurface>> obstacles;
  obstacles.push_back(std::make_unique<IPC::StaticObstacleSurface>(makeObstacleTriangle(0.1), makeObstacleFace()));
  obstacles.push_back(std::make_unique<IPC::LinearMovingObstacleSurface>(
    makeObstacleTriangle(0.2), makeObstacleFace(), ES::V3d(0.0, 0.0, 0.5)));
  obstacles.push_back(std::make_unique<IPC::StaticObstacleSurface>(makeObstacleTriangle(0.3), makeObstacleFace()));
  return obstacles;
}

struct ExternalMaxStepScene
{
  ES::MXd V;
  ES::MXi F;
  ES::MXd obstacleV;
  ES::MXi obstacleF;
};

ExternalMaxStepScene makeExternalMaxStepScene()
{
  ExternalMaxStepScene scene;
  scene.V.resize(4, 3);
  scene.V << 0.0, 0.0, 0.0,
             1.0, 0.0, 0.0,
             0.0, 0.0, 1.0,
             1.0, 0.0, 1.0;
  scene.F.resize(2, 3);
  scene.F << 0, 1, 2,
             1, 3, 2;

  scene.obstacleV.resize(4, 3);
  scene.obstacleV << -1.0, 0.5, -1.0,
                      2.0, 0.5, -1.0,
                     -1.0, 0.5,  2.0,
                      2.0, 0.5,  2.0;
  scene.obstacleF.resize(2, 3);
  scene.obstacleF << 0, 1, 2,
                     1, 3, 2;
  return scene;
}

std::unique_ptr<IPC::ObstacleSurface> makeStationaryTrajectoryObstacle(const ES::MXd &V, const ES::MXi &F)
{
  const ES::VXd rest = pgo::Contact::CIPCTest::flattenPositions(V);
  return std::make_unique<IPC::TrajectoryObstacleSurface>(
    V, F, IPC::makeLinearTrajectorySampler(rest, ES::V3d::Zero()));
}
}  // namespace

TEST(IPCPairGeneratorGTest, BuildsSelfActiveSet)
{
  const auto [V, F] = pgo::Contact::CIPCTest::makeTwoTriangleMesh();
  IPC::IPCPairGenerator::Parameters params;
  params.dhat = 0.5;
  params.dhatExternal = 0.5;

  IPC::IPCPairGenerator generator(params);
  generator.setMesh(V, F);
  const ES::VXd x = pgo::Contact::CIPCTest::flattenPositions(V);
  const IPC::SurfaceIPCActiveSet activeSet = generator.buildActiveSet(x);

  EXPECT_TRUE(activeSet.positions.isApprox(x));
  ASSERT_FALSE(activeSet.selfPairs.ptPairs.empty());
  EXPECT_GT(activeSet.selfPairs.size(), 0u);
  EXPECT_EQ(activeSet.externalPairs.size(), 0u);
}

TEST(IPCPairGeneratorGTest, ComputesMaxStepConstraint)
{
  const auto [V, F] = pgo::Contact::CIPCTest::makeTwoTriangleMesh();
  IPC::IPCPairGenerator::Parameters params;
  params.dhat = 0.1;
  params.dhatExternal = 0.1;
  params.slackness = 0.9;

  IPC::IPCPairGenerator generator(params);
  generator.setMesh(V, F);
  const ES::VXd x = pgo::Contact::CIPCTest::flattenPositions(V);
  ES::VXd dx = ES::VXd::Zero(x.size());
  for (int vi = 3; vi < 6; ++vi)
    dx[3 * vi + 2] = -0.1;

  NO::SolveDiagnostics diagnostics;
  const NO::StepConstraint constraint = generator.computeMaxStepLimit(x, dx, &diagnostics);
  EXPECT_EQ(constraint.source, NO::StepSource::Contact);
  EXPECT_GT(constraint.alpha, 0.0);
  EXPECT_LT(constraint.alpha, 1.0);
  EXPECT_TRUE(constraint.clamped());
  EXPECT_EQ(diagnostics.clampCounts[src(NO::StepSource::Contact)], 1);
  EXPECT_DOUBLE_EQ(diagnostics.minSourceFeasibleAlpha[src(NO::StepSource::Contact)], constraint.alpha);
  EXPECT_DOUBLE_EQ(diagnostics.lastMaxStep.alpha, constraint.alpha);
  EXPECT_EQ(diagnostics.lastMaxStep.source, NO::StepSource::Contact);
}

TEST(IPCPairGeneratorGTest, ObstacleViewsPreserveInputOrderAndMovingTimeUpdatesOnlyMovingObstacles)
{
  IPC::IPCPairGenerator generator(IPC::IPCPairGenerator::Parameters{}, makeMixedObstacles());
  IPC::IPCPairGenerator copyConstructed(generator);
  IPC::IPCPairGenerator assigned;
  assigned = generator;

  const std::vector<IPC::ObstacleSurfaceView> before = generator.obstacleViews();
  ASSERT_EQ(before.size(), 3u);
  EXPECT_EQ(before[0].objectId(), 0);
  EXPECT_EQ(before[1].objectId(), 1);
  EXPECT_EQ(before[2].objectId(), 2);
  const double static0Z = before[0].currentPositions()[2];
  const double movingZ = before[1].currentPositions()[2];
  const double static2Z = before[2].currentPositions()[2];

  generator.setMovingObstacleTime(2.0);

  const std::vector<IPC::ObstacleSurfaceView> after = generator.obstacleViews();
  ASSERT_EQ(after.size(), 3u);
  EXPECT_DOUBLE_EQ(after[0].currentPositions()[2], static0Z);
  EXPECT_DOUBLE_EQ(after[1].currentPositions()[2], movingZ + 1.0);
  EXPECT_DOUBLE_EQ(after[2].currentPositions()[2], static2Z);
  EXPECT_EQ(after[0].objectId(), 0);
  EXPECT_EQ(after[1].objectId(), 1);
  EXPECT_EQ(after[2].objectId(), 2);

  const std::vector<IPC::ObstacleSurfaceView> copyViews = copyConstructed.obstacleViews();
  const std::vector<IPC::ObstacleSurfaceView> assignedViews = assigned.obstacleViews();
  ASSERT_EQ(copyViews.size(), 3u);
  ASSERT_EQ(assignedViews.size(), 3u);
  EXPECT_DOUBLE_EQ(copyViews[1].currentPositions()[2], movingZ);
  EXPECT_DOUBLE_EQ(assignedViews[1].currentPositions()[2], movingZ);

  copyConstructed.setMovingObstacleTime(4.0);
  const std::vector<IPC::ObstacleSurfaceView> updatedCopyViews = copyConstructed.obstacleViews();
  ASSERT_EQ(updatedCopyViews.size(), 3u);
  EXPECT_DOUBLE_EQ(updatedCopyViews[1].currentPositions()[2], movingZ + 2.0);
  EXPECT_DOUBLE_EQ(generator.obstacleViews()[1].currentPositions()[2], movingZ + 1.0);
}

TEST(IPCPairGeneratorGTest, ExternalMaxStepMatchesLowLevelHelperAndReportsSink)
{
  const ExternalMaxStepScene scene = makeExternalMaxStepScene();

  IPC::IPCPairGenerator::Parameters generatorParams;
  generatorParams.dhat = 0.1;
  generatorParams.dhatExternal = 0.5;
  generatorParams.slackness = 1.0;

  std::vector<std::unique_ptr<IPC::ObstacleSurface>> generatorObstacles;
  generatorObstacles.push_back(makeStationaryTrajectoryObstacle(scene.obstacleV, scene.obstacleF));
  IPC::IPCPairGenerator generator(generatorParams, std::move(generatorObstacles));
  generator.setMesh(scene.V, scene.F);

  const ES::VXd x = pgo::Contact::CIPCTest::flattenPositions(scene.V);
  ES::VXd dx = ES::VXd::Zero(x.size());
  for (int vi = 0; vi < scene.V.rows(); ++vi)
    dx[3 * vi + 1] = 1.0;

  NO::SolveDiagnostics diagnostics;
  const NO::StepConstraint generatorConstraint = generator.computeMaxStepLimit(x, dx, &diagnostics);
  const double externalAlpha = IPC::computeExternalMaxStep(
    generator.topology(), x, dx, generator.obstacleViews(), generatorParams.dhatExternal, generatorParams.slackness);

  EXPECT_EQ(generatorConstraint.source, NO::StepSource::Contact);
  EXPECT_LT(generatorConstraint.alpha, 1.0);
  EXPECT_TRUE(generatorConstraint.clamped());
  EXPECT_NEAR(generatorConstraint.alpha, externalAlpha, 1e-12);
  EXPECT_DOUBLE_EQ(diagnostics.lastMaxStep.alpha, generatorConstraint.alpha);
  EXPECT_EQ(diagnostics.lastMaxStep.source, NO::StepSource::Contact);
  EXPECT_EQ(diagnostics.clampCounts[src(NO::StepSource::Contact)], 1);
}

TEST(IPCPairGeneratorGTest, ConstructorInjectedObstaclesAssignSequentialSlots)
{
  ES::MXd dynV(4, 3);
  dynV << 0.0, 0.0, 0.0,
          1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          1.0, 1.0, 0.0;
  ES::MXi dynF(2, 3);
  dynF << 0, 1, 2,
          1, 3, 2;

  auto buildPlaneObstacle = [](double zOffset) {
    ES::MXd obsV(3, 3);
    obsV << 0.0, 0.0, zOffset,
            1.0, 0.0, zOffset,
            0.0, 1.0, zOffset;
    ES::MXi obsF(1, 3);
    obsF << 0, 1, 2;
    return std::make_unique<IPC::StaticObstacleSurface>(obsV, obsF);
  };

  IPC::IPCPairGenerator::Parameters params;
  params.dhat = 0.1;
  params.dhatExternal = 1.0;

  std::vector<std::unique_ptr<IPC::ObstacleSurface>> obstacles;
  obstacles.push_back(buildPlaneObstacle(0.3));
  obstacles.push_back(buildPlaneObstacle(-0.3));
  IPC::IPCPairGenerator generator(params, std::move(obstacles));
  generator.setMesh(dynV, dynF);

  const IPC::SurfaceIPCActiveSet activeSet =
    generator.buildActiveSet(pgo::Contact::CIPCTest::flattenPositions(dynV));

  bool sawSlot0 = false;
  bool sawSlot1 = false;
  auto scanSlots = [&](auto &&vec) {
    for (const auto &pair : vec) {
      if (pair.obstacleSlot == 0)
        sawSlot0 = true;
      if (pair.obstacleSlot == 1)
        sawSlot1 = true;
    }
  };
  scanSlots(activeSet.externalPairs.ptPairs);
  scanSlots(activeSet.externalPairs.tpPairs);
  scanSlots(activeSet.externalPairs.eePairs);
  EXPECT_TRUE(sawSlot0);
  EXPECT_TRUE(sawSlot1);
}

TEST(IPCPairGeneratorGTest, ObstacleSurfaceEmptySamplerThrows)
{
  ES::MXd V(3, 3);
  V.setZero();
  ES::MXi F(1, 3);
  F << 0, 1, 2;

  EXPECT_THROW(
    IPC::TrajectoryObstacleSurface(V, F, IPC::ObstacleSurface::TrajectorySampler{}),
    std::invalid_argument);
}

TEST(IPCPairGeneratorGTest, ObstacleSurfaceStoresRestPositionsRowWise)
{
  ES::MXd V(2, 3);
  V << 1.0, 2.0, 3.0,
       4.0, 5.0, 6.0;
  ES::MXi F(0, 3);

  auto sampler = [](double, ES::RefVecXd out) { out.setZero(); };
  IPC::TrajectoryObstacleSurface obs(V, F, sampler);

  ES::VXd expected(6);
  expected << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  EXPECT_TRUE(obs.restPositions().isApprox(expected));
}

TEST(IPCPairGeneratorGTest, ObstacleSurfaceInvalidVertexColumnCountThrows)
{
  ES::MXd V(2, 4);
  V.setZero();
  ES::MXi F(0, 3);

  auto sampler = [](double, ES::RefVecXd out) { out.setZero(); };
  EXPECT_THROW(IPC::TrajectoryObstacleSurface(V, F, sampler), std::invalid_argument);
}

TEST(IPCPairGeneratorGTest, ObstacleSurfaceUpdateRefreshesBroadPhaseCache)
{
  ES::MXd V(4, 3);
  V << 0.0, 0.0, 0.0,
       1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       1.0, 1.0, 0.0;
  ES::MXi F(2, 3);
  F << 0, 1, 2,
       1, 3, 2;

  const ES::VXd rest = pgo::Contact::CIPCTest::flattenPositions(V);
  IPC::TrajectoryObstacleSurface obs(V, F, IPC::makeLinearTrajectorySampler(rest, ES::V3d(0.0, 0.0, 1.0)));
  obs.setTime(0.5);
  const auto &cache = obs.cache();

  ASSERT_EQ(static_cast<int>(cache.vertBoxes.size()), V.rows());
  ASSERT_EQ(static_cast<int>(cache.triBoxes.size()), F.rows());
  ASSERT_EQ(static_cast<int>(cache.edgeBoxes.size()), obs.contactEdges().rows());
  EXPECT_GT(cache.cellSize, 0.0);

  for (int fi = 0; fi < F.rows(); ++fi) {
    const auto &box = cache.triBoxes[fi];
    for (int j = 0; j < 3; ++j) {
      const ES::V3d v = obs.currentPositions().segment<3>(3 * F(fi, j));
      EXPECT_LE(box.lo.x(), v.x());
      EXPECT_GE(box.hi.x(), v.x());
      EXPECT_LE(box.lo.y(), v.y());
      EXPECT_GE(box.hi.y(), v.y());
      EXPECT_LE(box.lo.z(), v.z());
      EXPECT_GE(box.hi.z(), v.z());
    }
  }

  IPC::SpatialHashGrid::AABB queryBox;
  queryBox.init(obs.currentPositions().segment<3>(3 * F(0, 0)), 1e-3);
  queryBox.expand(obs.currentPositions().segment<3>(3 * F(0, 1)), 1e-3);
  queryBox.expand(obs.currentPositions().segment<3>(3 * F(0, 2)), 1e-3);
  std::vector<int> visited(F.rows(), 0);
  std::vector<int> candidates;
  cache.triHash.query(queryBox, -1, visited, 1, candidates);
  EXPECT_NE(std::find(candidates.begin(), candidates.end(), 0), candidates.end());

  obs.setTime(1.0);
  const auto &cache2 = obs.cache();
  EXPECT_EQ(static_cast<int>(cache2.triBoxes.size()), F.rows());
  for (int fi = 0; fi < F.rows(); ++fi) {
    const auto &box = cache2.triBoxes[fi];
    const ES::V3d v0 = obs.currentPositions().segment<3>(3 * F(fi, 0));
    EXPECT_LE(box.lo.z(), v0.z());
    EXPECT_GE(box.hi.z(), v0.z());
  }
}

TEST(IPCPairGeneratorGTest, ObstacleSurfaceKeepsOnlyFeatureEdgesForExternalEECache)
{
  ES::MXd V(4, 3);
  V << 0.0, 0.0, 0.0,
       1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       1.0, 1.0, 0.0;
  ES::MXi F(2, 3);
  F << 0, 1, 2,
       1, 3, 2;

  const ES::VXd rest = pgo::Contact::CIPCTest::flattenPositions(V);
  IPC::TrajectoryObstacleSurface obs(V, F, IPC::makeLinearTrajectorySampler(rest, ES::V3d::Zero()));

  ASSERT_EQ(obs.uniqueEdges().rows(), 5);
  ASSERT_EQ(obs.contactEdges().rows(), 4);
  EXPECT_EQ(static_cast<int>(obs.cache().edgeBoxes.size()), obs.contactEdges().rows());
  EXPECT_EQ(static_cast<int>(obs.cache().edgeLengths.size()), obs.contactEdges().rows());

  for (int ei = 0; ei < obs.contactEdges().rows(); ++ei) {
    const int a = std::min(obs.contactEdges()(ei, 0), obs.contactEdges()(ei, 1));
    const int b = std::max(obs.contactEdges()(ei, 0), obs.contactEdges()(ei, 1));
    EXPECT_FALSE(a == 1 && b == 2);
  }
}

TEST(IPCPairGeneratorGTest, ObstacleSurfaceKeepsSharpInteriorEdgesForExternalEECache)
{
  ES::MXd V(4, 3);
  V << 0.0, 0.0, 0.0,
       1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       1.0, 0.0, 1.0;
  ES::MXi F(2, 3);
  F << 0, 1, 2,
       1, 3, 2;

  const ES::VXd rest = pgo::Contact::CIPCTest::flattenPositions(V);
  IPC::TrajectoryObstacleSurface obs(V, F, IPC::makeLinearTrajectorySampler(rest, ES::V3d::Zero()));

  ASSERT_EQ(obs.uniqueEdges().rows(), 5);
  ASSERT_EQ(obs.contactEdges().rows(), 5);

  bool sawSharedSharpEdge = false;
  for (int ei = 0; ei < obs.contactEdges().rows(); ++ei) {
    const int a = std::min(obs.contactEdges()(ei, 0), obs.contactEdges()(ei, 1));
    const int b = std::max(obs.contactEdges()(ei, 0), obs.contactEdges()(ei, 1));
    if (a == 1 && b == 2)
      sawSharedSharpEdge = true;
  }
  EXPECT_TRUE(sawSharedSharpEdge);
}

TEST(IPCPairGeneratorGTest, StaticAndLinearMovingObstacleSurfacesAreReadyAndCloneable)
{
  IPC::StaticObstacleSurface staticObstacle(makeObstacleTriangle(0.1), makeObstacleFace());
  IPC::LinearMovingObstacleSurface movingObstacle(
    makeObstacleTriangle(0.2), makeObstacleFace(), ES::V3d(0.0, 0.0, 0.5));

  EXPECT_EQ(staticObstacle.currentPositions().size(), 9);
  EXPECT_TRUE(staticObstacle.cache().hasSurfaceBox);
  EXPECT_EQ(movingObstacle.currentPositions().size(), 9);
  EXPECT_TRUE(movingObstacle.cache().hasSurfaceBox);

  const double before = movingObstacle.currentPositions()[2];
  movingObstacle.setTime(2.0);
  EXPECT_DOUBLE_EQ(movingObstacle.currentPositions()[2], before + 1.0);

  auto staticClone = staticObstacle.cloneStatic();
  auto movingClone = movingObstacle.cloneMoving();
  EXPECT_DOUBLE_EQ(staticClone->currentPositions()[2], staticObstacle.currentPositions()[2]);
  EXPECT_DOUBLE_EQ(movingClone->currentPositions()[2], movingObstacle.currentPositions()[2]);
  EXPECT_TRUE(staticClone->cache().hasSurfaceBox);
  EXPECT_TRUE(movingClone->cache().hasSurfaceBox);
}
