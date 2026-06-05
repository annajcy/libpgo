#include "ipc/broadPhase/surfaceIPCBroadPhase.h"
#include "ipc/core/surfaceIPCCore.h"
#include "ipc/external/obstacleSurface.h"
#include "ipc/topology/surfaceIPCTopology.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace ES = pgo::EigenSupport;
using pgo::Contact::IPC::ExternalEEPair;
using pgo::Contact::IPC::ExternalPairSet;
using pgo::Contact::IPC::ExternalPTPair;
using pgo::Contact::IPC::ExternalTPPair;
using pgo::Contact::IPC::ObstacleSurface;
using pgo::Contact::IPC::ObstacleSurfaceView;
using pgo::Contact::IPC::SurfaceIPCCore;
using pgo::Contact::IPC::SurfaceIPCTopology;
using pgo::Contact::IPC::TrajectoryObstacleSurface;

static std::vector<ObstacleSurfaceView> obstacleViews(const std::vector<std::unique_ptr<ObstacleSurface>> &obstacles)
{
  std::vector<ObstacleSurfaceView> views;
  views.reserve(obstacles.size());
  for (const auto &obstacle : obstacles)
    views.push_back(pgo::Contact::IPC::makeObstacleSurfaceView(*obstacle));
  return views;
}

static ES::VXd flattenRows(const ES::MXd &V)
{
  ES::VXd x(V.rows() * 3);
  for (int vi = 0; vi < V.rows(); ++vi)
    x.segment<3>(3 * vi) = V.row(vi).transpose();
  return x;
}

static std::pair<ES::MXd, ES::MXi> makeUnitSquareMesh()
{
  ES::MXd V(4, 3);
  V << 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    1.0, 1.0, 0.0;
  ES::MXi F(2, 3);
  F << 0, 1, 2,
    1, 3, 2;
  return { V, F };
}

static std::pair<ES::MXd, ES::MXi> makeSmallBoxObstacle()
{
  ES::MXd V(8, 3);
  V << -0.5, -0.5, -0.5,
    0.5, -0.5, -0.5,
    0.5, 0.5, -0.5,
    -0.5, 0.5, -0.5,
    -0.5, -0.5, 0.5,
    0.5, -0.5, 0.5,
    0.5, 0.5, 0.5,
    -0.5, 0.5, 0.5;
  ES::MXi F(12, 3);
  F << 0, 2, 1, 0, 3, 2,
    4, 5, 6, 4, 6, 7,
    0, 1, 5, 0, 5, 4,
    1, 2, 6, 1, 6, 5,
    2, 3, 7, 2, 7, 6,
    3, 0, 4, 3, 4, 7;
  return { V, F };
}

static long long weightKey(double weight)
{
  return std::llround(weight * 1e12);
}

static auto canonicalPT(const std::vector<ExternalPTPair> &pairs)
{
  std::vector<std::tuple<int32_t, int, int, int, int, long long>> keys;
  keys.reserve(pairs.size());
  for (const auto &pair : pairs)
    keys.emplace_back(
      pair.obstacleSlot, pair.dynVertex, pair.obsTri[0], pair.obsTri[1], pair.obsTri[2], weightKey(pair.weight));
  std::sort(keys.begin(), keys.end());
  return keys;
}

static auto canonicalTP(const std::vector<ExternalTPPair> &pairs)
{
  std::vector<std::tuple<int32_t, int, int, int, int, long long>> keys;
  keys.reserve(pairs.size());
  for (const auto &pair : pairs)
    keys.emplace_back(
      pair.obstacleSlot, pair.dynTri[0], pair.dynTri[1], pair.dynTri[2], pair.obsVertex, weightKey(pair.weight));
  std::sort(keys.begin(), keys.end());
  return keys;
}

static auto canonicalEE(const std::vector<ExternalEEPair> &pairs)
{
  std::vector<std::tuple<int32_t, int, int, int, int, long long>> keys;
  keys.reserve(pairs.size());
  for (const auto &pair : pairs)
    keys.emplace_back(
      pair.obstacleSlot, pair.dynEdge[0], pair.dynEdge[1], pair.obsEdge[0], pair.obsEdge[1], weightKey(pair.weight));
  std::sort(keys.begin(), keys.end());
  return keys;
}

static bool containsExternalPT(const std::vector<ExternalPTPair> &pairs, const ExternalPTPair &target)
{
  const auto keys = canonicalPT(pairs);
  const auto targetKey = canonicalPT(std::vector<ExternalPTPair>{ target }).front();
  return std::binary_search(keys.begin(), keys.end(), targetKey);
}

static bool containsExternalTP(const std::vector<ExternalTPPair> &pairs, const ExternalTPPair &target)
{
  const auto keys = canonicalTP(pairs);
  const auto targetKey = canonicalTP(std::vector<ExternalTPPair>{ target }).front();
  return std::binary_search(keys.begin(), keys.end(), targetKey);
}

static bool containsExternalEE(const std::vector<ExternalEEPair> &pairs, const ExternalEEPair &target)
{
  const auto keys = canonicalEE(pairs);
  const auto targetKey = canonicalEE(std::vector<ExternalEEPair>{ target }).front();
  return std::binary_search(keys.begin(), keys.end(), targetKey);
}

TEST(SurfaceIPCExternalBroadPhaseGTest, BuilderMatchesSurfaceIPCCoreExternalPairs)
{
  auto [V, F] = makeUnitSquareMesh();
  auto [obsV, obsF] = makeSmallBoxObstacle();
  const ES::VXd obsRest = flattenRows(obsV);

  auto makeObs = [&]() {
    return std::make_unique<TrajectoryObstacleSurface>(
      obsV, obsF,
      pgo::Contact::IPC::makeLinearTrajectorySampler(obsRest, ES::V3d::Zero()));
  };

  SurfaceIPCCore::Parameters params;
  params.dhat_external = 1.0;

  std::vector<std::unique_ptr<ObstacleSurface>> coreObstacles;
  coreObstacles.push_back(makeObs());
  SurfaceIPCCore core(params, std::move(coreObstacles));
  core.setMesh(V, F);

  const ES::VXd x = flattenRows(V);
  const auto activeSet = core.buildActiveSet(x);

  SurfaceIPCTopology topology;
  topology.setMesh(V, F);
  std::vector<std::unique_ptr<ObstacleSurface>> obstacles;
  obstacles.push_back(makeObs());
  obstacles.front()->setObjectId(0);

  ExternalPairSet pairs;
  buildExternalPairs(topology, x, obstacleViews(obstacles), params.dhat_external, pairs);

  EXPECT_EQ(canonicalPT(pairs.ptPairs), canonicalPT(activeSet.externalPairs.ptPairs));
  EXPECT_EQ(canonicalTP(pairs.tpPairs), canonicalTP(activeSet.externalPairs.tpPairs));
  EXPECT_EQ(canonicalEE(pairs.eePairs), canonicalEE(activeSet.externalPairs.eePairs));
  EXPECT_GT(pairs.size(), 0u);
}

TEST(SurfaceIPCExternalBroadPhaseGTest, MovingObstacleProducesGoldenPairsAndWeights)
{
  ES::MXd V(3, 3);
  V << 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0;
  ES::MXi F(1, 3);
  F << 0, 1, 2;

  ES::MXd obsV(3, 3);
  obsV << 0.0, 0.0, 0.2,
    1.0, 0.0, 0.2,
    0.0, 1.0, 0.2;
  ES::MXi obsF(1, 3);
  obsF << 0, 1, 2;

  auto obs = std::make_unique<TrajectoryObstacleSurface>(
    obsV, obsF,
    pgo::Contact::IPC::makeLinearTrajectorySampler(flattenRows(obsV), ES::V3d(0.0, 0.0, -0.15)));
  obs->setObjectId(7);
  obs->setTime(1.0);

  SurfaceIPCTopology topology;
  topology.setMesh(V, F);
  std::vector<std::unique_ptr<ObstacleSurface>> obstacles;
  obstacles.push_back(std::move(obs));

  ExternalPairSet pairs;
  buildExternalPairs(topology, flattenRows(V), obstacleViews(obstacles), 0.2, pairs);

  const long long oneTwelfth = weightKey(1.0 / 12.0);
  const long long half = weightKey(0.5);
  const long long one = weightKey(1.0);
  const long long sqrtTwo = weightKey(std::sqrt(2.0));
  const long long two = weightKey(2.0);

  EXPECT_EQ(canonicalPT(pairs.ptPairs),
    (std::vector<std::tuple<int32_t, int, int, int, int, long long>>{
      { 7, 0, 0, 1, 2, oneTwelfth },
      { 7, 1, 0, 1, 2, oneTwelfth },
      { 7, 2, 0, 1, 2, oneTwelfth },
    }));
  EXPECT_EQ(canonicalTP(pairs.tpPairs),
    (std::vector<std::tuple<int32_t, int, int, int, int, long long>>{
      { 7, 0, 1, 2, 0, half },
      { 7, 0, 1, 2, 1, half },
      { 7, 0, 1, 2, 2, half },
    }));
  EXPECT_EQ(canonicalEE(pairs.eePairs),
    (std::vector<std::tuple<int32_t, int, int, int, int, long long>>{
      { 7, 0, 1, 0, 1, one },
      { 7, 0, 1, 0, 2, one },
      { 7, 0, 1, 1, 2, sqrtTwo },
      { 7, 0, 2, 0, 1, one },
      { 7, 0, 2, 0, 2, one },
      { 7, 0, 2, 1, 2, sqrtTwo },
      { 7, 1, 2, 0, 1, sqrtTwo },
      { 7, 1, 2, 0, 2, sqrtTwo },
      { 7, 1, 2, 1, 2, two },
    }));
}

TEST(SurfaceIPCExternalBroadPhaseGTest, ExternalEEDoesNotUseCoplanarInteriorObstacleDiagonal)
{
  auto [V, F] = makeUnitSquareMesh();
  V.col(2).array() = 0.05;

  auto [obsV, obsF] = makeUnitSquareMesh();
  const ES::VXd obsRest = flattenRows(obsV);
  auto obs = std::make_unique<TrajectoryObstacleSurface>(
    obsV, obsF,
    pgo::Contact::IPC::makeLinearTrajectorySampler(obsRest, ES::V3d::Zero()));
  obs->setObjectId(0);

  SurfaceIPCTopology topology;
  topology.setMesh(V, F);

  std::vector<std::unique_ptr<ObstacleSurface>> obstacles;
  obstacles.push_back(std::move(obs));

  ExternalPairSet pairs;
  buildExternalPairs(topology, flattenRows(V), obstacleViews(obstacles), 0.2, pairs);

  bool sawBoundaryEdge = false;
  for (const auto &pair : pairs.eePairs) {
    const int a = std::min(pair.obsEdge[0], pair.obsEdge[1]);
    const int b = std::max(pair.obsEdge[0], pair.obsEdge[1]);
    EXPECT_FALSE(a == 1 && b == 2);
    if (a == 0 && b == 1)
      sawBoundaryEdge = true;
  }

  EXPECT_TRUE(sawBoundaryEdge);
}

TEST(SurfaceIPCExternalBroadPhaseGTest, LineSearchSupersetContainsExactExternalPairsAtTrialStates)
{
  ES::MXd dynV(4, 3);
  dynV << 0.0, 0.0, 0.32,
    1.0, 0.0, 0.32,
    0.0, 1.0, 0.32,
    1.0, 1.0, 0.32;
  ES::MXi dynF(2, 3);
  dynF << 0, 1, 2,
    1, 3, 2;

  auto [obsV, obsF] = makeUnitSquareMesh();
  const ES::VXd obsRest = flattenRows(obsV);
  auto obs = std::make_unique<TrajectoryObstacleSurface>(
    obsV, obsF,
    pgo::Contact::IPC::makeLinearTrajectorySampler(obsRest, ES::V3d::Zero()));
  obs->setObjectId(3);

  SurfaceIPCTopology topology;
  topology.setMesh(dynV, dynF);

  const ES::VXd x = flattenRows(dynV);
  ES::VXd dx = ES::VXd::Zero(x.size());
  for (int vi = 0; vi < dynV.rows(); ++vi)
    dx[3 * vi + 2] = -0.28;

  std::vector<std::unique_ptr<ObstacleSurface>> obstacles;
  obstacles.push_back(std::move(obs));

  ExternalPairSet superset;
  buildExternalPairsLineSearchSuperset(topology, x, dx, obstacleViews(obstacles), 0.35, superset);

  bool sawExactPairs = false;
  for (double alpha : { 0.0, 0.25, 0.5, 1.0 }) {
    ExternalPairSet exact;
    buildExternalPairs(topology, x + alpha * dx, obstacleViews(obstacles), 0.35, exact);
    sawExactPairs = sawExactPairs || exact.size() > 0;

    for (const auto &pair : exact.ptPairs)
      EXPECT_TRUE(containsExternalPT(superset.ptPairs, pair));
    for (const auto &pair : exact.tpPairs)
      EXPECT_TRUE(containsExternalTP(superset.tpPairs, pair));
    for (const auto &pair : exact.eePairs)
      EXPECT_TRUE(containsExternalEE(superset.eePairs, pair));
  }

  EXPECT_TRUE(sawExactPairs);
}

TEST(SurfaceIPCExternalBroadPhaseGTest, ZeroDisplacementLineSearchSupersetContainsNormalPairsOnFixture)
{
  auto [V, F] = makeUnitSquareMesh();
  auto [obsV, obsF] = makeSmallBoxObstacle();
  const ES::VXd obsRest = flattenRows(obsV);
  auto obs = std::make_unique<TrajectoryObstacleSurface>(
    obsV, obsF,
    pgo::Contact::IPC::makeLinearTrajectorySampler(obsRest, ES::V3d::Zero()));
  obs->setObjectId(0);

  SurfaceIPCTopology topology;
  topology.setMesh(V, F);

  std::vector<std::unique_ptr<ObstacleSurface>> obstacles;
  obstacles.push_back(std::move(obs));

  const ES::VXd x = flattenRows(V);
  const ES::VXd dx = ES::VXd::Zero(x.size());

  ExternalPairSet normalPairs;
  ExternalPairSet supersetPairs;
  buildExternalPairs(topology, x, obstacleViews(obstacles), 1.0, normalPairs);
  buildExternalPairsLineSearchSuperset(topology, x, dx, obstacleViews(obstacles), 1.0, supersetPairs);

  EXPECT_GE(supersetPairs.ptPairs.size(), normalPairs.ptPairs.size());
  EXPECT_GE(supersetPairs.tpPairs.size(), normalPairs.tpPairs.size());
  EXPECT_GE(supersetPairs.eePairs.size(), normalPairs.eePairs.size());
  for (const auto &pair : normalPairs.ptPairs)
    EXPECT_TRUE(containsExternalPT(supersetPairs.ptPairs, pair));
  for (const auto &pair : normalPairs.tpPairs)
    EXPECT_TRUE(containsExternalTP(supersetPairs.tpPairs, pair));
  for (const auto &pair : normalPairs.eePairs)
    EXPECT_TRUE(containsExternalEE(supersetPairs.eePairs, pair));
}

TEST(SurfaceIPCExternalBroadPhaseGTest, ObstaclePoseCacheTracksSurfaceBounds)
{
  auto [obsV, obsF] = makeSmallBoxObstacle();
  TrajectoryObstacleSurface obs(
    obsV, obsF,
    pgo::Contact::IPC::makeLinearTrajectorySampler(flattenRows(obsV), ES::V3d(1.0, 2.0, 3.0)));
  obs.setTime(0.5);

  const auto &surfaceBox = obs.cache().surfaceBox;
  EXPECT_NEAR(surfaceBox.lo.x(), 0.0, 1e-12);
  EXPECT_NEAR(surfaceBox.lo.y(), 0.5, 1e-12);
  EXPECT_NEAR(surfaceBox.lo.z(), 1.0, 1e-12);
  EXPECT_NEAR(surfaceBox.hi.x(), 1.0, 1e-12);
  EXPECT_NEAR(surfaceBox.hi.y(), 1.5, 1e-12);
  EXPECT_NEAR(surfaceBox.hi.z(), 2.0, 1e-12);

  obs.setTime(1.0);
  const auto &updatedBox = obs.cache().surfaceBox;
  EXPECT_NEAR(updatedBox.lo.x(), 0.5, 1e-12);
  EXPECT_NEAR(updatedBox.lo.y(), 1.5, 1e-12);
  EXPECT_NEAR(updatedBox.lo.z(), 2.5, 1e-12);
  EXPECT_NEAR(updatedBox.hi.x(), 1.5, 1e-12);
  EXPECT_NEAR(updatedBox.hi.y(), 2.5, 1e-12);
  EXPECT_NEAR(updatedBox.hi.z(), 3.5, 1e-12);
}
