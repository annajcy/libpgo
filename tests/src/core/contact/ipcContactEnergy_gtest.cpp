#include <gtest/gtest.h>

#include "ipc/ipcContactEnergy.h"
#include "energy/energySet.h"
#include "pgoLogging.h"
#include "scopedProfileSection.h"
#include "ipc/profiling/surfaceIPCProfiling.h"
#include "testCIPCHelpers.h"

#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
using pgo::Contact::IPC::IPCContactAssembler;
using pgo::Contact::IPC::IPCContactEnergy;
using pgo::Contact::IPC::IPCPairGenerator;
using pgo::Contact::ContactModelKind;
using pgo::Contact::StatefulContactEnergy;
using pgo::Contact::CIPCTest::flattenPositions;
using pgo::Contact::CIPCTest::makeTwoTriangleMesh;
using pgo::Contact::CIPCTest::relativeError;
using pgo::Contact::CIPCTest::sparseToDense;
using pgo::NonlinearOptimization::PotentialEnergy;
using pgo::NonlinearOptimization::EnergySet;

using pgo::Profiling::ProfileStat;

const ProfileStat *findStat(const ProfileStat &stat, std::string_view name)
{
  if (stat.name == name || stat.localName == name)
    return &stat;
  for (const ProfileStat &child : stat.children) {
    if (const ProfileStat *found = findStat(child, name))
      return found;
  }
  return nullptr;
}

struct IPCContactEnergyTestParameters
{
  IPCPairGenerator::Parameters pair;
  IPCContactAssembler::Parameters assembler;
};

IPCContactEnergyTestParameters makeParams()
{
  IPCContactEnergyTestParameters params;
  params.pair.dhat = 0.1;
  params.pair.dhatExternal = 0.1;
  params.pair.slackness = 0.8;

  params.assembler.dhat = 0.1;
  params.assembler.dhatExternal = 0.1;
  params.assembler.kappa = 2.0;
  params.assembler.epsEE = 0.0;

  return params;
}

IPCContactEnergy makeEnergy(
  const ES::MXd &surfaceRestVertices,
  const ES::MXi &surfaceTriangles,
  const ES::SpMatD &surfaceFromSimulationDispMap)
{
  const IPCContactEnergyTestParameters params = makeParams();
  return IPCContactEnergy(
    surfaceRestVertices,
    surfaceTriangles,
    surfaceFromSimulationDispMap,
    params.pair,
    params.assembler);
}

ES::SpMatD makeIdentityEmbedding(int n3)
{
  std::vector<ES::TripletD> triplets;
  triplets.reserve(n3);
  for (int i = 0; i < n3; ++i)
    triplets.emplace_back(i, i, 1.0);

  ES::SpMatD W(n3, n3);
  W.setFromTriplets(triplets.begin(), triplets.end());
  return W;
}
}  // namespace

TEST(IPCContactEnergyGTest, SparseEmbeddingPullsBackGradientAndHessian)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd surfaceRestPositions = flattenPositions(V);

  std::vector<ES::TripletD> triplets;
  triplets.emplace_back(0, 0, 1.0);
  triplets.emplace_back(1, 1, 1.0);
  triplets.emplace_back(2, 2, 1.0);
  triplets.emplace_back(3, 0, 0.25);
  triplets.emplace_back(3, 3, 0.75);
  triplets.emplace_back(4, 1, 0.25);
  triplets.emplace_back(4, 4, 0.75);
  triplets.emplace_back(5, 2, 0.25);
  triplets.emplace_back(5, 5, 0.75);
  triplets.emplace_back(6, 6, 1.0);
  triplets.emplace_back(7, 7, 1.0);
  triplets.emplace_back(8, 8, 1.0);
  triplets.emplace_back(9, 9, 1.0);
  triplets.emplace_back(10, 10, 1.0);
  triplets.emplace_back(11, 11, 1.0);
  triplets.emplace_back(12, 12, 1.0);
  triplets.emplace_back(13, 13, 1.0);
  triplets.emplace_back(14, 14, 1.0);
  triplets.emplace_back(15, 15, 1.0);
  triplets.emplace_back(16, 16, 1.0);
  triplets.emplace_back(17, 17, 1.0);

  ES::SpMatD W(surfaceRestPositions.size(), surfaceRestPositions.size());
  W.setFromTriplets(triplets.begin(), triplets.end());

  ES::VXd simulationDisplacements = ES::VXd::Zero(surfaceRestPositions.size());
  simulationDisplacements[11] = 0.01;
  simulationDisplacements[14] = 0.02;
  simulationDisplacements[17] = 0.015;

  IPCContactEnergy adapter = makeEnergy(V, F, W);

  const IPCContactEnergyTestParameters params = makeParams();
  IPCPairGenerator pairGenerator(params.pair);
  pairGenerator.setMesh(V, F);
  IPCContactAssembler assembler(params.assembler);

  const ES::VXd surfacePositions = surfaceRestPositions + W * simulationDisplacements;
  const auto activeSet = pairGenerator.buildActiveSet(surfacePositions);

  ES::VXd surfaceGradient(surfacePositions.size());
  assembler.computeGradient(pairGenerator.topology(), pairGenerator.obstacleViews(), activeSet, surfaceGradient);
  ES::SpMatD surfaceHessian;
  assembler.computeHessian(pairGenerator.topology(), pairGenerator.obstacleViews(), activeSet, surfaceHessian);

  ES::VXd simulationGradient(adapter.getNumDOFs());
  adapter.gradient(simulationDisplacements, simulationGradient);
  ES::SpMatD simulationHessian;
  adapter.hessian(simulationDisplacements, simulationHessian);

  EXPECT_LT(relativeError(simulationGradient, W.transpose() * surfaceGradient), 1e-12);
  EXPECT_LT(relativeError(sparseToDense(simulationHessian), sparseToDense(W.transpose() * surfaceHessian * W)), 1e-12);
}

TEST(IPCContactEnergyGTest, ProfilingRecordsAdapterSections)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd rest = flattenPositions(V);
  ES::VXd u = ES::VXd::Zero(rest.size());
  for (int vi = 3; vi < 6; ++vi)
    u[3 * vi + 2] = 0.01;

  IPCContactEnergy adapter = makeEnergy(V, F, makeIdentityEmbedding(rest.size()));

  pgo::Profiling::setProfilingEnabled(true);
  pgo::Profiling::resetProfileStatistics();

  ES::VXd gradient(rest.size());
  adapter.gradient(u, gradient);
  ES::SpMatD hessian;
  adapter.hessian(u, hessian);
  (void)adapter.func(u);

  const auto stats = pgo::Profiling::snapshotProfileStatistics();
  EXPECT_NE(findStat(stats, "contact.adapter.func"), nullptr);
  EXPECT_NE(findStat(stats, "contact.adapter.gradient"), nullptr);
  EXPECT_NE(findStat(stats, "contact.adapter.hessian_direct"), nullptr);
  EXPECT_NE(findStat(stats, "contact.adapter.map_to_surface"), nullptr);
  EXPECT_NE(findStat(stats, "contact.adapter.pullback_gradient"), nullptr);
  EXPECT_NE(findStat(stats, "contact.adapter.pullback_hessian"), nullptr);

  pgo::Profiling::setProfilingEnabled(false);
  pgo::Profiling::resetProfileStatistics();
}

TEST(IPCContactEnergyGTest, DirectEvaluationPreparesActiveSet)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd rest = flattenPositions(V);
  const ES::VXd simDispl = ES::VXd::Zero(rest.size());

  IPCContactEnergy energy = makeEnergy(V, F, makeIdentityEmbedding(rest.size()));

  EXPECT_NO_THROW((void)energy.func(simDispl));
}

TEST(IPCContactEnergyGTest, SeparateEvaluationsReuseExplicitActiveSetForSameState)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd rest = flattenPositions(V);
  ES::VXd u = ES::VXd::Zero(rest.size());
  for (int vi = 3; vi < 6; ++vi)
    u[3 * vi + 2] = 0.01;

  IPCContactEnergy adapter = makeEnergy(V, F, makeIdentityEmbedding(rest.size()));

  pgo::Profiling::setProfilingEnabled(true);
  pgo::Profiling::resetProfileStatistics();

  const double energy0 = adapter.func(u);
  ES::VXd gradient0 = ES::VXd::Zero(adapter.getNumDOFs());
  adapter.gradient(u, gradient0);
  ES::SpMatD hessian0;
  adapter.hessian(u, hessian0);

  const double energy1 = adapter.func(u);
  ES::VXd gradient1 = ES::VXd::Zero(adapter.getNumDOFs());
  adapter.gradient(u, gradient1);
  ES::SpMatD hessian1;
  adapter.hessian(u, hessian1);

  const auto stats = pgo::Profiling::snapshotProfileStatistics();
  const ProfileStat *activeSetBuild = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kBuildActiveSet);

  pgo::Profiling::setProfilingEnabled(false);
  pgo::Profiling::resetProfileStatistics();

  ASSERT_NE(activeSetBuild, nullptr);
  EXPECT_EQ(activeSetBuild->callCount, 1u);
  EXPECT_NEAR(energy1, energy0, 1e-12);
  EXPECT_LT(relativeError(gradient1, gradient0), 1e-12);
  EXPECT_LT(relativeError(sparseToDense(hessian1), sparseToDense(hessian0)), 1e-12);
}

TEST(IPCContactEnergyGTest, DirectEvaluationRebuildsActiveSetForChangedState)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd rest = flattenPositions(V);
  ES::VXd u0 = ES::VXd::Zero(rest.size());
  ES::VXd u1 = u0;
  for (int vi = 3; vi < 6; ++vi)
    u1[3 * vi + 2] = 0.01;

  IPCContactEnergy energy = makeEnergy(V, F, makeIdentityEmbedding(rest.size()));

  pgo::Profiling::setProfilingEnabled(true);
  pgo::Profiling::resetProfileStatistics();

  (void)energy.func(u0);
  (void)energy.func(u1);
  (void)energy.func(u1);

  const auto stats = pgo::Profiling::snapshotProfileStatistics();
  const ProfileStat *activeSetBuild = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kBuildActiveSet);

  pgo::Profiling::setProfilingEnabled(false);
  pgo::Profiling::resetProfileStatistics();

  ASSERT_NE(activeSetBuild, nullptr);
  EXPECT_EQ(activeSetBuild->callCount, 2u);
}

TEST(IPCContactEnergyGTest, FuncGradFusesOneBroadPhaseForEnergyAndGradient)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd rest = flattenPositions(V);
  ES::VXd simDispl = ES::VXd::Zero(rest.size());
  for (int vi = 3; vi < 6; ++vi)
    simDispl[3 * vi + 2] = 0.01;

  IPCContactEnergy energy = makeEnergy(V, F, makeIdentityEmbedding(rest.size()));
  ES::VXd g = ES::VXd::Zero(simDispl.size());

  pgo::Profiling::setProfilingEnabled(true);
  pgo::Profiling::resetProfileStatistics();

  const double e = energy.funcGradient(simDispl, g);

  const auto stats = pgo::Profiling::snapshotProfileStatistics();
  const ProfileStat *activeSetBuild = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kBuildActiveSet);

  pgo::Profiling::setProfilingEnabled(false);
  pgo::Profiling::resetProfileStatistics();

  ASSERT_NE(activeSetBuild, nullptr);
  EXPECT_EQ(activeSetBuild->callCount, 1u);

  const double eRef = energy.func(simDispl);
  ES::VXd gRef = ES::VXd::Zero(simDispl.size());
  energy.gradient(simDispl, gRef);

  EXPECT_NEAR(e, eRef, 1e-12);
  EXPECT_LT(relativeError(g, gRef), 1e-12);
}

TEST(IPCContactEnergyGTest, ExposesOnlyLineSearchAndStepLifecycleContactCapabilities)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd rest = flattenPositions(V);

  IPCContactEnergy energy = makeEnergy(V, F, makeIdentityEmbedding(rest.size()));
  const StatefulContactEnergy &stateful = energy;
  EXPECT_EQ(stateful.contactModelKind(), ContactModelKind::IPC);
  EXPECT_EQ(dynamic_cast<const pgo::NonlinearOptimization::EvaluationStateAwareEnergy *>(&energy), nullptr);
  EXPECT_NE(dynamic_cast<const pgo::NonlinearOptimization::LineSearchAwareEnergy *>(&energy), nullptr);
  EXPECT_NE(dynamic_cast<pgo::NonlinearOptimization::StepAwareEnergy *>(&energy), nullptr);
  EXPECT_FALSE(energy.isStepDependent());
}

TEST(IPCContactEnergyGTest, FuncGradHessianFusesOneBroadPhaseForAllThree)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd rest = flattenPositions(V);
  ES::VXd simDispl = ES::VXd::Zero(rest.size());
  for (int vi = 3; vi < 6; ++vi)
    simDispl[3 * vi + 2] = 0.01;

  IPCContactEnergy energy = makeEnergy(V, F, makeIdentityEmbedding(rest.size()));
  ES::VXd g = ES::VXd::Zero(simDispl.size());
  ES::SpMatD H;

  pgo::Profiling::setProfilingEnabled(true);
  pgo::Profiling::resetProfileStatistics();

  const double e = energy.funcGradientHessian(simDispl, g, H);

  const auto stats = pgo::Profiling::snapshotProfileStatistics();
  const ProfileStat *activeSetBuild = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kBuildActiveSet);
  const ProfileStat *combinedStat = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kActiveSetCombined);

  pgo::Profiling::setProfilingEnabled(false);
  pgo::Profiling::resetProfileStatistics();

  ASSERT_NE(activeSetBuild, nullptr);
  ASSERT_NE(combinedStat, nullptr);
  EXPECT_EQ(activeSetBuild->callCount, 1u);
  EXPECT_EQ(combinedStat->callCount, 1u);

  const double eRef = energy.func(simDispl);
  ES::VXd gRef = ES::VXd::Zero(simDispl.size());
  energy.gradient(simDispl, gRef);
  ES::SpMatD HRef;
  energy.hessian(simDispl, HRef);

  EXPECT_NEAR(e, eRef, 1e-12);
  EXPECT_LT(relativeError(g, gRef), 1e-12);
  EXPECT_LT(relativeError(sparseToDense(H), sparseToDense(HRef)), 1e-12);
}

TEST(IPCContactEnergyGTest, EnergyOnlyEvaluationSeedsNextCombinedActiveSet)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd rest = flattenPositions(V);
  ES::VXd simDispl = ES::VXd::Zero(rest.size());
  for (int vi = 3; vi < 6; ++vi)
    simDispl[3 * vi + 2] = 0.01;

  IPCContactEnergy energy = makeEnergy(V, F, makeIdentityEmbedding(rest.size()));
  ES::VXd g = ES::VXd::Zero(simDispl.size());
  ES::SpMatD H;

  pgo::Profiling::setProfilingEnabled(true);
  pgo::Profiling::resetProfileStatistics();

  const double e0 = energy.func(simDispl);
  const double e1 = energy.funcGradientHessian(simDispl, g, H);

  const auto stats = pgo::Profiling::snapshotProfileStatistics();
  const ProfileStat *activeSetBuild = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kBuildActiveSet);
  const ProfileStat *surfaceEnergy = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kEnergy);
  const ProfileStat *activeSetEnergy = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kActiveSetEnergy);
  const ProfileStat *combinedStat = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kActiveSetCombined);

  pgo::Profiling::setProfilingEnabled(false);
  pgo::Profiling::resetProfileStatistics();

  ASSERT_NE(activeSetBuild, nullptr);
  ASSERT_NE(surfaceEnergy, nullptr);
  ASSERT_NE(activeSetEnergy, nullptr);
  ASSERT_NE(combinedStat, nullptr);
  EXPECT_EQ(activeSetBuild->callCount, 1u);
  EXPECT_EQ(surfaceEnergy->callCount, 1u);
  EXPECT_EQ(activeSetEnergy->callCount, 1u);
  EXPECT_EQ(combinedStat->callCount, 1u);
  EXPECT_NEAR(e1, e0, 1e-12);

  const double eRef = energy.func(simDispl);
  ES::VXd gRef = ES::VXd::Zero(simDispl.size());
  energy.gradient(simDispl, gRef);
  ES::SpMatD HRef;
  energy.hessian(simDispl, HRef);

  EXPECT_NEAR(e1, eRef, 1e-12);
  EXPECT_LT(relativeError(g, gRef), 1e-12);
  EXPECT_LT(relativeError(sparseToDense(H), sparseToDense(HRef)), 1e-12);
}

TEST(IPCContactEnergyGTest, LineSearchSupersetReusesOneActiveSetAcrossTrialEnergies)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd rest = flattenPositions(V);
  const ES::VXd u0 = ES::VXd::Zero(rest.size());
  ES::VXd du = ES::VXd::Zero(rest.size());
  for (int vi = 3; vi < 6; ++vi)
    du[3 * vi + 2] = -0.02;

  IPCContactEnergy energy = makeEnergy(V, F, makeIdentityEmbedding(rest.size()));

  const std::array<double, 3> alphas = { 1.0, 0.5, 0.25 };
  std::array<double, 3> exactEnergies = {};
  for (std::size_t i = 0; i < alphas.size(); ++i) {
    exactEnergies[i] = energy.func(u0 + alphas[i] * du);
  }

  pgo::Profiling::setProfilingEnabled(true);
  pgo::Profiling::resetProfileStatistics();

  energy.beginLineSearch(u0, du);
  for (std::size_t i = 0; i < alphas.size(); ++i) {
    const double lineSearchEnergy = energy.func(u0 + alphas[i] * du);
    EXPECT_NEAR(lineSearchEnergy, exactEnergies[i], 1e-12);
  }

  energy.endLineSearch();
  pgo::Profiling::resetProfileStatistics();

  ES::VXd g = ES::VXd::Zero(rest.size());
  ES::SpMatD H;
  const double acceptedEnergy = energy.funcGradientHessian(u0 + alphas.back() * du, g, H);

  const auto stats = pgo::Profiling::snapshotProfileStatistics();
  const ProfileStat *activeSetBuild = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kBuildActiveSet);
  const ProfileStat *combinedStat = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kActiveSetCombined);

  pgo::Profiling::setProfilingEnabled(false);
  pgo::Profiling::resetProfileStatistics();

  ASSERT_NE(activeSetBuild, nullptr);
  ASSERT_NE(combinedStat, nullptr);
  EXPECT_EQ(activeSetBuild->callCount, 1u);
  EXPECT_EQ(combinedStat->callCount, 1u);
  EXPECT_NEAR(acceptedEnergy, exactEnergies.back(), 1e-12);
  EXPECT_EQ(g.size(), rest.size());
  EXPECT_EQ(H.rows(), rest.size());
}

TEST(IPCContactEnergyGTest, GradientHessianFusesOneBroadPhaseForGradAndHess)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd rest = flattenPositions(V);
  ES::VXd simDispl = ES::VXd::Zero(rest.size());
  for (int vi = 3; vi < 6; ++vi)
    simDispl[3 * vi + 2] = 0.01;

  IPCContactEnergy energy = makeEnergy(V, F, makeIdentityEmbedding(rest.size()));
  const PotentialEnergy &baseEnergy = energy;
  ES::VXd g = ES::VXd::Zero(simDispl.size());
  ES::SpMatD H;

  pgo::Profiling::setProfilingEnabled(true);
  pgo::Profiling::resetProfileStatistics();

  baseEnergy.gradientHessian(simDispl, g, H);

  const auto stats = pgo::Profiling::snapshotProfileStatistics();
  const ProfileStat *activeSetBuild = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kBuildActiveSet);
  const ProfileStat *activeSetGradient = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kActiveSetGradient);
  const ProfileStat *activeSetHessian = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kActiveSetHessian);
  const ProfileStat *combinedStat = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kActiveSetCombined);

  pgo::Profiling::setProfilingEnabled(false);
  pgo::Profiling::resetProfileStatistics();

  ASSERT_NE(activeSetBuild, nullptr);
  ASSERT_NE(activeSetGradient, nullptr);
  ASSERT_NE(activeSetHessian, nullptr);
  EXPECT_EQ(combinedStat, nullptr);
  EXPECT_EQ(activeSetBuild->callCount, 1u);
  EXPECT_EQ(activeSetGradient->callCount, 1u);
  EXPECT_EQ(activeSetHessian->callCount, 1u);

  ES::VXd gRef = ES::VXd::Zero(simDispl.size());
  energy.gradient(simDispl, gRef);
  ES::SpMatD HRef;
  energy.hessian(simDispl, HRef);

  EXPECT_LT(relativeError(g, gRef), 1e-12);
  EXPECT_LT(relativeError(sparseToDense(H), sparseToDense(HRef)), 1e-12);
}

TEST(IPCContactEnergyGTest, AggregatedGradientHessianPreservesIPCFusion)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd rest = flattenPositions(V);
  ES::VXd simDispl = ES::VXd::Zero(rest.size());
  for (int vi = 3; vi < 6; ++vi)
    simDispl[3 * vi + 2] = 0.01;

  const IPCContactEnergyTestParameters params = makeParams();
  auto ipcEnergy = std::make_shared<IPCContactEnergy>(
    V,
    F,
    makeIdentityEmbedding(rest.size()),
    params.pair,
    params.assembler);
  EnergySet aggregate(static_cast<int>(rest.size()), {{ipcEnergy, 1.0}});

  const PotentialEnergy &baseEnergy = aggregate;
  ES::VXd g = ES::VXd::Zero(simDispl.size());
  ES::SpMatD H;

  pgo::Profiling::setProfilingEnabled(true);
  pgo::Profiling::resetProfileStatistics();

  baseEnergy.gradientHessian(simDispl, g, H);

  const auto stats = pgo::Profiling::snapshotProfileStatistics();
  const ProfileStat *activeSetBuild = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kBuildActiveSet);
  const ProfileStat *activeSetGradient = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kActiveSetGradient);
  const ProfileStat *activeSetHessian = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kActiveSetHessian);
  const ProfileStat *combinedStat = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kActiveSetCombined);

  pgo::Profiling::setProfilingEnabled(false);
  pgo::Profiling::resetProfileStatistics();

  ASSERT_NE(activeSetBuild, nullptr);
  ASSERT_NE(activeSetGradient, nullptr);
  ASSERT_NE(activeSetHessian, nullptr);
  EXPECT_EQ(combinedStat, nullptr);
  EXPECT_EQ(activeSetBuild->callCount, 1u);
  EXPECT_EQ(activeSetGradient->callCount, 1u);
  EXPECT_EQ(activeSetHessian->callCount, 1u);

  ES::VXd gRef = ES::VXd::Zero(simDispl.size());
  aggregate.gradient(simDispl, gRef);
  ES::SpMatD HRef;
  aggregate.hessian(simDispl, HRef);

  EXPECT_LT(relativeError(g, gRef), 1e-12);
  EXPECT_LT(relativeError(sparseToDense(H), sparseToDense(HRef)), 1e-12);
}

TEST(IPCContactEnergyGTest, InvalidEmbeddingRowsThrow)
{
  const auto [V, F] = makeTwoTriangleMesh();
  ES::SpMatD W(3 * V.rows() - 1, 3 * V.rows());

  EXPECT_THROW(
    makeEnergy(V, F, W),
    std::invalid_argument);
}
