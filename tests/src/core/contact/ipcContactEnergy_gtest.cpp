#include <gtest/gtest.h>

#include "ipc/ipcContactEnergy.h"
#include "energy/energySet.h"
#include "pgoLogging.h"
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


TEST(IPCContactEnergyGTest, DirectEvaluationPreparesActiveSet)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd rest = flattenPositions(V);
  const ES::VXd simDispl = ES::VXd::Zero(rest.size());

  IPCContactEnergy energy = makeEnergy(V, F, makeIdentityEmbedding(rest.size()));

  EXPECT_NO_THROW((void)energy.func(simDispl));
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


TEST(IPCContactEnergyGTest, InvalidEmbeddingRowsThrow)
{
  const auto [V, F] = makeTwoTriangleMesh();
  ES::SpMatD W(3 * V.rows() - 1, 3 * V.rows());

  EXPECT_THROW(
    makeEnergy(V, F, W),
    std::invalid_argument);
}
