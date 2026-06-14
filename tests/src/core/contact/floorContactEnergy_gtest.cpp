#include <gtest/gtest.h>

#include "evaluationStateAwareEnergy.h"
#include "floor/floorContactEnergy.h"
#include "lineSearchAwareEnergy.h"
#include "statefulContactEnergy.h"
#include "testCIPCHelpers.h"

#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
namespace Contact = pgo::Contact;
namespace NO = pgo::NonlinearOptimization;
using pgo::Contact::Floor::FloorContactEnergy;
using pgo::Contact::Floor::FloorAxis;
using pgo::Contact::Floor::FloorPenaltyParameters;
using pgo::Contact::Floor::FloorSide;
using pgo::Contact::CIPCTest::computeFloorEnergy;
using pgo::Contact::CIPCTest::computeFloorGradient;
using pgo::Contact::CIPCTest::computeFloorHessian;
using pgo::Contact::CIPCTest::flattenPositions;
using pgo::Contact::CIPCTest::makeTwoTriangleMesh;
using pgo::Contact::CIPCTest::relativeError;
using pgo::Contact::CIPCTest::sparseToDense;

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

int axisToIndex(FloorAxis axis)
{
  return static_cast<int>(axis);
}

FloorPenaltyParameters makeFloorParams(FloorAxis axis = FloorAxis::Z)
{
  FloorPenaltyParameters params;
  params.floorHeight = 0.02;
  params.floorKappa = 7.5;
  params.floorAxis = axis;
  return params;
}

double floorSideTestSign(FloorSide floorSide)
{
  return floorSide == FloorSide::KEEP_ABOVE ? 1.0 : -1.0;
}

double computeSidedFloorEnergy(const ES::VXd &x, double floorHeight, double floorKappa, int floorAxis, FloorSide floorSide)
{
  const double sideSign = floorSideTestSign(floorSide);
  double energy = 0.0;
  for (int vi = 0; vi < x.size() / 3; ++vi) {
    const double dzEff = sideSign * (x[3 * vi + floorAxis] - floorHeight);
    if (dzEff < 0.0)
      energy += 0.5 * floorKappa * dzEff * dzEff;
  }
  return energy;
}

ES::VXd computeSidedFloorGradient(const ES::VXd &x, double floorHeight, double floorKappa, int floorAxis, FloorSide floorSide)
{
  const double sideSign = floorSideTestSign(floorSide);
  ES::VXd g = ES::VXd::Zero(x.size());
  for (int vi = 0; vi < x.size() / 3; ++vi) {
    const double dzEff = sideSign * (x[3 * vi + floorAxis] - floorHeight);
    if (dzEff < 0.0)
      g[3 * vi + floorAxis] = floorKappa * dzEff * sideSign;
  }
  return g;
}

ES::MXd computeSidedFloorHessian(const ES::VXd &x, double floorHeight, double floorKappa, int floorAxis, FloorSide floorSide)
{
  const double sideSign = floorSideTestSign(floorSide);
  ES::MXd H = ES::MXd::Zero(x.size(), x.size());
  for (int vi = 0; vi < x.size() / 3; ++vi) {
    const double dzEff = sideSign * (x[3 * vi + floorAxis] - floorHeight);
    if (dzEff < 0.0)
      H(3 * vi + floorAxis, 3 * vi + floorAxis) = floorKappa;
  }
  return H;
}
}  // namespace

TEST(FloorContactEnergyGTest, UsesOnlyStatefulContactBoundary)
{
  const auto [V, F] = makeTwoTriangleMesh();
  (void)F;
  const ES::VXd rest = flattenPositions(V);
  Contact::Floor::FloorPenaltyParameters params;
  params.floorAxis = Contact::Floor::FloorAxis::Z;
  params.floorSide = Contact::Floor::FloorSide::KEEP_ABOVE;
  params.floorHeight = 0.0;
  params.floorKappa = 1.0;

  FloorContactEnergy energy(V, makeIdentityEmbedding(rest.size()), params);

  EXPECT_NE(dynamic_cast<Contact::StatefulContactEnergy *>(&energy), nullptr);
  EXPECT_EQ(dynamic_cast<NO::LineSearchAwareEnergy *>(&energy), nullptr);
  EXPECT_EQ(dynamic_cast<NO::EvaluationStateAwareEnergy *>(&energy), nullptr);
}

TEST(FloorContactEnergyGTest, IdentityEmbeddingMatchesReferenceFloorPenaltyOnAllAxes)
{
  const auto [V, F] = makeTwoTriangleMesh();
  (void)F;
  const ES::VXd rest = flattenPositions(V);

  for (const FloorAxis axis : { FloorAxis::X, FloorAxis::Y, FloorAxis::Z }) {
    ES::VXd u = ES::VXd::Zero(rest.size());
    const int ai = axisToIndex(axis);
    u[0 * 3 + ai] = -0.03;
    u[1 * 3 + ai] = -0.01;
    u[3 * 3 + ai] = 0.04;
    u[4 * 3 + ai] = -0.02;
    u[5 * 3 + ai] = -0.05;

    ES::VXd du = ES::VXd::Zero(rest.size());
    du[0 * 3 + ai] = -0.5;
    du[4 * 3 + ai] = 0.25;

    const FloorPenaltyParameters params = makeFloorParams(axis);
    FloorContactEnergy energy(V, makeIdentityEmbedding(rest.size()), params);

    const ES::VXd surfacePositions = rest + u;
    const double expectedEnergy = computeFloorEnergy(surfacePositions, params.floorHeight, params.floorKappa, ai);
    const ES::VXd expectedGradient = computeFloorGradient(surfacePositions, params.floorHeight, params.floorKappa, ai);
    const ES::MXd expectedHessian = computeFloorHessian(surfacePositions, params.floorHeight, params.floorKappa, ai);

    ES::VXd gradient(energy.getNumDOFs());
    energy.gradient(u, gradient);
    ES::SpMatD hessian;
    energy.hessian(u, hessian);

    EXPECT_NEAR(energy.func(u), expectedEnergy, 1e-12);
    EXPECT_LT(relativeError(gradient, expectedGradient), 1e-12);
    EXPECT_LT(relativeError(sparseToDense(hessian), expectedHessian), 1e-12);
    EXPECT_DOUBLE_EQ(energy.computeMaxStepLimit(u, du).alpha, 1.0);
  }
}

TEST(FloorContactEnergyGTest, SparseEmbeddingPullsBackGradientAndHessianOnYAxis)
{
  const auto [V, F] = makeTwoTriangleMesh();
  (void)F;
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
  simulationDisplacements[1] = -0.05;
  simulationDisplacements[4] = -0.03;
  simulationDisplacements[10] = 0.01;
  simulationDisplacements[13] = -0.02;

  const FloorPenaltyParameters params = makeFloorParams(FloorAxis::Y);
  FloorContactEnergy energy(V, W, params);

  const ES::VXd surfacePositions = surfaceRestPositions + W * simulationDisplacements;
  const ES::VXd surfaceGradient = computeFloorGradient(surfacePositions, params.floorHeight, params.floorKappa, 1);
  const ES::MXd surfaceHessian = computeFloorHessian(surfacePositions, params.floorHeight, params.floorKappa, 1);

  ES::VXd simulationGradient(energy.getNumDOFs());
  energy.gradient(simulationDisplacements, simulationGradient);
  ES::SpMatD simulationHessian;
  energy.hessian(simulationDisplacements, simulationHessian);

  const ES::VXd expectedGradient = W.transpose() * surfaceGradient;
  const ES::MXd expectedHessian = W.transpose() * surfaceHessian * ES::MXd(W);

  EXPECT_LT(relativeError(simulationGradient, expectedGradient), 1e-12);
  EXPECT_LT(relativeError(sparseToDense(simulationHessian), expectedHessian), 1e-12);
}

TEST(FloorContactEnergyGTest, HessianInPlaceAndAllocDoNotThrow)
{
  const auto [V, F] = makeTwoTriangleMesh();
  (void)F;
  const ES::VXd rest = flattenPositions(V);

  FloorPenaltyParameters params = makeFloorParams(FloorAxis::Z);
  FloorContactEnergy energy(V, makeIdentityEmbedding(rest.size()), params);
  const ES::VXd x = ES::VXd::Zero(rest.size());

  ES::SpMatD allocatedHessian;
  EXPECT_NO_THROW(energy.hessianAlloc(allocatedHessian));
  EXPECT_EQ(allocatedHessian.rows(), rest.size());
  EXPECT_EQ(allocatedHessian.cols(), rest.size());

  ES::SpMatD inPlaceHessian;
  EXPECT_NO_THROW(energy.hessianInPlace(x, inPlaceHessian));
  EXPECT_EQ(inPlaceHessian.rows(), rest.size());
  EXPECT_EQ(inPlaceHessian.cols(), rest.size());
}

TEST(FloorContactEnergyGTest, UpperSidePenalizesPointsAboveHeightAndPushesTowardNegativeAxis)
{
  const auto [V, F] = makeTwoTriangleMesh();
  (void)F;
  const ES::VXd rest = flattenPositions(V);

  ES::VXd u = ES::VXd::Zero(rest.size());
  u[0 * 3 + 0] = 0.04;
  u[1 * 3 + 0] = 0.12;
  u[2 * 3 + 0] = -0.05;
  u[4 * 3 + 0] = 0.08;

  FloorPenaltyParameters params = makeFloorParams(FloorAxis::X);
  params.floorHeight = 0.05;
  params.floorSide = FloorSide::KEEP_BELOW;
  FloorContactEnergy energy(V, makeIdentityEmbedding(rest.size()), params);

  const ES::VXd surfacePositions = rest + u;
  const double expectedEnergy = computeSidedFloorEnergy(surfacePositions, params.floorHeight, params.floorKappa, 0, params.floorSide);
  const ES::VXd expectedGradient = computeSidedFloorGradient(surfacePositions, params.floorHeight, params.floorKappa, 0, params.floorSide);
  const ES::MXd expectedHessian = computeSidedFloorHessian(surfacePositions, params.floorHeight, params.floorKappa, 0, params.floorSide);

  ES::VXd gradient(energy.getNumDOFs());
  energy.gradient(u, gradient);
  ES::SpMatD hessian;
  energy.hessian(u, hessian);

  EXPECT_NEAR(energy.func(u), expectedEnergy, 1e-12);
  EXPECT_LT(relativeError(gradient, expectedGradient), 1e-12);
  EXPECT_LT(relativeError(sparseToDense(hessian), expectedHessian), 1e-12);
  EXPECT_GT(gradient[1 * 3 + 0], 0.0);
}

TEST(FloorContactEnergyGTest, SetFloorHeightChangesNextEvaluation)
{
  const auto [V, F] = makeTwoTriangleMesh();
  (void)F;
  const ES::VXd rest = flattenPositions(V);

  FloorPenaltyParameters params = makeFloorParams(FloorAxis::Z);
  params.floorHeight = -0.05;
  FloorContactEnergy energy(V, makeIdentityEmbedding(rest.size()), params);

  const ES::VXd u = ES::VXd::Zero(rest.size());
  const double initialEnergy = energy.func(u);
  energy.setFloorHeight(0.20);

  EXPECT_DOUBLE_EQ(energy.floorHeight(), 0.20);
  EXPECT_GT(energy.func(u), initialEnergy);
}

TEST(FloorContactEnergyGTest, NonFiniteParametersThrow)
{
  const auto [V, F] = makeTwoTriangleMesh();
  (void)F;
  const ES::SpMatD W = makeIdentityEmbedding(V.rows() * 3);

  FloorPenaltyParameters missingHeight;
  missingHeight.floorHeight = std::numeric_limits<double>::quiet_NaN();
  missingHeight.floorKappa = 1.0;
  EXPECT_THROW(FloorContactEnergy(V, W, missingHeight), std::invalid_argument);

  FloorPenaltyParameters missingKappa;
  missingKappa.floorHeight = 0.0;
  missingKappa.floorKappa = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(FloorContactEnergy(V, W, missingKappa), std::invalid_argument);

  FloorPenaltyParameters nonFiniteSetter = makeFloorParams();
  FloorContactEnergy energy(V, W, nonFiniteSetter);
  EXPECT_THROW(energy.setFloorHeight(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

TEST(FloorContactEnergyGTest, EmptySurfaceOrZeroSimulationDofsThrow)
{
  FloorPenaltyParameters params = makeFloorParams();

  ES::MXd emptyVertices(0, 3);
  const ES::SpMatD emptySurfaceMap(0, 3);
  EXPECT_THROW(FloorContactEnergy(emptyVertices, emptySurfaceMap, params), std::invalid_argument);

  const auto [V, F] = makeTwoTriangleMesh();
  (void)F;
  const ES::SpMatD zeroSimulationDofMap(V.rows() * 3, 0);
  EXPECT_THROW(FloorContactEnergy(V, zeroSimulationDofMap, params), std::invalid_argument);
}

TEST(FloorContactEnergyGTest, InvalidAxisThrows)
{
  const auto [V, F] = makeTwoTriangleMesh();
  (void)F;
  const ES::SpMatD W = makeIdentityEmbedding(V.rows() * 3);

  FloorPenaltyParameters invalidAxis = makeFloorParams();
  invalidAxis.floorAxis = static_cast<FloorAxis>(3);
  EXPECT_THROW(FloorContactEnergy(V, W, invalidAxis), std::invalid_argument);

  FloorPenaltyParameters invalidSide = makeFloorParams();
  invalidSide.floorSide = static_cast<FloorSide>(99);
  EXPECT_THROW(FloorContactEnergy(V, W, invalidSide), std::invalid_argument);
}
