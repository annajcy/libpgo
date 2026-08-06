#include <gtest/gtest.h>

#include "ipc/broadPhase/surfaceIPCBroadPhase.h"
#include "ipc/core/surfaceIPCSelfBarrierAssembler.h"
#include "ipc/core/surfaceIPCExternalBarrierAssembler.h"
#include "ipc/core/surfaceIPCBarrierKernels.h"
#include "ipc/external/obstacleSurface.h"
#include "ipc/topology/surfaceIPCTopology.h"

#include "testCIPCHelpers.h"

#include <algorithm>
#include <memory>
#include <string_view>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
using pgo::Contact::IPC::ExternalPairSet;
using pgo::Contact::IPC::EEPair;
using pgo::Contact::IPC::ObstacleSurface;
using pgo::Contact::IPC::ObstacleSurfaceView;
using pgo::Contact::IPC::PTPair;
using pgo::Contact::IPC::SelfPairSet;
using pgo::Contact::IPC::SurfaceIPCTopology;
using pgo::Contact::IPC::TrajectoryObstacleSurface;
using pgo::Contact::CIPCTest::flattenPositions;
using pgo::Contact::CIPCTest::makeTwoTriangleMesh;
using pgo::Contact::CIPCTest::relativeError;
using pgo::Contact::CIPCTest::sparseToDense;
namespace kernels = pgo::Contact::IPC::barrier_kernels;

std::vector<ObstacleSurfaceView> obstacleViews(const std::vector<std::unique_ptr<ObstacleSurface>> &obstacles)
{
  std::vector<ObstacleSurfaceView> views;
  views.reserve(obstacles.size());
  for (const auto &obstacle : obstacles)
    views.push_back(pgo::Contact::IPC::makeObstacleSurfaceView(*obstacle));
  return views;
}


}  // namespace

TEST(SurfaceIPCBarrierAssemblerGTest, SelfAssemblyEnergyGradientHessianAreConsistent)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd x = flattenPositions(V);

  SurfaceIPCTopology topology;
  topology.setMesh(V, F);

  constexpr double dhat = 0.1;
  constexpr double kappa = 1.0;
  constexpr double epsEE = 0.0;
  SelfPairSet pairs;
  buildSelfPairs(topology, x, dhat, pairs);

  const double helperEnergy = computeSelfEnergy(x, pairs, topology.numVerts, dhat, kappa, epsEE);
  ES::VXd helperGradient(x.size());
  computeSelfGradient(x, pairs, topology.numVerts, dhat, kappa, epsEE, helperGradient);
  ES::SpMatD helperHessian;
  computeSelfHessian(x, pairs, topology.numVerts, dhat, kappa, epsEE, helperHessian);

  double allEnergy = -1.0;
  ES::VXd allGradient = ES::VXd::Constant(1, -1.0);
  ES::SpMatD allHessian;
  computeSelfAll(x, pairs, topology.numVerts, dhat, kappa, epsEE, allEnergy, allGradient, allHessian);

  EXPECT_NEAR(helperEnergy, allEnergy, 1e-12);
  EXPECT_LT(relativeError(helperGradient, allGradient), 1e-12);
  EXPECT_LT(relativeError(sparseToDense(helperHessian), sparseToDense(allHessian)), 1e-12);
}

TEST(SurfaceIPCBarrierAssemblerGTest, KernelPTActiveAndInactive)
{
  // Vertex at origin, triangle 0.05 above — close enough to activate with dhat=0.1
  ES::V3d p(0.0, 0.0, 0.0);
  ES::V3d t0(0.0, 0.0, 0.05);
  ES::V3d t1(1.0, 0.0, 0.05);
  ES::V3d t2(0.0, 1.0, 0.05);

  const double dhat2 = 0.01;   // 0.1²
  const double kappa = 1.0;
  const double weight = 1.0;

  // Energy-only: should be active with positive energy
  auto kE = kernels::pointTriangle(p, t0, t1, t2, weight, dhat2, kappa, false, false);
  ASSERT_TRUE(kE.active);
  EXPECT_GT(kE.energy, 0.0);

  // Energy + gradient: gradient w.r.t. p should point toward triangle (+z)
  auto kG = kernels::pointTriangle(p, t0, t1, t2, weight, dhat2, kappa, true, false);
  ASSERT_TRUE(kG.active);
  EXPECT_NEAR(kG.energy, kE.energy, 1e-12);
  EXPECT_GT(kG.gradient[2], 0.0);  // p.z gradient positive

  // Energy + hessian (PSD-projected): p-p diagonal should be positive
  auto kH = kernels::pointTriangle(p, t0, t1, t2, weight, dhat2, kappa, false, true);
  ASSERT_TRUE(kH.active);
  EXPECT_GT(kH.hessian(2, 2), 0.0);

  // Too far: triangle 1.0 above — should be inactive
  ES::V3d far0(0.0, 0.0, 1.0);
  ES::V3d far1(1.0, 0.0, 1.0);
  ES::V3d far2(0.0, 1.0, 1.0);
  auto kFar = kernels::pointTriangle(p, far0, far1, far2, weight, dhat2, kappa, false, false);
  EXPECT_FALSE(kFar.active);

  // Coincident (d2=0) — should be inactive
  auto kZero = kernels::pointTriangle(p, p, t1, t2, weight, dhat2, kappa, false, false);
  EXPECT_FALSE(kZero.active);
}

TEST(SurfaceIPCBarrierAssemblerGTest, KernelEEActiveAndInactive)
{
  // Two edges 0.02 apart — close enough to activate with dhat=0.1
  ES::V3d ea0(0.0, -0.01, 0.0);
  ES::V3d ea1(1.0, -0.01, 0.0);
  ES::V3d eb0(0.0,  0.01, 0.0);
  ES::V3d eb1(1.0,  0.01, 0.0);

  const double dhat2 = 0.01;
  const double kappa = 1.0;
  const double weight = 1.0;

  // Without mollifier
  auto k = kernels::edgeEdge(ea0, ea1, eb0, eb1, weight, dhat2, kappa, 0.0, true, true);
  ASSERT_TRUE(k.active);
  EXPECT_GT(k.energy, 0.0);
  // Gradient should push edges apart (opposite y directions)
  EXPECT_GT(std::abs(k.gradient[1]), 0.0);
  EXPECT_GT(std::abs(k.gradient[7]), 0.0);
  EXPECT_LT(k.gradient[1] * k.gradient[7], 0.0);  // opposite signs
  // Hessian diagonal should be positive (PSD)
  EXPECT_GT(k.hessian(1, 1), 0.0);

  // With mollifier: energy should differ
  auto kM = kernels::edgeEdge(ea0, ea1, eb0, eb1, weight, dhat2, kappa, 1e-3, true, true);
  ASSERT_TRUE(kM.active);
  EXPECT_NE(kM.energy, k.energy);

  // Too far apart — inactive
  ES::V3d far0(0.0, -1.0, 0.0);
  ES::V3d far1(1.0, -1.0, 0.0);
  ES::V3d far2(0.0,  1.0, 0.0);
  ES::V3d far3(1.0,  1.0, 0.0);
  auto kFar = kernels::edgeEdge(far0, far1, far2, far3, weight, dhat2, kappa, 0.0, false, false);
  EXPECT_FALSE(kFar.active);
}

TEST(SurfaceIPCBarrierAssemblerGTest, ExternalDynamicPointKernelMatchesGenericSubBlock)
{
  const ES::V3d p(0.10, 0.20, 0.0);
  const ES::V3d t0(0.0, 0.0, 0.04);
  const ES::V3d t1(1.0, 0.0, 0.04);
  const ES::V3d t2(0.0, 1.0, 0.04);

  const double dhat2 = 0.01;
  const double kappa = 2.0;
  const double weight = 0.75;

  const auto generic = kernels::pointTriangle(p, t0, t1, t2, weight, dhat2, kappa, true, true);
  const auto external = kernels::pointStaticTriangle(p, t0, t1, t2, weight, dhat2, kappa, true, true);

  ASSERT_TRUE(generic.active);
  ASSERT_EQ(external.active, generic.active);
  EXPECT_NEAR(external.energy, generic.energy, 1e-12);
  EXPECT_LT((external.gradient - generic.gradient.head<3>()).norm(), 1e-12);
  EXPECT_LT((external.hessian - generic.hessian.block<3, 3>(0, 0)).norm(), 1e-12);
}

TEST(SurfaceIPCBarrierAssemblerGTest, ExternalDynamicTriangleKernelMatchesGenericSubBlock)
{
  const ES::V3d p(0.10, 0.20, 0.0);
  const ES::V3d t0(0.0, 0.0, 0.04);
  const ES::V3d t1(1.0, 0.0, 0.04);
  const ES::V3d t2(0.0, 1.0, 0.04);

  const double dhat2 = 0.01;
  const double kappa = 2.0;
  const double weight = 0.75;

  const auto generic = kernels::pointTriangle(p, t0, t1, t2, weight, dhat2, kappa, true, true);
  const auto external = kernels::staticPointTriangle(p, t0, t1, t2, weight, dhat2, kappa, true, true);

  ASSERT_TRUE(generic.active);
  ASSERT_EQ(external.active, generic.active);
  EXPECT_NEAR(external.energy, generic.energy, 1e-12);
  EXPECT_LT((external.gradient - generic.gradient.segment<9>(3)).norm(), 1e-12);
  EXPECT_LT((external.hessian - generic.hessian.block<9, 9>(3, 3)).norm(), 1e-12);
}

TEST(SurfaceIPCBarrierAssemblerGTest, ExternalDynamicEdgeKernelMatchesGenericSubBlock)
{
  const ES::V3d ea0(0.0, -0.01, 0.0);
  const ES::V3d ea1(1.0, -0.01, 0.0);
  const ES::V3d eb0(0.0,  0.01, 0.0);
  const ES::V3d eb1(1.0,  0.01, 0.0);

  const double dhat2 = 0.01;
  const double kappa = 1.5;
  const double weight = 0.8;
  const double epsEE = 1e-3;

  const auto generic = kernels::edgeEdge(ea0, ea1, eb0, eb1, weight, dhat2, kappa, epsEE, true, true);
  const auto external = kernels::edgeStaticEdge(ea0, ea1, eb0, eb1, weight, dhat2, kappa, epsEE, true, true);

  ASSERT_TRUE(generic.active);
  ASSERT_EQ(external.active, generic.active);
  EXPECT_NEAR(external.energy, generic.energy, 1e-12);
  EXPECT_LT((external.gradient - generic.gradient.head<6>()).norm(), 1e-12);
  EXPECT_LT((external.hessian - generic.hessian.block<6, 6>(0, 0)).norm(), 1e-12);
}

TEST(SurfaceIPCBarrierAssemblerGTest, EmptyExternalAllPreservesExistingOutputsAndSizesFreshOutputs)
{
  ES::MXd V(2, 3);
  V << 0.0, 0.0, 0.0,
       1.0, 0.0, 0.0;
  const ES::VXd x = flattenPositions(V);
  const ExternalPairSet pairs;
  const std::vector<ObstacleSurfaceView> obstacles;

  double energy = -1.0;
  ES::VXd grad = ES::VXd::Constant(x.size(), 3.0);
  ES::SpMatD hess(x.size(), x.size());
  std::vector<ES::TripletD> entries;
  entries.emplace_back(0, 0, 2.0);
  entries.emplace_back(3, 4, -1.0);
  hess.setFromTriplets(entries.begin(), entries.end());

  computeExternalAll(x, obstacles, pairs, static_cast<int>(V.rows()), 0.1, 1.0, 0.0, energy, grad, hess);

  EXPECT_EQ(energy, 0.0);
  EXPECT_TRUE(grad.isApprox(ES::VXd::Constant(x.size(), 3.0), 0.0));
  EXPECT_EQ(hess.rows(), x.size());
  EXPECT_EQ(hess.cols(), x.size());
  EXPECT_EQ(hess.nonZeros(), 2);
  EXPECT_DOUBLE_EQ(hess.coeff(0, 0), 2.0);
  EXPECT_DOUBLE_EQ(hess.coeff(3, 4), -1.0);

  ES::VXd freshGrad;
  ES::SpMatD freshHess;
  computeExternalAll(x, obstacles, pairs, static_cast<int>(V.rows()), 0.1, 1.0, 0.0, energy, freshGrad, freshHess);

  EXPECT_EQ(energy, 0.0);
  ASSERT_EQ(freshGrad.size(), x.size());
  EXPECT_TRUE(freshGrad.isZero(0.0));
  EXPECT_EQ(freshHess.rows(), x.size());
  EXPECT_EQ(freshHess.cols(), x.size());
  EXPECT_EQ(freshHess.nonZeros(), 0);
}

TEST(SurfaceIPCBarrierAssemblerGTest, SinglePairGradientScatterIsCorrect)
{
  // Create a minimal mesh with 2 vertices, build one PT pair manually
  // Then verify gradient is scattered to the correct DOFs
  ES::MXd V(2, 3);
  V << 0.0, 0.0, 0.0,
       0.0, 0.0, 0.1;
  ES::MXi F(0, 3);  // no triangles — we build the pair manually

  SurfaceIPCTopology topology;
  topology.setMesh(V, F);

  SelfPairSet pairs;
  PTPair ptPair = { 0, 1, 1, 1, 1.0 };  // vertex 0 vs (degenerate) "triangle" using vertex 1×3
  pairs.ptPairs.push_back(ptPair);

  const ES::VXd x = flattenPositions(V);
  const double dhat = 1.0;  // large enough to activate
  const double kappa = 1.0;
  const double epsEE = 0.0;

  ES::VXd grad = ES::VXd::Zero(x.size());
  computeSelfGradient(x, pairs, topology.numVerts, dhat, kappa, epsEE, grad);

  // The gradient should be non-zero (barrier active)
  double gradNorm = grad.norm();
  EXPECT_GT(gradNorm, 0.0);
}

TEST(SurfaceIPCBarrierAssemblerGTest, SinglePairHessianScatterProducesSymmetricMatrix)
{
  ES::MXd V(2, 3);
  V << 0.0, 0.0, 0.0,
       0.0, 0.0, 0.1;
  ES::MXi F(0, 3);

  SurfaceIPCTopology topology;
  topology.setMesh(V, F);

  SelfPairSet pairs;
  PTPair ptPair = { 0, 1, 1, 1, 1.0 };
  pairs.ptPairs.push_back(ptPair);

  const ES::VXd x = flattenPositions(V);
  const double dhat = 1.0;
  const double kappa = 1.0;
  const double epsEE = 0.0;

  ES::SpMatD hess;
  computeSelfHessian(x, pairs, topology.numVerts, dhat, kappa, epsEE, hess);

  // Hessian should be symmetric
  ES::MXd denseH = sparseToDense(hess);
  EXPECT_LT((denseH - denseH.transpose()).norm(), 1e-12);
  EXPECT_GT(hess.nonZeros(), 0);
}

