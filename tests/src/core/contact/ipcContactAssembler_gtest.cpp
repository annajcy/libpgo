#include <gtest/gtest.h>

#include "ipc/ipcContactAssembler.h"
#include "ipc/ipcPairGenerator.h"
#include "ipc/external/obstacleSurface.h"
#include "testCIPCHelpers.h"

#include <cmath>
#include <memory>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
namespace IPC = pgo::Contact::IPC;

constexpr double kFDStep = 1e-5;
constexpr double kGradTol = 1e-4;
constexpr double kProjectedHessFdTol = 1e-1;
constexpr double kProjectedHessMinEigenTol = 1e-8;

struct ExternalScene
{
  ES::MXd V;
  ES::MXi F;
  ES::MXd obstacleV;
  ES::MXi obstacleF;
};

ExternalScene makeExternalPlaneScene()
{
  ExternalScene scene;
  scene.V.resize(4, 3);
  scene.V << 0.0, 0.0, 0.32,
             1.0, 0.0, 0.32,
             0.0, 1.0, 0.32,
             1.0, 1.0, 0.32;
  scene.F.resize(2, 3);
  scene.F << 0, 1, 2,
             1, 3, 2;

  scene.obstacleV.resize(4, 3);
  scene.obstacleV << 0.0, 0.0, 0.0,
                     1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     1.0, 1.0, 0.0;
  scene.obstacleF.resize(2, 3);
  scene.obstacleF << 0, 1, 2,
                     1, 3, 2;
  return scene;
}

std::unique_ptr<IPC::ObstacleSurface> makeStaticObstacle(const ExternalScene &scene)
{
  return std::make_unique<IPC::StaticObstacleSurface>(scene.obstacleV, scene.obstacleF);
}
}  // namespace

TEST(IPCContactAssemblerGTest, ComputesFiniteEnergyGradientAndHessianForGeneratedActiveSet)
{
  const auto [V, F] = pgo::Contact::CIPCTest::makeTwoTriangleMesh();
  IPC::IPCPairGenerator::Parameters generatorParams;
  generatorParams.dhat = 0.1;
  generatorParams.dhatExternal = 0.1;

  IPC::IPCPairGenerator generator(generatorParams);
  generator.setMesh(V, F);
  const ES::VXd x = pgo::Contact::CIPCTest::flattenPositions(V);
  const IPC::SurfaceIPCActiveSet activeSet = generator.buildActiveSet(x);

  IPC::IPCContactAssembler::Parameters assemblerParams;
  assemblerParams.dhat = 0.1;
  assemblerParams.dhatExternal = 0.1;
  assemblerParams.kappa = 1.0;
  assemblerParams.epsEE = 0.0;
  IPC::IPCContactAssembler assembler(assemblerParams);

  ES::VXd g = ES::VXd::Zero(x.size());
  ES::SpMatD H;
  double e = 0.0;
  assembler.computeAll(generator.topology(), generator.obstacleViews(), activeSet, e, g, H);

  EXPECT_TRUE(std::isfinite(e));
  EXPECT_EQ(g.size(), x.size());
  EXPECT_EQ(H.rows(), x.size());
  EXPECT_EQ(H.cols(), x.size());
}

TEST(IPCContactAssemblerGTest, EnergyGradientMatchesFiniteDifference)
{
  const auto [V, F] = pgo::Contact::CIPCTest::makeTwoTriangleMesh();
  IPC::IPCPairGenerator::Parameters generatorParams;
  generatorParams.dhat = 0.1;
  generatorParams.dhatExternal = 0.1;

  IPC::IPCPairGenerator generator(generatorParams);
  generator.setMesh(V, F);
  const ES::VXd x = pgo::Contact::CIPCTest::flattenPositions(V);

  IPC::IPCContactAssembler::Parameters assemblerParams;
  assemblerParams.dhat = 0.1;
  assemblerParams.dhatExternal = 0.1;
  assemblerParams.kappa = 1.0;
  assemblerParams.epsEE = 0.0;
  IPC::IPCContactAssembler assembler(assemblerParams);

  const auto gradientAt = [&](const ES::VXd &state) {
    const IPC::SurfaceIPCActiveSet activeSet = generator.buildActiveSet(state);
    ES::VXd gradient(state.size());
    assembler.computeGradient(generator.topology(), generator.obstacleViews(), activeSet, gradient);
    return gradient;
  };
  const auto energyAt = [&](const ES::VXd &state) {
    const IPC::SurfaceIPCActiveSet activeSet = generator.buildActiveSet(state);
    return assembler.computeEnergy(generator.topology(), generator.obstacleViews(), activeSet);
  };

  const ES::VXd analyticGradient = gradientAt(x);
  const ES::VXd fdGradient = pgo::Contact::CIPCTest::finiteDifferenceGradient(energyAt, x, kFDStep);
  EXPECT_LT(pgo::Contact::CIPCTest::relativeError(analyticGradient, fdGradient), kGradTol);
}

TEST(IPCContactAssemblerGTest, ProjectedHessianRemainsSymmetricPSDAndTracksFiniteDifference)
{
  const auto [V, F] = pgo::Contact::CIPCTest::makeTwoTriangleMesh();
  IPC::IPCPairGenerator::Parameters generatorParams;
  generatorParams.dhat = 0.1;
  generatorParams.dhatExternal = 0.1;

  IPC::IPCPairGenerator generator(generatorParams);
  generator.setMesh(V, F);
  const ES::VXd x = pgo::Contact::CIPCTest::flattenPositions(V);

  IPC::IPCContactAssembler::Parameters assemblerParams;
  assemblerParams.dhat = 0.1;
  assemblerParams.dhatExternal = 0.1;
  assemblerParams.kappa = 1.0;
  assemblerParams.epsEE = 0.0;
  IPC::IPCContactAssembler assembler(assemblerParams);

  const IPC::SurfaceIPCActiveSet activeSet = generator.buildActiveSet(x);
  ES::SpMatD H;
  assembler.computeHessian(generator.topology(), generator.obstacleViews(), activeSet, H);
  const ES::MXd analyticH = pgo::Contact::CIPCTest::sparseToDense(H);

  const ES::MXd fdH = pgo::Contact::CIPCTest::finiteDifferenceHessian(
    [&](const ES::VXd &state) {
      const IPC::SurfaceIPCActiveSet stateActiveSet = generator.buildActiveSet(state);
      ES::VXd gradient(state.size());
      assembler.computeGradient(generator.topology(), generator.obstacleViews(), stateActiveSet, gradient);
      return gradient;
    },
    x,
    kFDStep);

  const ES::MXd symAnalyticH = 0.5 * (analyticH + analyticH.transpose());
  const ES::MXd symFdH = 0.5 * (fdH + fdH.transpose());
  const Eigen::SelfAdjointEigenSolver<ES::MXd> eig(symAnalyticH);

  EXPECT_LT(pgo::Contact::CIPCTest::relativeError(analyticH, analyticH.transpose()), 1e-12);
  ASSERT_EQ(eig.info(), Eigen::Success);
  EXPECT_GE(eig.eigenvalues().minCoeff(), -kProjectedHessMinEigenTol);
  EXPECT_LT(pgo::Contact::CIPCTest::relativeError(symAnalyticH, symFdH), kProjectedHessFdTol);
}

TEST(IPCContactAssemblerGTest, ActiveSetConsumersWithExternalObstaclesAreConsistent)
{
  const ExternalScene scene = makeExternalPlaneScene();

  IPC::IPCPairGenerator::Parameters generatorParams;
  generatorParams.dhat = 0.1;
  generatorParams.dhatExternal = 0.35;

  std::vector<std::unique_ptr<IPC::ObstacleSurface>> generatorObstacles;
  generatorObstacles.push_back(makeStaticObstacle(scene));
  IPC::IPCPairGenerator generator(generatorParams, std::move(generatorObstacles));
  generator.setMesh(scene.V, scene.F);

  IPC::IPCContactAssembler::Parameters assemblerParams;
  assemblerParams.dhat = generatorParams.dhat;
  assemblerParams.dhatExternal = generatorParams.dhatExternal;
  assemblerParams.kappa = 1.0;
  assemblerParams.epsEE = 0.0;
  IPC::IPCContactAssembler assembler(assemblerParams);

  const ES::VXd x = pgo::Contact::CIPCTest::flattenPositions(scene.V);
  const IPC::SurfaceIPCActiveSet activeSet = generator.buildActiveSet(x);
  ASSERT_GT(activeSet.externalPairs.size(), 0u);

  const double assemblerEnergy = assembler.computeEnergy(generator.topology(), generator.obstacleViews(), activeSet);
  EXPECT_TRUE(std::isfinite(assemblerEnergy));
  EXPECT_GT(assemblerEnergy, 0.0);

  ES::VXd assemblerGradient = ES::VXd::Constant(x.size(), 123.0);
  assembler.computeGradient(generator.topology(), generator.obstacleViews(), activeSet, assemblerGradient);
  EXPECT_EQ(assemblerGradient.size(), x.size());
  EXPECT_GT(assemblerGradient.norm(), 0.0);

  ES::SpMatD assemblerHessian(x.size(), x.size());
  assemblerHessian.setIdentity();
  assembler.computeHessian(generator.topology(), generator.obstacleViews(), activeSet, assemblerHessian);
  EXPECT_EQ(assemblerHessian.rows(), x.size());
  EXPECT_EQ(assemblerHessian.cols(), x.size());

  double assemblerAllEnergy = -7.0;
  ES::VXd assemblerAllGradient = ES::VXd::Constant(1, -2.0);
  ES::SpMatD assemblerAllHessian;
  assembler.computeAll(generator.topology(), generator.obstacleViews(), activeSet, assemblerAllEnergy, assemblerAllGradient, assemblerAllHessian);

  EXPECT_NEAR(assemblerAllEnergy, assemblerEnergy, 1e-12);
  EXPECT_LT(pgo::Contact::CIPCTest::relativeError(assemblerAllGradient, assemblerGradient), 1e-12);
  EXPECT_LT(pgo::Contact::CIPCTest::relativeError(
              pgo::Contact::CIPCTest::sparseToDense(assemblerAllHessian),
              pgo::Contact::CIPCTest::sparseToDense(assemblerHessian)),
    1e-12);
}
