#include <gtest/gtest.h>

#include "contactEnergyFactory.h"
#include "ipc/ipcContactEnergy.h"
#include "sampled_penalty/sampledPenaltyContactEnergy.h"
#include "statefulContactEnergy.h"
#include "evaluationStateAwareEnergy.h"
#include "stepAwareEnergy.h"

#include <numeric>
#include <stdexcept>

namespace
{
namespace ES = pgo::EigenSupport;
namespace Contact = pgo::Contact;
namespace NO = pgo::NonlinearOptimization;

ES::MXd makeTriangle()
{
  ES::MXd V(3, 3);
  V << 0.0, 0.0, 0.0,
       1.0, 0.0, 0.0,
       0.0, 1.0, 0.0;
  return V;
}

ES::MXi makeTriangleFaces()
{
  ES::MXi F(1, 3);
  F << 0, 1, 2;
  return F;
}

ES::SpMatD makeIdentityMap(int n)
{
  std::vector<ES::TripletD> triplets;
  triplets.reserve(n);
  for (int i = 0; i < n; i++)
    triplets.emplace_back(i, i, 1.0);
  ES::SpMatD W(n, n);
  W.setFromTriplets(triplets.begin(), triplets.end());
  return W;
}

class TestStatefulContactEnergy : public Contact::StatefulContactEnergy
{
public:
  explicit TestStatefulContactEnergy(int n): n_(n) {}

  double func(ES::ConstRefVecXd) const override { return 0.0; }
  void gradient(ES::ConstRefVecXd, ES::RefVecXd grad) const override { grad.setZero(); }
  void hessianInPlace(ES::ConstRefVecXd, ES::SpMatD &) const override {}
  void hessianAlloc(ES::SpMatD &hess) const override { hess.resize(n_, n_); }
  void getDOFs(std::vector<int> &dofs) const override
  {
    dofs.resize(n_);
    std::iota(dofs.begin(), dofs.end(), 0);
  }
  int getNumDOFs() const override { return n_; }
  Contact::ContactModelKind contactModelKind() const override { return Contact::ContactModelKind::IPC; }

private:
  int n_;
};
}  // namespace

TEST(ContactEnergyFactoryGTest, EmbeddedDofMapMapsSimulationDisplacementsToSurfacePositions)
{
  const ES::MXd rest = makeTriangle();
  const ES::SpMatD W = makeIdentityMap(9);
  Contact::EmbeddedDofMap map(rest, W);

  ES::VXd u = ES::VXd::Zero(9);
  u[2] = -0.25;
  const ES::VXd positions = map.surfacePositions(u);

  EXPECT_EQ(map.numSimulationDofs(), 9);
  EXPECT_EQ(map.numSurfaceDofs(), 9);
  EXPECT_DOUBLE_EQ(positions[2], -0.25);
  EXPECT_DOUBLE_EQ(positions[3], 1.0);
}

TEST(ContactEnergyFactoryGTest, CreateFloorEnergyBuildsDisplacementEnergy)
{
  Contact::ContactSurfaceSpec surface;
  surface.restVertices = makeTriangle();
  surface.surfaceFromSimulationDispMap = makeIdentityMap(9);

  Contact::FloorContactSpec floor;
  floor.axis = Contact::FloorAxis::Z;
  floor.side = Contact::FloorSide::KeepAbove;
  floor.height = 0.1;
  floor.stiffness = 2.0;

  const auto energy = Contact::createFloorEnergy(surface, floor);
  ASSERT_NE(energy, nullptr);
  EXPECT_EQ(energy->stateKind(), NO::EnergyStateKind::Displacement);

  ES::VXd u = ES::VXd::Zero(energy->getNumDOFs());
  EXPECT_GT(energy->func(u), 0.0);
}

TEST(ContactEnergyFactoryGTest, StatefulContactEnergyIsOnlyCommonContactBoundary)
{
  TestStatefulContactEnergy energy(3);
  const auto *stepAware = dynamic_cast<NO::StepAwareEnergy *>(&energy);
  const auto *evaluationAware = dynamic_cast<const NO::EvaluationStateAwareEnergy *>(&energy);
  const auto *lineSearchAware = dynamic_cast<const NO::LineSearchAwareEnergy *>(&energy);

  EXPECT_EQ(stepAware, nullptr);
  EXPECT_EQ(evaluationAware, nullptr);
  EXPECT_EQ(lineSearchAware, nullptr);
  EXPECT_EQ(energy.stateKind(), NO::EnergyStateKind::Displacement);
}

TEST(ContactEnergyFactoryGTest, CreateSampledPenaltyEnergyBuildsNormalAndFrictionalModels)
{
  Contact::ContactSurfaceSpec surface;
  surface.restVertices = makeTriangle();
  surface.surfaceFromSimulationDispMap = makeIdentityMap(9);

  Contact::SampledPenaltyContactSpec params;
  params.stiffness = 10.0;
  params.samples = 1;

  auto normal = Contact::SampledPenalty::createSampledPenaltyEnergy(surface, makeTriangleFaces(), params);
  ASSERT_NE(normal, nullptr);
  EXPECT_EQ(normal->contactModelKind(), Contact::ContactModelKind::SampledPenalty);
  EXPECT_NE(dynamic_cast<const NO::EvaluationStateAwareEnergy *>(normal.get()), nullptr);
  EXPECT_NE(dynamic_cast<const NO::LineSearchAwareEnergy *>(normal.get()), nullptr);
  EXPECT_EQ(dynamic_cast<NO::StepAwareEnergy *>(normal.get()), nullptr);
  EXPECT_EQ(dynamic_cast<NO::StepDependentEnergy *>(normal.get()), nullptr);

  Contact::FrictionContactSpec friction;
  friction.frictionCoeff = 0.5;
  friction.velocityEps = 1e-5;
  auto frictional = Contact::SampledPenalty::createFrictionalSampledPenaltyEnergy(surface, makeTriangleFaces(), params, friction);
  ASSERT_NE(frictional, nullptr);
  EXPECT_EQ(frictional->contactModelKind(), Contact::ContactModelKind::SampledPenalty);
  EXPECT_NE(dynamic_cast<const NO::EvaluationStateAwareEnergy *>(frictional.get()), nullptr);
  EXPECT_NE(dynamic_cast<const NO::LineSearchAwareEnergy *>(frictional.get()), nullptr);
  EXPECT_NE(dynamic_cast<NO::StepAwareEnergy *>(frictional.get()), nullptr);
  EXPECT_NE(dynamic_cast<NO::StepDependentEnergy *>(frictional.get()), nullptr);
}

TEST(ContactEnergyFactoryGTest, CreateIPCEnergyBuildsStaticAndMovingObstacleSpecs)
{
  Contact::ContactSurfaceSpec surface;
  surface.restVertices = makeTriangle();
  surface.restVertices.col(2).array() += 0.02;
  surface.surfaceFromSimulationDispMap = makeIdentityMap(9);

  Contact::IPCContactSpec params;
  params.dhat = 0.1;
  params.dhatExternal = 0.1;
  params.kappa = 1.0;

  Contact::StaticObstacleSpec staticObstacle;
  staticObstacle.restVertices = makeTriangle();
  staticObstacle.triangles = makeTriangleFaces();

  Contact::LinearMovingObstacleSpec movingObstacle;
  movingObstacle.restVertices = makeTriangle();
  movingObstacle.restVertices.col(2).array() -= 0.2;
  movingObstacle.triangles = makeTriangleFaces();
  movingObstacle.velocity = ES::V3d(0.0, 0.0, 0.1);

  std::vector<Contact::ObstacleSpec> obstacles;
  obstacles.emplace_back(staticObstacle);
  obstacles.emplace_back(movingObstacle);

  auto ipc = Contact::IPC::createIPCEnergy(surface, makeTriangleFaces(), params, std::move(obstacles));
  ASSERT_NE(ipc, nullptr);
  EXPECT_EQ(ipc->contactModelKind(), Contact::ContactModelKind::IPC);
  EXPECT_EQ(ipc->stateKind(), NO::EnergyStateKind::Displacement);
  auto *ipcConcrete = dynamic_cast<Contact::IPC::IPCContactEnergy *>(ipc.get());
  ASSERT_NE(ipcConcrete, nullptr);

  ES::VXd x = ES::VXd::Zero(ipc->getNumDOFs());
  NO::StepState state;
  state.time = 0.0;
  state.timestep = 0.1;
  ipcConcrete->beginStep(state);
  EXPECT_NO_THROW((void)ipc->func(x));

  ipcConcrete->setMovingObstacleTime(0.25);
  EXPECT_NO_THROW((void)ipc->func(x));
}

TEST(ContactEnergyFactoryGTest, SampledPenaltyFactoryAcceptsNonIdentitySurfaceMap)
{
  Contact::ContactSurfaceSpec surface;
  surface.restVertices = makeTriangle();
  surface.surfaceFromSimulationDispMap = makeIdentityMap(9);
  surface.surfaceFromSimulationDispMap.conservativeResize(9, 12);
  surface.surfaceFromSimulationDispMap.coeffRef(0, 9) = 0.25;
  surface.surfaceFromSimulationDispMap.makeCompressed();

  Contact::SampledPenaltyContactSpec params;
  params.stiffness = 10.0;
  params.samples = 1;

  auto energy = Contact::SampledPenalty::createSampledPenaltyEnergy(surface, makeTriangleFaces(), params);
  ASSERT_NE(energy, nullptr);
  EXPECT_EQ(energy->getNumDOFs(), 12);
  EXPECT_EQ(energy->contactModelKind(), Contact::ContactModelKind::SampledPenalty);
  EXPECT_NE(dynamic_cast<const NO::EvaluationStateAwareEnergy *>(energy.get()), nullptr);
  EXPECT_NE(dynamic_cast<const NO::LineSearchAwareEnergy *>(energy.get()), nullptr);
  EXPECT_EQ(dynamic_cast<NO::StepAwareEnergy *>(energy.get()), nullptr);

  ES::VXd u = ES::VXd::Zero(12);
  EXPECT_NO_THROW((void)energy->func(u));
}

TEST(ContactEnergyFactoryGTest, FrictionalSampledPenaltyFactoryMapsPreviousSimulationState)
{
  Contact::ContactSurfaceSpec surface;
  surface.restVertices = makeTriangle();
  surface.surfaceFromSimulationDispMap = makeIdentityMap(9);
  surface.surfaceFromSimulationDispMap.conservativeResize(9, 12);
  surface.surfaceFromSimulationDispMap.coeffRef(0, 9) = 0.25;
  surface.surfaceFromSimulationDispMap.makeCompressed();

  Contact::SampledPenaltyContactSpec params;
  params.stiffness = 10.0;
  params.samples = 1;

  Contact::FrictionContactSpec friction;
  friction.frictionCoeff = 0.5;
  friction.velocityEps = 1e-5;
  auto energy = Contact::SampledPenalty::createFrictionalSampledPenaltyEnergy(surface, makeTriangleFaces(), params, friction);
  ASSERT_NE(energy, nullptr);
  EXPECT_EQ(energy->getNumDOFs(), 12);
  EXPECT_NE(dynamic_cast<NO::StepAwareEnergy *>(energy.get()), nullptr);
  EXPECT_NE(dynamic_cast<NO::StepDependentEnergy *>(energy.get()), nullptr);

  ES::VXd previous = ES::VXd::Zero(12);
  NO::StepState state;
  state.previousX = &previous;
  state.timestep = 0.1;
  EXPECT_NO_THROW(dynamic_cast<NO::StepAwareEnergy *>(energy.get())->beginStep(state));
}
