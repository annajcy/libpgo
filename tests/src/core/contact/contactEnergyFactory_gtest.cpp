#include <gtest/gtest.h>

#include "contactEnergyFactory.h"
#include "ipc/ipcContactEnergy.h"
#include "sampled_penalty/sampledPenaltyContactEnergy.h"
#include "statefulContactEnergy.h"
#include "evaluationStateAwareEnergy.h"
#include "stepAwareEnergy.h"

#include <numeric>

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
  void beginStep(const NO::StepState &) override { beginStepCalls++; }
  void refreshActiveSet(ES::ConstRefVecXd x) const override
  {
    refreshCalls++;
    lastRefresh = x;
  }
  void clearActiveSet() const override { clearCalls++; }

  int beginStepCalls = 0;
  mutable int refreshCalls = 0;
  mutable int clearCalls = 0;
  mutable ES::VXd lastRefresh;

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

TEST(ContactEnergyFactoryGTest, StatefulContactEnergyPrepareEvaluationRefreshesActiveSet)
{
  TestStatefulContactEnergy energy(3);
  const auto *stepAware = dynamic_cast<NO::StepAwareEnergy *>(&energy);
  const auto *evaluationAware = dynamic_cast<const NO::EvaluationStateAwareEnergy *>(&energy);
  const auto *lineSearchAware = dynamic_cast<const NO::LineSearchAwareEnergy *>(&energy);

  EXPECT_NE(stepAware, nullptr);
  EXPECT_NE(evaluationAware, nullptr);
  EXPECT_NE(lineSearchAware, nullptr);
  EXPECT_EQ(energy.stateKind(), NO::EnergyStateKind::Displacement);

  ES::VXd x(3);
  x << 1.0, 2.0, 3.0;
  energy.prepareEvaluationState(x);

  ASSERT_EQ(energy.refreshCalls, 1);
  EXPECT_DOUBLE_EQ(energy.lastRefresh[1], 2.0);
}

TEST(ContactEnergyFactoryGTest, CreateSampledPenaltyEnergyBuildsNormalAndFrictionalModels)
{
  Contact::ContactSurfaceSpec surface;
  surface.restVertices = makeTriangle();
  surface.surfaceFromSimulationDispMap = makeIdentityMap(9);

  Contact::SampledPenalty::ParametersSpec params;
  params.stiffness = 10.0;
  params.samples = 1;

  auto normal = Contact::SampledPenalty::createSampledPenaltyEnergy(surface, makeTriangleFaces(), params);
  ASSERT_NE(normal, nullptr);
  EXPECT_EQ(normal->contactModelKind(), Contact::ContactModelKind::SampledPenalty);
  EXPECT_EQ(dynamic_cast<NO::StepDependentEnergy *>(normal.get()), nullptr);

  Contact::SampledPenalty::FrictionParametersSpec friction;
  friction.frictionCoeff = 0.5;
  friction.velocityEps = 1e-5;
  auto frictional = Contact::SampledPenalty::createFrictionalSampledPenaltyEnergy(surface, makeTriangleFaces(), params, friction);
  ASSERT_NE(frictional, nullptr);
  EXPECT_EQ(frictional->contactModelKind(), Contact::ContactModelKind::SampledPenalty);
  EXPECT_NE(dynamic_cast<NO::StepDependentEnergy *>(frictional.get()), nullptr);
}

TEST(ContactEnergyFactoryGTest, CreateIPCEnergyBuildsStaticAndMovingObstacleSpecs)
{
  Contact::ContactSurfaceSpec surface;
  surface.restVertices = makeTriangle();
  surface.restVertices.col(2).array() += 0.02;
  surface.surfaceFromSimulationDispMap = makeIdentityMap(9);

  Contact::IPC::ParametersSpec params;
  params.dhat = 0.1;
  params.dhat_external = 0.1;
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

  ES::VXd x = ES::VXd::Zero(ipc->getNumDOFs());
  NO::StepState state;
  state.time = 0.0;
  state.timestep = 0.1;
  ipc->beginStep(state);
  ipc->refreshActiveSet(x);
  EXPECT_NO_THROW((void)ipc->func(x));

  ipc->setMovingObstacleTime(0.25);
  ipc->refreshActiveSet(x);
  EXPECT_NO_THROW((void)ipc->func(x));
}
