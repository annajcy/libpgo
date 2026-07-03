#include <gtest/gtest.h>

#include "sampled_penalty/sampledPenaltyContactBuilder.h"
#include "sampled_penalty/sampledPenaltyContactEnergy.h"
#include "energy/stepAwareEnergy.h"

#include <stdexcept>

namespace
{
namespace ES = pgo::EigenSupport;
namespace Contact = pgo::Contact;
namespace SP = pgo::Contact::SampledPenalty;
namespace NO = pgo::NonlinearOptimization;

ES::MXd makeSingleTriangleVertices()
{
  ES::MXd V(3, 3);
  V << 0.0, 0.0, 0.0,
       1.0, 0.0, 0.0,
       0.0, 1.0, 0.0;
  return V;
}

ES::MXi makeSingleTriangleFaces()
{
  ES::MXi F(1, 3);
  F << 0, 1, 2;
  return F;
}

ES::SpMatD makeIdentityMap(int n)
{
  std::vector<ES::TripletD> triplets;
  triplets.reserve(n);
  for (int i = 0; i < n; ++i)
    triplets.emplace_back(i, i, 1.0);
  ES::SpMatD W(n, n);
  W.setFromTriplets(triplets.begin(), triplets.end());
  return W;
}

pgo::Mesh::TriMeshGeo makeOpposingObstacleTriangle()
{
  std::vector<pgo::Vec3d> vertices = {
    pgo::Vec3d(0.0, 0.0, 0.0),
    pgo::Vec3d(1.0, 0.0, 0.0),
    pgo::Vec3d(0.0, 1.0, 0.0),
  };
  std::vector<pgo::Vec3i> triangles = {
    pgo::Vec3i(0, 2, 1),
  };
  return pgo::Mesh::TriMeshGeo(std::move(vertices), std::move(triangles));
}

pgo::Mesh::TriMeshGeo makeRaisedSurfaceTriangle()
{
  std::vector<pgo::Vec3d> vertices = {
    pgo::Vec3d(0.0, 0.0, 1.0),
    pgo::Vec3d(1.0, 0.0, 1.0),
    pgo::Vec3d(0.0, 1.0, 1.0),
  };
  std::vector<pgo::Vec3i> triangles = {
    pgo::Vec3i(0, 1, 2),
  };
  return pgo::Mesh::TriMeshGeo(std::move(vertices), std::move(triangles));
}

}  // namespace

TEST(SampledPenaltyContactEnergyGTest, SurfacePositionEnergyUsesAbsolutePositions)
{
  const ES::MXd V = makeSingleTriangleVertices();
  const ES::MXi F = makeSingleTriangleFaces();
  SP::SampledPenaltyContactEnergyOptions options;
  options.params.stiffness = 10.0;
  options.params.samples = 1;

  SP::SampledPenaltyContactEnergy energy(V, F, makeIdentityMap(9), options);

  const ES::VXd u = ES::VXd::Zero(9);
  EXPECT_EQ(energy.getNumDOFs(), u.size());
  EXPECT_EQ(energy.contactModelKind(), Contact::ContactModelKind::SampledPenalty);
  EXPECT_FALSE(energy.isStepDependent());
  EXPECT_NE(dynamic_cast<NO::StepAwareEnergy *>(&energy), nullptr);

  ES::VXd g = ES::VXd::Ones(u.size());
  ES::SpMatD H;
  EXPECT_DOUBLE_EQ(energy.func(u), 0.0);
  energy.gradient(u, g);
  energy.hessian(u, H);

  EXPECT_EQ(g.norm(), 0.0);
  EXPECT_EQ(H.rows(), u.size());
  EXPECT_EQ(H.cols(), u.size());
}

TEST(SampledPenaltyContactEnergyGTest, SurfacePositionEnergySupportsSubdividedSamples)
{
  const ES::MXd V = makeSingleTriangleVertices();
  const ES::MXi F = makeSingleTriangleFaces();
  SP::SampledPenaltyContactEnergyOptions options;
  options.params.stiffness = 10.0;
  options.params.samples = 2;

  SP::SampledPenaltyContactEnergy energy(V, F, makeIdentityMap(9), options);

  const ES::VXd u = ES::VXd::Zero(9);
  EXPECT_NO_THROW((void)energy.func(u));
}

TEST(SampledPenaltyContactEnergyGTest, ExternalContactSupportsVertexSamples)
{
  ES::MXd V = makeSingleTriangleVertices();
  V.col(2).array() += 0.1;
  const ES::MXi F = makeSingleTriangleFaces();
  SP::SampledPenaltyContactEnergyOptions options;
  options.params.stiffness = 10.0;
  options.params.samples = 1;
  options.params.enableSelfContact = false;
  options.params.enableExternalContact = true;

  std::vector<pgo::Mesh::TriMeshGeo> externalSurfaces;
  externalSurfaces.push_back(makeOpposingObstacleTriangle());
  SP::SampledPenaltyContactEnergy energy(
    V, F, makeIdentityMap(9), options, std::move(externalSurfaces));

  const ES::VXd u = ES::VXd::Zero(9);
  EXPECT_NO_THROW((void)energy.func(u));
}

TEST(SampledPenaltyContactEnergyGTest, BuilderTreatsInputAsAbsoluteSurfacePositions)
{
  SP::ParametersSpec params;
  params.stiffness = 10.0;
  params.samples = 1;
  params.enableSelfContact = false;
  params.enableExternalContact = true;

  std::vector<pgo::Mesh::TriMeshGeo> externalSurfaces;
  externalSurfaces.push_back(makeOpposingObstacleTriangle());

  SP::SampledPenaltyContactBuilder builder(
    makeRaisedSurfaceTriangle(),
    9,
    params,
    std::move(externalSurfaces),
    {},
    {});

  SP::SampledPenaltyEnergyConfigurator configurator;
  configurator.configureExternal = [](Contact::PointPenetrationEnergy &) {};
  configurator.configureSelf = [](Contact::PointTrianglePairCouplingEnergyWithCollision &, ES::ConstRefVecXd) {};

  ES::VXd absolutePositions(9);
  absolutePositions << 0.0, 0.0, -0.1,
                       1.0, 0.0, -0.1,
                       0.0, 1.0, -0.1;

  std::unique_ptr<SP::SampledPenaltyEvaluationBundle> bundle =
    builder.buildFromPositions(absolutePositions, configurator);
  EXPECT_EQ(bundle->externalEnergy, nullptr);
}

TEST(SampledPenaltyContactEnergyGTest, ContactBundleIsFrozenWithinStep)
{
  const ES::MXd V = makeSingleTriangleVertices();
  const ES::MXi F = makeSingleTriangleFaces();
  SP::SampledPenaltyContactEnergyOptions options;
  options.params.stiffness = 10.0;
  options.params.samples = 1;
  options.params.enableSelfContact = false;
  options.params.enableExternalContact = true;

  std::vector<pgo::Mesh::TriMeshGeo> externalSurfaces;
  externalSurfaces.push_back(makeOpposingObstacleTriangle());

  SP::SampledPenaltyContactEnergy energy(
    V, F, makeIdentityMap(9), options, std::move(externalSurfaces));

  ES::VXd start = ES::VXd::Zero(9);
  start[2] = start[5] = start[8] = -0.1;

  ES::VXd trial = ES::VXd::Zero(9);
  trial[2] = trial[5] = trial[8] = 0.1;

  SP::SampledPenaltyContactEnergy freshEnergy(
    V, F, makeIdentityMap(9), options, { makeOpposingObstacleTriangle() });
  EXPECT_GT(freshEnergy.func(trial), 0.0);

  NO::StepState state;
  state.currentX = &start;
  EXPECT_NO_THROW(energy.beginStep(state));
  EXPECT_DOUBLE_EQ(energy.func(trial), 0.0);
  ES::VXd gradient = ES::VXd::Ones(9);
  energy.gradient(trial, gradient);
  EXPECT_DOUBLE_EQ(gradient.norm(), 0.0);
  ES::SpMatD hessian;
  EXPECT_DOUBLE_EQ(energy.func_grad_hessian(trial, gradient, hessian), 0.0);
  EXPECT_DOUBLE_EQ(gradient.norm(), 0.0);

  state.currentX = &trial;
  EXPECT_NO_THROW(energy.beginStep(state));
  EXPECT_GT(energy.func(trial), 0.0);
}

TEST(SampledPenaltyContactEnergyGTest, FrictionalSurfacePositionEnergyReceivesPreviousAbsolutePositions)
{
  const ES::MXd V = makeSingleTriangleVertices();
  const ES::MXi F = makeSingleTriangleFaces();
  SP::SampledPenaltyContactEnergyOptions options;
  options.params.stiffness = 10.0;
  options.params.samples = 1;
  SP::FrictionParametersSpec friction;
  friction.frictionCoeff = 0.4;
  friction.velocityEps = 1e-5;
  options.friction = friction;

  SP::SampledPenaltyContactEnergy energy(V, F, makeIdentityMap(9), options);
  EXPECT_TRUE(energy.isStepDependent());
  EXPECT_NE(dynamic_cast<NO::StepAwareEnergy *>(&energy), nullptr);

  NO::StepState missingPrevious;
  missingPrevious.timestep = 0.1;
  EXPECT_THROW(energy.beginStep(missingPrevious), std::invalid_argument);

  ES::VXd previous = ES::VXd::Zero(9);
  NO::StepState nonPositiveTimestep;
  nonPositiveTimestep.previousX = &previous;
  nonPositiveTimestep.timestep = 0.0;
  EXPECT_THROW(energy.beginStep(nonPositiveTimestep), std::invalid_argument);

  NO::StepState valid;
  valid.previousX = &previous;
  valid.timestep = 0.1;
  EXPECT_NO_THROW(energy.beginStep(valid));
}
