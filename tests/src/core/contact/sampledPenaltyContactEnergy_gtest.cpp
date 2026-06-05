#include <gtest/gtest.h>

#include "sampled_penalty/sampledPenaltyContactEnergy.h"
#include "stepDependentEnergy.h"
#include "testCIPCHelpers.h"
#include "triMeshGeo.h"

#include <stdexcept>

namespace
{
namespace ES = pgo::EigenSupport;
namespace Contact = pgo::Contact;
namespace SP = pgo::Contact::SampledPenalty;
namespace NO = pgo::NonlinearOptimization;

pgo::Mesh::TriMeshGeo makeSingleTriangleMesh()
{
  std::vector<pgo::Vec3d> vertices = {
    { 0.0, 0.0, 0.0 },
    { 1.0, 0.0, 0.0 },
    { 0.0, 1.0, 0.0 },
  };
  std::vector<pgo::Vec3i> triangles = {
    { 0, 1, 2 },
  };
  return pgo::Mesh::TriMeshGeo(std::move(vertices), std::move(triangles));
}

ES::VXd flattenMeshPositions(const pgo::Mesh::TriMeshGeo &mesh)
{
  ES::VXd x(mesh.numVertices() * 3);
  for (int vi = 0; vi < mesh.numVertices(); vi++)
    x.segment<3>(vi * 3) = mesh.pos(vi);
  return x;
}
}  // namespace

TEST(SampledPenaltyContactEnergyGTest, NormalEnergyIsStatefulDisplacementContactButNotStepDependent)
{
  const pgo::Mesh::TriMeshGeo mesh = makeSingleTriangleMesh();
  const ES::VXd rest = flattenMeshPositions(mesh);
  SP::ParametersSpec params;
  params.stiffness = 10.0;
  params.samples = 1;

  SP::SampledPenaltyContactEnergy energy(mesh, rest, params);

  EXPECT_EQ(energy.contactModelKind(), Contact::ContactModelKind::SampledPenalty);
  EXPECT_EQ(energy.stateKind(), NO::EnergyStateKind::Displacement);
  EXPECT_EQ(dynamic_cast<NO::StepDependentEnergy *>(&energy), nullptr);
  EXPECT_EQ(energy.isHessianTopologyFixed(), 0);

  NO::StepState state;
  state.time = 2.0;
  state.timestep = 0.25;

  const ES::VXd u = ES::VXd::Zero(rest.size());

  ES::VXd g = ES::VXd::Ones(rest.size());
  ES::SpMatD H;
  EXPECT_DOUBLE_EQ(energy.func(u), 0.0);
  energy.gradient(u, g);
  energy.hessian(u, H);

  EXPECT_EQ(g.norm(), 0.0);
  EXPECT_EQ(H.rows(), rest.size());
  EXPECT_EQ(H.cols(), rest.size());
}

TEST(SampledPenaltyContactEnergyGTest, FrictionalEnergyRequiresPreviousStateAndPositiveTimestep)
{
  const pgo::Mesh::TriMeshGeo mesh = makeSingleTriangleMesh();
  const ES::VXd rest = flattenMeshPositions(mesh);
  SP::ParametersSpec params;
  params.stiffness = 10.0;
  params.samples = 1;
  SP::FrictionParametersSpec friction;
  friction.frictionCoeff = 0.4;
  friction.velocityEps = 1e-5;

  SP::FrictionalSampledPenaltyContactEnergy energy(mesh, rest, params, friction);
  EXPECT_NE(dynamic_cast<NO::StepDependentEnergy *>(&energy), nullptr);

  NO::StepState missingPrevious;
  missingPrevious.timestep = 0.1;
  EXPECT_THROW(energy.beginStep(missingPrevious), std::invalid_argument);

  ES::VXd previous = ES::VXd::Zero(rest.size());
  NO::StepState nonPositiveTimestep;
  nonPositiveTimestep.previousX = &previous;
  nonPositiveTimestep.timestep = 0.0;
  EXPECT_THROW(energy.beginStep(nonPositiveTimestep), std::invalid_argument);

  NO::StepState valid;
  valid.previousX = &previous;
  valid.timestep = 0.1;
  EXPECT_NO_THROW(energy.beginStep(valid));
}
