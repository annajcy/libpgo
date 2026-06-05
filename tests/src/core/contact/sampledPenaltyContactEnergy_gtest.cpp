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

TEST(SampledPenaltyContactEnergyGTest, SurfacePositionEnergyUsesAbsolutePositions)
{
  const pgo::Mesh::TriMeshGeo mesh = makeSingleTriangleMesh();
  SP::ParametersSpec params;
  params.stiffness = 10.0;
  params.samples = 1;

  SP::SampledPenaltySurfaceContactEnergy energy(mesh, params);

  const ES::VXd x = flattenMeshPositions(mesh);
  EXPECT_EQ(energy.getNumDOFs(), x.size());
  EXPECT_EQ(energy.contactModelKind(), Contact::ContactModelKind::SampledPenalty);

  ES::VXd g = ES::VXd::Ones(x.size());
  ES::SpMatD H;
  EXPECT_DOUBLE_EQ(energy.func(x), 0.0);
  energy.gradient(x, g);
  energy.hessian(x, H);

  EXPECT_EQ(g.norm(), 0.0);
  EXPECT_EQ(H.rows(), x.size());
  EXPECT_EQ(H.cols(), x.size());
}

TEST(SampledPenaltyContactEnergyGTest, SurfacePositionEnergySupportsSubdividedSamples)
{
  const pgo::Mesh::TriMeshGeo mesh = makeSingleTriangleMesh();
  SP::ParametersSpec params;
  params.stiffness = 10.0;
  params.samples = 2;

  SP::SampledPenaltySurfaceContactEnergy energy(mesh, params);

  const ES::VXd x = flattenMeshPositions(mesh);
  EXPECT_NO_THROW((void)energy.func(x));
}

TEST(SampledPenaltyContactEnergyGTest, FrictionalSurfacePositionEnergyReceivesPreviousAbsolutePositions)
{
  const pgo::Mesh::TriMeshGeo mesh = makeSingleTriangleMesh();
  SP::ParametersSpec params;
  params.stiffness = 10.0;
  params.samples = 1;
  SP::FrictionParametersSpec friction;
  friction.frictionCoeff = 0.4;
  friction.velocityEps = 1e-5;

  SP::FrictionalSampledPenaltySurfaceContactEnergy energy(mesh, params, friction);
  EXPECT_NE(dynamic_cast<NO::StepDependentEnergy *>(&energy), nullptr);

  NO::StepState missingPrevious;
  missingPrevious.timestep = 0.1;
  EXPECT_THROW(energy.beginStep(missingPrevious), std::invalid_argument);

  ES::VXd previous = flattenMeshPositions(mesh);
  NO::StepState nonPositiveTimestep;
  nonPositiveTimestep.previousX = &previous;
  nonPositiveTimestep.timestep = 0.0;
  EXPECT_THROW(energy.beginStep(nonPositiveTimestep), std::invalid_argument);

  NO::StepState valid;
  valid.previousX = &previous;
  valid.timestep = 0.1;
  EXPECT_NO_THROW(energy.beginStep(valid));
}
