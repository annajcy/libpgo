#include "contactEnergyFactory.h"

#include "floor/floorContactEnergy.h"
#include "ipc/ipcContactEnergy.h"
#include "ipc/core/surfaceIPCCore.h"
#include "mappedContactEnergy.h"
#include "sampled_penalty/sampledPenaltyContactEnergy.h"
#include "triMeshGeo.h"

#include <stdexcept>
#include <memory>
#include <type_traits>
#include <utility>

namespace pgo
{
namespace Contact
{
namespace
{

Floor::FloorAxis convertFloorAxis(FloorAxis axis)
{
  switch (axis) {
    case FloorAxis::X:
      return Floor::FloorAxis::X;
    case FloorAxis::Y:
      return Floor::FloorAxis::Y;
    case FloorAxis::Z:
      return Floor::FloorAxis::Z;
    default:
      throw std::invalid_argument("FloorContactSpec.axis must be X, Y, or Z.");
  }
}

Floor::FloorSide convertFloorSide(FloorSide side)
{
  switch (side) {
    case FloorSide::KeepAbove:
      return Floor::FloorSide::KEEP_ABOVE;
    case FloorSide::KeepBelow:
      return Floor::FloorSide::KEEP_BELOW;
    default:
      throw std::invalid_argument("FloorContactSpec.side must be KeepAbove or KeepBelow.");
  }
}

Mesh::TriMeshGeo makeSurfaceMesh(const ContactSurfaceSpec &surface, const EigenSupport::MXi &surfaceTriangles)
{
  if (surface.restVertices.cols() != 3)
    throw std::invalid_argument("ContactSurfaceSpec.restVertices must have shape (#vertices, 3).");
  if (surfaceTriangles.cols() != 3)
    throw std::invalid_argument("surfaceTriangles must have shape (#triangles, 3).");
  return Mesh::matricesToTriMeshGeo(surface.restVertices, surfaceTriangles);
}

void validateObstacleMesh(const EigenSupport::MXd &restVertices, const EigenSupport::MXi &triangles)
{
  if (restVertices.cols() != 3)
    throw std::invalid_argument("ObstacleSpec.restVertices must have shape (#vertices, 3).");
  if (triangles.cols() != 3)
    throw std::invalid_argument("ObstacleSpec.triangles must have shape (#triangles, 3).");
  if (triangles.size() > 0 && (triangles.minCoeff() < 0 || triangles.maxCoeff() >= restVertices.rows()))
    throw std::invalid_argument("ObstacleSpec.triangles contains an out-of-range vertex index.");
}

std::unique_ptr<IPC::ObstacleSurface> makeStaticObstacleSurface(const StaticObstacleSpec &spec)
{
  validateObstacleMesh(spec.restVertices, spec.triangles);
  return std::make_unique<IPC::StaticObstacleSurface>(spec.restVertices, spec.triangles);
}

std::unique_ptr<IPC::ObstacleSurface> makeLinearMovingObstacleSurface(const LinearMovingObstacleSpec &spec)
{
  validateObstacleMesh(spec.restVertices, spec.triangles);
  return std::make_unique<IPC::LinearMovingObstacleSurface>(
    spec.restVertices,
    spec.triangles,
    spec.velocity,
    spec.t0);
}

std::vector<std::unique_ptr<IPC::ObstacleSurface>> makeObstacleSurfaces(std::vector<ObstacleSpec> obstacleSpecs)
{
  std::vector<std::unique_ptr<IPC::ObstacleSurface>> obstacles;
  obstacles.reserve(obstacleSpecs.size());
  for (const ObstacleSpec &spec : obstacleSpecs) {
    obstacles.push_back(std::visit(
      [](const auto &typedSpec) {
        using Spec = std::decay_t<decltype(typedSpec)>;
        if constexpr (std::is_same_v<Spec, StaticObstacleSpec>) {
          return makeStaticObstacleSurface(typedSpec);
        }
        else {
          return makeLinearMovingObstacleSurface(typedSpec);
        }
      },
      spec));
  }
  return obstacles;
}

IPC::SurfaceIPCCore::Parameters toIPCParameters(const IPCContactSpec &spec)
{
  IPC::SurfaceIPCCore::Parameters params;
  params.dhat = spec.dhat;
  params.dhat_external = spec.dhatExternal;
  params.kappa = spec.kappa;
  params.eps_ee = spec.epsEE;
  params.slackness = spec.slackness;
  params.ccd_thickness = spec.ccdThickness;
  return params;
}

SampledPenalty::ParametersSpec toSampledPenaltyParameters(const SampledPenaltyContactSpec &spec)
{
  SampledPenalty::ParametersSpec params;
  params.stiffness = spec.stiffness;
  params.samples = spec.samples;
  params.enableSelfContact = spec.enableSelfContact;
  params.enableExternalContact = spec.enableExternalContact;
  return params;
}

SampledPenalty::FrictionParametersSpec toFrictionParameters(const FrictionContactSpec &spec)
{
  SampledPenalty::FrictionParametersSpec friction;
  friction.frictionCoeff = spec.frictionCoeff;
  friction.velocityEps = spec.velocityEps;
  return friction;
}

}  // namespace

std::shared_ptr<NonlinearOptimization::PotentialEnergy> createFloorEnergy(
  const ContactSurfaceSpec &surface,
  const FloorContactSpec &floor)
{
  Floor::FloorPenaltyParameters params;
  params.floorAxis = convertFloorAxis(floor.axis);
  params.floorSide = convertFloorSide(floor.side);
  params.floorHeight = floor.height;
  params.floorKappa = floor.stiffness;

  return std::make_shared<Floor::FloorContactEnergy>(
    surface.restVertices,
    surface.surfaceFromSimulationDispMap,
    params);
}

namespace IPC
{

std::shared_ptr<StatefulContactEnergy> createIPCEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const IPCContactSpec &params,
  std::vector<ObstacleSpec> obstacles)
{
  return std::make_shared<IPCContactEnergy>(
    surface.restVertices,
    surfaceTriangles,
    surface.surfaceFromSimulationDispMap,
    toIPCParameters(params),
    makeObstacleSurfaces(std::move(obstacles)));
}

}  // namespace IPC

namespace SampledPenalty
{

std::shared_ptr<StatefulContactEnergy> createSampledPenaltyEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const SampledPenaltyContactSpec &params,
  std::vector<Mesh::TriMeshGeo> externalSurfaces)
{
  auto surfaceEnergy = std::make_shared<SampledPenaltySurfaceContactEnergy>(
    makeSurfaceMesh(surface, surfaceTriangles),
    toSampledPenaltyParameters(params),
    std::move(externalSurfaces));

  return makeMappedContactEnergy(
    surface.restVertices,
    surface.surfaceFromSimulationDispMap,
    std::move(surfaceEnergy));
}

std::shared_ptr<StatefulContactEnergy> createFrictionalSampledPenaltyEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const SampledPenaltyContactSpec &params,
  const FrictionContactSpec &friction,
  std::vector<Mesh::TriMeshGeo> externalSurfaces)
{
  auto surfaceEnergy = std::make_shared<FrictionalSampledPenaltySurfaceContactEnergy>(
    makeSurfaceMesh(surface, surfaceTriangles),
    toSampledPenaltyParameters(params),
    toFrictionParameters(friction),
    std::move(externalSurfaces));

  return makeMappedContactEnergy(
    surface.restVertices,
    surface.surfaceFromSimulationDispMap,
    std::move(surfaceEnergy));
}

}  // namespace SampledPenalty

}  // namespace Contact
}  // namespace pgo
