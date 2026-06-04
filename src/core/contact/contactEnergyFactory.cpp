#include "contactEnergyFactory.h"

#include "embeddedSurfaceFloorPotentialEnergy.h"
#include "ipc/ipcContactEnergy.h"
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

IPC::FloorAxis convertFloorAxis(FloorAxis axis)
{
  switch (axis) {
    case FloorAxis::X:
      return IPC::FloorAxis::X;
    case FloorAxis::Y:
      return IPC::FloorAxis::Y;
    case FloorAxis::Z:
      return IPC::FloorAxis::Z;
    default:
      throw std::invalid_argument("FloorContactSpec.axis must be X, Y, or Z.");
  }
}

IPC::FloorSide convertFloorSide(FloorSide side)
{
  switch (side) {
    case FloorSide::KeepAbove:
      return IPC::FloorSide::KEEP_ABOVE;
    case FloorSide::KeepBelow:
      return IPC::FloorSide::KEEP_BELOW;
    default:
      throw std::invalid_argument("FloorContactSpec.side must be KeepAbove or KeepBelow.");
  }
}

EigenSupport::VXd flattenRestVertices(const EigenSupport::MXd &restVertices)
{
  EigenSupport::VXd rest(restVertices.rows() * 3);
  for (Eigen::Index vi = 0; vi < restVertices.rows(); ++vi)
    rest.segment<3>(vi * 3) = restVertices.row(vi).transpose();
  return rest;
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

}  // namespace

std::shared_ptr<NonlinearOptimization::PotentialEnergy> createFloorEnergy(
  const ContactSurfaceSpec &surface,
  const FloorContactSpec &floor)
{
  IPC::FloorPenaltyParameters params;
  params.floorAxis = convertFloorAxis(floor.axis);
  params.floorSide = convertFloorSide(floor.side);
  params.floorHeight = floor.height;
  params.floorKappa = floor.stiffness;

  return std::make_shared<IPC::EmbeddedSurfaceFloorPotentialEnergy>(
    surface.restVertices,
    surface.surfaceFromSimulationDispMap,
    params);
}

namespace IPC
{

std::shared_ptr<IPCContactEnergy> createIPCEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const ParametersSpec &params,
  std::vector<ObstacleSpec> obstacles)
{
  return std::make_shared<IPCContactEnergy>(
    surface.restVertices,
    surfaceTriangles,
    surface.surfaceFromSimulationDispMap,
    params,
    makeObstacleSurfaces(std::move(obstacles)));
}

}  // namespace IPC

namespace SampledPenalty
{

std::shared_ptr<SampledPenaltyContactEnergy> createSampledPenaltyEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const ParametersSpec &params)
{
  if (surface.surfaceFromSimulationDispMap.cols() != surface.restVertices.rows() * 3)
    throw std::invalid_argument("Sampled penalty factory currently requires an identity/surface-sized simulation map.");

  return std::make_shared<SampledPenaltyContactEnergy>(
    makeSurfaceMesh(surface, surfaceTriangles),
    flattenRestVertices(surface.restVertices),
    params);
}

std::shared_ptr<FrictionalSampledPenaltyContactEnergy> createFrictionalSampledPenaltyEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const ParametersSpec &params,
  const FrictionParametersSpec &friction)
{
  if (surface.surfaceFromSimulationDispMap.cols() != surface.restVertices.rows() * 3)
    throw std::invalid_argument("Sampled penalty factory currently requires an identity/surface-sized simulation map.");

  return std::make_shared<FrictionalSampledPenaltyContactEnergy>(
    makeSurfaceMesh(surface, surfaceTriangles),
    flattenRestVertices(surface.restVertices),
    params,
    friction);
}

}  // namespace SampledPenalty

}  // namespace Contact
}  // namespace pgo
