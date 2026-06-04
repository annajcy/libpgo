/*
  Public construction facade for contact energies.
*/

#pragma once

#include "embeddedDofMap.h"
#include "ipc/core/surfaceIPCCore.h"

#include "EigenDef.h"
#include "potentialEnergy.h"

#include <memory>
#include <variant>
#include <vector>

namespace pgo
{
namespace Contact
{
namespace IPC
{
class IPCContactEnergy;
using ParametersSpec = SurfaceIPCCore::Parameters;
}  // namespace IPC

namespace SampledPenalty
{
struct ParametersSpec;
struct FrictionParametersSpec;
class SampledPenaltyContactEnergy;
class FrictionalSampledPenaltyContactEnergy;
}  // namespace SampledPenalty

enum class FloorAxis
{
  X,
  Y,
  Z,
};

enum class FloorSide
{
  KeepAbove,
  KeepBelow,
};

struct ContactSurfaceSpec
{
  EigenSupport::MXd restVertices;
  EigenSupport::SpMatD surfaceFromSimulationDispMap;
};

struct FloorContactSpec
{
  FloorAxis axis = FloorAxis::Z;
  FloorSide side = FloorSide::KeepAbove;
  double height = 0.0;
  double stiffness = 1.0;
};

struct StaticObstacleSpec
{
  EigenSupport::MXd restVertices;
  EigenSupport::MXi triangles;
};

struct LinearMovingObstacleSpec
{
  EigenSupport::MXd restVertices;
  EigenSupport::MXi triangles;
  EigenSupport::V3d velocity = EigenSupport::V3d::Zero();
  double t0 = 0.0;
};

using ObstacleSpec = std::variant<StaticObstacleSpec, LinearMovingObstacleSpec>;

std::shared_ptr<NonlinearOptimization::PotentialEnergy> createFloorEnergy(
  const ContactSurfaceSpec &surface,
  const FloorContactSpec &floor);

namespace IPC
{
std::shared_ptr<IPCContactEnergy> createIPCEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const ParametersSpec &params,
  std::vector<ObstacleSpec> obstacles = {});
}  // namespace IPC

namespace SampledPenalty
{
std::shared_ptr<SampledPenaltyContactEnergy> createSampledPenaltyEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const ParametersSpec &params);

std::shared_ptr<FrictionalSampledPenaltyContactEnergy> createFrictionalSampledPenaltyEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const ParametersSpec &params,
  const FrictionParametersSpec &friction);
}  // namespace SampledPenalty

}  // namespace Contact
}  // namespace pgo
