/*
  Public construction facade for contact energies.
*/

#pragma once

#include "embeddedDofMap.h"
#include "statefulContactEnergy.h"

#include "EigenDef.h"
#include "potentialEnergy.h"

#include <memory>
#include <variant>
#include <vector>

namespace pgo
{
namespace Contact
{

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

struct IPCContactSpec
{
  double dhat = 1e-1;
  double dhatExternal = 1e-1;
  double kappa = 0.1;
  double epsEE = 0.0;
  double slackness = 1.0;
  double ccdThickness = 0.0;
};

struct SampledPenaltyContactSpec
{
  double stiffness = 1.0;
  int samples = 1;
  bool enableSelfContact = true;
  bool enableExternalContact = true;
};

struct FrictionContactSpec
{
  double frictionCoeff = 1.0;
  double velocityEps = 1.0;
};

std::shared_ptr<NonlinearOptimization::PotentialEnergy> createFloorEnergy(
  const ContactSurfaceSpec &surface,
  const FloorContactSpec &floor);

namespace IPC
{
std::shared_ptr<StatefulContactEnergy> createIPCEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const IPCContactSpec &params,
  std::vector<ObstacleSpec> obstacles = {});
}  // namespace IPC

namespace SampledPenalty
{
std::shared_ptr<StatefulContactEnergy> createSampledPenaltyEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const SampledPenaltyContactSpec &params);

std::shared_ptr<StatefulContactEnergy> createFrictionalSampledPenaltyEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const SampledPenaltyContactSpec &params,
  const FrictionContactSpec &friction);
}  // namespace SampledPenalty

}  // namespace Contact
}  // namespace pgo
