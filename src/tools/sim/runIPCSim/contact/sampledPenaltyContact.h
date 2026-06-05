#pragma once

#include "EigenSupport.h"
#include "contactEnergyFactory.h"
#include "contact/contactBackend.h"
#include "triMeshGeo.h"

#include <memory>
#include <vector>

namespace pgo
{
class ConfigFileJSON;
}

namespace pgo::RunIPCSim
{
struct SampledPenaltyContactConfig
{
  double stiffness = 0.0;
  int samples = 0;
  double frictionCoeff = 0.0;
  double velocityEps = 0.0;
};

SampledPenaltyContactConfig parseSampledPenaltyContactConfig(const pgo::ConfigFileJSON &config);

std::shared_ptr<RunIPCSimContactBackend> makeSampledPenaltyContactBackend(
  const pgo::ConfigFileJSON &config,
  const SampledPenaltyContactConfig &contactConfig,
  Contact::ContactSurfaceSpec surfaceSpec,
  EigenSupport::MXi surfaceTriangles,
  double scale);
}  // namespace pgo::RunIPCSim
