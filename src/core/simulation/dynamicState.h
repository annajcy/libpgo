/*
  Value objects for the dynamic time-stepping service.

  DynamicState is the per-frame kinematic state (displacement / velocity /
  acceleration) carried across time steps. See
  time_integrator_api_refactor.plan.md Task T1.
*/

#pragma once

#include "EigenDef.h"

#include <cstdint>

namespace pgo
{
namespace Simulation
{

enum class DynamicStepperKind
{
  BackwardEuler,
  TRBDF2,
};

struct DynamicState
{
  EigenSupport::VXd displacement;
  EigenSupport::VXd velocity;
  EigenSupport::VXd acceleration;
  std::uint64_t timestepId = 0;
  double time = 0.0;  // simulation time, feeds StepAwareEnergy::beginStep
};

// Throws std::invalid_argument if the state is inconsistent with numDofs or
// contains non-finite values.
void validateDynamicState(const DynamicState &state, int numDofs);

}  // namespace Simulation
}  // namespace pgo
