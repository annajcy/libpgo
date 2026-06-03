/*
  Factory for DynamicStepper backends. See Task T7.5.
*/

#pragma once

#include "dynamicState.h"
#include "dynamicStepOptions.h"
#include "dynamicStepper.h"

#include <memory>

namespace pgo
{
namespace Simulation
{

std::unique_ptr<DynamicStepper> makeDynamicStepper(
  TimeIntegratorKind kind,
  DynamicProblem problem,
  double trbdf2Gamma = 0.5);

}  // namespace Simulation
}  // namespace pgo
