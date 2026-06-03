/*
  Shared step-lifecycle infrastructure for potential energies.

  StepAwareEnergy lets a long-lived energy observe the beginning of a dynamic
  time step (time, timestep, previous displacement). It is common optimization
  infrastructure, NOT contact-specific: the time integrator dispatches beginStep
  to every persistent energy, and contact energies (StatefulContactEnergy, added
  by contact_api_refactor.plan.md) derive from this to additionally manage their
  active set.

  See time_integrator_api_refactor.plan.md (B1) and contact_api_refactor.plan.md
  (cross-cutting decision 18.1).
*/

#pragma once

#include "EigenDef.h"

namespace pgo
{
namespace NonlinearOptimization
{

// Per-step state handed to StepAwareEnergy::beginStep at the start of a dynamic
// time step, before the stage residual is solved.
struct StepState
{
  double time = 0.0;
  double timestep = 0.0;
  const EigenSupport::VXd *previousX = nullptr;
};

class StepAwareEnergy
{
public:
  virtual ~StepAwareEnergy() = default;

  // Called once at the start of each dynamic time step.
  virtual void beginStep(const StepState &state) = 0;
};

}  // namespace NonlinearOptimization
}  // namespace pgo
