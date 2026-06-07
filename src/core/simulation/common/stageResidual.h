/*
  Stage residual for an implicit time-integration stage.

  Each stage minimizes  J_s(x) = ½xᵀA_s x + Φ(x) + l_sᵀx, expressed as a flat
  EnergySet { QuadraticEnergy(A_s, l_s), Φ-terms... }. The EnergySet (and its
  Hessian template) is built once via initStageResidual; per-step the quadratic
  term is updated in place via prepareStageResidual, so nothing is rebuilt and
  no per-frame allocation occurs (design decisions D2/D3). See Task T4.
*/

#pragma once

#include "dynamicStepOptions.h"
#include "energySet.h"
#include "quadraticPotentialEnergy.h"
#include "EigenDef.h"

#include <memory>
#include <vector>

namespace pgo
{
namespace Simulation
{

// Persistent per-stage objects, built once and held by the stepper.
struct StageResidualHandle
{
  std::shared_ptr<NonlinearOptimization::EnergySet> energySet;
  std::shared_ptr<PredefinedPotentialEnergies::QuadraticPotentialEnergy> stageQuad;
};

// Lightweight per-step description handed to the solver bridge. The energy is
// the same long-lived EnergySet held by the handle (kept owning so it can be
// wrapped in an OptimizationProblem). A / linear are kept for diagnostics and
// formula checks.
struct ImplicitStageProblem
{
  NonlinearOptimization::EnergySet_const_p energy;
  EigenSupport::VXd initialGuess;
  EigenSupport::SpMatD A;
  EigenSupport::VXd linear;
};

// Build the persistent EnergySet once. A_initial fixes the quadratic term's
// sparsity pattern (and thus the EnergySet Hessian template) for the lifetime
// of the stepper; l_initial is a placeholder overwritten each step.
StageResidualHandle initStageResidual(
  const EigenSupport::SpMatD &A_initial,
  const EigenSupport::VXd &l_initial,
  const std::vector<ImplicitModelTerm> &terms);

// Update the persistent quadratic term in place for this step. A_s must share
// A_initial's sparsity pattern (only values may change, e.g. variable timestep).
ImplicitStageProblem prepareStageResidual(
  StageResidualHandle &handle,
  EigenSupport::SpMatD A_s,
  EigenSupport::VXd l_s,
  EigenSupport::VXd initialGuess);

}  // namespace Simulation
}  // namespace pgo
