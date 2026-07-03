/*
  Immutable problem description (DynamicProblem) and per-step input
  (DynamicStepRequest) for the dynamic time-stepping service.

  The split keeps invariants (mass, persistent energies, per-term damping, and
  fixed-DOF index set) out of the per-step hot path so they are never re-copied
  each frame. Solver/backend configuration lives on the optimizer object passed
  to DynamicStepper::step().
*/

#pragma once

#include "dynamicState.h"
#include "energy/potentialEnergy.h"

#include <optional>
#include <vector>

namespace pgo
{
namespace Simulation
{

// One persistent energy term (elastic / attachments / contact ...) plus its
// per-term Rayleigh damping coefficients. Damping is metadata about how the
// term contributes to D_n, not a property of the energy itself.
struct ImplicitModelTerm
{
  NonlinearOptimization::PotentialEnergy_p energy;
  double stiffnessDamping = 0.0;
  double massDamping = 0.0;
};

// Immutable across steps. Construct a stepper once from this.
struct DynamicProblem
{
  EigenSupport::SpMatD mass;
  std::vector<ImplicitModelTerm> persistentTerms;  // elastic / attachments / contact
  std::vector<int> fixedDofs;                       // immutable across steps (D5)
  double timestep = 0.0;
};

// Per-step. Contact energies are already held persistently in
// DynamicProblem.persistentTerms (D1), so there is no transientTerms here.
struct DynamicStepRequest
{
  EigenSupport::VXd externalForce;
  // Prescribed values on fixedDofs; defaults to the current state's values.
  std::optional<EigenSupport::VXd> fixedValues;
};

void validateDynamicProblem(const DynamicProblem &problem, int numDofs);
void validateDynamicStepRequest(const DynamicStepRequest &request, const DynamicProblem &problem, int numDofs);

}  // namespace Simulation
}  // namespace pgo
