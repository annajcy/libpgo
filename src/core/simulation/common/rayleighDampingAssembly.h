/*
  Rayleigh damping assembly for the dynamic time-stepping service.

  D_n = (Σ_i massDamping_i) M + Σ_i stiffnessDamping_i ∇²Φ_i(u_n)

  All terms share the global mass for mass damping, fixed-topology terms
  contribute a stiffness Hessian evaluated at the current state, and
  non-fixed-topology terms (e.g. IPC contact) are skipped for stiffness damping.
  See Task T3.
*/

#pragma once

#include "dynamicStepOptions.h"
#include "EigenDef.h"

#include <vector>

namespace pgo
{
namespace Simulation
{

EigenSupport::SpMatD assembleRayleighDamping(
  const std::vector<ImplicitModelTerm> &terms,
  const EigenSupport::SpMatD &mass,
  EigenSupport::ConstRefVecXd state);

}  // namespace Simulation
}  // namespace pgo
