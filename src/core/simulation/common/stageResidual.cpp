#include "stageResidual.h"

#include <utility>

namespace pgo
{
namespace Simulation
{
namespace NO = pgo::NonlinearOptimization;

StageResidualHandle initStageResidual(
  const EigenSupport::SpMatD &A_initial,
  const EigenSupport::VXd &l_initial,
  const std::vector<ImplicitModelTerm> &terms)
{
  StageResidualHandle handle;
  handle.stageQuad = std::make_shared<PredefinedPotentialEnergies::QuadraticPotentialEnergy>(A_initial, l_initial);

  std::vector<NO::EnergySet::Term> setTerms;
  setTerms.reserve(terms.size() + 1);
  setTerms.push_back({handle.stageQuad, 1.0});
  for (const ImplicitModelTerm &term : terms)
    setTerms.push_back({term.energy, 1.0});

  handle.energySet = std::make_shared<NO::EnergySet>((int)A_initial.rows(), std::move(setTerms));
  return handle;
}

ImplicitStageProblem prepareStageResidual(
  StageResidualHandle &handle,
  EigenSupport::SpMatD A_s,
  EigenSupport::VXd l_s,
  EigenSupport::VXd initialGuess)
{
  // In-place update of the persistent quadratic term: no EnergySet rebuild,
  // Hessian template untouched.
  handle.stageQuad->setAValues(A_s);
  handle.stageQuad->setLinearTerm(l_s);

  ImplicitStageProblem problem;
  problem.energy = handle.energySet;
  problem.initialGuess = std::move(initialGuess);
  problem.A = std::move(A_s);
  problem.linear = std::move(l_s);
  return problem;
}

}  // namespace Simulation
}  // namespace pgo
