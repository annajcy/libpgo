#include "dynamicStepService.h"

#include <stdexcept>
#include <utility>

namespace pgo
{
namespace Simulation
{

std::unique_ptr<DynamicStepper> makeDynamicStepper(
  TimeIntegratorKind kind,
  DynamicProblem problem,
  double trbdf2Gamma)
{
  const int n = (int)problem.mass.rows();
  validateDynamicProblem(problem, n);

  switch (kind) {
  case TimeIntegratorKind::ImplicitEuler:
    return std::make_unique<ImplicitEulerStepper>(std::move(problem));
  case TimeIntegratorKind::TRBDF2:
    return std::make_unique<TRBDF2Stepper>(std::move(problem), trbdf2Gamma);
  }

  throw std::invalid_argument("makeDynamicStepper: unknown TimeIntegratorKind");
}

}  // namespace Simulation
}  // namespace pgo
