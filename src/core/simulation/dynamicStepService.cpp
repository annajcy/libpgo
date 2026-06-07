#include "dynamicStepService.h"

#include <stdexcept>
#include <utility>

namespace pgo
{
namespace Simulation
{

std::unique_ptr<DynamicStepper> makeDynamicStepper(
  DynamicStepperKind kind,
  DynamicProblem problem,
  double trbdf2Gamma)
{
  const int n = (int)problem.mass.rows();
  validateDynamicProblem(problem, n);

  switch (kind) {
  case DynamicStepperKind::ImplicitEuler:
    return std::make_unique<ImplicitEulerStepper>(std::move(problem));
  case DynamicStepperKind::TRBDF2:
    return std::make_unique<TRBDF2Stepper>(std::move(problem), trbdf2Gamma);
  }

  throw std::invalid_argument("makeDynamicStepper: unknown DynamicStepperKind");
}

}  // namespace Simulation
}  // namespace pgo
