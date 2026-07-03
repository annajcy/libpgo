#include "dynamicStepOptions.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace pgo
{
namespace Simulation
{
namespace
{
void requireFinite(const EigenSupport::VXd &v, const char *name)
{
  if (!v.allFinite())
    throw std::invalid_argument(std::string("DynamicState/Request: ") + name + " contains non-finite values");
}
}  // namespace

void validateDynamicState(const DynamicState &state, int numDofs)
{
  if (numDofs <= 0)
    throw std::invalid_argument("validateDynamicState: numDofs must be positive");

  if ((int)state.displacement.size() != numDofs)
    throw std::invalid_argument("validateDynamicState: displacement size mismatch");
  if ((int)state.velocity.size() != numDofs)
    throw std::invalid_argument("validateDynamicState: velocity size mismatch");
  if ((int)state.acceleration.size() != numDofs)
    throw std::invalid_argument("validateDynamicState: acceleration size mismatch");

  requireFinite(state.displacement, "displacement");
  requireFinite(state.velocity, "velocity");
  requireFinite(state.acceleration, "acceleration");

  if (!std::isfinite(state.time))
    throw std::invalid_argument("validateDynamicState: time is not finite");
}

void validateDynamicProblem(const DynamicProblem &problem, int numDofs)
{
  if (numDofs <= 0)
    throw std::invalid_argument("validateDynamicProblem: numDofs must be positive");

  if (problem.mass.rows() != numDofs || problem.mass.cols() != numDofs)
    throw std::invalid_argument("validateDynamicProblem: mass must be numDofs x numDofs");

  if (!(problem.timestep > 0.0) || !std::isfinite(problem.timestep))
    throw std::invalid_argument("validateDynamicProblem: timestep must be positive and finite");

  for (const ImplicitModelTerm &term : problem.persistentTerms) {
    if (!term.energy)
      throw std::invalid_argument("validateDynamicProblem: persistent term has null energy");
    if (!std::isfinite(term.stiffnessDamping) || term.stiffnessDamping < 0.0)
      throw std::invalid_argument("validateDynamicProblem: stiffnessDamping must be finite and non-negative");
    if (!std::isfinite(term.massDamping) || term.massDamping < 0.0)
      throw std::invalid_argument("validateDynamicProblem: massDamping must be finite and non-negative");
  }

  // fixedDofs must be in-range and free of duplicates (caller may rely on canonical order).
  std::vector<int> sorted = problem.fixedDofs;
  std::sort(sorted.begin(), sorted.end());
  for (size_t i = 0; i < sorted.size(); i++) {
    if (sorted[i] < 0 || sorted[i] >= numDofs)
      throw std::invalid_argument("validateDynamicProblem: fixedDofs index out of range");
    if (i > 0 && sorted[i] == sorted[i - 1])
      throw std::invalid_argument("validateDynamicProblem: duplicate fixedDofs index");
  }
}

void validateDynamicStepRequest(const DynamicStepRequest &request, const DynamicProblem &problem, int numDofs)
{
  if ((int)request.externalForce.size() != numDofs)
    throw std::invalid_argument("validateDynamicStepRequest: externalForce size mismatch");
  requireFinite(request.externalForce, "externalForce");

  if (request.fixedValues) {
    if ((int)request.fixedValues->size() != (int)problem.fixedDofs.size())
      throw std::invalid_argument("validateDynamicStepRequest: fixedValues size must match fixedDofs");
    requireFinite(*request.fixedValues, "fixedValues");
  }
}

}  // namespace Simulation
}  // namespace pgo
