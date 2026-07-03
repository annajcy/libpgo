/*
  Optional lifecycle hook for energies with evaluation-time state.
*/

#pragma once

#include "EigenDef.h"

namespace pgo
{
namespace NonlinearOptimization
{

class EvaluationStateAwareEnergy
{
public:
  virtual ~EvaluationStateAwareEnergy() = default;

  // Called immediately before evaluating an energy at x when the evaluator owns
  // the normal active/cache state. Line-search scopes may provide a separate
  // frozen state and deliberately skip this hook for trial points.
  virtual void prepareEvaluationState(EigenSupport::ConstRefVecXd x) const = 0;
};

}  // namespace NonlinearOptimization
}  // namespace pgo
