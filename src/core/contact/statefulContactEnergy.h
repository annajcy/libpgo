/*
  Common long-lived contact energy boundary.
*/

#pragma once

#include "evaluationStateAwareEnergy.h"
#include "lineSearchAwareEnergy.h"
#include "potentialEnergy.h"
#include "stepAwareEnergy.h"

namespace pgo
{
namespace Contact
{

enum class ContactModelKind
{
  Floor,
  IPC,
  SampledPenalty,
};

class StatefulContactEnergy:
  public NonlinearOptimization::PotentialEnergy,
  public NonlinearOptimization::LineSearchAwareEnergy,
  public NonlinearOptimization::StepAwareEnergy,
  public NonlinearOptimization::EvaluationStateAwareEnergy
{
public:
  virtual ~StatefulContactEnergy() = default;

  virtual ContactModelKind contactModelKind() const = 0;
  virtual void beginStep(const NonlinearOptimization::StepState &) override { clearActiveSet(); }
  virtual void refreshActiveSet(EigenSupport::ConstRefVecXd) const {}
  virtual void clearActiveSet() const {}

  virtual NonlinearOptimization::EnergyStateKind stateKind() const override
  {
    return NonlinearOptimization::EnergyStateKind::Displacement;
  }

  virtual void prepareEvaluationState(EigenSupport::ConstRefVecXd x) const override
  {
    refreshActiveSet(x);
  }

  virtual void beginLineSearch(EigenSupport::ConstRefVecXd, EigenSupport::ConstRefVecXd) const override {}
  virtual void endLineSearch() const override {}
};

}  // namespace Contact
}  // namespace pgo
