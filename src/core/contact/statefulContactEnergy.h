/*
  Common long-lived contact energy boundary.
*/

#pragma once

#include "evaluationStateAwareEnergy.h"
#include "lineSearchAwareEnergy.h"
#include "potentialEnergy.h"

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
  public NonlinearOptimization::PotentialEnergy
{
public:
  virtual ~StatefulContactEnergy() = default;

  virtual ContactModelKind contactModelKind() const = 0;

  virtual NonlinearOptimization::EnergyStateKind stateKind() const override
  {
    return NonlinearOptimization::EnergyStateKind::Displacement;
  }
};

class ActiveSetContactEnergy:
  public NonlinearOptimization::LineSearchAwareEnergy,
  public NonlinearOptimization::EvaluationStateAwareEnergy
{
public:
  virtual ~ActiveSetContactEnergy() = default;

  virtual void prepareEvaluationState(EigenSupport::ConstRefVecXd x) const override
  {
    prepareActiveSet(x);
  }

  virtual void beginLineSearch(
    EigenSupport::ConstRefVecXd x,
    EigenSupport::ConstRefVecXd dx) const override
  {
    clearPreparedActiveSet();
    beginActiveSetLineSearch(x, dx);
  }

  virtual void endLineSearch() const override
  {
    endActiveSetLineSearch();
  }

protected:
  virtual void prepareActiveSet(EigenSupport::ConstRefVecXd x) const = 0;
  virtual void clearPreparedActiveSet() const = 0;
  virtual void beginActiveSetLineSearch(
    EigenSupport::ConstRefVecXd x,
    EigenSupport::ConstRefVecXd dx) const = 0;
  virtual void endActiveSetLineSearch() const = 0;
};

}  // namespace Contact
}  // namespace pgo
