#pragma once

#include "energy/potentialEnergy.h"
#include "energy/stepAwareEnergy.h"

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
  public NonlinearOptimization::StepAwareEnergy
{
public:
  ~StatefulContactEnergy() override = default;

  virtual ContactModelKind contactModelKind() const = 0;
  virtual bool isStepDependent() const { return false; }

  virtual void beginStep(const NonlinearOptimization::StepState &) override {}

  NonlinearOptimization::EnergyStateKind stateKind() const override
  {
    return NonlinearOptimization::EnergyStateKind::Displacement;
  }
};

}  // namespace Contact
}  // namespace pgo
