/*
  Marker for energies whose evaluation depends on per-step history.
*/

#pragma once

namespace pgo
{
namespace NonlinearOptimization
{

class StepDependentEnergy
{
public:
  virtual ~StepDependentEnergy() = default;
};

}  // namespace NonlinearOptimization
}  // namespace pgo
