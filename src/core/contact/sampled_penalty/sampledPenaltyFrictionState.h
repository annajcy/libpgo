/*
  Frictional step state for sampled penalty contact.
*/

#pragma once

#include "EigenDef.h"
#include "sampled_penalty/sampledPenaltySpecs.h"
#include "stepAwareEnergy.h"

namespace pgo
{
namespace Contact
{
class PointPenetrationEnergy;
class PointTrianglePairCouplingEnergyWithCollision;

namespace SampledPenalty
{

class SampledPenaltyFrictionState
{
public:
  explicit SampledPenaltyFrictionState(const FrictionParametersSpec &params);

  void beginStep(const NonlinearOptimization::StepState &state, int expectedDofs);
  void configureExternalSurfacePositions(PointPenetrationEnergy &energy) const;
  void configureSelfSurfacePositions(PointTrianglePairCouplingEnergyWithCollision &energy) const;

private:
  FrictionParametersSpec params_;
  EigenSupport::VXd previousX_;
  double timestep_ = 0.0;
  bool hasStepState_ = false;
};

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
