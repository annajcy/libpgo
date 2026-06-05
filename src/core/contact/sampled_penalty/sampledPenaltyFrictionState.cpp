/*
  Frictional step state for sampled penalty contact.
*/

#include "sampled_penalty/sampledPenaltyFrictionState.h"

#include "sampled_penalty/kernels/pointPenetrationEnergy.h"
#include "sampled_penalty/kernels/pointTrianglePairCouplingEnergyWithCollision.h"

#include <stdexcept>

namespace pgo
{
namespace Contact
{
namespace SampledPenalty
{

SampledPenaltyFrictionState::SampledPenaltyFrictionState(const FrictionParametersSpec &params):
  params_(params)
{
  if (params_.frictionCoeff < 0.0)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy requires non-negative friction coefficient.");
  if (params_.velocityEps <= 0.0)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy requires positive velocity epsilon.");
}

void SampledPenaltyFrictionState::beginStep(const NonlinearOptimization::StepState &state, int expectedDofs)
{
  if (state.previousX == nullptr)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy::beginStep requires previousX.");
  if (state.timestep <= 0.0)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy::beginStep requires a positive timestep.");
  if (state.previousX->size() != expectedDofs)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy::beginStep previousX has unexpected size.");

  previousX_ = *state.previousX;
  timestep_ = state.timestep;
  hasStepState_ = true;
}

void SampledPenaltyFrictionState::configureExternalSurfacePositions(PointPenetrationEnergy &energy) const
{
  if (!hasStepState_)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy requires beginStep before active contact evaluation.");

  energy.setComputeLastPosFunction([this](const EigenSupport::V3d &, EigenSupport::V3d &p, int dofStart) {
    p = previousX_.segment<3>(dofStart);
  });
  energy.setFrictionCoeff(params_.frictionCoeff);
  energy.setTimestep(timestep_);
  energy.setVelEps(params_.velocityEps);
}

void SampledPenaltyFrictionState::configureSelfSurfacePositions(PointTrianglePairCouplingEnergyWithCollision &energy) const
{
  if (!hasStepState_)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy requires beginStep before active contact evaluation.");

  energy.setToLastPosFunction([this](const EigenSupport::V3d &, EigenSupport::V3d &p, int dofStart) {
    p = previousX_.segment<3>(dofStart);
  });
  energy.setFrictionCoeff(params_.frictionCoeff);
  energy.setTimestep(timestep_);
  energy.setVelEps(params_.velocityEps);
}

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
