/*
  Frictional step state for sampled penalty contact.
*/

#include "sampled_penalty/sampledPenaltyFrictionState.h"

#include "sampled_penalty/kernels/pointPenetrationEnergy.h"
#include "sampled_penalty/kernels/pointTrianglePairCouplingEnergyWithCollision.h"

#include <stdexcept>
#include <utility>

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

void SampledPenaltyFrictionState::configureExternal(
  PointPenetrationEnergy &energy,
  EigenSupport::ConstRefVecXd restPositions) const
{
  if (!hasStepState_)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy requires beginStep before active contact evaluation.");

  EigenSupport::VXd restCopy = restPositions;
  energy.setComputeLastPosFunction([this, rest = std::move(restCopy)](const EigenSupport::V3d &, EigenSupport::V3d &p, int dofStart) {
    p = previousX_.segment<3>(dofStart) + rest.segment<3>(dofStart);
  });
  energy.setFrictionCoeff(params_.frictionCoeff);
  energy.setTimestep(timestep_);
  energy.setVelEps(params_.velocityEps);
}

void SampledPenaltyFrictionState::configureSelf(
  PointTrianglePairCouplingEnergyWithCollision &energy,
  EigenSupport::ConstRefVecXd restPositions) const
{
  if (!hasStepState_)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy requires beginStep before active contact evaluation.");

  EigenSupport::VXd restCopy = restPositions;
  energy.setToLastPosFunction([this, rest = std::move(restCopy)](const EigenSupport::V3d &, EigenSupport::V3d &p, int dofStart) {
    p = previousX_.segment<3>(dofStart) + rest.segment<3>(dofStart);
  });
  energy.setFrictionCoeff(params_.frictionCoeff);
  energy.setTimestep(timestep_);
  energy.setVelEps(params_.velocityEps);
}

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
