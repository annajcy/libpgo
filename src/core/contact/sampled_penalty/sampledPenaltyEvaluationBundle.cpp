/*
  RAII holder for sampled penalty evaluation child energies.
*/

#include "sampled_penalty/sampledPenaltyEvaluationBundle.h"

#include "sampled_penalty/kernels/pointPenetrationEnergy.h"
#include "sampled_penalty/kernels/pointTrianglePairCouplingEnergyWithCollision.h"

namespace pgo
{
namespace Contact
{
namespace SampledPenalty
{

SampledPenaltyEvaluationBundle::~SampledPenaltyEvaluationBundle()
{
  clear();
}

bool SampledPenaltyEvaluationBundle::empty() const
{
  return !externalEnergy && !selfEnergy;
}

void SampledPenaltyEvaluationBundle::clear()
{
  if (externalEnergy && externalBuffer) {
    externalEnergy->freeBuffer(externalBuffer);
    externalEnergy->setBuffer(nullptr);
  }
  externalBuffer = nullptr;
  externalEnergy.reset();

  if (selfEnergy && selfBuffer) {
    selfEnergy->freeBuffer(selfBuffer);
    selfEnergy->setBuffer(nullptr);
  }
  selfBuffer = nullptr;
  selfEnergy.reset();
}

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
