/*
  RAII holder for sampled penalty active contact energies.
*/

#include "sampled_penalty/sampledPenaltyActiveSet.h"

#include "sampled_penalty/kernels/pointPenetrationEnergy.h"
#include "sampled_penalty/kernels/pointTrianglePairCouplingEnergyWithCollision.h"

namespace pgo
{
namespace Contact
{
namespace SampledPenalty
{

SampledPenaltyActiveSet::~SampledPenaltyActiveSet()
{
  clear();
}

bool SampledPenaltyActiveSet::empty() const
{
  return !externalEnergy && !selfEnergy;
}

void SampledPenaltyActiveSet::clear()
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
