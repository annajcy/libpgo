/*
  RAII holder for sampled penalty active contact energies.
*/

#pragma once

#include <memory>

namespace pgo
{
namespace Contact
{
class PointPenetrationEnergy;
class PointTrianglePairCouplingEnergyWithCollision;
struct PointPenetrationEnergyBuffer;
class PointTrianglePairCouplingEnergyWithCollisionBuffer;

namespace SampledPenalty
{

struct SampledPenaltyActiveSet
{
  std::shared_ptr<PointPenetrationEnergy> externalEnergy;
  PointPenetrationEnergyBuffer *externalBuffer = nullptr;
  std::shared_ptr<PointTrianglePairCouplingEnergyWithCollision> selfEnergy;
  PointTrianglePairCouplingEnergyWithCollisionBuffer *selfBuffer = nullptr;

  SampledPenaltyActiveSet() = default;
  SampledPenaltyActiveSet(const SampledPenaltyActiveSet &) = delete;
  SampledPenaltyActiveSet &operator=(const SampledPenaltyActiveSet &) = delete;
  SampledPenaltyActiveSet(SampledPenaltyActiveSet &&) noexcept = default;
  SampledPenaltyActiveSet &operator=(SampledPenaltyActiveSet &&) noexcept = default;
  ~SampledPenaltyActiveSet();

  bool empty() const;
  void clear();
};

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
