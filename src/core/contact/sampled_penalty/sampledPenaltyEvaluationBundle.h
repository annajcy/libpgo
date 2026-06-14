/*
  RAII holder for sampled penalty evaluation child energies.
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

struct SampledPenaltyEvaluationBundle
{
  std::shared_ptr<PointPenetrationEnergy> externalEnergy;
  PointPenetrationEnergyBuffer *externalBuffer = nullptr;
  std::shared_ptr<PointTrianglePairCouplingEnergyWithCollision> selfEnergy;
  PointTrianglePairCouplingEnergyWithCollisionBuffer *selfBuffer = nullptr;

  SampledPenaltyEvaluationBundle() = default;
  SampledPenaltyEvaluationBundle(const SampledPenaltyEvaluationBundle &) = delete;
  SampledPenaltyEvaluationBundle &operator=(const SampledPenaltyEvaluationBundle &) = delete;
  SampledPenaltyEvaluationBundle(SampledPenaltyEvaluationBundle &&) = delete;
  SampledPenaltyEvaluationBundle &operator=(SampledPenaltyEvaluationBundle &&) = delete;
  ~SampledPenaltyEvaluationBundle();

  bool empty() const;
  void clear();
};

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
