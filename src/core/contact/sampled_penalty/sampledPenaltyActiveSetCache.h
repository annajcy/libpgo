/*
  Active-set cache for sampled penalty contact evaluations.
*/

#pragma once

#include "EigenDef.h"
#include "sampled_penalty/sampledPenaltyActiveSet.h"

#include <functional>
#include <memory>

namespace pgo
{
namespace Contact
{
namespace SampledPenalty
{

class SampledPenaltyActiveSetCache
{
public:
  using ActiveSetBuilder =
    std::function<std::unique_ptr<SampledPenaltyActiveSet>(EigenSupport::ConstRefVecXd)>;

  const SampledPenaltyActiveSet &prepareExact(
    EigenSupport::ConstRefVecXd x,
    const ActiveSetBuilder &build);
  const SampledPenaltyActiveSet &forEvaluation(
    EigenSupport::ConstRefVecXd x,
    const ActiveSetBuilder &build);
  const SampledPenaltyActiveSet &beginLineSearch(
    EigenSupport::ConstRefVecXd x,
    const ActiveSetBuilder &build);
  void endLineSearch();
  void clearExact();
  void clearAll();
  bool hasExactFor(EigenSupport::ConstRefVecXd x) const;

private:
  static bool sameState(EigenSupport::ConstRefVecXd a, EigenSupport::ConstRefVecXd b);

  std::unique_ptr<SampledPenaltyActiveSet> exact_;
  EigenSupport::VXd exactState_;
  std::unique_ptr<SampledPenaltyActiveSet> lineSearch_;
};

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
