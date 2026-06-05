/*
  Active-set cache for sampled penalty contact evaluations.
*/

#include "sampled_penalty/sampledPenaltyActiveSetCache.h"

namespace pgo
{
namespace Contact
{
namespace SampledPenalty
{

bool SampledPenaltyActiveSetCache::sameState(EigenSupport::ConstRefVecXd a, EigenSupport::ConstRefVecXd b)
{
  return a.size() == b.size() && (a.array() == b.array()).all();
}

bool SampledPenaltyActiveSetCache::hasExactFor(EigenSupport::ConstRefVecXd x) const
{
  return exact_ && sameState(exactState_, x);
}

const SampledPenaltyActiveSet &SampledPenaltyActiveSetCache::prepareExact(
  EigenSupport::ConstRefVecXd x,
  const ActiveSetBuilder &build)
{
  exact_ = build(x);
  exactState_ = x;
  return *exact_;
}

const SampledPenaltyActiveSet &SampledPenaltyActiveSetCache::forEvaluation(
  EigenSupport::ConstRefVecXd x,
  const ActiveSetBuilder &build)
{
  if (lineSearch_)
    return *lineSearch_;
  if (!hasExactFor(x))
    return prepareExact(x, build);
  return *exact_;
}

const SampledPenaltyActiveSet &SampledPenaltyActiveSetCache::beginLineSearch(
  EigenSupport::ConstRefVecXd x,
  const ActiveSetBuilder &build)
{
  clearExact();
  lineSearch_ = build(x);
  return *lineSearch_;
}

void SampledPenaltyActiveSetCache::endLineSearch()
{
  lineSearch_.reset();
}

void SampledPenaltyActiveSetCache::clearExact()
{
  exact_.reset();
  exactState_.resize(0);
}

void SampledPenaltyActiveSetCache::clearAll()
{
  clearExact();
  endLineSearch();
}

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
