/*
  Active-set cache for IPC contact evaluations.
*/

#include "ipc/ipcActiveSetCache.h"

#include <utility>

namespace pgo
{
namespace Contact
{
namespace IPC
{

bool IPCActiveSetCache::samePositions(EigenSupport::ConstRefVecXd a, EigenSupport::ConstRefVecXd b)
{
  return a.size() == b.size() && (a.array() == b.array()).all();
}

bool IPCActiveSetCache::hasExactFor(EigenSupport::ConstRefVecXd surfacePositions) const
{
  return hasExact_ && samePositions(exact_.positions, surfacePositions);
}

const SurfaceIPCActiveSet &IPCActiveSetCache::prepareExact(
  EigenSupport::ConstRefVecXd surfacePositions,
  const ActiveSetBuilder &build)
{
  exact_ = build(surfacePositions);
  hasExact_ = true;
  return exact_;
}

const SurfaceIPCActiveSet &IPCActiveSetCache::forEvaluation(
  EigenSupport::ConstRefVecXd surfacePositions,
  const ActiveSetBuilder &build)
{
  if (hasLineSearch_) {
    lineSearch_.positions = surfacePositions;
    return lineSearch_;
  }

  if (!hasExactFor(surfacePositions))
    return prepareExact(surfacePositions, build);

  return exact_;
}

void IPCActiveSetCache::beginLineSearch(SurfaceIPCActiveSet activeSet)
{
  clearExact();
  lineSearch_ = std::move(activeSet);
  hasLineSearch_ = true;
}

void IPCActiveSetCache::endLineSearch()
{
  lineSearch_.clear();
  hasLineSearch_ = false;
}

void IPCActiveSetCache::clearExact()
{
  exact_.clear();
  hasExact_ = false;
}

void IPCActiveSetCache::clearAll()
{
  clearExact();
  endLineSearch();
}

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
