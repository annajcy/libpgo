/*
  Active-set cache for IPC contact evaluations.
*/

#pragma once

#include "EigenDef.h"
#include "ipc/core/surfaceIPCActiveSet.h"

#include <functional>

namespace pgo
{
namespace Contact
{
namespace IPC
{

class IPCActiveSetCache
{
public:
  using ActiveSetBuilder = std::function<SurfaceIPCActiveSet(EigenSupport::ConstRefVecXd)>;

  const SurfaceIPCActiveSet &prepareExact(
    EigenSupport::ConstRefVecXd surfacePositions,
    const ActiveSetBuilder &build);
  const SurfaceIPCActiveSet &forEvaluation(
    EigenSupport::ConstRefVecXd surfacePositions,
    const ActiveSetBuilder &build);

  void beginLineSearch(SurfaceIPCActiveSet activeSet);
  void endLineSearch();
  void clearExact();
  void clearAll();
  bool hasExactFor(EigenSupport::ConstRefVecXd surfacePositions) const;

private:
  static bool samePositions(EigenSupport::ConstRefVecXd a, EigenSupport::ConstRefVecXd b);

  bool hasExact_ = false;
  SurfaceIPCActiveSet exact_;
  bool hasLineSearch_ = false;
  SurfaceIPCActiveSet lineSearch_;
};

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
