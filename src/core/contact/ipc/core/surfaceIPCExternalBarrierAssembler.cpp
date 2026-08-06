#include "ipc/core/surfaceIPCExternalBarrierAssembler.h"
#include "ipc/core/surfaceIPCBarrierKernels.h"


#include <atomic>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <vector>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <utility>

namespace pgo
{
namespace Contact
{
namespace IPC
{
using namespace pgo::EigenSupport;

// =========================================================================
//  File-local helpers
// =========================================================================

static const VXd &obsPositions(
  const std::vector<ObstacleSurfaceView> &obstacles,
  int32_t slot)
{
  return obstacles.at(static_cast<std::size_t>(slot)).currentPositions();
}

static V3d obsVtx(const VXd &pos, int i)
{
  return pos.segment<3>(3 * i);
}

static V3d dynVtx(ConstRefVecXd x, int i)
{
  return x.segment<3>(3 * i);
}

// Gradient scatter helpers

static void scatterExternalPTGrad(const V3d &g_local, int dynVertex, RefVecXd g_surf)
{
  double *gd = g_surf.data();
  for (int d = 0; d < 3; ++d)
    std::atomic_ref<double>(gd[3 * dynVertex + d])
      .fetch_add(g_local[d], std::memory_order_relaxed);
}

static void scatterExternalTPGrad(const V9d &g_local, const std::array<int, 3> &dynTri, RefVecXd g_surf)
{
  double *gd = g_surf.data();
  for (int i = 0; i < 3; ++i)
    for (int d = 0; d < 3; ++d)
      std::atomic_ref<double>(gd[3 * dynTri[i] + d])
        .fetch_add(g_local[3 * i + d], std::memory_order_relaxed);
}

static void scatterExternalEEGrad(const V6d &g_local, const std::array<int, 2> &dynEdge, RefVecXd g_surf)
{
  double *gd = g_surf.data();
  for (int i = 0; i < 2; ++i)
    for (int d = 0; d < 3; ++d)
      std::atomic_ref<double>(gd[3 * dynEdge[i] + d])
        .fetch_add(g_local[3 * i + d], std::memory_order_relaxed);
}

// Hessian scatter helpers

struct ExtHessianScatterState
{
  std::vector<TripletD> triplets;
  int pairCount = 0;
};

static void scatterExternalPTHessian(int tripletOffset, const M3d &localH, int dynVertex, ExtHessianScatterState &state)
{
  TripletD *base = state.triplets.data() + tripletOffset;
  int k = 0;
  int ri = 3 * dynVertex;
  int cj = 3 * dynVertex;
  for (int di = 0; di < 3; ++di)
    for (int dj = 0; dj < 3; ++dj, ++k)
      base[k] = TripletD(ri + di, cj + dj, localH(di, dj));
}

static void scatterExternalTPHessian(int tripletOffset, const M9d &localH, const std::array<int, 3> &dynTri, ExtHessianScatterState &state)
{
  TripletD *base = state.triplets.data() + tripletOffset;
  int k = 0;
  for (int i = 0; i < 3; ++i) {
    int ri = 3 * dynTri[i];
    for (int j = 0; j < 3; ++j) {
      int cj = 3 * dynTri[j];
      for (int di = 0; di < 3; ++di)
        for (int dj = 0; dj < 3; ++dj, ++k)
          base[k] = TripletD(ri + di, cj + dj, localH(3 * i + di, 3 * j + dj));
    }
  }
}

static void scatterExternalEEHessian(int tripletOffset, const M6d &localH, const std::array<int, 2> &dynEdge, ExtHessianScatterState &state)
{
  TripletD *base = state.triplets.data() + tripletOffset;
  int k = 0;
  for (int i = 0; i < 2; ++i) {
    int ri = 3 * dynEdge[i];
    for (int j = 0; j < 2; ++j) {
      int cj = 3 * dynEdge[j];
      for (int di = 0; di < 3; ++di)
        for (int dj = 0; dj < 3; ++dj, ++k)
          base[k] = TripletD(ri + di, cj + dj, localH(3 * i + di, 3 * j + dj));
    }
  }
}

// =========================================================================
//  External Energy
// =========================================================================

double computeExternalEnergy(
  ConstRefVecXd dynPos,
  const std::vector<ObstacleSurfaceView> &obstacles,
  const ExternalPairSet &pairs,
  double dhat,
  double kappa,
  double eps_ee)
{
  (void)eps_ee;
  double dhat2 = dhat * dhat;

  double ptEnergy = tbb::parallel_reduce(tbb::blocked_range<decltype(0)>(0, (int)pairs.ptPairs.size(), 1), 0.0, [pgoRangeFn = [&](int rangeBegin, int rangeEnd, double localE) {
    for (int i = rangeBegin; i < rangeEnd; ++i) {
      auto &pair = pairs.ptPairs[i];
      const VXd &obsP = obsPositions(obstacles, pair.obstacleSlot);
      auto k = barrier_kernels::pointStaticTriangle(
        dynVtx(dynPos, pair.dynVertex),
        obsVtx(obsP, pair.obsTri[0]),
        obsVtx(obsP, pair.obsTri[1]),
        obsVtx(obsP, pair.obsTri[2]),
        pair.weight, dhat2, kappa, false, false);
      if (k.active)
        localE += k.energy;
    }
    return localE;
  }](const auto &pgoRange, auto pgoLocal) { return pgoRangeFn(pgoRange.begin(), pgoRange.end(), std::move(pgoLocal)); },
    [pgoJoinFn = std::plus<double>()](auto pgoLeft, auto pgoRight) { return pgoJoinFn(std::move(pgoLeft), std::move(pgoRight)); });

  double tpEnergy = tbb::parallel_reduce(tbb::blocked_range<decltype(0)>(0, (int)pairs.tpPairs.size(), 1), 0.0, [pgoRangeFn = [&](int rangeBegin, int rangeEnd, double localE) {
    for (int i = rangeBegin; i < rangeEnd; ++i) {
      auto &pair = pairs.tpPairs[i];
      const VXd &obsP = obsPositions(obstacles, pair.obstacleSlot);
      auto k = barrier_kernels::staticPointTriangle(
        obsVtx(obsP, pair.obsVertex),
        dynVtx(dynPos, pair.dynTri[0]),
        dynVtx(dynPos, pair.dynTri[1]),
        dynVtx(dynPos, pair.dynTri[2]),
        pair.weight, dhat2, kappa, false, false);
      if (k.active)
        localE += k.energy;
    }
    return localE;
  }](const auto &pgoRange, auto pgoLocal) { return pgoRangeFn(pgoRange.begin(), pgoRange.end(), std::move(pgoLocal)); },
    [pgoJoinFn = std::plus<double>()](auto pgoLeft, auto pgoRight) { return pgoJoinFn(std::move(pgoLeft), std::move(pgoRight)); });

  double eeEnergy = tbb::parallel_reduce(tbb::blocked_range<decltype(0)>(0, (int)pairs.eePairs.size(), 1), 0.0, [pgoRangeFn = [&](int rangeBegin, int rangeEnd, double localE) {
    for (int i = rangeBegin; i < rangeEnd; ++i) {
      auto &pair = pairs.eePairs[i];
      const VXd &obsP = obsPositions(obstacles, pair.obstacleSlot);
      auto k = barrier_kernels::edgeStaticEdge(
        dynVtx(dynPos, pair.dynEdge[0]),
        dynVtx(dynPos, pair.dynEdge[1]),
        obsVtx(obsP, pair.obsEdge[0]),
        obsVtx(obsP, pair.obsEdge[1]),
        pair.weight, dhat2, kappa, eps_ee, false, false);
      if (k.active)
        localE += k.energy;
    }
    return localE;
  }](const auto &pgoRange, auto pgoLocal) { return pgoRangeFn(pgoRange.begin(), pgoRange.end(), std::move(pgoLocal)); },
    [pgoJoinFn = std::plus<double>()](auto pgoLeft, auto pgoRight) { return pgoJoinFn(std::move(pgoLeft), std::move(pgoRight)); });

  return ptEnergy + tpEnergy + eeEnergy;
}

// =========================================================================
//  External Gradient
// =========================================================================

void computeExternalGradient(
  ConstRefVecXd dynPos,
  const std::vector<ObstacleSurfaceView> &obstacles,
  const ExternalPairSet &pairs,
  int numDynVerts,
  double dhat,
  double kappa,
  double eps_ee,
  RefVecXd grad)
{
  int n = 3 * numDynVerts;
  if (grad.size() != n)
    throw std::runtime_error("External gradient vector has wrong size");

  double dhat2 = dhat * dhat;

  // PT pairs
  tbb::parallel_for(0, (int)pairs.ptPairs.size(), [&](int i) {
    {
      auto &pair = pairs.ptPairs[i];
      const VXd &obsP = obsPositions(obstacles, pair.obstacleSlot);
      auto k = barrier_kernels::pointStaticTriangle(
        dynVtx(dynPos, pair.dynVertex),
        obsVtx(obsP, pair.obsTri[0]),
        obsVtx(obsP, pair.obsTri[1]),
        obsVtx(obsP, pair.obsTri[2]),
        pair.weight, dhat2, kappa, true, false);
      if (!k.active)
        return;
      scatterExternalPTGrad(k.gradient, pair.dynVertex, grad);
    }
  });

  // TP pairs
  tbb::parallel_for(0, (int)pairs.tpPairs.size(), [&](int i) {
    {
      auto &pair = pairs.tpPairs[i];
      const VXd &obsP = obsPositions(obstacles, pair.obstacleSlot);
      auto k = barrier_kernels::staticPointTriangle(
        obsVtx(obsP, pair.obsVertex),
        dynVtx(dynPos, pair.dynTri[0]),
        dynVtx(dynPos, pair.dynTri[1]),
        dynVtx(dynPos, pair.dynTri[2]),
        pair.weight, dhat2, kappa, true, false);
      if (!k.active)
        return;
      scatterExternalTPGrad(k.gradient, pair.dynTri, grad);
    }
  });

  // EE pairs
  tbb::parallel_for(0, (int)pairs.eePairs.size(), [&](int i) {
    {
      auto &pair = pairs.eePairs[i];
      const VXd &obsP = obsPositions(obstacles, pair.obstacleSlot);
      auto k = barrier_kernels::edgeStaticEdge(
        dynVtx(dynPos, pair.dynEdge[0]),
        dynVtx(dynPos, pair.dynEdge[1]),
        obsVtx(obsP, pair.obsEdge[0]),
        obsVtx(obsP, pair.obsEdge[1]),
        pair.weight, dhat2, kappa, eps_ee, true, false);
      if (!k.active)
        return;
      scatterExternalEEGrad(k.gradient, pair.dynEdge, grad);
    }
  });
}

// =========================================================================
//  External Hessian
// =========================================================================

void computeExternalHessian(
  ConstRefVecXd dynPos,
  const std::vector<ObstacleSurfaceView> &obstacles,
  const ExternalPairSet &pairs,
  int numDynVerts,
  double dhat,
  double kappa,
  double eps_ee,
  SpMatD &hess)
{
  int n = 3 * numDynVerts;
  int nPT = (int)pairs.ptPairs.size();
  int nTP = (int)pairs.tpPairs.size();
  int nEE = (int)pairs.eePairs.size();

  ExtHessianScatterState state;
  state.pairCount = nPT * 9 + nTP * 81 + nEE * 36;
  {
    state.triplets.resize(state.pairCount, TripletD(0, 0, 0.0));
  }

  double dhat2 = dhat * dhat;

  // PT pairs
  tbb::parallel_for(0, nPT, [&](int i) {
    {
      auto &pair = pairs.ptPairs[i];
      const VXd &obsP = obsPositions(obstacles, pair.obstacleSlot);
      auto k = barrier_kernels::pointStaticTriangle(
        dynVtx(dynPos, pair.dynVertex),
        obsVtx(obsP, pair.obsTri[0]),
        obsVtx(obsP, pair.obsTri[1]),
        obsVtx(obsP, pair.obsTri[2]),
        pair.weight, dhat2, kappa, false, true);
      if (!k.active)
        return;
      scatterExternalPTHessian(9 * i, k.hessian, pair.dynVertex, state);
    }
  });

  // TP pairs
  tbb::parallel_for(0, nTP, [&](int i) {
    {
      auto &pair = pairs.tpPairs[i];
      const VXd &obsP = obsPositions(obstacles, pair.obstacleSlot);
      auto k = barrier_kernels::staticPointTriangle(
        obsVtx(obsP, pair.obsVertex),
        dynVtx(dynPos, pair.dynTri[0]),
        dynVtx(dynPos, pair.dynTri[1]),
        dynVtx(dynPos, pair.dynTri[2]),
        pair.weight, dhat2, kappa, false, true);
      if (!k.active)
        return;
      scatterExternalTPHessian(9 * nPT + 81 * i, k.hessian, pair.dynTri, state);
    }
  });

  // EE pairs
  tbb::parallel_for(0, nEE, [&](int i) {
    {
      auto &pair = pairs.eePairs[i];
      const VXd &obsP = obsPositions(obstacles, pair.obstacleSlot);
      auto k = barrier_kernels::edgeStaticEdge(
        dynVtx(dynPos, pair.dynEdge[0]),
        dynVtx(dynPos, pair.dynEdge[1]),
        obsVtx(obsP, pair.obsEdge[0]),
        obsVtx(obsP, pair.obsEdge[1]),
        pair.weight, dhat2, kappa, eps_ee, false, true);
      if (!k.active)
        return;
      scatterExternalEEHessian(9 * nPT + 81 * nTP + 36 * i, k.hessian, pair.dynEdge, state);
    }
  });

  SpMatD hessExt(n, n);
  {
    hessExt.setFromTriplets(state.triplets.begin(), state.triplets.end());
  }
  if (hess.nonZeros() == 0) {
    hess = std::move(hessExt);
  }
  else {
    hess += hessExt;
  }
}

// =========================================================================
//  External Combined (energy + gradient + hessian in single pass)
// =========================================================================

void computeExternalAll(
  ConstRefVecXd dynPos,
  const std::vector<ObstacleSurfaceView> &obstacles,
  const ExternalPairSet &pairs,
  int numDynVerts,
  double dhat,
  double kappa,
  double eps_ee,
  double &energy,
  VXd &grad,
  SpMatD &hess)
{
  int n = 3 * numDynVerts;
  int nPT = (int)pairs.ptPairs.size();
  int nTP = (int)pairs.tpPairs.size();
  int nEE = (int)pairs.eePairs.size();


  energy = 0.0;
  if (grad.size() != n)
    grad.setZero(n);

  if (nPT == 0 && nTP == 0 && nEE == 0) {
    if (hess.rows() == 0 && hess.cols() == 0) {
      hess.resize(n, n);
      hess.setZero();
    }
    return;
  }

  ExtHessianScatterState hState;
  hState.pairCount = nPT * 9 + nTP * 81 + nEE * 36;
  {
    hState.triplets.resize(hState.pairCount, TripletD(0, 0, 0.0));
  }

  double dhat2 = dhat * dhat;

  // PT pairs
  double ptEnergy = 0.0;
  {
    ptEnergy = tbb::parallel_reduce(tbb::blocked_range<decltype(0)>(0, nPT, 1), 0.0, [pgoRangeFn = [&](int rangeBegin, int rangeEnd, double localE) {
      for (int i = rangeBegin; i < rangeEnd; ++i) {
        auto &pair = pairs.ptPairs[i];
        const VXd &obsP = obsPositions(obstacles, pair.obstacleSlot);
        auto k = barrier_kernels::pointStaticTriangle(
          dynVtx(dynPos, pair.dynVertex),
          obsVtx(obsP, pair.obsTri[0]),
          obsVtx(obsP, pair.obsTri[1]),
          obsVtx(obsP, pair.obsTri[2]),
          pair.weight, dhat2, kappa, true, true);
        if (!k.active)
          continue;
        localE += k.energy;
        scatterExternalPTGrad(k.gradient, pair.dynVertex, grad);
        scatterExternalPTHessian(9 * i, k.hessian, pair.dynVertex, hState);
      }
      return localE;
    }](const auto &pgoRange, auto pgoLocal) { return pgoRangeFn(pgoRange.begin(), pgoRange.end(), std::move(pgoLocal)); },
      [pgoJoinFn = std::plus<double>()](auto pgoLeft, auto pgoRight) { return pgoJoinFn(std::move(pgoLeft), std::move(pgoRight)); });
  }

  // TP pairs
  double tpEnergy = 0.0;
  {
    tpEnergy = tbb::parallel_reduce(tbb::blocked_range<decltype(0)>(0, nTP, 1), 0.0, [pgoRangeFn = [&](int rangeBegin, int rangeEnd, double localE) {
      for (int i = rangeBegin; i < rangeEnd; ++i) {
        auto &pair = pairs.tpPairs[i];
        const VXd &obsP = obsPositions(obstacles, pair.obstacleSlot);
        auto k = barrier_kernels::staticPointTriangle(
          obsVtx(obsP, pair.obsVertex),
          dynVtx(dynPos, pair.dynTri[0]),
          dynVtx(dynPos, pair.dynTri[1]),
          dynVtx(dynPos, pair.dynTri[2]),
          pair.weight, dhat2, kappa, true, true);
        if (!k.active)
          continue;
        localE += k.energy;
        scatterExternalTPGrad(k.gradient, pair.dynTri, grad);
        scatterExternalTPHessian(9 * nPT + 81 * i, k.hessian, pair.dynTri, hState);
      }
      return localE;
    }](const auto &pgoRange, auto pgoLocal) { return pgoRangeFn(pgoRange.begin(), pgoRange.end(), std::move(pgoLocal)); },
      [pgoJoinFn = std::plus<double>()](auto pgoLeft, auto pgoRight) { return pgoJoinFn(std::move(pgoLeft), std::move(pgoRight)); });
  }

  // EE pairs
  double eeEnergy = 0.0;
  {
    eeEnergy = tbb::parallel_reduce(tbb::blocked_range<decltype(0)>(0, nEE, 1), 0.0, [pgoRangeFn = [&](int rangeBegin, int rangeEnd, double localE) {
      for (int i = rangeBegin; i < rangeEnd; ++i) {
        auto &pair = pairs.eePairs[i];
        const VXd &obsP = obsPositions(obstacles, pair.obstacleSlot);
        auto k = barrier_kernels::edgeStaticEdge(
          dynVtx(dynPos, pair.dynEdge[0]),
          dynVtx(dynPos, pair.dynEdge[1]),
          obsVtx(obsP, pair.obsEdge[0]),
          obsVtx(obsP, pair.obsEdge[1]),
          pair.weight, dhat2, kappa, eps_ee, true, true);
        if (!k.active)
          continue;
        localE += k.energy;
        scatterExternalEEGrad(k.gradient, pair.dynEdge, grad);
        scatterExternalEEHessian(9 * nPT + 81 * nTP + 36 * i, k.hessian, pair.dynEdge, hState);
      }
      return localE;
    }](const auto &pgoRange, auto pgoLocal) { return pgoRangeFn(pgoRange.begin(), pgoRange.end(), std::move(pgoLocal)); },
      [pgoJoinFn = std::plus<double>()](auto pgoLeft, auto pgoRight) { return pgoJoinFn(std::move(pgoLeft), std::move(pgoRight)); });
  }

  energy = ptEnergy + tpEnergy + eeEnergy;

  SpMatD hessExt(n, n);
  {
    hessExt.setFromTriplets(hState.triplets.begin(), hState.triplets.end());
  }
  if (hess.nonZeros() == 0) {
    hess = std::move(hessExt);
  }
  else {
    hess += hessExt;
  }
}

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
