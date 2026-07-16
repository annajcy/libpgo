#include "surfaceIPCSelfBarrierAssembler.h"
#include "surfaceIPCBarrierKernels.h"

#include "scopedProfileSection.h"
#include "ipc/profiling/surfaceIPCProfiling.h"

#include <tbb/enumerable_thread_specific.h>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
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

static V3d vtx(ConstRefVecXd x, int i)
{
  return x.segment<3>(3 * i);
}

static void scatterSelfGrad(const V12d &local, const int idx[4], RefVecXd grad)
{
  double *gdata = grad.data();
  for (int i = 0; i < 4; ++i)
    if (idx[i] >= 0)
      for (int d = 0; d < 3; ++d)
        std::atomic_ref<double>(gdata[3 * idx[i] + d])
          .fetch_add(local[3 * i + d], std::memory_order_relaxed);
}

struct RowValue
{
  Eigen::Index col = 0;
  double value = 0.0;
};

struct SelfHessianThreadRows
{
  std::vector<std::vector<RowValue>> rows;

  explicit SelfHessianThreadRows(Eigen::Index n): rows(static_cast<std::size_t>(n))
  {
  }
};

struct RowMergeStats
{
  std::uint64_t contributions = 0;
  std::uint64_t activeRows = 0;
};

EigenSupport::SpMatD::StorageIndex checkedStorageIndexFromUint64(std::uint64_t value, const char *what)
{
  using StorageIndex = EigenSupport::SpMatD::StorageIndex;
  if (value > static_cast<std::uint64_t>(std::numeric_limits<StorageIndex>::max()))
    throw std::overflow_error(std::string("Self IPC direct sparse fill ") + what +
      " exceeds Eigen sparse storage index range.");
  return static_cast<StorageIndex>(value);
}

EigenSupport::SpMatD::StorageIndex checkedStorageIndexFromEigenIndex(Eigen::Index value, const char *what)
{
  using StorageIndex = EigenSupport::SpMatD::StorageIndex;
  if (value < 0 || value > static_cast<Eigen::Index>(std::numeric_limits<StorageIndex>::max()))
    throw std::overflow_error(std::string("Self IPC direct sparse fill ") + what +
      " exceeds Eigen sparse storage index range.");
  return static_cast<StorageIndex>(value);
}

Eigen::Index checkedEigenIndexFromUint64(std::uint64_t value, const char *what)
{
  if (value > static_cast<std::uint64_t>(std::numeric_limits<Eigen::Index>::max()))
    throw std::overflow_error(std::string("Self IPC sparse fill ") + what +
      " exceeds Eigen index range.");
  return static_cast<Eigen::Index>(value);
}

static void appendSelfHessianRows(
  const M12d &localH,
  const int idx[4],
  std::vector<std::vector<RowValue>> &rows)
{
  for (int i = 0; i < 4; ++i) {
    if (idx[i] < 0)
      continue;

    for (int di = 0; di < 3; ++di) {
      const Eigen::Index row = 3 * idx[i] + di;
      std::vector<RowValue> &rowValues = rows[static_cast<std::size_t>(row)];
      for (int j = 0; j < 4; ++j) {
        if (idx[j] < 0)
          continue;

        const Eigen::Index colBase = 3 * idx[j];
        for (int dj = 0; dj < 3; ++dj)
          rowValues.push_back(RowValue{ colBase + dj, localH(3 * i + di, 3 * j + dj) });
      }
    }
  }
}

static void fillSparseRowsDirect(
  const std::vector<std::vector<RowValue>> &rowBuffers,
  Eigen::Index n,
  std::uint64_t outputNnz,
  SpMatD &hess)
{
  using StorageIndex = SpMatD::StorageIndex;

  (void)checkedStorageIndexFromEigenIndex(n, "row count");
  const Eigen::Index totalNnz = checkedEigenIndexFromUint64(outputNnz, "nonzero count");

  std::vector<StorageIndex> outerOffsets(static_cast<std::size_t>(n) + 1u);
  std::uint64_t offset = 0;
  outerOffsets[0] = 0;
  for (Eigen::Index row = 0; row < n; ++row) {
    offset += static_cast<std::uint64_t>(rowBuffers[static_cast<std::size_t>(row)].size());
    outerOffsets[static_cast<std::size_t>(row) + 1u] =
      checkedStorageIndexFromUint64(offset, "row offset");
  }
  if (offset != outputNnz)
    throw std::logic_error("Self IPC direct sparse fill row offsets do not match output nnz.");

  hess.resize(n, n);
  hess.resizeNonZeros(totalNnz);
  std::copy(outerOffsets.begin(), outerOffsets.end(), hess.outerIndexPtr());

  tbb::parallel_for(tbb::blocked_range<decltype(Eigen::Index{ 0 })>(Eigen::Index{ 0 }, n, 1), [pgoBody = [&](Eigen::Index rangeBegin, Eigen::Index rangeEnd) {
    for (Eigen::Index row = rangeBegin; row < rangeEnd; ++row) {
      const std::vector<RowValue> &rowBuffer = rowBuffers[static_cast<std::size_t>(row)];
      const StorageIndex offset = outerOffsets[static_cast<std::size_t>(row)];
      for (std::size_t entryIndex = 0; entryIndex < rowBuffer.size(); ++entryIndex) {
        const RowValue &entry = rowBuffer[entryIndex];
        const StorageIndex storageIndex = offset + static_cast<StorageIndex>(entryIndex);
        hess.innerIndexPtr()[storageIndex] = static_cast<StorageIndex>(entry.col);
        hess.valuePtr()[storageIndex] = entry.value;
      }
    }
  }](const auto &pgoRange) { pgoBody(pgoRange.begin(), pgoRange.end()); });

  if (!hess.isCompressed())
    hess.makeCompressed();
}

static void buildSelfHessianFromThreadRows(
  tbb::enumerable_thread_specific<SelfHessianThreadRows> &threadRows,
  Eigen::Index n,
  SpMatD &hess)
{
  Profiling::ScopedProfileSection assemblyProfile(
    SurfaceIPCProfileSections::kActiveSetSelfDirectRowAssembly);

  std::vector<SelfHessianThreadRows *> scratchRows;
  for (SelfHessianThreadRows &scratch : threadRows)
    scratchRows.push_back(&scratch);

  std::vector<std::vector<RowValue>> rowBuffers(static_cast<std::size_t>(n));
  std::uint64_t contributionCount = 0;
  std::uint64_t activeRows = 0;
  {
    Profiling::ScopedProfileSection mergeProfile(
      SurfaceIPCProfileSections::kActiveSetSelfThreadRowMerge);

    const RowMergeStats mergeStats =
      tbb::parallel_reduce(tbb::blocked_range<decltype(Eigen::Index{ 0 })>(Eigen::Index{ 0 }, n, 1), RowMergeStats{}, [pgoRangeFn = [&](Eigen::Index rangeBegin, Eigen::Index rangeEnd, RowMergeStats localStats) {
        for (Eigen::Index row = rangeBegin; row < rangeEnd; ++row) {
          const std::size_t rowIndex = static_cast<std::size_t>(row);
          std::size_t rowSize = 0;
          for (const SelfHessianThreadRows *scratch : scratchRows)
            rowSize += scratch->rows[rowIndex].size();

          if (rowSize == 0)
            continue;

          localStats.contributions += static_cast<std::uint64_t>(rowSize);
          ++localStats.activeRows;

          std::vector<RowValue> &rowBuffer = rowBuffers[rowIndex];
          rowBuffer.reserve(rowSize);
          for (const SelfHessianThreadRows *scratch : scratchRows) {
            const std::vector<RowValue> &threadRow = scratch->rows[rowIndex];
            rowBuffer.insert(rowBuffer.end(), threadRow.begin(), threadRow.end());
          }
        }
        return localStats;
      }](const auto &pgoRange, auto pgoLocal) { return pgoRangeFn(pgoRange.begin(), pgoRange.end(), std::move(pgoLocal)); },
        [pgoJoinFn = [](const RowMergeStats &lhs, const RowMergeStats &rhs) {
          return RowMergeStats{
            lhs.contributions + rhs.contributions,
            lhs.activeRows + rhs.activeRows
          };
        }](auto pgoLeft, auto pgoRight) { return pgoJoinFn(std::move(pgoLeft), std::move(pgoRight)); });
    contributionCount = mergeStats.contributions;
    activeRows = mergeStats.activeRows;
  }

  {
    Profiling::ScopedProfileSection reduceProfile(
      SurfaceIPCProfileSections::kActiveSetSelfRowSortReduce);
    tbb::parallel_for(tbb::blocked_range<decltype(Eigen::Index{ 0 })>(Eigen::Index{ 0 }, n, 1), [pgoBody = [&](Eigen::Index rangeBegin, Eigen::Index rangeEnd) {
      for (Eigen::Index row = rangeBegin; row < rangeEnd; ++row) {
        std::vector<RowValue> &rowBuffer = rowBuffers[static_cast<std::size_t>(row)];
        if (rowBuffer.empty())
          continue;

        std::sort(rowBuffer.begin(), rowBuffer.end(),
          [](const RowValue &lhs, const RowValue &rhs) {
            return lhs.col < rhs.col;
          });

        std::size_t writeIndex = 0;
        for (std::size_t readIndex = 0; readIndex < rowBuffer.size();) {
          const RowValue &first = rowBuffer[readIndex];
          const Eigen::Index col = first.col;
          double value = first.value;
          ++readIndex;
          while (readIndex < rowBuffer.size() && rowBuffer[readIndex].col == col) {
            value += rowBuffer[readIndex].value;
            ++readIndex;
          }

          rowBuffer[writeIndex++] = RowValue{ col, value };
        }
        rowBuffer.resize(writeIndex);
      }
    }](const auto &pgoRange) { pgoBody(pgoRange.begin(), pgoRange.end()); });
  }

  std::uint64_t outputNnz = 0;
  for (Eigen::Index row = 0; row < n; ++row) {
    const std::vector<RowValue> &rowBuffer = rowBuffers[static_cast<std::size_t>(row)];
    outputNnz += static_cast<std::uint64_t>(rowBuffer.size());
  }

  {
    Profiling::ScopedProfileSection fillProfile(
      SurfaceIPCProfileSections::kActiveSetSelfDirectSparseFill);
    fillSparseRowsDirect(rowBuffers, n, outputNnz, hess);
  }

  Profiling::recordProfileCounter(
    SurfaceIPCProfileSections::kActiveSetSelfDirectRowContributions, contributionCount);
  Profiling::recordProfileCounter(
    SurfaceIPCProfileSections::kActiveSetSelfDirectActiveRows, activeRows);
}

// =========================================================================
//  Self Energy
// =========================================================================

double computeSelfEnergy(
  EigenSupport::ConstRefVecXd dynPos,
  const SelfPairSet &pairs,
  int numVerts,
  double dhat,
  double kappa,
  double eps_ee)
{
  (void)numVerts;
  double dhat2 = dhat * dhat;

  // PT pairs
  double ptEnergy = tbb::parallel_reduce(tbb::blocked_range<decltype(0)>(0, (int)pairs.ptPairs.size(), 1), 0.0, [pgoRangeFn = [&](int rangeBegin, int rangeEnd, double localE) {
    for (int i = rangeBegin; i < rangeEnd; ++i) {
      auto &pair = pairs.ptPairs[i];
      auto k = barrier_kernels::pointTriangle(
        vtx(dynPos, pair.p), vtx(dynPos, pair.t0), vtx(dynPos, pair.t1), vtx(dynPos, pair.t2),
        pair.weight, dhat2, kappa, false, false);
      if (k.active)
        localE += k.energy;
    }
    return localE;
  }](const auto &pgoRange, auto pgoLocal) { return pgoRangeFn(pgoRange.begin(), pgoRange.end(), std::move(pgoLocal)); },
    [pgoJoinFn = std::plus<double>()](auto pgoLeft, auto pgoRight) { return pgoJoinFn(std::move(pgoLeft), std::move(pgoRight)); });

  // EE pairs
  double eeEnergy = tbb::parallel_reduce(tbb::blocked_range<decltype(0)>(0, (int)pairs.eePairs.size(), 1), 0.0, [pgoRangeFn = [&](int rangeBegin, int rangeEnd, double localE) {
    for (int i = rangeBegin; i < rangeEnd; ++i) {
      auto &pair = pairs.eePairs[i];
      auto k = barrier_kernels::edgeEdge(
        vtx(dynPos, pair.ea0), vtx(dynPos, pair.ea1), vtx(dynPos, pair.eb0), vtx(dynPos, pair.eb1),
        pair.weight, dhat2, kappa, eps_ee, false, false);
      if (k.active)
        localE += k.energy;
    }
    return localE;
  }](const auto &pgoRange, auto pgoLocal) { return pgoRangeFn(pgoRange.begin(), pgoRange.end(), std::move(pgoLocal)); },
    [pgoJoinFn = std::plus<double>()](auto pgoLeft, auto pgoRight) { return pgoJoinFn(std::move(pgoLeft), std::move(pgoRight)); });

  return ptEnergy + eeEnergy;
}

// =========================================================================
//  Self Gradient
// =========================================================================

void computeSelfGradient(
  EigenSupport::ConstRefVecXd dynPos,
  const SelfPairSet &pairs,
  int numVerts,
  double dhat,
  double kappa,
  double eps_ee,
  EigenSupport::RefVecXd grad)
{
  int n = 3 * numVerts;
  if (grad.size() != n)
    throw std::runtime_error("Gradient vector has wrong size");

  grad.setZero();

  double dhat2 = dhat * dhat;

  // PT pairs
  tbb::parallel_for(0, (int)pairs.ptPairs.size(), [&](int i) {
    {
      auto &pair = pairs.ptPairs[i];
      auto k = barrier_kernels::pointTriangle(
        vtx(dynPos, pair.p), vtx(dynPos, pair.t0), vtx(dynPos, pair.t1), vtx(dynPos, pair.t2),
        pair.weight, dhat2, kappa, true, false);
      if (!k.active)
        return;
      int idx[4] = { pair.p, pair.t0, pair.t1, pair.t2 };
      scatterSelfGrad(k.gradient, idx, grad);
    }
  });

  // EE pairs
  tbb::parallel_for(0, (int)pairs.eePairs.size(), [&](int i) {
    {
      auto &pair = pairs.eePairs[i];
      auto k = barrier_kernels::edgeEdge(
        vtx(dynPos, pair.ea0), vtx(dynPos, pair.ea1), vtx(dynPos, pair.eb0), vtx(dynPos, pair.eb1),
        pair.weight, dhat2, kappa, eps_ee, true, false);
      if (!k.active)
        return;
      int idx[4] = { pair.ea0, pair.ea1, pair.eb0, pair.eb1 };
      scatterSelfGrad(k.gradient, idx, grad);
    }
  });
}

// =========================================================================
//  Self Hessian
// =========================================================================

void computeSelfHessian(
  EigenSupport::ConstRefVecXd dynPos,
  const SelfPairSet &pairs,
  int numVerts,
  double dhat,
  double kappa,
  double eps_ee,
  SpMatD &hess)
{
  int n = 3 * numVerts;
  int nPT = (int)pairs.ptPairs.size();
  int nEE = (int)pairs.eePairs.size();
  int totalPairs = nPT + nEE;

  (void)totalPairs;
  tbb::enumerable_thread_specific<SelfHessianThreadRows> threadRows(
    [n] { return SelfHessianThreadRows(n); });

  double dhat2 = dhat * dhat;

  // PT pairs
  tbb::parallel_for(tbb::blocked_range<decltype(0)>(0, nPT, 1), [pgoBody = [&](int rangeBegin, int rangeEnd) {
    std::vector<std::vector<RowValue>> &rows = threadRows.local().rows;
    for (int i = rangeBegin; i < rangeEnd; ++i) {
      auto &pair = pairs.ptPairs[i];
      auto k = barrier_kernels::pointTriangle(
        vtx(dynPos, pair.p), vtx(dynPos, pair.t0), vtx(dynPos, pair.t1), vtx(dynPos, pair.t2),
        pair.weight, dhat2, kappa, false, true);
      if (!k.active)
        continue;
      int idx[4] = { pair.p, pair.t0, pair.t1, pair.t2 };
      appendSelfHessianRows(k.hessian, idx, rows);
    }
  }](const auto &pgoRange) { pgoBody(pgoRange.begin(), pgoRange.end()); });

  // EE pairs
  tbb::parallel_for(tbb::blocked_range<decltype(0)>(0, nEE, 1), [pgoBody = [&](int rangeBegin, int rangeEnd) {
    std::vector<std::vector<RowValue>> &rows = threadRows.local().rows;
    for (int i = rangeBegin; i < rangeEnd; ++i) {
      auto &pair = pairs.eePairs[i];
      auto k = barrier_kernels::edgeEdge(
        vtx(dynPos, pair.ea0), vtx(dynPos, pair.ea1), vtx(dynPos, pair.eb0), vtx(dynPos, pair.eb1),
        pair.weight, dhat2, kappa, eps_ee, false, true);
      if (!k.active)
        continue;
      int idx[4] = { pair.ea0, pair.ea1, pair.eb0, pair.eb1 };
      appendSelfHessianRows(k.hessian, idx, rows);
    }
  }](const auto &pgoRange) { pgoBody(pgoRange.begin(), pgoRange.end()); });

  buildSelfHessianFromThreadRows(threadRows, n, hess);
}

// =========================================================================
//  Self Combined (energy + gradient + hessian in single pass)
// =========================================================================

void computeSelfAll(
  EigenSupport::ConstRefVecXd dynPos,
  const SelfPairSet &pairs,
  int numVerts,
  double dhat,
  double kappa,
  double eps_ee,
  double &energy,
  VXd &grad,
  SpMatD &hess)
{
  int n = 3 * numVerts;
  int nPT = (int)pairs.ptPairs.size();
  int nEE = (int)pairs.eePairs.size();
  int totalPairs = nPT + nEE;

  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kActiveSetSelfCombined);
  Profiling::recordProfileCounter(SurfaceIPCProfileSections::kActiveSetSelfPTPairCount, static_cast<std::uint64_t>(nPT));
  Profiling::recordProfileCounter(SurfaceIPCProfileSections::kActiveSetSelfEEPairCount, static_cast<std::uint64_t>(nEE));
  Profiling::recordProfileCounter(
    SurfaceIPCProfileSections::kActiveSetSelfTripletSlots, static_cast<std::uint64_t>(144 * totalPairs));

  energy = 0.0;
  grad.setZero(n);
  tbb::enumerable_thread_specific<SelfHessianThreadRows> threadRows(
    [n] { return SelfHessianThreadRows(n); });

  double dhat2 = dhat * dhat;

  // ---- PT pairs ----
  double ptEnergy = 0.0;
  {
    Profiling::ScopedProfileSection ptProfile(SurfaceIPCProfileSections::kActiveSetSelfPTCombined);
    ptEnergy = tbb::parallel_reduce(tbb::blocked_range<decltype(0)>(0, nPT, 1), 0.0, [pgoRangeFn = [&](int rangeBegin, int rangeEnd, double localE) {
      std::vector<std::vector<RowValue>> &rows = threadRows.local().rows;
      for (int i = rangeBegin; i < rangeEnd; ++i) {
        auto &pair = pairs.ptPairs[i];
        auto k = barrier_kernels::pointTriangle(
          vtx(dynPos, pair.p), vtx(dynPos, pair.t0), vtx(dynPos, pair.t1), vtx(dynPos, pair.t2),
          pair.weight, dhat2, kappa, true, true);
        if (!k.active)
          continue;
        localE += k.energy;
        int idx[4] = { pair.p, pair.t0, pair.t1, pair.t2 };
        scatterSelfGrad(k.gradient, idx, grad);
        appendSelfHessianRows(k.hessian, idx, rows);
      }
      return localE;
    }](const auto &pgoRange, auto pgoLocal) { return pgoRangeFn(pgoRange.begin(), pgoRange.end(), std::move(pgoLocal)); },
      [pgoJoinFn = std::plus<double>()](auto pgoLeft, auto pgoRight) { return pgoJoinFn(std::move(pgoLeft), std::move(pgoRight)); });
  }

  // ---- EE pairs ----
  double eeEnergy = 0.0;
  {
    Profiling::ScopedProfileSection eeProfile(SurfaceIPCProfileSections::kActiveSetSelfEECombined);
    eeEnergy = tbb::parallel_reduce(tbb::blocked_range<decltype(0)>(0, nEE, 1), 0.0, [pgoRangeFn = [&](int rangeBegin, int rangeEnd, double localE) {
      std::vector<std::vector<RowValue>> &rows = threadRows.local().rows;
      for (int i = rangeBegin; i < rangeEnd; ++i) {
        auto &pair = pairs.eePairs[i];
        auto k = barrier_kernels::edgeEdge(
          vtx(dynPos, pair.ea0), vtx(dynPos, pair.ea1), vtx(dynPos, pair.eb0), vtx(dynPos, pair.eb1),
          pair.weight, dhat2, kappa, eps_ee, true, true);
        if (!k.active)
          continue;
        localE += k.energy;
        int idx[4] = { pair.ea0, pair.ea1, pair.eb0, pair.eb1 };
        scatterSelfGrad(k.gradient, idx, grad);
        appendSelfHessianRows(k.hessian, idx, rows);
      }
      return localE;
    }](const auto &pgoRange, auto pgoLocal) { return pgoRangeFn(pgoRange.begin(), pgoRange.end(), std::move(pgoLocal)); },
      [pgoJoinFn = std::plus<double>()](auto pgoLeft, auto pgoRight) { return pgoJoinFn(std::move(pgoLeft), std::move(pgoRight)); });
  }

  energy = ptEnergy + eeEnergy;

  buildSelfHessianFromThreadRows(threadRows, n, hess);
  Profiling::recordProfileCounter(
    SurfaceIPCProfileSections::kActiveSetSelfHessianNnz, static_cast<std::uint64_t>(hess.nonZeros()));
}

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
