#include "energy/energySet.h"
#include "EigenSupport.h"
#include "scopedProfileSection.h"
#include "parallelism/parallelFor.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace pgo::NonlinearOptimization
{
namespace ES = pgo::EigenSupport;

namespace
{
struct PatternSnapshot
{
  bool valid = false;
  Eigen::Index rows = 0;
  Eigen::Index cols = 0;
  Eigen::Index nnz = 0;
  std::size_t hash = 0;
  std::vector<ES::SpMatD::StorageIndex> outerIndices;
  std::vector<ES::SpMatD::StorageIndex> innerIndices;
};

struct DynamicAssemblyCache
{
  bool valid = false;
  ES::SpMatD hessianTemplate;
  PatternSnapshot fullPattern;
  std::vector<ES::SpMatI, Eigen::aligned_allocator<ES::SpMatI>> termMappings;
  std::vector<PatternSnapshot> termPatterns;
};

void hashCombine(std::size_t &seed, std::size_t value)
{
  seed ^= value + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2);
}

PatternSnapshot snapshotPattern(const ES::SpMatD &mat)
{
  PatternSnapshot snapshot;
  snapshot.valid = true;
  snapshot.rows = mat.rows();
  snapshot.cols = mat.cols();
  snapshot.nnz = mat.nonZeros();

  snapshot.outerIndices.assign(mat.outerIndexPtr(), mat.outerIndexPtr() + mat.outerSize() + 1);
  snapshot.innerIndices.assign(mat.innerIndexPtr(), mat.innerIndexPtr() + mat.nonZeros());

  std::size_t hash = 1469598103934665603ull;
  hashCombine(hash, static_cast<std::size_t>(snapshot.rows));
  hashCombine(hash, static_cast<std::size_t>(snapshot.cols));
  hashCombine(hash, static_cast<std::size_t>(snapshot.nnz));
  for (ES::SpMatD::StorageIndex index : snapshot.outerIndices)
    hashCombine(hash, static_cast<std::size_t>(index));
  for (ES::SpMatD::StorageIndex index : snapshot.innerIndices)
    hashCombine(hash, static_cast<std::size_t>(index));
  snapshot.hash = hash;

  return snapshot;
}

PatternSnapshot emptyPattern(Eigen::Index rows, Eigen::Index cols)
{
  ES::SpMatD empty(rows, cols);
  empty.makeCompressed();
  return snapshotPattern(empty);
}

bool patternMatches(const ES::SpMatD &mat, const PatternSnapshot &snapshot)
{
  if (!snapshot.valid)
    return false;
  if (!mat.isCompressed())
    return false;
  if (mat.rows() != snapshot.rows || mat.cols() != snapshot.cols || mat.nonZeros() != snapshot.nnz)
    return false;

  using StorageIndex = ES::SpMatD::StorageIndex;
  const std::size_t outerBytes = static_cast<std::size_t>(mat.outerSize() + 1) * sizeof(StorageIndex);
  if (std::memcmp(mat.outerIndexPtr(), snapshot.outerIndices.data(), outerBytes) != 0)
    return false;

  const std::size_t innerBytes = static_cast<std::size_t>(mat.nonZeros()) * sizeof(StorageIndex);
  return innerBytes == 0 ||
    std::memcmp(mat.innerIndexPtr(), snapshot.innerIndices.data(), innerBytes) == 0;
}

bool emptyPatternMatches(Eigen::Index rows, Eigen::Index cols, const PatternSnapshot &snapshot)
{
  return snapshot.valid && snapshot.rows == rows && snapshot.cols == cols && snapshot.nnz == 0;
}

void zeroSparseValues(ES::SpMatD &mat)
{
  constexpr Eigen::Index kParallelZeroThreshold = 100000;
  const Eigen::Index nnz = mat.nonZeros();
  if (nnz < kParallelZeroThreshold) {
    std::memset(mat.valuePtr(), 0, sizeof(double) * nnz);
    return;
  }

  double *values = mat.valuePtr();
  tbb::parallel_for(tbb::blocked_range<Eigen::Index>(0, nnz, 8192),
    [values](const tbb::blocked_range<Eigen::Index> &range) {
      std::fill(values + range.begin(), values + range.end(), 0.0);
    });
}

Eigen::Index findRequiredSparseOffset(const ES::SpMatD &mat, Eigen::Index row, Eigen::Index col)
{
  const ES::SpMatD::StorageIndex *rowStart = mat.innerIndexPtr() + mat.outerIndexPtr()[row];
  const ES::SpMatD::StorageIndex *rowEnd = mat.innerIndexPtr() + mat.outerIndexPtr()[row + 1];
  const auto it = std::lower_bound(rowStart, rowEnd, static_cast<ES::SpMatD::StorageIndex>(col));

  if (it == rowEnd || *it != col)
    throw std::domain_error("Different sparse matrix topology");

  return it - mat.innerIndexPtr();
}

void buildSmallToBigMappingFast(const ES::SpMatD &Asmall, const ES::SpMatD &Abig, const std::vector<int> &dofs, ES::SpMatI &mapping)
{
  mapping.resize(Asmall.rows(), Asmall.cols());
  if (Asmall.nonZeros() == 0) {
    mapping.makeCompressed();
    return;
  }

  mapping.reserve(Asmall.nonZeros());
  for (Eigen::Index outeri = 0; outeri < Asmall.outerSize(); ++outeri) {
    mapping.startVec(outeri);
    for (ES::SpMatD::InnerIterator it(Asmall, outeri); it; ++it) {
      const Eigen::Index smallCol = it.col();
      mapping.insertBackByOuterInner(outeri, smallCol) = 0;
    }
  }
  mapping.finalize();
  mapping.makeCompressed();

  tbb::parallel_for(Eigen::Index(0), Asmall.outerSize(), [&](Eigen::Index outeri) {
    for (Eigen::Index k = Asmall.outerIndexPtr()[outeri]; k < Asmall.outerIndexPtr()[outeri + 1]; ++k) {
      const Eigen::Index smallRow = outeri;
      const Eigen::Index smallCol = Asmall.innerIndexPtr()[k];
      const Eigen::Index bigRow = dofs[smallRow];
      const Eigen::Index bigCol = dofs[smallCol];

      mapping.valuePtr()[k] = findRequiredSparseOffset(Abig, bigRow, bigCol);
    }
  });
}

void composeSmallToTemplateMapping(const ES::SpMatI &smallToAll, const ES::SpMatI &allToTemplate, ES::SpMatI &smallToTemplate)
{
  smallToTemplate = smallToAll;
  tbb::parallel_for(Eigen::Index(0), smallToAll.nonZeros(), [&](Eigen::Index k) {
    const Eigen::Index oldOffset = smallToAll.valuePtr()[k];
    if (oldOffset < 0 || oldOffset >= allToTemplate.nonZeros())
      throw std::domain_error("Different sparse matrix topology");
    smallToTemplate.valuePtr()[k] = allToTemplate.valuePtr()[oldOffset];
  });
}


void buildEnergySetHessianTemplateRowWise(
  int nAll,
  const ES::SpMatD &hessianAll,
  const std::vector<PotentialEnergy_const_p> &potentialEnergies,
  const std::vector<double> &energyCoeffs,
  const std::vector<std::vector<int>> &energyDOFs,
  const std::vector<ES::SpMatD> &dynamicHessianMatrices,
  ES::SpMatD &hessianTemplate)
{
  using StorageIndex = ES::SpMatD::StorageIndex;

  std::vector<std::vector<StorageIndex>> rowColumns(static_cast<std::size_t>(nAll));

  // ── Parallel row-collection helpers (count → prefix-sum → fill flat → append) ──
  // Both fixed and dynamic collection use the same two-pass pattern to avoid
  // per-row locks during push_back: first count entries per row in parallel
  // (atomics), then prefix-sum into offsets, then fill a flat array in parallel,
  // and finally copy the flat slices into rowColumns.
  //
  // Fixed-term collection ──────────────────────────────────────────────────────

  {
    Profiling::ScopedProfileSection fixedProfile("energy_set.fgh.cache.rebuild_template.row_collect_fixed");

    const auto nRows = static_cast<std::size_t>(nAll);
    const Eigen::Index fixedNnz = hessianAll.nonZeros();

    // Count fixed entries per row (parallel over columns with grain size).
    std::vector<std::atomic<Eigen::Index>> rowCounts(nRows);
    tbb::parallel_for(tbb::blocked_range<Eigen::Index>(0, hessianAll.outerSize(), 256),
      [&](const tbb::blocked_range<Eigen::Index> &range) {
        for (Eigen::Index outeri = range.begin(); outeri < range.end(); ++outeri) {
          for (ES::SpMatD::InnerIterator it(hessianAll, outeri); it; ++it)
            rowCounts[static_cast<std::size_t>(it.row())].fetch_add(1, std::memory_order_relaxed);
        }
      });

    // Prefix-sum counts → offsets.
    std::vector<Eigen::Index> offsets(nRows + 1u, 0);
    for (std::size_t r = 0; r < nRows; ++r)
      offsets[r + 1u] = offsets[r] + rowCounts[r].load(std::memory_order_relaxed);

    // Fill flat array (parallel over columns with grain size, atomic write-positions).
    std::vector<StorageIndex> flatEntries(static_cast<std::size_t>(fixedNnz));
    {
      std::vector<std::atomic<Eigen::Index>> writePos(nRows);
      for (std::size_t r = 0; r < nRows; ++r)
        writePos[r].store(offsets[r], std::memory_order_relaxed);

      tbb::parallel_for(tbb::blocked_range<Eigen::Index>(0, hessianAll.outerSize(), 256),
        [&](const tbb::blocked_range<Eigen::Index> &range) {
          for (Eigen::Index outeri = range.begin(); outeri < range.end(); ++outeri) {
            for (ES::SpMatD::InnerIterator it(hessianAll, outeri); it; ++it) {
              const std::size_t r = static_cast<std::size_t>(it.row());
              const Eigen::Index pos = writePos[r].fetch_add(1, std::memory_order_relaxed);
              flatEntries[static_cast<std::size_t>(pos)] = static_cast<StorageIndex>(it.col());
            }
          }
        });
    }

    // Append flat slices into rowColumns.
    for (std::size_t r = 0; r < nRows; ++r)
      if (rowCounts[r].load(std::memory_order_relaxed) > 0)
        rowColumns[r].insert(rowColumns[r].end(),
          flatEntries.begin() + static_cast<std::ptrdiff_t>(offsets[r]),
          flatEntries.begin() + static_cast<std::ptrdiff_t>(offsets[r + 1u]));
  }

  // Dynamic-term collection ────────────────────────────────────────────────────

  {
    Profiling::ScopedProfileSection dynamicProfile("energy_set.fgh.cache.rebuild_template.row_collect_dynamic");

    const auto nRows = static_cast<std::size_t>(nAll);
    const std::size_t nTerms = potentialEnergies.size();

    // Count dynamic entries per global row.
    // Iterate terms serially (only 1-2 dynamic terms) but parallelize over the
    // columns inside each term — each term has thousands of columns.
    std::vector<std::atomic<Eigen::Index>> dynRowCounts(nRows);
    for (std::size_t i = 0; i < nTerms; ++i) {
      if (energyCoeffs[i] == 0 || potentialEnergies[i]->isHessianTopologyFixed())
        continue;
      const ES::SpMatD &Ki = dynamicHessianMatrices[i];
      const std::vector<int> &dofs = energyDOFs[i];
      tbb::parallel_for(Eigen::Index(0), Ki.outerSize(), [&](Eigen::Index outeri) {
        for (ES::SpMatD::InnerIterator it(Ki, outeri); it; ++it)
          dynRowCounts[static_cast<std::size_t>(dofs[it.row()])].fetch_add(1, std::memory_order_relaxed);
      });
    }

    // Prefix-sum counts → offsets.
    std::vector<Eigen::Index> dynOffsets(nRows + 1u, 0);
    for (std::size_t r = 0; r < nRows; ++r)
      dynOffsets[r + 1u] = dynOffsets[r] + dynRowCounts[r].load(std::memory_order_relaxed);

    const Eigen::Index totalDynNnz = dynOffsets[nRows];

    // Fill flat array.
    // Same pattern as the count pass: serial over terms, parallel over columns.
    std::vector<StorageIndex> dynFlatEntries(static_cast<std::size_t>(totalDynNnz));
    {
      std::vector<std::atomic<Eigen::Index>> dynWritePos(nRows);
      for (std::size_t r = 0; r < nRows; ++r)
        dynWritePos[r].store(dynOffsets[r], std::memory_order_relaxed);

      for (std::size_t i = 0; i < nTerms; ++i) {
        if (energyCoeffs[i] == 0 || potentialEnergies[i]->isHessianTopologyFixed())
          continue;
        const ES::SpMatD &Ki = dynamicHessianMatrices[i];
        const std::vector<int> &dofs = energyDOFs[i];
        tbb::parallel_for(Eigen::Index(0), Ki.outerSize(), [&](Eigen::Index outeri) {
          for (ES::SpMatD::InnerIterator it(Ki, outeri); it; ++it) {
            const std::size_t r = static_cast<std::size_t>(dofs[it.row()]);
            const Eigen::Index pos = dynWritePos[r].fetch_add(1, std::memory_order_relaxed);
            dynFlatEntries[static_cast<std::size_t>(pos)] = static_cast<StorageIndex>(dofs[it.col()]);
          }
        });
      }
    }

    // Append flat slices into rowColumns.
    for (std::size_t r = 0; r < nRows; ++r)
      if (dynRowCounts[r].load(std::memory_order_relaxed) > 0)
        rowColumns[r].insert(rowColumns[r].end(),
          dynFlatEntries.begin() + static_cast<std::ptrdiff_t>(dynOffsets[r]),
          dynFlatEntries.begin() + static_cast<std::ptrdiff_t>(dynOffsets[r + 1u]));
  }

  std::vector<Eigen::Index> rowNonZeros(rowColumns.size(), 0);
  {
    Profiling::ScopedProfileSection sortProfile("energy_set.fgh.cache.rebuild_template.row_sort_unique");
    tbb::parallel_for(std::size_t(0), rowColumns.size(), [&](std::size_t row) {
      auto &columns = rowColumns[row];
      std::sort(columns.begin(), columns.end());
      columns.erase(std::unique(columns.begin(), columns.end()), columns.end());
      rowNonZeros[row] = static_cast<Eigen::Index>(columns.size());
    });
  }
  const Eigen::Index totalNonZeros = std::accumulate(rowNonZeros.begin(), rowNonZeros.end(), Eigen::Index(0));

  {
    Profiling::ScopedProfileSection insertProfile("energy_set.fgh.cache.rebuild_template.row_insert");

    // Build outer-index offsets via prefix sum over per-row nnz.
    std::vector<StorageIndex> outerOffsets(static_cast<std::size_t>(nAll) + 1u);
    outerOffsets[0] = 0;
    for (int row = 0; row < nAll; ++row)
      outerOffsets[static_cast<std::size_t>(row) + 1u] =
        outerOffsets[static_cast<std::size_t>(row)] +
        static_cast<StorageIndex>(rowNonZeros[static_cast<std::size_t>(row)]);

    hessianTemplate.resize(nAll, nAll);
    hessianTemplate.resizeNonZeros(totalNonZeros);

    // When the matrix is already compressed after resizeNonZeros and has no
    // separate inner-nonzero array, fill the CSR arrays directly in parallel.
    // Otherwise fall back to the serial Eigen insert path.
    const bool canDirectFill =
      hessianTemplate.isCompressed() && hessianTemplate.innerNonZeroPtr() == nullptr;

    if (canDirectFill) {
      std::copy(outerOffsets.begin(), outerOffsets.end(), hessianTemplate.outerIndexPtr());

      pgo::parallel::parallelFor(0, nAll,
        pgo::parallel::Options{
          .nestedKernelPolicy = pgo::parallel::NestedKernelPolicy::Inherit,
        },
        [&](int row) {
          const auto &columns = rowColumns[static_cast<std::size_t>(row)];
          StorageIndex offset = outerOffsets[static_cast<std::size_t>(row)];
          for (StorageIndex k = 0; k < static_cast<StorageIndex>(columns.size()); ++k) {
            hessianTemplate.innerIndexPtr()[offset + k] = columns[k];
            hessianTemplate.valuePtr()[offset + k] = 1.0;
          }
        });
    }
    else {
      // Serial fallback: Eigen's standard insert API.
      hessianTemplate.reserve(totalNonZeros);
      for (int row = 0; row < nAll; ++row) {
        hessianTemplate.startVec(row);
        for (StorageIndex col : rowColumns[static_cast<std::size_t>(row)])
          hessianTemplate.insertBackByOuterInner(row, col) = 1.0;
      }
      hessianTemplate.finalize();
    }

    hessianTemplate.makeCompressed();
  }
}

}  // namespace

class EnergySetBuffer
{
public:
  std::vector<double> energyBuffers;

  std::vector<ES::VXd> xlocals;
  std::vector<ES::VXd> vecs;

  std::vector<ES::VXd> gradients;
  std::vector<ES::VXd> hessianVectors;
  std::vector<ES::SpMatD> hessianMatrices;
  std::vector<ES::SpMatD> dynamicHessianMatrices;
  std::vector<ES::TripletD> fixedTemplateEntries;
  DynamicAssemblyCache dynamicAssemblyCache;
};

EnergySet::EnergySet(int numDofs, std::vector<Term> terms)
  : terms_(std::move(terms)), nAll(numDofs)
{
  if (terms_.empty())
    throw std::invalid_argument("EnergySet requires at least one term");

  buffer_ = std::make_shared<EnergySetBuffer>();

  potentialEnergies.reserve(terms_.size());
  energyCoeffs.reserve(terms_.size());
  for (const auto &term : terms_) {
    potentialEnergies.push_back(term.energy);
    energyCoeffs.push_back(term.weight);
  }

  init_();
}

EnergySet::EnergySet(int numDofs, std::vector<Term> terms, std::shared_ptr<EnergySetBuffer> buffer)
  : terms_(std::move(terms)), nAll(numDofs), buffer_(std::move(buffer))
{
  if (terms_.empty())
    throw std::invalid_argument("EnergySet requires at least one term");

  potentialEnergies.reserve(terms_.size());
  energyCoeffs.reserve(terms_.size());
  for (const auto &term : terms_) {
    potentialEnergies.push_back(term.energy);
    energyCoeffs.push_back(term.weight);
  }

  init_();
}

EnergySet::~EnergySet()
{
}

void EnergySet::setWeight(int i, double w)
{
  energyCoeffs.at(i) = w;
  terms_[i].weight = w;
}

void EnergySet::init_()
{
  std::vector<ES::TripletD> entries;
  for (auto energy : potentialEnergies) {
    ES::SpMatD h;

    std::vector<int> dofs;
    energy->getDOFs(dofs);

    if (energy->isHessianTopologyFixed()) {
      energy->hessianAlloc(h);
      entries.reserve(entries.size() + static_cast<std::size_t>(h.nonZeros()));
      for (ES::IDX outeri = 0; outeri < h.outerSize(); ++outeri) {
        for (ES::SpMatD::InnerIterator it(h, outeri); it; ++it) {
          entries.emplace_back(
            static_cast<ES::SpMatD::StorageIndex>(dofs[it.row()]),
            static_cast<ES::SpMatD::StorageIndex>(dofs[it.col()]),
            1.0);
        }
      }
    }

    buffer_->hessianMatrices.push_back(h);
    buffer_->dynamicHessianMatrices.emplace_back();
  }

  hessianAll.resize(nAll, nAll);
  hessianAll.setFromTriplets(entries.begin(), entries.end());

  buffer_->fixedTemplateEntries.clear();
  buffer_->fixedTemplateEntries.reserve(static_cast<std::size_t>(hessianAll.nonZeros()));
  for (Eigen::Index outeri = 0; outeri < hessianAll.outerSize(); ++outeri) {
    for (ES::SpMatD::InnerIterator it(hessianAll, outeri); it; ++it) {
      buffer_->fixedTemplateEntries.emplace_back(
        static_cast<ES::SpMatD::StorageIndex>(it.row()),
        static_cast<ES::SpMatD::StorageIndex>(it.col()),
        1.0);
    }
  }

  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    std::vector<int> dofs;
    potentialEnergies[i]->getDOFs(dofs);

    ES::SpMatI mapping;
    if (potentialEnergies[i]->isHessianTopologyFixed() && buffer_->hessianMatrices[i].nonZeros()) {
      ES::small2Big(buffer_->hessianMatrices[i], hessianAll, dofs, mapping);
    }

    hessianMatrixMappings.push_back(mapping);

    buffer_->xlocals.push_back(ES::VXd::Zero(dofs.size()));
    buffer_->vecs.push_back(ES::VXd::Zero(dofs.size()));
    buffer_->gradients.push_back(ES::VXd::Zero(dofs.size()));
    buffer_->hessianVectors.push_back(ES::VXd::Zero(dofs.size()));
    energyDOFs.emplace_back(std::move(dofs));
  }

  allDOFs.resize(nAll);
  std::iota(allDOFs.begin(), allDOFs.end(), 0);

  isQuadraticEnergy = 1;
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (potentialEnergies[i]->isQuadratic() == 0) {
      isQuadraticEnergy = 0;
      break;
    }
  }

  hasHessianVectorProduct = 1;
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (potentialEnergies[i]->hasHessianVector() == 0) {
      hasHessianVectorProduct = 0;
      break;
    }
  }
}

void EnergySet::mapx(ES::ConstRefVecXd x, const std::vector<int> &dofs, ES::RefVecXd xlocal) const
{
  for (int i = 0; i < static_cast<int>(dofs.size()); i++) {
    xlocal(i) = x(dofs[i]);
  }
}

double EnergySet::func(ES::ConstRefVecXd x) const
{
  double energyAll = 0;
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (energyCoeffs[i] == 0) {
      continue;
    }

    mapx(x, energyDOFs[i], buffer_->xlocals[i]);
    double eng = potentialEnergies[i]->func(buffer_->xlocals[i]) * energyCoeffs[i];
    energyAll += eng;
  }

  return energyAll;
}

void EnergySet::gradient(ES::ConstRefVecXd x, ES::RefVecXd grad) const
{
  grad.setZero();

  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (energyCoeffs[i] == 0) {
      continue;
    }

    mapx(x, energyDOFs[i], buffer_->xlocals[i]);
    potentialEnergies[i]->gradient(buffer_->xlocals[i], buffer_->gradients[i]);

    for (Eigen::Index j = 0; j < buffer_->gradients[i].size(); j++)
      grad[energyDOFs[i][j]] += buffer_->gradients[i][j] * energyCoeffs[i];
  }
}

void EnergySet::hessianInPlace(ES::ConstRefVecXd x, ES::SpMatD &hess) const
{
  std::memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (energyCoeffs[i] == 0) {
      continue;
    }

    if (buffer_->hessianMatrices[i].nonZeros() == 0)
      continue;

    mapx(x, energyDOFs[i], buffer_->xlocals[i]);
    potentialEnergies[i]->hessianInPlace(buffer_->xlocals[i], buffer_->hessianMatrices[i]);

    ES::addSmallToBig(energyCoeffs[i], buffer_->hessianMatrices[i], hess, 1.0, hessianMatrixMappings[i]);
  }
}

void EnergySet::printEnergy(EigenSupport::ConstRefVecXd x) const
{
  prepareEvaluationState(x);
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    mapx(x, energyDOFs[i], buffer_->xlocals[i]);
    double energy = potentialEnergies[i]->func(buffer_->xlocals[i]);
    std::cout << "Energy " << i << ": " << energy << ',' << energy * energyCoeffs[i] << std::endl;
  }
}

void EnergySet::hessianVector(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd vec, EigenSupport::RefVecXd hessVec) const
{
  hessVec.setZero();

  if (hasHessianVectorProduct) {
    for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
      if (energyCoeffs[i] == 0) {
        continue;
      }

      mapx(x, energyDOFs[i], buffer_->xlocals[i]);
      mapx(vec, energyDOFs[i], buffer_->vecs[i]);
      potentialEnergies[i]->hessianVector(buffer_->xlocals[i], buffer_->vecs[i], buffer_->hessianVectors[i]);

      for (Eigen::Index j = 0; j < buffer_->hessianVectors[i].size(); j++)
        hessVec[energyDOFs[i][j]] += buffer_->hessianVectors[i][j] * energyCoeffs[i];
    }
  }
}

int EnergySet::isHessianTopologyFixed() const
{
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (!potentialEnergies[i]->isHessianTopologyFixed())
      return 0;
  }
  return 1;
}

void EnergySet::hessian(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const
{
  hess = hessianAll;
  std::memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (energyCoeffs[i] == 0)
      continue;

    if (buffer_->hessianMatrices[i].nonZeros() == 0)
      continue;

    if (potentialEnergies[i]->isHessianTopologyFixed()) {
      mapx(x, energyDOFs[i], buffer_->xlocals[i]);
      potentialEnergies[i]->hessianInPlace(buffer_->xlocals[i], buffer_->hessianMatrices[i]);
      ES::addSmallToBig(energyCoeffs[i], buffer_->hessianMatrices[i], hess, 1.0, hessianMatrixMappings[i]);
    }
  }

  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (energyCoeffs[i] == 0)
      continue;

    if (!potentialEnergies[i]->isHessianTopologyFixed()) {
      mapx(x, energyDOFs[i], buffer_->xlocals[i]);
      ES::SpMatD Ki;
      potentialEnergies[i]->hessian(buffer_->xlocals[i], Ki);
      if (Ki.nonZeros() == 0)
        continue;

      ES::SpMatD KiGlobal(nAll, nAll);
      std::vector<ES::TripletD> entries;
      entries.reserve(Ki.nonZeros());
      for (Eigen::Index outeri = 0; outeri < Ki.outerSize(); outeri++) {
        for (ES::SpMatD::InnerIterator it(Ki, outeri); it; ++it) {
          entries.emplace_back(
            static_cast<ES::SpMatD::StorageIndex>(energyDOFs[i][it.row()]),
            static_cast<ES::SpMatD::StorageIndex>(energyDOFs[i][it.col()]),
            it.value() * energyCoeffs[i]);
        }
      }
      KiGlobal.setFromTriplets(entries.begin(), entries.end());
      hess = hess + KiGlobal;
    }
  }
}

void EnergySet::gradient_hessian(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad, EigenSupport::SpMatD &hess) const
{
  grad.setZero();

  hess = hessianAll;
  std::memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (energyCoeffs[i] == 0)
      continue;

    mapx(x, energyDOFs[i], buffer_->xlocals[i]);
    buffer_->gradients[i].setZero();

    if (potentialEnergies[i]->isHessianTopologyFixed()) {
      potentialEnergies[i]->gradient(buffer_->xlocals[i], buffer_->gradients[i]);

      if (buffer_->hessianMatrices[i].nonZeros()) {
        potentialEnergies[i]->hessianInPlace(buffer_->xlocals[i], buffer_->hessianMatrices[i]);
        ES::addSmallToBig(energyCoeffs[i], buffer_->hessianMatrices[i], hess, 1.0, hessianMatrixMappings[i]);
      }
    }
    else {
      ES::SpMatD Ki;
      potentialEnergies[i]->gradient_hessian(buffer_->xlocals[i], buffer_->gradients[i], Ki);
      if (Ki.nonZeros()) {
        ES::SpMatD KiGlobal(nAll, nAll);
        std::vector<ES::TripletD> entries;
        entries.reserve(Ki.nonZeros());
        for (Eigen::Index outeri = 0; outeri < Ki.outerSize(); outeri++) {
          for (ES::SpMatD::InnerIterator it(Ki, outeri); it; ++it) {
            entries.emplace_back(
              static_cast<ES::SpMatD::StorageIndex>(energyDOFs[i][it.row()]),
              static_cast<ES::SpMatD::StorageIndex>(energyDOFs[i][it.col()]),
              it.value() * energyCoeffs[i]);
          }
        }
        KiGlobal.setFromTriplets(entries.begin(), entries.end());
        hess = hess + KiGlobal;
      }
    }

    for (Eigen::Index j = 0; j < buffer_->gradients[i].size(); j++)
      grad[energyDOFs[i][j]] += buffer_->gradients[i][j] * energyCoeffs[i];
  }
}

double EnergySet::func_grad_hessian(
  EigenSupport::ConstRefVecXd x,
  EigenSupport::RefVecXd grad,
  EigenSupport::SpMatD &hess) const
{
  Profiling::ScopedProfileSection profile("energy_set.func_grad_hessian");

  auto assembleCachedHessian = [&](ES::SpMatD &cachedHessian) {
    DynamicAssemblyCache &cache = buffer_->dynamicAssemblyCache;

    bool patternsChanged = !cache.valid || cache.termPatterns.size() != potentialEnergies.size() ||
      cache.termMappings.size() != potentialEnergies.size();

    {
      Profiling::ScopedProfileSection checkProfile("energy_set.fgh.cache.check_patterns");
      if (!patternsChanged) {
        for (std::size_t i = 0; i < potentialEnergies.size(); ++i) {
          const ES::SpMatD *termHessian = nullptr;
          if (potentialEnergies[i]->isHessianTopologyFixed()) {
            if (buffer_->hessianMatrices[i].nonZeros())
              termHessian = &buffer_->hessianMatrices[i];
          }
          else {
            if (energyCoeffs[i] != 0 && buffer_->dynamicHessianMatrices[i].nonZeros())
              termHessian = &buffer_->dynamicHessianMatrices[i];
          }

          const bool matches = termHessian != nullptr ?
            patternMatches(*termHessian, cache.termPatterns[i]) :
            emptyPatternMatches(potentialEnergies[i]->getNumDOFs(), potentialEnergies[i]->getNumDOFs(), cache.termPatterns[i]);
          if (!matches) {
            patternsChanged = true;
            break;
          }
        }
      }
    }

    if (patternsChanged) {
      std::vector<PatternSnapshot> currentTermPatterns(potentialEnergies.size());
      {
        Profiling::ScopedProfileSection snapshotProfile("energy_set.fgh.cache.snapshot_patterns");
        for (std::size_t i = 0; i < potentialEnergies.size(); ++i) {
          const ES::SpMatD *termHessian = nullptr;
          if (potentialEnergies[i]->isHessianTopologyFixed()) {
            if (buffer_->hessianMatrices[i].nonZeros())
              termHessian = &buffer_->hessianMatrices[i];
          }
          else {
            if (energyCoeffs[i] != 0 && buffer_->dynamicHessianMatrices[i].nonZeros())
              termHessian = &buffer_->dynamicHessianMatrices[i];
          }

          currentTermPatterns[i] = termHessian != nullptr ?
            snapshotPattern(*termHessian) :
            emptyPattern(potentialEnergies[i]->getNumDOFs(), potentialEnergies[i]->getNumDOFs());
        }
      }

      ES::SpMatD hessianTemplate;
      {
        Profiling::ScopedProfileSection templateProfile("energy_set.fgh.cache.rebuild_template");
        buildEnergySetHessianTemplateRowWise(
          nAll,
          hessianAll,
          potentialEnergies,
          energyCoeffs,
          energyDOFs,
          buffer_->dynamicHessianMatrices,
          hessianTemplate);
      }

      {
        Profiling::ScopedProfileSection mappingProfile("energy_set.fgh.cache.rebuild_mappings");
        ES::SpMatI hessianAllToTemplateMapping;
        {
          Profiling::ScopedProfileSection baseProfile("energy_set.fgh.cache.rebuild_mappings.base");
          if (hessianAll.nonZeros())
            buildSmallToBigMappingFast(hessianAll, hessianTemplate, allDOFs, hessianAllToTemplateMapping);
        }

        cache.termMappings.clear();
        cache.termMappings.resize(potentialEnergies.size());

        {
          Profiling::ScopedProfileSection fixedComposeProfile("energy_set.fgh.cache.rebuild_mappings.fixed_compose");
          for (std::size_t i = 0; i < potentialEnergies.size(); ++i) {
            if (!potentialEnergies[i]->isHessianTopologyFixed())
              continue;
            if (buffer_->hessianMatrices[i].nonZeros())
              composeSmallToTemplateMapping(hessianMatrixMappings[i], hessianAllToTemplateMapping, cache.termMappings[i]);
          }
        }

        {
          Profiling::ScopedProfileSection dynamicProfile("energy_set.fgh.cache.rebuild_mappings.dynamic");
          for (std::size_t i = 0; i < potentialEnergies.size(); ++i) {
            if (potentialEnergies[i]->isHessianTopologyFixed())
              continue;
            if (buffer_->dynamicHessianMatrices[i].nonZeros())
              buildSmallToBigMappingFast(buffer_->dynamicHessianMatrices[i], hessianTemplate, energyDOFs[i], cache.termMappings[i]);
          }
        }
      }

      cache.hessianTemplate = std::move(hessianTemplate);
      cache.hessianTemplate.makeCompressed();
      cache.fullPattern = snapshotPattern(cache.hessianTemplate);
      cache.termPatterns = std::move(currentTermPatterns);
      cache.valid = true;
    }

    {
      Profiling::ScopedProfileSection prepareProfile("energy_set.fgh.cache.prepare_output");
      if (!patternMatches(cachedHessian, cache.fullPattern))
        cachedHessian = cache.hessianTemplate;
      zeroSparseValues(cachedHessian);
    }

    {
      Profiling::ScopedProfileSection fixedProfile("energy_set.fgh.cache.add_fixed");
      for (std::size_t i = 0; i < potentialEnergies.size(); ++i) {
        if (energyCoeffs[i] == 0 || !potentialEnergies[i]->isHessianTopologyFixed())
          continue;
        if (buffer_->hessianMatrices[i].nonZeros())
          ES::addSmallToBig(energyCoeffs[i], buffer_->hessianMatrices[i], cachedHessian, 1.0, cache.termMappings[i], 1);
      }
    }

    {
      Profiling::ScopedProfileSection dynamicProfile("energy_set.fgh.cache.add_dynamic");
      for (std::size_t i = 0; i < potentialEnergies.size(); ++i) {
        if (energyCoeffs[i] == 0 || potentialEnergies[i]->isHessianTopologyFixed())
          continue;
        if (buffer_->dynamicHessianMatrices[i].nonZeros())
          ES::addSmallToBig(energyCoeffs[i], buffer_->dynamicHessianMatrices[i], cachedHessian, 1.0, cache.termMappings[i], 1);
      }
    }

    cachedHessian.makeCompressed();
  };

  // Single fused pass: each term is evaluated once. This matters for
  // non-fixed-topology terms (e.g. IPC contact), where the previous
  // gradient_hessian(...)+func(...) split triggered two active-set builds per
  // call. Mirrors gradient_hessian but accumulates the objective value too.
  double energyAll = 0;
  {
    Profiling::ScopedProfileSection resetProfile("energy_set.fgh.reset_grad_hessian");
    grad.setZero();
  }

  if (buffer_->dynamicHessianMatrices.size() != potentialEnergies.size())
    buffer_->dynamicHessianMatrices.resize(potentialEnergies.size());

  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (energyCoeffs[i] == 0) {
      if (!potentialEnergies[i]->isHessianTopologyFixed())
        buffer_->dynamicHessianMatrices[i].resize(potentialEnergies[i]->getNumDOFs(), potentialEnergies[i]->getNumDOFs());
      continue;
    }

    {
      Profiling::ScopedProfileSection mapProfile("energy_set.fgh.map_x");
      mapx(x, energyDOFs[i], buffer_->xlocals[i]);
    }
    {
      Profiling::ScopedProfileSection clearProfile("energy_set.fgh.clear_local_gradient");
      buffer_->gradients[i].setZero();
    }

    if (potentialEnergies[i]->isHessianTopologyFixed()) {
      // Fixed-topology terms do not build an active set, so func+grad here is cheap.
      {
        Profiling::ScopedProfileSection termProfile("energy_set.fgh.fixed.func_grad");
        energyAll += potentialEnergies[i]->func_grad(buffer_->xlocals[i], buffer_->gradients[i]) * energyCoeffs[i];
      }

      if (buffer_->hessianMatrices[i].nonZeros()) {
        {
          Profiling::ScopedProfileSection hessianProfile("energy_set.fgh.fixed.hessian");
          potentialEnergies[i]->hessianInPlace(buffer_->xlocals[i], buffer_->hessianMatrices[i]);
        }
      }
    }
    else {
      ES::SpMatD &Ki = buffer_->dynamicHessianMatrices[i];
      Ki.resize(0, 0);
      // One call -> one active-set build for this term.
      {
        Profiling::ScopedProfileSection termProfile("energy_set.fgh.dynamic.func_grad_hessian");
        energyAll += potentialEnergies[i]->func_grad_hessian(buffer_->xlocals[i], buffer_->gradients[i], Ki) * energyCoeffs[i];
      }
      {
        Profiling::ScopedProfileSection compressProfile("energy_set.fgh.dynamic.make_compressed");
        Ki.makeCompressed();
      }
    }

    {
      Profiling::ScopedProfileSection gradProfile("energy_set.fgh.add_gradient");
      for (Eigen::Index j = 0; j < buffer_->gradients[i].size(); j++)
        grad[energyDOFs[i][j]] += buffer_->gradients[i][j] * energyCoeffs[i];
    }
  }

  assembleCachedHessian(hess);

  return energyAll;
}

StepConstraint EnergySet::computeMaxStepLimit(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx, StepConstraintSink *sink) const
{
  // Across energies of possibly different sources, keep the binding (min-alpha) one.
  StepConstraint binding = {};
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    mapx(x, energyDOFs[i], buffer_->xlocals[i]);
    mapx(dx, energyDOFs[i], buffer_->vecs[i]);
    const StepConstraint c = potentialEnergies[i]->computeMaxStepLimit(buffer_->xlocals[i], buffer_->vecs[i], sink);
    if (c.alpha < binding.alpha)
      binding = c;
  }
  return binding;
}

void EnergySet::beginStep(const StepState &state)
{
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    auto *aware = dynamic_cast<StepAwareEnergy *>(
      const_cast<PotentialEnergy *>(potentialEnergies[i].get()));
    if (!aware)
      continue;

    StepState localState = state;
    if (state.previousX) {
      mapx(*state.previousX, energyDOFs[i], buffer_->vecs[i]);
      localState.previousX = &buffer_->vecs[i];
    }
    if (state.currentX) {
      mapx(*state.currentX, energyDOFs[i], buffer_->xlocals[i]);
      localState.currentX = &buffer_->xlocals[i];
    }
    aware->beginStep(localState);
  }
}

void EnergySet::beginLineSearch(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx) const
{
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    const auto *aware = dynamic_cast<const LineSearchAwareEnergy *>(potentialEnergies[i].get());
    if (!aware)
      continue;
    mapx(x, energyDOFs[i], buffer_->xlocals[i]);
    mapx(dx, energyDOFs[i], buffer_->vecs[i]);
    aware->beginLineSearch(buffer_->xlocals[i], buffer_->vecs[i]);
  }
}

void EnergySet::prepareEvaluationState(EigenSupport::ConstRefVecXd x) const
{
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    const auto *aware = dynamic_cast<const EvaluationStateAwareEnergy *>(potentialEnergies[i].get());
    if (!aware)
      continue;
    mapx(x, energyDOFs[i], buffer_->xlocals[i]);
    aware->prepareEvaluationState(buffer_->xlocals[i]);
  }
}

void EnergySet::endLineSearch() const
{
  for (const auto &energy : potentialEnergies) {
    if (const auto *aware = dynamic_cast<const LineSearchAwareEnergy *>(energy.get()))
      aware->endLineSearch();
  }
}

EnergyStateKind EnergySet::stateKind() const
{
  EnergyStateKind kind = terms_[0].energy->stateKind();
  for (std::size_t i = 1; i < terms_.size(); i++) {
    if (terms_[i].energy->stateKind() != kind)
      return EnergyStateKind::Generic;
  }
  return kind;
}
}  // namespace pgo::NonlinearOptimization
