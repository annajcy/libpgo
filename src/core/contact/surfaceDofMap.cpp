#include "surfaceDofMap.h"


#include <tbb/enumerable_thread_specific.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace pgo
{
namespace Contact
{

namespace
{

std::uint64_t asCounter(Eigen::Index value)
{
  return static_cast<std::uint64_t>(value);
}

struct RowValue
{
  Eigen::Index col = 0;
  double value = 0.0;
};

EigenSupport::SpMatD::StorageIndex checkedStorageIndexFromUint64(std::uint64_t value, const char *what)
{
  using StorageIndex = EigenSupport::SpMatD::StorageIndex;
  if (value > static_cast<std::uint64_t>(std::numeric_limits<StorageIndex>::max()))
    throw std::overflow_error(std::string("SurfaceDofMap direct sparse fill ") + what +
      " exceeds Eigen sparse storage index range.");
  return static_cast<StorageIndex>(value);
}

EigenSupport::SpMatD::StorageIndex checkedStorageIndexFromEigenIndex(Eigen::Index value, const char *what)
{
  using StorageIndex = EigenSupport::SpMatD::StorageIndex;
  if (value < 0 || value > static_cast<Eigen::Index>(std::numeric_limits<StorageIndex>::max()))
    throw std::overflow_error(std::string("SurfaceDofMap direct sparse fill ") + what +
      " exceeds Eigen sparse storage index range.");
  return static_cast<StorageIndex>(value);
}

Eigen::Index checkedEigenIndexFromUint64(std::uint64_t value, const char *what)
{
  if (value > static_cast<std::uint64_t>(std::numeric_limits<Eigen::Index>::max()))
    throw std::overflow_error(std::string("SurfaceDofMap sparse fill ") + what +
      " exceeds Eigen index range.");
  return static_cast<Eigen::Index>(value);
}

void fillSparseRowsDirect(
  const std::vector<std::vector<RowValue>> &rowBuffers,
  Eigen::Index numRows,
  Eigen::Index numCols,
  std::uint64_t outputNnz,
  EigenSupport::SpMatD &matrix)
{
  using StorageIndex = EigenSupport::SpMatD::StorageIndex;

  (void)checkedStorageIndexFromEigenIndex(numRows, "row count");
  (void)checkedStorageIndexFromEigenIndex(numCols, "column count");
  const Eigen::Index totalNnz = checkedEigenIndexFromUint64(outputNnz, "nonzero count");

  std::vector<StorageIndex> outerOffsets(static_cast<std::size_t>(numRows) + 1u);
  std::uint64_t offset = 0;
  outerOffsets[0] = 0;
  for (Eigen::Index row = 0; row < numRows; ++row) {
    offset += static_cast<std::uint64_t>(rowBuffers[static_cast<std::size_t>(row)].size());
    outerOffsets[static_cast<std::size_t>(row) + 1u] =
      checkedStorageIndexFromUint64(offset, "row offset");
  }
  if (offset != outputNnz)
    throw std::logic_error("SurfaceDofMap direct sparse fill row offsets do not match output nnz.");

  matrix.resize(numRows, numCols);
  matrix.resizeNonZeros(totalNnz);
  std::copy(outerOffsets.begin(), outerOffsets.end(), matrix.outerIndexPtr());

  tbb::parallel_for(tbb::blocked_range<decltype(Eigen::Index{ 0 })>(Eigen::Index{ 0 }, numRows, 1), [pgoBody = [&](Eigen::Index rangeBegin, Eigen::Index rangeEnd) {
    for (Eigen::Index row = rangeBegin; row < rangeEnd; ++row) {
      const std::vector<RowValue> &rowBuffer = rowBuffers[static_cast<std::size_t>(row)];
      StorageIndex offset = outerOffsets[static_cast<std::size_t>(row)];
      for (std::size_t entryIndex = 0; entryIndex < rowBuffer.size(); ++entryIndex) {
        const RowValue &entry = rowBuffer[entryIndex];
        const StorageIndex storageIndex = offset + static_cast<StorageIndex>(entryIndex);
        matrix.innerIndexPtr()[storageIndex] = static_cast<StorageIndex>(entry.col);
        matrix.valuePtr()[storageIndex] = entry.value;
      }
    }
  }](const auto &pgoRange) { pgoBody(pgoRange.begin(), pgoRange.end()); });

  if (!matrix.isCompressed())
    matrix.makeCompressed();
}

void fillSimulationHessianDirect(
  const std::vector<std::vector<RowValue>> &rowBuffers,
  Eigen::Index numOutputRows,
  std::uint64_t outputNnz,
  EigenSupport::SpMatD &simulationHessian)
{
  using StorageIndex = EigenSupport::SpMatD::StorageIndex;

  std::vector<StorageIndex> outerOffsets;
  {

    (void)checkedStorageIndexFromEigenIndex(numOutputRows, "row count");
    const StorageIndex totalNnzStorage = checkedStorageIndexFromUint64(outputNnz, "nonzero count");
    const Eigen::Index totalNnz = checkedEigenIndexFromUint64(outputNnz, "nonzero count");

    outerOffsets.resize(static_cast<std::size_t>(numOutputRows) + 1u);
    std::uint64_t offset = 0;
    outerOffsets[0] = 0;
    for (Eigen::Index row = 0; row < numOutputRows; ++row) {
      offset += static_cast<std::uint64_t>(rowBuffers[static_cast<std::size_t>(row)].size());
      outerOffsets[static_cast<std::size_t>(row) + 1u] =
        checkedStorageIndexFromUint64(offset, "row offset");
    }
    if (offset != outputNnz || outerOffsets.back() != totalNnzStorage)
      throw std::logic_error("SurfaceDofMap direct sparse fill row offsets do not match output nnz.");

    simulationHessian.resize(numOutputRows, numOutputRows);
    simulationHessian.resizeNonZeros(totalNnz);
    std::copy(outerOffsets.begin(), outerOffsets.end(), simulationHessian.outerIndexPtr());

  }

  {
    tbb::parallel_for(tbb::blocked_range<decltype(Eigen::Index{ 0 })>(Eigen::Index{ 0 }, numOutputRows, 1), [pgoBody = [&](Eigen::Index rangeBegin, Eigen::Index rangeEnd) {
      for (Eigen::Index row = rangeBegin; row < rangeEnd; ++row) {
        const std::vector<RowValue> &rowBuffer = rowBuffers[static_cast<std::size_t>(row)];
        StorageIndex offset = outerOffsets[static_cast<std::size_t>(row)];
        for (std::size_t entryIndex = 0; entryIndex < rowBuffer.size(); ++entryIndex) {
          const RowValue &entry = rowBuffer[entryIndex];
          const StorageIndex storageIndex = offset + static_cast<StorageIndex>(entryIndex);
          simulationHessian.innerIndexPtr()[storageIndex] = static_cast<StorageIndex>(entry.col);
          simulationHessian.valuePtr()[storageIndex] = entry.value;
        }
      }
    }](const auto &pgoRange) { pgoBody(pgoRange.begin(), pgoRange.end()); });
  }

  if (!simulationHessian.isCompressed())
    simulationHessian.makeCompressed();
}

}  // namespace

SurfaceDofMap::SurfaceDofMap(
  const EigenSupport::MXd &surfaceRestVertices,
  const EigenSupport::SpMatD &surfaceFromSimulationDispMap):
  surfaceFromSimulationDispMap_(surfaceFromSimulationDispMap)
{
  if (surfaceRestVertices.cols() != 3)
    throw std::invalid_argument("SurfaceDofMap rest vertices must have shape (#vertices, 3).");
  if (surfaceRestVertices.rows() <= 0)
    throw std::invalid_argument("SurfaceDofMap rest vertices must contain at least one vertex.");
  if (surfaceFromSimulationDispMap_.rows() != surfaceRestVertices.rows() * 3)
    throw std::invalid_argument("SurfaceDofMap surface-from-simulation map row count must be 3 * #surface vertices.");
  if (surfaceFromSimulationDispMap_.cols() <= 0)
    throw std::invalid_argument("SurfaceDofMap surface-from-simulation map must contain at least one simulation DOF.");
  surfaceFromSimulationDispMap_.makeCompressed();

  surfaceRestPositions_.resize(surfaceRestVertices.rows() * 3);
  for (Eigen::Index i = 0; i < surfaceRestVertices.rows(); i++)
    surfaceRestPositions_.segment<3>(i * 3) = surfaceRestVertices.row(i).transpose();

  simulationDofs_.resize(surfaceFromSimulationDispMap_.cols());
  std::iota(simulationDofs_.begin(), simulationDofs_.end(), 0);
  surfaceFromSimulationDispMapRowNnzStats_ = computeRowNnzStats(surfaceFromSimulationDispMap_);
  buildSurfaceFromSimulationDispMapRows();
}

SurfaceDofMap::RowNnzStats SurfaceDofMap::computeRowNnzStats(const EigenSupport::SpMatD &matrix)
{
  RowNnzStats stats;
  if (matrix.rows() <= 0)
    return stats;

  stats.min = std::numeric_limits<std::uint64_t>::max();
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    std::uint64_t rowNnz = 0;
    for (EigenSupport::SpMatD::InnerIterator it(matrix, row); it; ++it)
      ++rowNnz;

    stats.min = std::min(stats.min, rowNnz);
    stats.max = std::max(stats.max, rowNnz);
    stats.total += rowNnz;
    if (rowNnz > 0)
      ++stats.nonzeroRows;
  }

  return stats;
}

void SurfaceDofMap::buildSurfaceFromSimulationDispMapRows()
{
  surfaceFromSimulationDispMapRows_.assign(
    static_cast<std::size_t>(surfaceFromSimulationDispMap_.rows()), {});
  simulationToSurfaceDispMapRows_.assign(
    static_cast<std::size_t>(surfaceFromSimulationDispMap_.cols()), {});

  for (Eigen::Index row = 0; row < surfaceFromSimulationDispMap_.rows(); ++row) {
    std::vector<SurfaceMapRowEntry> &rowEntries =
      surfaceFromSimulationDispMapRows_[static_cast<std::size_t>(row)];
    rowEntries.reserve(static_cast<std::size_t>(
      surfaceFromSimulationDispMap_.outerIndexPtr()[row + 1] - surfaceFromSimulationDispMap_.outerIndexPtr()[row]));

    for (EigenSupport::SpMatD::InnerIterator it(surfaceFromSimulationDispMap_, row); it; ++it) {
      rowEntries.push_back(SurfaceMapRowEntry{ it.col(), it.value() });
      simulationToSurfaceDispMapRows_[static_cast<std::size_t>(it.col())].push_back(
        SimulationMapRowEntry{ row, it.value() });
    }
  }
}

void SurfaceDofMap::parallelSurfaceHessianMapMultiply(
  const EigenSupport::SpMatD &surfaceHessian,
  EigenSupport::SpMatD &tmp) const
{
  const Eigen::Index numRows = surfaceHessian.rows();
  const Eigen::Index numCols = surfaceFromSimulationDispMap_.cols();
  std::vector<std::vector<RowValue>> rowBuffers;
  std::vector<std::uint64_t> rowContributionCounts;

  {
    rowBuffers.resize(static_cast<std::size_t>(numRows));
    rowContributionCounts.assign(static_cast<std::size_t>(numRows), 0u);
  }

  {
    tbb::parallel_for(tbb::blocked_range<decltype(Eigen::Index{ 0 })>(Eigen::Index{ 0 }, numRows, 1), [pgoBody = [&](Eigen::Index rangeBegin, Eigen::Index rangeEnd) {
      for (Eigen::Index row = rangeBegin; row < rangeEnd; ++row) {
        std::vector<RowValue> &rowBuffer = rowBuffers[static_cast<std::size_t>(row)];

        std::uint64_t contributionCount = 0;
        for (EigenSupport::SpMatD::InnerIterator it(surfaceHessian, row); it; ++it) {
          contributionCount +=
            static_cast<std::uint64_t>(surfaceFromSimulationDispMapRows_[static_cast<std::size_t>(it.col())].size());
        }

        rowContributionCounts[static_cast<std::size_t>(row)] = contributionCount;
        rowBuffer.clear();
        if (contributionCount == 0)
          continue;

        rowBuffer.reserve(std::max(rowBuffer.capacity(), static_cast<std::size_t>(contributionCount)));
        for (EigenSupport::SpMatD::InnerIterator it(surfaceHessian, row); it; ++it) {
          const double hessianValue = it.value();
          const std::vector<SurfaceMapRowEntry> &mapRow =
            surfaceFromSimulationDispMapRows_[static_cast<std::size_t>(it.col())];
          for (const SurfaceMapRowEntry &entry : mapRow)
            rowBuffer.push_back(RowValue{ entry.simulationCol, hessianValue * entry.weight });
        }

        std::sort(rowBuffer.begin(), rowBuffer.end(),
          [](const RowValue &lhs, const RowValue &rhs) {
            return lhs.col < rhs.col;
          });

        std::size_t writeIndex = 0;
        for (std::size_t readIndex = 0; readIndex < rowBuffer.size();) {
          const Eigen::Index col = rowBuffer[readIndex].col;
          double value = 0.0;
          do {
            value += rowBuffer[readIndex].value;
            ++readIndex;
          } while (readIndex < rowBuffer.size() && rowBuffer[readIndex].col == col);

          rowBuffer[writeIndex++] = RowValue{ col, value };
        }
        rowBuffer.resize(writeIndex);
      }
    }](const auto &pgoRange) { pgoBody(pgoRange.begin(), pgoRange.end()); });
  }

  std::uint64_t outputNnz = 0;
  std::uint64_t contributionCount = 0;
  std::uint64_t activeRows = 0;
  for (Eigen::Index row = 0; row < numRows; ++row) {
    const std::size_t rowIndex = static_cast<std::size_t>(row);
    outputNnz += static_cast<std::uint64_t>(rowBuffers[rowIndex].size());
    contributionCount += rowContributionCounts[rowIndex];
    if (!rowBuffers[rowIndex].empty())
      ++activeRows;
  }

  {
    fillSparseRowsDirect(rowBuffers, numRows, numCols, outputNnz, tmp);
  }

}

void SurfaceDofMap::parallelTransposeMapMultiply(
  const EigenSupport::SpMatD &tmp,
  EigenSupport::SpMatD &simulationHessian) const
{
  struct MergeStream
  {
    Eigen::Index col = 0;
    Eigen::Index index = 0;
    Eigen::Index end = 0;
    double weight = 0.0;
  };

  struct ThreadScratch
  {
    std::vector<MergeStream> streams;
  };

  const auto heapCompare = [](const MergeStream &a, const MergeStream &b) {
    if (a.col != b.col)
      return a.col > b.col;
    return a.index > b.index;
  };

  const Eigen::Index numOutputRows = surfaceFromSimulationDispMap_.cols();
  std::vector<std::vector<RowValue>> rowBuffers;
  std::vector<std::uint64_t> rowContributionCounts;
  std::vector<std::uint64_t> rowMergeStreamCounts;
  std::vector<std::uint8_t> rowWorkspaceReused;
  tbb::enumerable_thread_specific<ThreadScratch> threadScratch;

  {
    {
      rowBuffers.resize(static_cast<std::size_t>(numOutputRows));
      rowContributionCounts.assign(static_cast<std::size_t>(numOutputRows), 0);
      rowMergeStreamCounts.assign(static_cast<std::size_t>(numOutputRows), 0);
      rowWorkspaceReused.assign(static_cast<std::size_t>(numOutputRows), 0);
    }

    {
      tbb::parallel_for(tbb::blocked_range<decltype(Eigen::Index{ 0 })>(Eigen::Index{ 0 }, numOutputRows, 1), [pgoBody = [&](Eigen::Index rangeBegin, Eigen::Index rangeEnd) {
        for (Eigen::Index outputRow = rangeBegin; outputRow < rangeEnd; ++outputRow) {
          const std::vector<SimulationMapRowEntry> &adjacentSurfaceRows =
            simulationToSurfaceDispMapRows_[static_cast<std::size_t>(outputRow)];
          std::vector<RowValue> &rowBuffer = rowBuffers[static_cast<std::size_t>(outputRow)];
          ThreadScratch &scratch = threadScratch.local();
          const std::size_t previousStreamCapacity = scratch.streams.capacity();
          scratch.streams.clear();

          std::uint64_t contributionCount = 0;
          std::size_t maxStreamNnz = 0;
          for (const SimulationMapRowEntry &adjacent : adjacentSurfaceRows) {
            const Eigen::Index begin = tmp.outerIndexPtr()[adjacent.surfaceRow];
            const Eigen::Index end = tmp.outerIndexPtr()[adjacent.surfaceRow + 1];
            const Eigen::Index streamNnz = end - begin;
            contributionCount += asCounter(streamNnz);
            if (streamNnz <= 0)
              continue;

            maxStreamNnz = std::max(maxStreamNnz, static_cast<std::size_t>(streamNnz));
            scratch.streams.push_back(
              MergeStream{ tmp.innerIndexPtr()[begin], begin, end, adjacent.weight });
          }

          rowBuffer.clear();
          const std::size_t streamCount = scratch.streams.size();
          rowContributionCounts[static_cast<std::size_t>(outputRow)] = contributionCount;
          rowMergeStreamCounts[static_cast<std::size_t>(outputRow)] = static_cast<std::uint64_t>(streamCount);
          rowWorkspaceReused[static_cast<std::size_t>(outputRow)] =
            (streamCount > 0 && previousStreamCapacity >= streamCount) ? 1u : 0u;
          if (streamCount == 0)
            continue;

          if (streamCount == 1) {
            const MergeStream &stream = scratch.streams.front();
            rowBuffer.reserve(std::max(rowBuffer.capacity(), static_cast<std::size_t>(stream.end - stream.index)));
            for (Eigen::Index index = stream.index; index < stream.end; ++index)
              rowBuffer.push_back(
                RowValue{ tmp.innerIndexPtr()[index], stream.weight * tmp.valuePtr()[index] });
            continue;
          }

          rowBuffer.reserve(std::max(rowBuffer.capacity(), maxStreamNnz));
          std::make_heap(scratch.streams.begin(), scratch.streams.end(), heapCompare);

          while (!scratch.streams.empty()) {
            std::pop_heap(scratch.streams.begin(), scratch.streams.end(), heapCompare);
            MergeStream stream = scratch.streams.back();
            scratch.streams.pop_back();

            const Eigen::Index col = stream.col;
            double value = 0.0;
            for (;;) {
              value += stream.weight * tmp.valuePtr()[stream.index];
              ++stream.index;
              if (stream.index < stream.end) {
                stream.col = tmp.innerIndexPtr()[stream.index];
                scratch.streams.push_back(stream);
                std::push_heap(scratch.streams.begin(), scratch.streams.end(), heapCompare);
              }

              if (scratch.streams.empty() || scratch.streams.front().col != col)
                break;

              std::pop_heap(scratch.streams.begin(), scratch.streams.end(), heapCompare);
              stream = scratch.streams.back();
              scratch.streams.pop_back();
            }

            rowBuffer.push_back(RowValue{ col, value });
          }
        }
      }](const auto &pgoRange) { pgoBody(pgoRange.begin(), pgoRange.end()); });
    }
  }

  std::uint64_t outputNnz = 0;
  std::uint64_t contributionCount = 0;
  std::uint64_t activeOutputRows = 0;
  std::uint64_t mergeStreamCount = 0;
  std::uint64_t workspaceReusedRows = 0;
  for (Eigen::Index row = 0; row < numOutputRows; ++row) {
    const std::size_t rowIndex = static_cast<std::size_t>(row);
    outputNnz += static_cast<std::uint64_t>(rowBuffers[rowIndex].size());
    contributionCount += rowContributionCounts[rowIndex];
    mergeStreamCount += rowMergeStreamCounts[rowIndex];
    workspaceReusedRows += rowWorkspaceReused[rowIndex];
    if (!rowBuffers[rowIndex].empty())
      ++activeOutputRows;
  }

  {
    fillSimulationHessianDirect(rowBuffers, numOutputRows, outputNnz, simulationHessian);
  }

}

void SurfaceDofMap::validateSimulationDisplacementSize(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  if (simulationDisplacements.size() != surfaceFromSimulationDispMap_.cols())
    throw std::invalid_argument("Simulation displacement size does not match SurfaceDofMap column count.");
}

void SurfaceDofMap::validateSurfaceVectorSize(EigenSupport::ConstRefVecXd surfaceVector) const
{
  if (surfaceVector.size() != surfaceFromSimulationDispMap_.rows())
    throw std::invalid_argument("Surface vector size does not match SurfaceDofMap row count.");
}

void SurfaceDofMap::parallelSurfaceMapVectorMultiply(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd surfaceDisplacements) const
{
  const Eigen::Index numRows = surfaceFromSimulationDispMap_.rows();
  if (surfaceDisplacements.size() != numRows)
    throw std::invalid_argument("Surface displacement output size does not match SurfaceDofMap row count.");

  tbb::parallel_for(tbb::blocked_range<decltype(Eigen::Index{ 0 })>(Eigen::Index{ 0 }, numRows, 1), [pgoBody = [&](Eigen::Index rangeBegin, Eigen::Index rangeEnd) {
    for (Eigen::Index row = rangeBegin; row < rangeEnd; ++row) {
      double value = 0.0;
      for (const SurfaceMapRowEntry &entry : surfaceFromSimulationDispMapRows_[static_cast<std::size_t>(row)])
        value += entry.weight * simulationDisplacements[entry.simulationCol];
      surfaceDisplacements[row] = value;
    }
  }](const auto &pgoRange) { pgoBody(pgoRange.begin(), pgoRange.end()); });
}

void SurfaceDofMap::parallelTransposeMapVectorMultiply(
  EigenSupport::ConstRefVecXd surfaceVector,
  EigenSupport::RefVecXd simulationVector) const
{
  const Eigen::Index numRows = surfaceFromSimulationDispMap_.cols();
  if (simulationVector.size() != numRows)
    throw std::invalid_argument("Simulation vector output size does not match SurfaceDofMap column count.");

  tbb::parallel_for(tbb::blocked_range<decltype(Eigen::Index{ 0 })>(Eigen::Index{ 0 }, numRows, 1), [pgoBody = [&](Eigen::Index rangeBegin, Eigen::Index rangeEnd) {
    for (Eigen::Index row = rangeBegin; row < rangeEnd; ++row) {
      double value = 0.0;
      for (const SimulationMapRowEntry &entry : simulationToSurfaceDispMapRows_[static_cast<std::size_t>(row)])
        value += entry.weight * surfaceVector[entry.surfaceRow];
      simulationVector[row] = value;
    }
  }](const auto &pgoRange) { pgoBody(pgoRange.begin(), pgoRange.end()); });
}

EigenSupport::VXd SurfaceDofMap::surfaceDisplacements(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  validateSimulationDisplacementSize(simulationDisplacements);
  EigenSupport::VXd surfaceDisplacements(surfaceFromSimulationDispMap_.rows());
  parallelSurfaceMapVectorMultiply(simulationDisplacements, surfaceDisplacements);
  return surfaceDisplacements;
}

EigenSupport::VXd SurfaceDofMap::surfacePositions(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  return surfaceRestPositions_ + surfaceDisplacements(simulationDisplacements);
}

EigenSupport::VXd SurfaceDofMap::pullbackGradient(EigenSupport::ConstRefVecXd surfaceGradient) const
{
  validateSurfaceVectorSize(surfaceGradient);
  EigenSupport::VXd simulationGradient(surfaceFromSimulationDispMap_.cols());
  parallelTransposeMapVectorMultiply(surfaceGradient, simulationGradient);
  return simulationGradient;
}

void SurfaceDofMap::pullbackHessian(const EigenSupport::SpMatD &surfaceHessian, EigenSupport::SpMatD &simulationHessian) const
{
  {
    if (surfaceHessian.rows() != surfaceFromSimulationDispMap_.rows() ||
      surfaceHessian.cols() != surfaceFromSimulationDispMap_.rows())
      throw std::invalid_argument("Surface Hessian shape does not match SurfaceDofMap row count.");
  }


  EigenSupport::SpMatD tmp;
  {
    parallelSurfaceHessianMapMultiply(surfaceHessian, tmp);
  }
  tmp.makeCompressed();

  parallelTransposeMapMultiply(tmp, simulationHessian);
}

}  // namespace Contact
}  // namespace pgo
