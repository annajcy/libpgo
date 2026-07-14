/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "formulations/dof/dofLayout.h"
#include "parallel/parallelFor.h"
#include "EigenSupport.h"

#include <algorithm>
#include <numeric>
#include <stdexcept>


namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

void buildHessianBlockOffsetsForGroups(const EigenSupport::SpMatD &tmpl,
  int localDofs, int globalDofs, const std::vector<DofGroup> &groups,
  std::vector<HessianBlockOffset> &blocks)
{
  blocks.clear();
  blocks.reserve(groups.size() * groups.size());

  for (const auto &group : groups) {
    if (group.size <= 0 || group.localStart < 0 || group.localStart + group.size > localDofs)
      throw std::runtime_error("Invalid Hessian DOF group");
    if (group.globalStart < 0 || group.globalStart + group.size > globalDofs)
      throw std::runtime_error("Invalid Hessian DOF group");
  }

  for (const auto &rowGroup : groups) {
    for (const auto &colGroup : groups) {
      HessianBlockOffset block;
      block.rowLocalStart = rowGroup.localStart;
      block.colLocalStart = colGroup.localStart;
      block.rowSize = rowGroup.size;
      block.colSize = colGroup.size;
      std::vector<std::ptrdiff_t> rowBases(rowGroup.size);

      for (int r = 0; r < rowGroup.size; r++) {
        const std::ptrdiff_t base = ES::findEntryOffset(tmpl, rowGroup.globalDof(r), colGroup.globalStart);
        if (base < 0)
          throw std::runtime_error("Hessian block offset is missing");
        rowBases[r] = base;
        // Verify BCSR block-row contiguity: entries of the same
        // (rowGroup, colGroup) block must be stored consecutively in
        // valuePtr[] so that valuePtr[base + c] addresses column c
        // without a per-entry column-index lookup.
        for (int c = 1; c < colGroup.size; c++) {
          const std::ptrdiff_t offset = ES::findEntryOffset(tmpl, rowGroup.globalDof(r), colGroup.globalDof(c));
          if (offset != base + c)
            throw std::runtime_error("Hessian block offsets must be contiguous");
        }
      }

      block.offsets = std::move(rowBases);
      blocks.push_back(std::move(block));
    }
  }
}

void buildCompressedHessianTemplate(int numDOFs,
  const std::set<HessianBlockKey> &blocks, EigenSupport::SpMatD &tmpl)
{
  std::vector<std::vector<int>> rowColumns(numDOFs);
  for (const auto &block : blocks) {
    for (int row : block.rows) {
      auto &columns = rowColumns[row];
      columns.insert(columns.end(), block.cols.begin(), block.cols.end());
    }
  }

  std::vector<Eigen::Index> rowNonZeros(rowColumns.size(), 0);
  pgo::parallel::parallelFor(std::size_t(0), rowColumns.size(), [&](std::size_t row) {
    auto &columns = rowColumns[row];
    std::sort(columns.begin(), columns.end());
    columns.erase(std::unique(columns.begin(), columns.end()), columns.end());
    rowNonZeros[row] = static_cast<Eigen::Index>(columns.size());
  });
  const Eigen::Index totalNonZeros = std::accumulate(rowNonZeros.begin(), rowNonZeros.end(), Eigen::Index(0));

  tmpl.resize(numDOFs, numDOFs);
  tmpl.reserve(totalNonZeros);
  for (int row = 0; row < numDOFs; row++) {
    tmpl.startVec(row);
    for (int col : rowColumns[row])
      tmpl.insertBackByOuterInner(row, col) = 1.0;
  }
  tmpl.finalize();
  tmpl.makeCompressed();
}

}  // namespace SolidDeformationModel
}  // namespace pgo
