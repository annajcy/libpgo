#pragma once

#include "EigenDef.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <set>
#include <span>
#include <stdexcept>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

// =========================================================================
// DOF mapping primitives — local ↔ global gather/scatter
// =========================================================================
//
// Theory summary:
//   x_e = G_e x          (gather: global → local)
//   ∇_x E += G_e^T ∇_x_e E_e   (scatter: local → global, atomic)
//
// This interface deliberately supports identity-only gather/scatter maps:
// each local DOF maps to exactly one global DOF with unit coefficient. G_e is
// therefore not stored as a matrix; it is represented by partitioning each
// element's local DOFs into DofGroup blocks that map to contiguous global
// ranges by a simple offset. Rotated or otherwise dense element transforms
// (G_e entries with non-unit coefficients) are not supported by this API.

struct DofGroup
{
  int localStart = 0;
  int globalStart = 0;
  int size = 0;

  int globalDof(int localOffset) const { return globalStart + localOffset; }
};

// Expand a DofGroup into a vector of global DOF indices.
inline std::vector<int> globalDofs(const DofGroup &group)
{
  std::vector<int> dofs(group.size);
  for (int i = 0; i < group.size; i++)
    dofs[i] = group.globalDof(i);
  return dofs;
}

// =========================================================================
// Sparse assembly pipeline
// =========================================================================
//
// The global sparse Hessian ∇²_x E = Σ_e G_e^T H_e G_e is stored in Eigen's
// standard CSR format (Eigen::RowMajor).  The DofGroup structure gives it BCSR-like block
// semantics: entries of the same (rowGroup, colGroup) block are contiguous
// in valuePtr[], so the inner assembly loop is
// valuePtr[offsets[r] + c] += ... — block-aware flat-offset indexing
// without per-entry column lookups.
//
// Pipeline:  collectSparseBlockPairs  →  buildSparseMatrixTemplate
//          →  buildAllSparseBlockOffsets

// Identifies a unique (rowGroup, colGroup) pair of global DOF index sets
// for Hessian sparsity deduplication.
struct SparseBlockKey
{
  std::vector<int> rows;
  std::vector<int> cols;

  bool operator<(const SparseBlockKey &other) const
  {
    if (rows != other.rows)
      return rows < other.rows;
    return cols < other.cols;
  }
};

// Build a compressed sparse Hessian template from a deduplicated set of
// SparseBlockKey entries.  Each key contributes a dense block of columns to
// every row it references; duplicates within a row are merged.
void buildSparseMatrixTemplate(int numDOFs,
  const std::set<SparseBlockKey> &blocks, EigenSupport::SpMatD &tmpl);

// Maps a (rowGroup, colGroup) pair of DofGroups to a contiguous block in
// a compressed sparse Hessian matrix.  Precomputed at setup time and stored
// per-element for fast atomic scatter of the local dense Hessian.
struct SparseBlockOffset
{
  int rowLocalStart = 0;
  int colLocalStart = 0;
  int rowSize = 0;
  int colSize = 0;
  std::vector<std::ptrdiff_t> offsets;

  std::ptrdiff_t offset(int localRow, int localCol) const
  {
    return offsets[localRow] + localCol;
  }
};

// Given a DofGroup list and a compressed sparse Hessian template, build a
// vector of SparseBlockOffset entries (one per (rowGroup, colGroup) pair).
// Throws std::runtime_error if the template does not contain a contiguous
// dense block for any (rowGroup, colGroup) pair.
void buildSparseBlockOffsetsForGroups(const EigenSupport::SpMatD &tmpl,
  int localDofs, int globalDofs, const std::vector<DofGroup> &groups,
  std::vector<SparseBlockOffset> &blocks);

struct SparseAssemblyCache
{
  EigenSupport::SpMatD matrixTemplate;
  std::vector<std::vector<SparseBlockOffset>> elementBlockOffsets;
};

// =========================================================================
// DofLayout — abstract interface for gather / scatter / Hessian setup
// =========================================================================

class DofLayout
{
public:
  virtual ~DofLayout() = default;

  virtual int numGlobalDofs() const = 0;
  virtual int numLocalDofs(int ele) const = 0;

  virtual void getDofGroups(int ele, std::vector<DofGroup> &groups) const = 0;

  // === gather / scatterAddGradient (identity-only) ===
  //
  // Built on top of getDofGroups() — the single mapping definition.  They
  // are non-virtual because this interface intentionally supports only the
  // identity index mapping represented by DofGroup. A transformation T_e
  // for rotated element axes requires a separate transform-aware layout API;
  // it cannot be expressed by overriding getDofGroups().
  //
  // Identity-mapping convention: the default implementations assume
  // local[k] = global[globalStart + k] within each DofGroup.  This is
  // sufficient for meshes where local axes align with global axes. Rotated
  // local axes are intentionally outside this layout contract.

  void gather(int ele, std::span<const double> global, std::span<double> local,
    std::vector<DofGroup> &groups) const
  {
    const int expectedLocalDofs = numLocalDofs(ele);
    if (global.size() != static_cast<std::size_t>(numGlobalDofs()))
      throw std::invalid_argument("DofLayout::gather global buffer has unexpected size.");
    if (local.size() != static_cast<std::size_t>(expectedLocalDofs))
      throw std::invalid_argument("DofLayout::gather local buffer has unexpected size.");
    std::fill(local.begin(), local.end(), 0.0);
    getDofGroups(ele, groups);
    for (const DofGroup &group : groups)
      for (int i = 0; i < group.size; i++)
        local[group.localStart + i] = global[group.globalDof(i)];
  }

  void scatterAddGradient(int ele, std::span<const double> local, std::span<double> global,
    std::vector<DofGroup> &groups) const
  {
    if (global.size() != static_cast<std::size_t>(numGlobalDofs()))
      throw std::invalid_argument("DofLayout::scatterAddGradient global buffer has unexpected size.");
    if (local.size() != static_cast<std::size_t>(numLocalDofs(ele)))
      throw std::invalid_argument("DofLayout::scatterAddGradient local buffer has unexpected size.");
    getDofGroups(ele, groups);
    for (const DofGroup &group : groups) {
      for (int i = 0; i < group.size; i++) {
        std::atomic_ref<double> atomicGrad(global[group.globalDof(i)]);
        atomicGrad.fetch_add(local[group.localStart + i]);
      }
    }
  }

  // Build sparse block offsets for element `ele` against the given compressed
  // sparse template.  Uses getDofGroups() and numGlobalDofs() internally.
  void buildSparseBlockOffsets(int ele, const EigenSupport::SpMatD &tmpl,
    std::vector<SparseBlockOffset> &blocks) const
  {
    std::vector<DofGroup> groups;
    getDofGroups(ele, groups);
    buildSparseBlockOffsetsForGroups(tmpl, numLocalDofs(ele), numGlobalDofs(), groups, blocks);
  }

  // Collect all unique (rowGroup, colGroup) global block pairs across elements
  // 0 .. nele-1.  Used by the assembler to build the compressed sparse Hessian
  // sparsity template.
  void collectSparseBlockPairs(int nele,
    std::set<SparseBlockKey> &blocks) const
  {
    std::vector<DofGroup> groups;
    for (int ele = 0; ele < nele; ele++) {
      getDofGroups(ele, groups);
      for (const auto &rowGroup : groups)
        for (const auto &colGroup : groups)
          blocks.insert({ globalDofs(rowGroup), globalDofs(colGroup) });
    }
  }

  // Build block offsets for all elements 0 .. nele-1 against the given template.
  void buildAllSparseBlockOffsets(int nele, const EigenSupport::SpMatD &tmpl,
    std::vector<std::vector<SparseBlockOffset>> &allBlocks) const
  {
    allBlocks.resize(nele);
    for (int ele = 0; ele < nele; ele++)
      buildSparseBlockOffsets(ele, tmpl, allBlocks[ele]);
  }
};

}  // namespace SolidDeformationModel
}  // namespace pgo
