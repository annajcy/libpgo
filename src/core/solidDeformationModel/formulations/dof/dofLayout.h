#pragma once

#include "EigenDef.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <set>
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
// G_e is not stored as a matrix.  It is implemented by partitioning each
// element's local DOFs into vertex-aligned DofGroup blocks that map to
// contiguous global ranges by a simple offset.

// Note: DynamicIndexMatrix is defined here (instead of the assembler header)
// so DofLayout can declare it without a circular dependency.
using DynamicIndexMatrix = Eigen::Matrix<std::ptrdiff_t, Eigen::Dynamic, Eigen::Dynamic>;

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
// Hessian sparsity pipeline
// =========================================================================
//
// The global sparse Hessian ∇²_x E = Σ_e G_e^T H_e G_e is stored in Eigen's
// standard CSR format (Eigen::RowMajor).  The DofGroup structure gives it BCSR-like block
// semantics: entries of the same (rowGroup, colGroup) block are contiguous
// in valuePtr[], so the inner assembly loop is
// valuePtr[offsets[r] + c] += ... — block-aware flat-offset indexing
// without per-entry column lookups.
//
// Pipeline:  collectHessianBlockPairs  →  buildCompressedHessianTemplate
//          →  buildAllHessianBlockOffsets

// Identifies a unique (rowGroup, colGroup) pair of global DOF index sets
// for Hessian sparsity deduplication.
struct HessianBlockKey
{
  std::vector<int> rows;
  std::vector<int> cols;

  bool operator<(const HessianBlockKey &other) const
  {
    if (rows != other.rows)
      return rows < other.rows;
    return cols < other.cols;
  }
};

// Build a compressed sparse Hessian template from a deduplicated set of
// HessianBlockKey entries.  Each key contributes a dense block of columns to
// every row it references; duplicates within a row are merged.
void buildCompressedHessianTemplate(int numDOFs,
  const std::set<HessianBlockKey> &blocks, EigenSupport::SpMatD &tmpl);

// Maps a (rowGroup, colGroup) pair of DofGroups to a contiguous block in
// a compressed sparse Hessian matrix.  Precomputed at setup time and stored
// per-element for fast atomic scatter of the local dense Hessian.
struct HessianBlockOffset
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
// vector of HessianBlockOffset entries (one per (rowGroup, colGroup) pair).
// Throws std::runtime_error if the template does not contain a contiguous
// dense block for any (rowGroup, colGroup) pair.
void buildHessianBlockOffsetsForGroups(const EigenSupport::SpMatD &tmpl,
  int localDofs, int globalDofs, const std::vector<DofGroup> &groups,
  std::vector<HessianBlockOffset> &blocks);

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

  // === gather / scatterAddGradient (non-virtual) ===
  //
  // Built on top of getDofGroups() — the single mapping definition.  They
  // are non-virtual because their semantics are fully determined by the
  // DofGroup layout.  A subclass that needs a different local↔global
  // mapping (e.g. a transformation T_e for rotated element axes) should
  // override getDofGroups, not these.
  //
  // Identity-mapping convention: the default implementations assume
  // local[k] = global[globalStart + k] within each DofGroup.  This is
  // sufficient for regular-grid meshes where local axes align with global
  // axes.  A general unstructured mesh with rotated local axes would need a
  // new subclass that overrides getDofGroups to apply T_e.

  void gather(int ele, const double *global, double *local,
    std::vector<DofGroup> &groups) const
  {
    std::fill(local, local + numLocalDofs(ele), 0.0);
    getDofGroups(ele, groups);
    for (const DofGroup &group : groups)
      for (int i = 0; i < group.size; i++)
        local[group.localStart + i] = global[group.globalDof(i)];
  }

  void scatterAddGradient(int ele, const double *local, double *global,
    std::vector<DofGroup> &groups) const
  {
    getDofGroups(ele, groups);
    for (const DofGroup &group : groups) {
      for (int i = 0; i < group.size; i++) {
        std::atomic_ref<double> atomicGrad(global[group.globalDof(i)]);
        atomicGrad.fetch_add(local[group.localStart + i]);
      }
    }
  }

  // Build Hessian block offsets for element `ele` against the given compressed
  // sparse template.  Uses getDofGroups() and numGlobalDofs() internally.
  void buildHessianBlockOffsets(int ele, const EigenSupport::SpMatD &tmpl,
    std::vector<HessianBlockOffset> &blocks) const
  {
    std::vector<DofGroup> groups;
    getDofGroups(ele, groups);
    buildHessianBlockOffsetsForGroups(tmpl, numLocalDofs(ele), numGlobalDofs(), groups, blocks);
  }

  // Collect all unique (rowGroup, colGroup) global block pairs across elements
  // 0 .. nele-1.  Used by the assembler to build the compressed sparse Hessian
  // sparsity template.
  void collectHessianBlockPairs(int nele,
    std::set<HessianBlockKey> &blocks) const
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
  void buildAllHessianBlockOffsets(int nele, const EigenSupport::SpMatD &tmpl,
    std::vector<std::vector<HessianBlockOffset>> &allBlocks) const
  {
    allBlocks.resize(nele);
    for (int ele = 0; ele < nele; ele++)
      buildHessianBlockOffsets(ele, tmpl, allBlocks[ele]);
  }
};

}  // namespace SolidDeformationModel
}  // namespace pgo
