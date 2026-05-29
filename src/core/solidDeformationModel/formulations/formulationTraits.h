#pragma once

#include "deformationFormulations.h"

#include <string_view>

namespace pgo
{
namespace SolidDeformationModel
{

// FormulationTraits<Formulation>
//
// Compile-time metadata for each formulation tag.
// After virtual-dispatch refactor, Basis/Quadrature/Kernel/ElementModel type
// aliases are removed — construction uses runtime polymorphic Basis/Quadrature.
//
// Constraints:
//   - Metadata (name, node count, local DOFs) only.
//   - No concrete elastic or plastic model types.
//   - No mathematical formula implementation — that lives in kernels / element models.

template<class Formulation>
struct FormulationTraits;

// ============================================================
// TetP1
// ============================================================

template<>
struct FormulationTraits<TetP1>
{
  static constexpr int nodesPerElement = 4;
  static constexpr int localDofs = 12;
  static constexpr std::string_view name = "tet_p1";
};

// ============================================================
// HexTrilinear
// ============================================================

template<>
struct FormulationTraits<HexTrilinear>
{
  static constexpr int nodesPerElement = 8;
  static constexpr int localDofs = 24;
  static constexpr std::string_view name = "hex_trilinear";
};

// ============================================================
// ShellKoiter
//
// Shell-specific stack: stencil-based, no Basis/Quadrature.
// ============================================================

template<>
struct FormulationTraits<ShellKoiter>
{
  static constexpr int nodesPerElement = 6;
  static constexpr int localDofs = 18;
  static constexpr std::string_view name = "shell_koiter";
};

}  // namespace SolidDeformationModel
}  // namespace pgo
