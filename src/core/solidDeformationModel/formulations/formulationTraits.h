#pragma once

#include "deformationFormulations.h"
#include "basis/tetP1Basis.h"
#include "basis/hexTrilinearBasis.h"
#include "quadrature/tetP1DefaultQuadrature.h"
#include "quadrature/gaussLegendreHexQuadrature.h"
#include "kernels/deformationGradientKernel.h"
#include "elements/deformationGradientElementModel.h"

#include <string_view>

namespace pgo
{
namespace SolidDeformationModel
{

// FormulationTraits<Formulation>
//
// Compile-time metadata + type routing for each formulation tag.
//
// Constraints:
//   - Only DofLayout, Basis, Quadrature, Kernel, ElementModel type aliases.
//   - Metadata (name, node count, local DOFs).
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
  using DofLayout = class Vertex3DofLayout;
  using Basis = TetP1Basis;
  using Quadrature = TetP1DefaultQuadrature;
  using Kernel = DeformationGradientKernel<Basis, Quadrature>;
  using ElementModel = DeformationGradientElementModel<Kernel>;

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
  using DofLayout = class Vertex3DofLayout;
  using Basis = HexTrilinearBasis;
  using Quadrature = GaussLegendreHexQuadrature2;
  using Kernel = DeformationGradientKernel<Basis, Quadrature>;
  using ElementModel = DeformationGradientElementModel<Kernel>;

  static constexpr int nodesPerElement = 8;
  static constexpr int localDofs = 24;
  static constexpr std::string_view name = "hex_trilinear";
};

// ============================================================
// ShellKoiter — routes to existing KoiterDeformationModel path.
// Does not declare volumetric Basis or Quadrature.
// ============================================================

template<>
struct FormulationTraits<ShellKoiter>
{
  using DofLayout = class Vertex3DofLayout;

  static constexpr std::string_view name = "shell_koiter";
};

}  // namespace SolidDeformationModel
}  // namespace pgo
