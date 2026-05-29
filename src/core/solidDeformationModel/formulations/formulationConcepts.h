#pragma once

#include "deformationFormulations.h"
#include "formulationTraits.h"

#include <concepts>

namespace pgo
{
namespace SolidDeformationModel
{

// ============================================================
// Per-tag concepts: which formulation tags are valid for a given mesh type.
// Used as template constraints on topology-specific factory functions.
// Supports || extension (e.g. CubicFormulation = HexTrilinear || HexTricubicHermite).
// ============================================================

template<class F>
concept TetFormulation = std::same_as<F, TetP1>;

template<class F>
concept CubicFormulation = std::same_as<F, HexTrilinear>;

template<class F>
concept ShellFormulation = std::same_as<F, ShellKoiter>;

// ============================================================
// Per-category concepts: what alias set does this formulation category provide.
// Used inside std::visit / if constexpr dispatch at runtime boundaries.
// ============================================================

template<class F>
concept VolumetricFormulationCategory = requires {
  typename FormulationTraits<F>::Basis;
  typename FormulationTraits<F>::Quadrature;
  typename FormulationTraits<F>::Kernel;
  typename FormulationTraits<F>::ElementModel;
};

template<class F>
concept ShellFormulationCategory = !VolumetricFormulationCategory<F> && requires {
  typename FormulationTraits<F>::Kernel;
  typename FormulationTraits<F>::ElementModel;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
