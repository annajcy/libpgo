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
// Per-category concepts: which top-level category a formulation belongs to.
// Volumetric formulations use DeformationGradientElementModel (Basis + Quadrature).
// Shell formulations use KoiterShellElementModel (stencil-based).
// ============================================================

template<class F>
concept VolumetricFormulationCategory = TetFormulation<F> || CubicFormulation<F>;

template<class F>
concept ShellFormulationCategory = ShellFormulation<F>;

}  // namespace SolidDeformationModel
}  // namespace pgo
