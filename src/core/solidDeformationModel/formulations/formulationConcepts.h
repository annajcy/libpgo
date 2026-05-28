#pragma once

#include "deformationFormulations.h"

#include <concepts>

namespace pgo
{
namespace SolidDeformationModel
{

// Compile-time formulation dispatch.
// Each concept accepts exactly the tag objects that are valid for a given topology.

template<class F>
concept TetFormulation = std::same_as<F, TetP1>;

template<class F>
concept CubicFormulation = std::same_as<F, HexTrilinear>;

template<class F>
concept ShellFormulation = std::same_as<F, ShellKoiter>;

}  // namespace SolidDeformationModel
}  // namespace pgo
