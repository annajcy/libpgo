#pragma once

#include "deformationFormulations.h"

#include <variant>

namespace pgo
{
namespace SolidDeformationModel
{

// Runtime boundary variants — used at Python / config / CLI dispatch boundaries.
// Core C++ code calls the constrained template overloads directly.

using TetFormulationVariant = std::variant<TetP1>;
using CubicFormulationVariant = std::variant<HexTrilinear>;
using ShellFormulationVariant = std::variant<ShellKoiter>;

}  // namespace SolidDeformationModel
}  // namespace pgo
