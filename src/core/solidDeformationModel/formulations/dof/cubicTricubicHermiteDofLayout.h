#pragma once

#include "perVertexDofLayout.h"

namespace pgo
{
namespace SolidDeformationModel
{

// Regular-grid tricubic Hermite layout: 8 Hermite modes x 3 coords per vertex.
using CubicTricubicHermiteDofLayout = PerVertexDofLayout<24>;

}  // namespace SolidDeformationModel
}  // namespace pgo
