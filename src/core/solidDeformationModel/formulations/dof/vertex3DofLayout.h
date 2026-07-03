#pragma once

#include "perVertexDofLayout.h"

namespace pgo
{
namespace SolidDeformationModel
{

// DOF layout where each vertex carries 3 displacement DOFs (x, y, z).
using Vertex3DofLayout = PerVertexDofLayout<3>;

}  // namespace SolidDeformationModel
}  // namespace pgo
