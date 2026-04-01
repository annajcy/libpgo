#pragma once

#include "EigenDef.h"

#include <functional>
#include <vector>

namespace pgo {
namespace Contact {

using PosFunction = std::function<void(const EigenSupport::V3d&, EigenSupport::V3d&, int dofStart)>;

int validateAndGetVertexEmbeddingArity(const std::vector<int>& vertexEmbeddingIndices,
                                       const std::vector<double>& vertexEmbeddingWeights, int numSurfaceVertices,
                                       const char* context);

}  // namespace Contact
}  // namespace pgo
