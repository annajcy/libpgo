#include "contactEnergyUtilities.h"

#include "EigenSupport.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace ES = pgo::EigenSupport;

int pgo::Contact::validateAndGetVertexEmbeddingArity(const std::vector<int>& vertexEmbeddingIndices,
                                                     const std::vector<double>& vertexEmbeddingWeights,
                                                     int numSurfaceVertices, const char* context) {
    if (vertexEmbeddingIndices.size() != vertexEmbeddingWeights.size()) {
        std::ostringstream oss;
        oss << context << ": embedding index/weight size mismatch (surface vertices=" << numSurfaceVertices
            << ", indices size=" << vertexEmbeddingIndices.size()
            << ", weights size=" << vertexEmbeddingWeights.size() << ")";
        throw std::runtime_error(oss.str());
    }

    if (numSurfaceVertices <= 0) {
        std::ostringstream oss;
        oss << context << ": invalid surface vertex count (" << numSurfaceVertices << ")";
        throw std::runtime_error(oss.str());
    }

    if (vertexEmbeddingIndices.size() % static_cast<size_t>(numSurfaceVertices) != 0) {
        std::ostringstream oss;
        oss << context << ": embedding data is not divisible by surface vertex count (surface vertices="
            << numSurfaceVertices << ", indices size=" << vertexEmbeddingIndices.size()
            << ", weights size=" << vertexEmbeddingWeights.size() << ")";
        throw std::runtime_error(oss.str());
    }

    const int embeddingArity = static_cast<int>(vertexEmbeddingIndices.size() / numSurfaceVertices);
    if (embeddingArity < 1) {
        std::ostringstream oss;
        oss << context << ": derived invalid embedding arity " << embeddingArity << " (surface vertices="
            << numSurfaceVertices << ", indices size=" << vertexEmbeddingIndices.size()
            << ", weights size=" << vertexEmbeddingWeights.size() << ")";
        throw std::runtime_error(oss.str());
    }

    return embeddingArity;
}
