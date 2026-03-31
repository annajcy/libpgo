#include "volumetricMeshInterpolation.h"

#include "EigenSupport.h"

#include <tbb/parallel_for.h>

#include <algorithm>
#include <cfloat>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace pgo::VolumetricMeshes {
namespace interpolation {
namespace detail {

std::vector<int> containing_elements_impl(const VolumetricMesh& mesh, std::span<const Vec3d> targetLocations,
                                          bool useClosestElementIfOutside) {
    std::vector<int> elements(targetLocations.size(), -1);
    for (int i = 0; i < static_cast<int>(targetLocations.size()); i++) {
        int element = mesh.getContainingElement(targetLocations[static_cast<size_t>(i)]);
        if (useClosestElementIfOutside && (element < 0))
            element = mesh.getClosestElement(targetLocations[static_cast<size_t>(i)]);
        elements[static_cast<size_t>(i)] = element;
    }
    return elements;
}

InterpolationWeights generate_weights_impl(const VolumetricMesh& mesh, std::span<const Vec3d> targetLocations,
                                           std::span<const int> elements, double zeroThreshold, int verbose) {
    InterpolationWeights result;
    result.numElementVertices = mesh.getNumElementVertices();
    result.indices.resize(static_cast<size_t>(mesh.getNumElementVertices()) * targetLocations.size());
    result.weights.resize(static_cast<size_t>(mesh.getNumElementVertices()) * targetLocations.size());
    result.elements.assign(elements.begin(), elements.end());

    std::vector<double> barycentricWeights(static_cast<size_t>(mesh.getNumElementVertices()));

    for (int i = 0; i < static_cast<int>(targetLocations.size()); i++) {
        if (verbose && (i % 100 == 0)) {
            printf("%d ", i);
            fflush(nullptr);
        }

        const Vec3d& pos = targetLocations[static_cast<size_t>(i)];
        int element = elements[static_cast<size_t>(i)];
        if (element < 0)
            throw std::runtime_error("Invalid element index in generate_weights.");

        mesh.computeBarycentricWeights(element, pos, barycentricWeights.data());

        if (zeroThreshold > 0) {
            double minDistance = DBL_MAX;
            for (int ii = 0; ii < mesh.getNumElementVertices(); ii++) {
                const Vec3d& vpos = mesh.getVertex(element, ii);
                minDistance = std::min(minDistance, (vpos - pos).norm());
            }
            if (minDistance > zeroThreshold)
                std::fill(barycentricWeights.begin(), barycentricWeights.end(), 0.0);
        }

        for (int ii = 0; ii < mesh.getNumElementVertices(); ii++) {
            result.indices[static_cast<size_t>(mesh.getNumElementVertices() * i + ii)] = mesh.getVertexIndex(element, ii);
            result.weights[static_cast<size_t>(mesh.getNumElementVertices() * i + ii)] = barycentricWeights[static_cast<size_t>(ii)];
        }
    }

    return result;
}

}  // namespace detail

InterpolationWeights generate_weights(const VolumetricMesh& mesh, std::span<const Vec3d> targetLocations,
                                      double zeroThreshold, bool useClosestElementIfOutside, int verbose) {
    const auto elements = detail::containing_elements_impl(mesh, targetLocations, useClosestElementIfOutside);
    return detail::generate_weights_impl(mesh, targetLocations, elements, zeroThreshold, verbose);
}

InterpolationWeights generate_weights(const VolumetricMesh& mesh, std::span<const Vec3d> targetLocations,
                                      std::span<const int> elements, double zeroThreshold, int verbose) {
    return detail::generate_weights_impl(mesh, targetLocations, elements, zeroThreshold, verbose);
}

std::vector<int> containing_elements(const VolumetricMesh& mesh, std::span<const Vec3d> targetLocations,
                                     bool useClosestElementIfOutside, int verbose) {
    (void)verbose;
    return detail::containing_elements_impl(mesh, targetLocations, useClosestElementIfOutside);
}

int get_num_interpolation_element_vertices(const std::filesystem::path& filename) {
    std::ifstream fin(filename);
    if (!fin) {
        printf("Error: unable to open file %s.\n", filename.string().c_str());
        return -1;
    }

    std::string line;
    if (!std::getline(fin, line)) {
        printf("Error: incorrect first line of file %s.\n", filename.string().c_str());
        return -2;
    }

    int count = 0;
    for (char c : line) {
        if (c == ' ')
            count++;
    }
    if (count % 2 == 1) {
        printf("Error: odd number of whitespaces in the first line of file %s.\n", filename.string().c_str());
        return -3;
    }
    return count / 2;
}

InterpolationWeights load_weights(const std::filesystem::path& filename, int numTargetLocations, int numElementVertices) {
    std::ifstream fin(filename);
    if (!fin)
        throw std::runtime_error("Unable to open interpolation weights file.");

    InterpolationWeights result;
    result.numElementVertices = numElementVertices;
    result.indices.resize(static_cast<size_t>(numTargetLocations * numElementVertices));
    result.weights.resize(static_cast<size_t>(numTargetLocations * numElementVertices));

    for (int currentVertex = 0; currentVertex < numTargetLocations; currentVertex++) {
        int vertexIndex = -1;
        fin >> vertexIndex;
        if (!fin || vertexIndex != currentVertex)
            throw std::runtime_error("Invalid interpolation weight file format.");

        for (int j = 0; j < numElementVertices; j++) {
            fin >> result.indices[static_cast<size_t>(currentVertex * numElementVertices + j)]
                >> result.weights[static_cast<size_t>(currentVertex * numElementVertices + j)];
            if (!fin)
                throw std::runtime_error("Invalid interpolation weight file format.");
        }
    }

    return result;
}

InterpolationWeights load_weights_binary(const std::filesystem::path& filename) {
    std::ifstream fin(filename, std::ios::binary);
    if (!fin)
        throw std::runtime_error("Unable to open interpolation weights file.");

    int buffer[2];
    fin.read(reinterpret_cast<char*>(buffer), sizeof(buffer));
    if (!fin)
        throw std::runtime_error("Invalid binary interpolation weight header.");

    InterpolationWeights result;
    const int numTargetLocations = buffer[0];
    result.numElementVertices = buffer[1];
    result.indices.resize(static_cast<size_t>(numTargetLocations * result.numElementVertices));
    result.weights.resize(static_cast<size_t>(numTargetLocations * result.numElementVertices));

    fin.read(reinterpret_cast<char*>(result.indices.data()), sizeof(int) * result.indices.size());
    fin.read(reinterpret_cast<char*>(result.weights.data()), sizeof(double) * result.weights.size());
    if (!fin)
        throw std::runtime_error("Invalid binary interpolation weight payload.");

    return result;
}

int save_weights(const std::filesystem::path& filename, const InterpolationWeights& interpolation) {
    std::ofstream fout(filename);
    if (!fout) {
        printf("Error: unable to open file %s.\n", filename.string().c_str());
        return 1;
    }

    const int numTargetLocations = static_cast<int>(interpolation.indices.size()) / interpolation.numElementVertices;
    for (int currentVertex = 0; currentVertex < numTargetLocations; currentVertex++) {
        fout << currentVertex;
        for (int j = 0; j < interpolation.numElementVertices; j++) {
            fout << ' ' << interpolation.indices[static_cast<size_t>(currentVertex * interpolation.numElementVertices + j)]
                 << ' ' << interpolation.weights[static_cast<size_t>(currentVertex * interpolation.numElementVertices + j)];
        }
        fout << '\n';
    }
    return 0;
}

int save_weights_binary(const std::filesystem::path& filename, const InterpolationWeights& interpolation) {
    std::ofstream fout(filename, std::ios::binary);
    if (!fout) {
        printf("Error: unable to open file %s.\n", filename.string().c_str());
        return 1;
    }

    const int numTargetLocations = static_cast<int>(interpolation.indices.size()) / interpolation.numElementVertices;
    int buffer[2] = {numTargetLocations, interpolation.numElementVertices};
    fout.write(reinterpret_cast<const char*>(buffer), sizeof(buffer));
    fout.write(reinterpret_cast<const char*>(interpolation.indices.data()), sizeof(int) * interpolation.indices.size());
    fout.write(reinterpret_cast<const char*>(interpolation.weights.data()),
               sizeof(double) * interpolation.weights.size());
    return fout ? 0 : 1;
}

void apply(const double* u, double* uTarget, int numTargetLocations, int numElementVertices, const int* vertices,
           const double* weights) {
    tbb::parallel_for(0, numTargetLocations, [&](int i) {
        Vec3d defo(0, 0, 0);
        for (int j = 0; j < numElementVertices; j++) {
            int volumetricMeshVertexIndex = vertices[numElementVertices * i + j];
            defo += weights[numElementVertices * i + j] * asVec3d(u + 3 * volumetricMeshVertexIndex);
        }
        (Eigen::Map<Vec3d>(uTarget + 3 * i)) = defo;
    });
}

}  // namespace interpolation
}  // namespace pgo::VolumetricMeshes
