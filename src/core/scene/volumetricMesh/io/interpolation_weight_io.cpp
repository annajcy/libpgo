#include "io/interpolation_weight_io.h"

#include <cstdio>
#include <fstream>
#include <stdexcept>

namespace pgo::VolumetricMeshes::interpolation {

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

}  // namespace pgo::VolumetricMeshes::interpolation
