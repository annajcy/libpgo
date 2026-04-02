#include "uniformCubicMeshGenerator.h"

#include "cubicMesherUtils.h"
#include "pgoLogging.h"

#include <memory>
#include <vector>

namespace cubic_mesher {
namespace {

std::vector<int> buildUniformVoxelIndices(int resolution, int numVoxels) {
    std::vector<int> voxels(static_cast<size_t>(numVoxels) * 3u);
    int              idx = 0;

    for (int i = 0; i < resolution; i++) {
        for (int j = 0; j < resolution; j++) {
            for (int k = 0; k < resolution; k++) {
                voxels[idx * 3 + 0] = i;
                voxels[idx * 3 + 1] = j;
                voxels[idx * 3 + 2] = k;
                idx++;
            }
        }
    }

    return voxels;
}

bool applyUniformTransform(pgo::VolumetricMeshes::CubicMesh& cubicMesh, double size, const std::array<double, 3>& offset) {
    if (size <= 0.0) {
        return false;
    }

    if (size == 1.0 && offset[0] == 0.0 && offset[1] == 0.0 && offset[2] == 0.0) {
        return true;
    }

    double translation[3] = {offset[0], offset[1], offset[2]};
    double transform[9]   = {size, 0.0, 0.0, 0.0, size, 0.0, 0.0, 0.0, size};
    cubicMesh.applyLinearTransformation(translation, transform);
    return true;
}

}  // namespace

std::unique_ptr<pgo::VolumetricMeshes::CubicMesh> createUniformCubicMesh(const UniformCubicMeshOptions& options) {
    int numVoxels = 0;
    if (cubic_mesher::computeNumVoxels(options.resolution, numVoxels) == false) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(),
                            "Resolution must be positive and small enough that N^3 fits in a signed int. Got N={}",
                            options.resolution);
        return nullptr;
    }

    std::vector<int> voxels = buildUniformVoxelIndices(options.resolution, numVoxels);
    auto cubicMesh = std::unique_ptr<pgo::VolumetricMeshes::CubicMesh>(pgo::VolumetricMeshes::CubicMesh::createFromUniformGrid(
        options.resolution, numVoxels, voxels.data(), options.E, options.nu, options.density));

    if (cubicMesh == nullptr) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Failed to create cubic mesh from uniform grid");
        return nullptr;
    }

    if (applyUniformTransform(*cubicMesh, options.size, options.offset) == false) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Uniform transform requires positive size. Got size={}",
                            options.size);
        return nullptr;
    }

    return cubicMesh;
}

}  // namespace cubic_mesher
