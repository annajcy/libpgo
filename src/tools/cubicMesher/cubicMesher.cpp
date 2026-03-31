#include "cubicMesh.h"
#include "generateSurfaceMesh.h"
#include "pgoLogging.h"
#include "triMeshGeo.h"

#include <argparse/argparse.hpp>

#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace {
bool computeNumVoxels(int resolution, int& numVoxels) {
    if (resolution <= 0) {
        return false;
    }

    const int64_t n   = static_cast<int64_t>(resolution);
    const int64_t num = n * n * n;
    if (num > static_cast<int64_t>(std::numeric_limits<int>::max())) {
        return false;
    }

    numVoxels = static_cast<int>(num);
    return true;
}

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

bool writeSurfaceMesh(const pgo::VolumetricMeshes::CubicMesh* cubicMesh, const std::string& outputSurface) {
    std::vector<pgo::EigenSupport::V3d> surfVerts;
    std::vector<std::vector<int>>       surfFaces;
    pgo::VolumetricMeshes::GenerateSurfaceMesh::computeMesh(cubicMesh, surfVerts, surfFaces, true, false);

    std::vector<pgo::Vec3i> triangles;
    triangles.reserve(surfFaces.size());
    for (const auto& face : surfFaces) {
        if (face.size() != 3) {
            SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Surface extraction returned a non-triangle face with {} vertices",
                                face.size());
            return false;
        }
        triangles.emplace_back(face[0], face[1], face[2]);
    }

    pgo::Mesh::TriMeshGeo rawSurface(std::move(surfVerts), std::move(triangles));
    pgo::Mesh::TriMeshGeo surface = pgo::Mesh::removeIsolatedVertices(rawSurface.ref());
    if (surface.save(outputSurface) != true) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Failed to save surface mesh to {}", outputSurface);
        return false;
    }

    return true;
}
}  // namespace

int main(int argc, char* argv[]) {
    argparse::ArgumentParser program("Cubic mesh generator");

    argparse::ArgumentParser uniformCmd("uniform");
    uniformCmd.add_description("Generate a fully filled uniform cubic mesh");
    uniformCmd.add_argument("-r", "--resolution")
        .help("Grid resolution (NxNxN)")
        .required()
        .scan<'i', int>()
        .metavar("N");
    uniformCmd.add_argument("-o", "--output-mesh").help("Output cubic mesh filename (.veg)").required().metavar("PATH");
    uniformCmd.add_argument("-s", "--output-surface")
        .help("Output surface mesh filename (.obj), optional")
        .metavar("PATH");
    uniformCmd.add_argument("--E").help("Young's modulus").default_value(1e6).scan<'g', double>();
    uniformCmd.add_argument("--nu").help("Poisson's ratio").default_value(0.45).scan<'g', double>();
    uniformCmd.add_argument("--density").help("Material density").default_value(1000.0).scan<'g', double>();

    program.add_subparser(uniformCmd);

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }

    pgo::Logging::init();

    if (program.is_subcommand_used(uniformCmd)) {
        const int         resolution = uniformCmd.get<int>("--resolution");
        const double      E          = uniformCmd.get<double>("--E");
        const double      nu         = uniformCmd.get<double>("--nu");
        const double      density    = uniformCmd.get<double>("--density");
        const std::string outputMesh = uniformCmd.get<std::string>("--output-mesh");

        int numVoxels = 0;
        if (computeNumVoxels(resolution, numVoxels) == false) {
            SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(),
                                "Resolution must be positive and small enough that N^3 fits in a signed int. Got N={}",
                                resolution);
            return 1;
        }

        std::vector<int> voxels = buildUniformVoxelIndices(resolution, numVoxels);
        std::unique_ptr<pgo::VolumetricMeshes::CubicMesh> cubicMesh =
            pgo::VolumetricMeshes::CubicMesh::createFromUniformGrid(resolution, voxels, E, nu, density);

        if (cubicMesh == nullptr) {
            SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Failed to create cubic mesh from uniform grid");
            return 1;
        }

        if (cubicMesh->save(outputMesh.c_str()) != 0) {
            SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Failed to save cubic mesh to {}", outputMesh);
            return 1;
        }

        pgo::VolumetricMeshes::CubicMesh loadedMesh(outputMesh.c_str());
        if (loadedMesh.getNumVertices() != cubicMesh->getNumVertices() ||
            loadedMesh.getNumElements() != cubicMesh->getNumElements()) {
            SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Saved mesh reload check failed for {}", outputMesh);
            return 1;
        }

        if (uniformCmd.is_used("--output-surface")) {
            const std::string outputSurface = uniformCmd.get<std::string>("--output-surface");
            if (writeSurfaceMesh(cubicMesh.get(), outputSurface) == false) {
                return 1;
            }
        }

        SPDLOG_LOGGER_INFO(pgo::Logging::lgr(), "Generated cubic mesh: N={}, vertices={}, elements={} -> {}",
                           resolution, cubicMesh->getNumVertices(), cubicMesh->getNumElements(), outputMesh);
    }

    return 0;
}
