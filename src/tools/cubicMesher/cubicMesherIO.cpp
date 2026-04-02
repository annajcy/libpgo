#include "cubicMesherIO.h"

#include "generateSurfaceMesh.h"
#include "pgoLogging.h"
#include "triMeshGeo.h"

#include <utility>
#include <vector>

namespace cubic_mesher {

bool saveAndValidateCubicMesh(const pgo::VolumetricMeshes::CubicMesh& cubicMesh, const std::string& outputMesh) {
    if (cubicMesh.save(outputMesh.c_str()) != 0) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Failed to save cubic mesh to {}", outputMesh);
        return false;
    }

    pgo::VolumetricMeshes::CubicMesh loadedMesh(outputMesh.c_str());
    if (loadedMesh.getNumVertices() != cubicMesh.getNumVertices() ||
        loadedMesh.getNumElements() != cubicMesh.getNumElements()) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Saved mesh reload check failed for {}", outputMesh);
        return false;
    }

    return true;
}

bool writeSurfaceMesh(const pgo::VolumetricMeshes::CubicMesh& cubicMesh, const std::string& outputSurface) {
    std::vector<pgo::EigenSupport::V3d> surfVerts;
    std::vector<std::vector<int>>       surfFaces;
    pgo::VolumetricMeshes::GenerateSurfaceMesh::computeMesh(&cubicMesh, surfVerts, surfFaces, true, false);

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

}  // namespace cubic_mesher
