#include "triangleMeshVoxelizer.h"

#include "arrayRef.h"
#include "boundingVolumeTree.h"
#include "pgoLogging.h"
#include "predicates.h"
#include "triMeshGeo.h"
#include "triMeshNeighbor.h"

#if defined(PGO_HAS_CGAL)
#include "cgalBasic.h"
#include "cgalInterface.h"

#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Surface_mesh.h>
#include <boost/graph/graph_traits.hpp>
#endif

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace cubic_mesher {
namespace {

#if !defined(PGO_HAS_CGAL)
constexpr char kGeometryStackUnsupportedMessage[] = "geometry stack is not supported.";
#endif

#if defined(PGO_HAS_CGAL)
struct VoxelDomain {
    pgo::Mesh::BoundingBox worldBox;
    int                    resolution = 0;
    double                 voxelSide  = 0.0;

    pgo::Vec3d voxelMin(int i, int j, int k) const {
        return worldBox.bmin() + voxelSide * pgo::Vec3d(static_cast<double>(i), static_cast<double>(j),
                                                        static_cast<double>(k));
    }

    pgo::Vec3d voxelCenter(int i, int j, int k) const {
        return voxelMin(i, j, k) + pgo::Vec3d::Constant(0.5 * voxelSide);
    }

    pgo::Mesh::BoundingBox voxelBoundingBox(int i, int j, int k) const {
        const pgo::Vec3d bmin = voxelMin(i, j, k);
        const pgo::Vec3d bmax = bmin + pgo::Vec3d::Constant(voxelSide);
        return pgo::Mesh::BoundingBox(bmin, bmax);
    }

    double side() const { return worldBox.sides()[0]; }
};

std::string toLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

bool applyDomainTransform(pgo::VolumetricMeshes::CubicMesh& cubicMesh, const VoxelDomain& domain) {
    if (domain.side() <= 0.0) {
        return false;
    }

    double translation[3] = {domain.worldBox.center()[0], domain.worldBox.center()[1], domain.worldBox.center()[2]};
    double transform[9] = {
        domain.side(), 0.0, 0.0, 0.0, domain.side(), 0.0, 0.0, 0.0, domain.side(),
    };
    cubicMesh.applyLinearTransformation(translation, transform);
    return true;
}

bool applyUserTransform(pgo::VolumetricMeshes::CubicMesh& cubicMesh, double scale, const std::array<double, 3>& offset) {
    if (scale <= 0.0) {
        return false;
    }

    if (scale == 1.0 && offset[0] == 0.0 && offset[1] == 0.0 && offset[2] == 0.0) {
        return true;
    }

    double translation[3] = {offset[0], offset[1], offset[2]};
    double transform[9]   = {scale, 0.0, 0.0, 0.0, scale, 0.0, 0.0, 0.0, scale};
    cubicMesh.applyLinearTransformation(translation, transform);
    return true;
}

bool loadAndSanitizeTriangleMesh(const std::string& inputMeshPath, pgo::Mesh::TriMeshGeo& mesh) {
    const std::filesystem::path meshPath(inputMeshPath);
    if (toLower(meshPath.extension().string()) != ".obj") {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "triangle-mesh currently only supports .obj input. Got '{}'",
                            inputMeshPath);
        return false;
    }

    pgo::Mesh::TriMeshGeo loadedMesh;
    if (loadedMesh.load(inputMeshPath) == false) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Failed to load triangle mesh from {}", inputMeshPath);
        return false;
    }

    const int inputVertices  = loadedMesh.numVertices();
    const int inputTriangles = loadedMesh.numTriangles();

    pgo::Mesh::TriMeshGeo withoutInvalidTriangles = pgo::Mesh::removeInvalidTriangles(loadedMesh.ref());
    mesh                                           = pgo::Mesh::removeIsolatedVertices(withoutInvalidTriangles.ref());

    if (mesh.numTriangles() == 0) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Triangle mesh '{}' has no valid triangles after cleanup",
                            inputMeshPath);
        return false;
    }

    if (mesh.numVertices() != inputVertices || mesh.numTriangles() != inputTriangles) {
        SPDLOG_LOGGER_WARN(pgo::Logging::lgr(),
                           "Triangle mesh cleanup removed invalid primitives: vertices {} -> {}, triangles {} -> {}",
                           inputVertices, mesh.numVertices(), inputTriangles, mesh.numTriangles());
    }

    return true;
}

bool validateTriangleMeshForVoxelization(const pgo::Mesh::TriMeshGeo& mesh, const std::string& inputMeshPath) {
    if (pgo::CGALInterface::isManifold(mesh) == false) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(),
                            "Triangle mesh '{}' must be manifold before voxelization",
                            inputMeshPath);
        return false;
    }

    const auto exteriorEdges = pgo::Mesh::getExteriorEdges(pgo::BasicAlgorithms::makeArrayRef(mesh.triangles()));
    if (exteriorEdges.empty() == false) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(),
                            "Triangle mesh '{}' must be closed before voxelization. Found {} exterior edges",
                            inputMeshPath, exteriorEdges.size());
        return false;
    }

    if (pgo::CGALInterface::isSelfIntersected(mesh)) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(),
                            "Triangle mesh '{}' has self intersections and cannot be voxelized safely",
                            inputMeshPath);
        return false;
    }

    return true;
}

bool buildVoxelDomain(const pgo::Mesh::TriMeshGeo& mesh, int resolution, int paddingVoxels, VoxelDomain& domain) {
    if (resolution <= 0) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Resolution must be positive. Got N={}", resolution);
        return false;
    }

    if (paddingVoxels < 0) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Padding voxels must be non-negative. Got K={}", paddingVoxels);
        return false;
    }

    const int interiorResolution = resolution - 2 * paddingVoxels;
    if (interiorResolution <= 0) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(),
                            "Resolution must be larger than twice the padding. Got N={}, padding={}", resolution,
                            paddingVoxels);
        return false;
    }

    pgo::Mesh::BoundingBox baseBox = mesh.ref().computeTriangleBoundingBox();
    baseBox.regularize();

    const double baseSide = baseBox.sides()[0];
    if (baseSide <= 0.0) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(),
                            "Triangle mesh bounding box is degenerate and cannot define a cubic voxelization domain");
        return false;
    }

    domain.resolution = resolution;
    domain.voxelSide  = baseSide / static_cast<double>(interiorResolution);

    const double     domainSide = domain.voxelSide * static_cast<double>(resolution);
    const pgo::Vec3d halfSide   = pgo::Vec3d::Constant(0.5 * domainSide);
    domain.worldBox             = pgo::Mesh::BoundingBox(baseBox.center() - halfSide, baseBox.center() + halfSide);

    return true;
}
bool voxelIntersectsTriangleSurface(const pgo::Mesh::TriMeshGeo& mesh, const pgo::Mesh::BoundingBox& voxelBox,
                                    const std::vector<pgo::Mesh::BoundingBox>& triangleBoundingBoxes,
                                    const pgo::Mesh::BoundingBoxBVTree&        triangleBoxTree,
                                    std::vector<int>&                          candidateTriangleIDs) {
    candidateTriangleIDs.clear();
    triangleBoxTree.intersectAABB(pgo::BasicAlgorithms::makeArrayRef(triangleBoundingBoxes), voxelBox,
                                  candidateTriangleIDs);

    for (int triID : candidateTriangleIDs) {
        if (pgo::Mesh::intersectTriAABB(mesh.pos(triID, 0).data(), mesh.pos(triID, 1).data(),
                                        mesh.pos(triID, 2).data(), voxelBox.bmin().data(), voxelBox.bmax().data())) {
            return true;
        }
    }

    return false;
}

std::vector<int> buildOccupiedVoxelIndices(const pgo::Mesh::TriMeshGeo& mesh, const VoxelDomain& domain,
                                           TriangleMeshClassifyMode classifyMode) {
    using Kernel             = pgo::CGALInterface::KernelInexact;
    using SurfaceMesh        = CGAL::Surface_mesh<Kernel::Point_3>;
    using VertexDescriptor   = boost::graph_traits<SurfaceMesh>::vertex_descriptor;
    using VertexIndexMap     = SurfaceMesh::Property_map<VertexDescriptor, int>;
    using SideOfTriangleMesh = CGAL::Side_of_triangle_mesh<SurfaceMesh, Kernel>;

    SurfaceMesh surfaceMesh;
    pgo::CGALInterface::triangleMesh2SurfaceMesh<SurfaceMesh, VertexIndexMap>(mesh, surfaceMesh, nullptr);
    SideOfTriangleMesh insideQuery(surfaceMesh);

    std::vector<pgo::Mesh::BoundingBox> triangleBoundingBoxes;
    pgo::Mesh::BoundingBoxBVTree        triangleBoxTree;
    if (classifyMode == TriangleMeshClassifyMode::Conservative) {
        triangleBoundingBoxes = mesh.ref().getTriangleBoundingBoxes();
        triangleBoxTree.buildByInertiaPartition(pgo::BasicAlgorithms::makeArrayRef(triangleBoundingBoxes));
    }

    std::vector<int> occupiedVoxels;
    std::vector<int> candidateTriangleIDs;

    for (int i = 0; i < domain.resolution; i++) {
        for (int j = 0; j < domain.resolution; j++) {
            for (int k = 0; k < domain.resolution; k++) {
                const pgo::Vec3d voxelCenter = domain.voxelCenter(i, j, k);
                const auto side = insideQuery(Kernel::Point_3(voxelCenter[0], voxelCenter[1], voxelCenter[2]));

                bool occupied = side == CGAL::ON_BOUNDED_SIDE || side == CGAL::ON_BOUNDARY;
                if (!occupied && classifyMode == TriangleMeshClassifyMode::Conservative) {
                    occupied = voxelIntersectsTriangleSurface(mesh, domain.voxelBoundingBox(i, j, k),
                                                              triangleBoundingBoxes, triangleBoxTree,
                                                              candidateTriangleIDs);
                }

                if (occupied) {
                    occupiedVoxels.push_back(i);
                    occupiedVoxels.push_back(j);
                    occupiedVoxels.push_back(k);
                }
            }
        }
    }

    return occupiedVoxels;
}
#endif

}  // namespace

const char* toString(TriangleMeshClassifyMode mode) {
    switch (mode) {
        case TriangleMeshClassifyMode::Center:
            return "center";
        case TriangleMeshClassifyMode::Conservative:
            return "conservative";
    }

    return "unknown";
}

std::unique_ptr<pgo::VolumetricMeshes::CubicMesh> createTriangleMeshCubicMesh(
    const TriangleMeshVoxelizerOptions& options) {
#if !defined(PGO_HAS_CGAL)
    (void)options;
    std::cerr << kGeometryStackUnsupportedMessage << std::endl;
    return nullptr;
#else
    pgo::Mesh::TriMeshGeo mesh;
    if (loadAndSanitizeTriangleMesh(options.inputMesh, mesh) == false) {
        return nullptr;
    }

    if (validateTriangleMeshForVoxelization(mesh, options.inputMesh) == false) {
        return nullptr;
    }

    VoxelDomain domain;
    if (buildVoxelDomain(mesh, options.resolution, options.paddingVoxels, domain) == false) {
        return nullptr;
    }

    std::vector<int> occupiedVoxels = buildOccupiedVoxelIndices(mesh, domain, options.classifyMode);
    if (occupiedVoxels.empty()) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(),
                            "Triangle mesh voxelization produced no occupied voxels. "
                            "Try increasing --resolution or switching to --classify-mode conservative");
        return nullptr;
    }

    auto cubicMesh = std::unique_ptr<pgo::VolumetricMeshes::CubicMesh>(pgo::VolumetricMeshes::CubicMesh::createFromUniformGrid(
        options.resolution, static_cast<int>(occupiedVoxels.size() / 3), occupiedVoxels.data(), options.E, options.nu,
        options.density));
    if (cubicMesh == nullptr) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Failed to create cubic mesh from triangle mesh voxelization");
        return nullptr;
    }

    if (applyDomainTransform(*cubicMesh, domain) == false) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Failed to transform cubic mesh back to world coordinates");
        return nullptr;
    }

    if (applyUserTransform(*cubicMesh, options.scale, options.offset) == false) {
        SPDLOG_LOGGER_ERROR(
            pgo::Logging::lgr(),
            "Triangle-mesh transform requires positive scale. Got scale={}, offset=({}, {}, {})", options.scale,
            options.offset[0], options.offset[1], options.offset[2]);
        return nullptr;
    }

    return cubicMesh;
#endif
}

}  // namespace cubic_mesher
