#include "boundingVolumeTree.h"
#include "geometryQuery.h"
#include "cubicMesh.h"
#include "tetMesh.h"
#include "triMeshGeo.h"
#include "triMeshNeighbor.h"
#include "volumetricMeshENuMaterial.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

namespace pgo::Mesh::test {

namespace {
std::filesystem::path uniqueTempPath(const std::string& suffix) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() / ("libpgo_core_scene_" + std::to_string(stamp) + suffix);
}

void expectVertexVectorsEqual(const std::vector<pgo::Vec3d>& lhs, const std::vector<pgo::Vec3d>& rhs) {
    ASSERT_EQ(lhs.size(), rhs.size());
    for (size_t i = 0; i < lhs.size(); ++i) {
        EXPECT_DOUBLE_EQ(lhs[i][0], rhs[i][0]);
        EXPECT_DOUBLE_EQ(lhs[i][1], rhs[i][1]);
        EXPECT_DOUBLE_EQ(lhs[i][2], rhs[i][2]);
    }
}
}  // namespace

TEST(CoreSceneGeometryTest, TriangleAreaNormalAndCenter) {
    const pgo::Vec3d p0(0.0, 0.0, 0.0);
    const pgo::Vec3d p1(1.0, 0.0, 0.0);
    const pgo::Vec3d p2(0.0, 1.0, 0.0);

    EXPECT_NEAR(pgo::Mesh::getTriangleArea(p0, p1, p2), 0.5, 1e-12);

    const pgo::Vec3d n = pgo::Mesh::getTriangleNormal(p0, p1, p2);
    EXPECT_NEAR(n[0], 0.0, 1e-12);
    EXPECT_NEAR(n[1], 0.0, 1e-12);
    EXPECT_NEAR(n[2], 1.0, 1e-12);

    const pgo::Vec3d c = pgo::Mesh::getTriangleCenterOfMass(p0, p1, p2);
    EXPECT_NEAR(c[0], 1.0 / 3.0, 1e-12);
    EXPECT_NEAR(c[1], 1.0 / 3.0, 1e-12);
    EXPECT_NEAR(c[2], 0.0, 1e-12);
}

TEST(CoreSceneGeometryTest, TriMeshNeighborSharedEdge) {
    std::vector<pgo::Vec3d> vertices{pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(1.0, 0.0, 0.0), pgo::Vec3d(1.0, 1.0, 0.0),
                                     pgo::Vec3d(0.0, 1.0, 0.0)};

    std::vector<pgo::Vec3i> triangles{pgo::Vec3i(0, 1, 2), pgo::Vec3i(0, 2, 3)};

    pgo::Mesh::TriMeshGeo      mesh(std::move(vertices), std::move(triangles));
    pgo::Mesh::TriMeshNeighbor neighbor(mesh);

    const pgo::Vec3i n0          = neighbor.getTriangleNeighbors(0);
    int              sharedCount = 0;
    bool             hasTri1     = false;
    for (int i = 0; i < 3; i++) {
        if (n0[i] >= 0) {
            sharedCount++;
            if (n0[i] == 1) {
                hasTri1 = true;
            }
        }
    }

    EXPECT_EQ(sharedCount, 1);
    EXPECT_TRUE(hasTri1);
}

TEST(CoreSceneGeometryTest, TriMeshBVTreeClosestTriangle) {
    std::vector<pgo::Vec3d> vertices{pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(1.0, 0.0, 0.0), pgo::Vec3d(1.0, 1.0, 0.0),
                                     pgo::Vec3d(0.0, 1.0, 0.0)};

    std::vector<pgo::Vec3i> triangles{pgo::Vec3i(0, 1, 2), pgo::Vec3i(0, 2, 3)};

    pgo::Mesh::TriMeshGeo    mesh(std::move(vertices), std::move(triangles));
    pgo::Mesh::TriMeshBVTree bvTree;
    bvTree.buildByInertiaPartition(mesh);

    const auto result = bvTree.closestTriangleQuery(mesh, pgo::Vec3d(0.2, 0.2, 0.8));
    EXPECT_GE(result.triID, 0);
    EXPECT_LT(result.triID, mesh.numTriangles());
    EXPECT_TRUE(std::isfinite(result.dist2));
    EXPECT_NEAR(result.closestPosition[2], 0.0, 1e-12);
}

TEST(CoreSceneGeometryTest, TetMeshAsciiAndBinaryRoundTripPreserveGeometry) {
    using namespace pgo::VolumetricMeshes;

    const std::vector<pgo::Vec3d> vertices{
        pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(1.0, 0.0, 0.0), pgo::Vec3d(0.0, 1.0, 0.0), pgo::Vec3d(0.0, 0.0, 1.0)};
    const std::vector<pgo::Vec4i> tets{pgo::Vec4i(0, 1, 2, 3)};
    const TetMesh                  source(vertices, tets, 1234.0, 0.31, 7.5);

    const std::filesystem::path asciiPath  = uniqueTempPath(".veg");
    const std::filesystem::path binaryPath = uniqueTempPath(".vegb");
    ASSERT_EQ(source.saveToAscii(asciiPath), 0);
    ASSERT_EQ(source.saveToBinary(binaryPath), 0);

    const TetMesh asciiMesh(asciiPath);
    const TetMesh binaryMesh(binaryPath);

    const auto sourceGeometry = source.VolumetricMesh::exportMeshGeometry();
    const auto asciiGeometry  = asciiMesh.VolumetricMesh::exportMeshGeometry();
    const auto binaryGeometry = binaryMesh.VolumetricMesh::exportMeshGeometry();

    EXPECT_EQ(asciiMesh.getElementType(), VolumetricMesh::ElementType::Tet);
    EXPECT_EQ(binaryMesh.getElementType(), VolumetricMesh::ElementType::Tet);
    EXPECT_EQ(asciiGeometry.numElementVertices, 4);
    EXPECT_EQ(binaryGeometry.numElementVertices, 4);
    expectVertexVectorsEqual(asciiGeometry.vertices, sourceGeometry.vertices);
    EXPECT_EQ(asciiGeometry.elements, sourceGeometry.elements);
    expectVertexVectorsEqual(binaryGeometry.vertices, sourceGeometry.vertices);
    EXPECT_EQ(binaryGeometry.elements, sourceGeometry.elements);

    std::filesystem::remove(asciiPath);
    std::filesystem::remove(binaryPath);
}

TEST(CoreSceneGeometryTest, CubicMeshBinaryRoundTripAndGeometryExportPreserveData) {
    using namespace pgo::VolumetricMeshes;

    const std::vector<pgo::Vec3d> vertices{pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(1.0, 0.0, 0.0),
                                           pgo::Vec3d(1.0, 1.0, 0.0), pgo::Vec3d(0.0, 1.0, 0.0),
                                           pgo::Vec3d(0.0, 0.0, 1.0), pgo::Vec3d(1.0, 0.0, 1.0),
                                           pgo::Vec3d(1.0, 1.0, 1.0), pgo::Vec3d(0.0, 1.0, 1.0)};
    const std::vector<VolumetricMesh::CubicElement> elements{{0, 1, 2, 3, 4, 5, 6, 7}};
    const CubicMesh                                 source(vertices, elements, 500.0, 0.25, 3.0);

    const auto sourceGeometry = source.VolumetricMesh::exportMeshGeometry();
    EXPECT_EQ(sourceGeometry.numElementVertices, 8);
    expectVertexVectorsEqual(sourceGeometry.vertices, vertices);
    ASSERT_EQ(sourceGeometry.elements.size(), 8u);
    EXPECT_EQ(sourceGeometry.elements[0], 0);
    EXPECT_EQ(sourceGeometry.elements[7], 7);

    const std::filesystem::path binaryPath = uniqueTempPath(".vegb");
    ASSERT_EQ(source.saveToBinary(binaryPath), 0);

    const CubicMesh binaryMesh(binaryPath);
    const auto      binaryGeometry = binaryMesh.VolumetricMesh::exportMeshGeometry();

    EXPECT_EQ(binaryMesh.getElementType(), VolumetricMesh::ElementType::Cubic);
    EXPECT_DOUBLE_EQ(binaryMesh.getCubeSize(), 1.0);
    EXPECT_EQ(binaryGeometry.numElementVertices, 8);
    expectVertexVectorsEqual(binaryGeometry.vertices, sourceGeometry.vertices);
    EXPECT_EQ(binaryGeometry.elements, sourceGeometry.elements);

    std::filesystem::remove(binaryPath);
}

TEST(CoreSceneGeometryTest, VolumetricMeshCloneOwnsDeepCopiedMaterial) {
    using namespace pgo::VolumetricMeshes;

    const std::vector<pgo::Vec3d> vertices{
        pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(1.0, 0.0, 0.0), pgo::Vec3d(0.0, 1.0, 0.0), pgo::Vec3d(0.0, 0.0, 1.0)};
    const std::vector<pgo::Vec4i> tets{pgo::Vec4i(0, 1, 2, 3)};
    TetMesh                       source(vertices, tets, 1234.0, 0.31, 7.5);

    std::unique_ptr<VolumetricMesh> clone = source.clone();
    auto* const sourceMaterial = downcastENuMaterial(source.getMaterial(0));
    auto* const cloneMaterial  = downcastENuMaterial(clone->getMaterial(0));
    ASSERT_NE(sourceMaterial, nullptr);
    ASSERT_NE(cloneMaterial, nullptr);
    ASSERT_NE(sourceMaterial, cloneMaterial);

    cloneMaterial->setE(4321.0);
    cloneMaterial->setNu(0.21);
    cloneMaterial->setDensity(9.25);

    EXPECT_DOUBLE_EQ(sourceMaterial->getE(), 1234.0);
    EXPECT_DOUBLE_EQ(sourceMaterial->getNu(), 0.31);
    EXPECT_DOUBLE_EQ(sourceMaterial->getDensity(), 7.5);
    EXPECT_DOUBLE_EQ(cloneMaterial->getE(), 4321.0);
    EXPECT_DOUBLE_EQ(cloneMaterial->getNu(), 0.21);
    EXPECT_DOUBLE_EQ(cloneMaterial->getDensity(), 9.25);
}

}  // namespace pgo::Mesh::test
