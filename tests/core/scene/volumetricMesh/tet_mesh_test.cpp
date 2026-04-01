#include "tetMesh.h"
#include "io/mesh_save.h"
#include "ops/mesh_export_ops.h"
#include "ops/tet_geometry_export.h"

#include "volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <filesystem>

namespace pgo::Mesh::test {

TEST(CoreSceneVolumetricMeshTest, TetMeshAsciiAndBinaryRoundTripPreserveGeometry) {
    using namespace pgo::VolumetricMeshes;

    const TetMesh source = makeSingleTetMesh();

    const std::filesystem::path ascii_path  = uniqueTempPath(".veg");
    const std::filesystem::path binary_path = uniqueTempPath(".vegb");
    ASSERT_EQ(io::save_to_ascii(source, ascii_path), 0);
    ASSERT_EQ(io::save_to_binary(source, binary_path), 0);

    const TetMesh ascii_mesh(ascii_path);
    const TetMesh binary_mesh(binary_path);

    const auto source_geometry = ops::geometry(source);
    const auto ascii_geometry  = ops::geometry(ascii_mesh);
    const auto binary_geometry = ops::geometry(binary_mesh);

    EXPECT_EQ(ascii_mesh.getElementType(), VolumetricMesh::ElementType::Tet);
    EXPECT_EQ(binary_mesh.getElementType(), VolumetricMesh::ElementType::Tet);
    EXPECT_EQ(ascii_geometry.numElementVertices, 4);
    EXPECT_EQ(binary_geometry.numElementVertices, 4);
    expectVertexVectorsEqual(ascii_geometry.vertices, source_geometry.vertices);
    EXPECT_EQ(ascii_geometry.elements, source_geometry.elements);
    expectVertexVectorsEqual(binary_geometry.vertices, source_geometry.vertices);
    EXPECT_EQ(binary_geometry.elements, source_geometry.elements);

    std::filesystem::remove(ascii_path);
    std::filesystem::remove(binary_path);
}

TEST(CoreSceneVolumetricMeshTest, TetMeshBinaryMemoryConstructorPreservesGeometry) {
    using namespace pgo::VolumetricMeshes;

    const TetMesh source = makeSingleTetMesh();

    const std::filesystem::path binary_path = uniqueTempPath(".vegb");
    ASSERT_EQ(io::save_to_binary(source, binary_path), 0);

    const auto bytes = readBinaryFile(binary_path);
    ASSERT_FALSE(bytes.empty());

    const TetMesh memory_mesh(std::span<const std::byte>(bytes.data(), bytes.size()));
    const auto    source_geometry = ops::geometry(source);
    const auto    memory_geometry = ops::geometry(memory_mesh);

    EXPECT_EQ(memory_mesh.getElementType(), VolumetricMesh::ElementType::Tet);
    EXPECT_EQ(memory_geometry.numElementVertices, 4);
    expectVertexVectorsEqual(memory_geometry.vertices, source_geometry.vertices);
    EXPECT_EQ(memory_geometry.elements, source_geometry.elements);

    std::filesystem::remove(binary_path);
}

TEST(CoreSceneVolumetricMeshTest, TetMeshBarycentricWeightsForInteriorPoint) {
    pgo::VolumetricMeshes::TetMesh mesh = makeSingleTetMesh();
    const pgo::Vec3d               interior_point(0.25, 0.25, 0.25);

    double weights[4] = {};
    mesh.computeBarycentricWeights(0, interior_point, weights);

    expectWeightsNormalized(weights, 4);
    EXPECT_TRUE(mesh.containsVertex(0, interior_point));
    EXPECT_EQ(mesh.getContainingElement(interior_point), 0);
}

TEST(CoreSceneVolumetricMeshTest, TetMeshExteriorPointIsNotContained) {
    pgo::VolumetricMeshes::TetMesh mesh = makeSingleTetMesh();
    const pgo::Vec3d               exterior_point(1.5, 1.5, 1.5);

    EXPECT_FALSE(mesh.containsVertex(0, exterior_point));
    EXPECT_EQ(mesh.getContainingElement(exterior_point), -1);
}

TEST(CoreSceneVolumetricMeshTest, TetMeshToEleExportWritesTetgenFiles) {
    using namespace pgo::VolumetricMeshes;

    const TetMesh source = makeSingleTetMesh();

    const std::filesystem::path base_path = uniqueTempPath("");
    const std::filesystem::path ele_path  = base_path.string() + ".ele";
    const std::filesystem::path node_path = base_path.string() + ".node";

    ASSERT_EQ(io::save_to_node_ele(source, base_path), 0);
    EXPECT_TRUE(std::filesystem::exists(ele_path));
    EXPECT_TRUE(std::filesystem::exists(node_path));

    std::filesystem::remove(ele_path);
    std::filesystem::remove(node_path);
}

}  // namespace pgo::Mesh::test
