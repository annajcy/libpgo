#include "cubicMesh.h"
#include "io/mesh_save.h"
#include "ops/mesh_export_ops.h"

#include "volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace pgo::Mesh::test {

TEST(CoreSceneVolumetricMeshTest, CubicMeshBinaryRoundTripAndGeometryExportPreserveData) {
    using namespace pgo::VolumetricMeshes;

    const CubicMesh source = makeSingleCubeMesh();

    const auto source_geometry = ops::geometry(source);
    EXPECT_EQ(source_geometry.numElementVertices, 8);
    expectVertexVectorsEqual(source_geometry.vertices, std::vector<pgo::Vec3d>(source.getVertices().begin(),
                                                                               source.getVertices().end()));
    ASSERT_EQ(source_geometry.elements.size(), 8u);
    EXPECT_EQ(source_geometry.elements[0], 0);
    EXPECT_EQ(source_geometry.elements[7], 7);

    const std::filesystem::path binary_path = uniqueTempPath(".vegb");
    ASSERT_EQ(io::save_to_binary(source, binary_path), 0);

    const CubicMesh binary_mesh(binary_path);
    const auto      binary_geometry = ops::geometry(binary_mesh);

    EXPECT_EQ(binary_mesh.getElementType(), ElementType::Cubic);
    EXPECT_DOUBLE_EQ(binary_mesh.getCubeSize(), 1.0);
    EXPECT_EQ(binary_geometry.numElementVertices, 8);
    expectVertexVectorsEqual(binary_geometry.vertices, source_geometry.vertices);
    EXPECT_EQ(binary_geometry.elements, source_geometry.elements);

    std::filesystem::remove(binary_path);
}

TEST(CoreSceneVolumetricMeshTest, CubicMeshBinaryMemoryConstructorPreservesGeometry) {
    using namespace pgo::VolumetricMeshes;

    const CubicMesh source = makeSingleCubeMesh();

    const std::filesystem::path binary_path = uniqueTempPath(".vegb");
    ASSERT_EQ(io::save_to_binary(source, binary_path), 0);

    const auto bytes = readBinaryFile(binary_path);
    ASSERT_FALSE(bytes.empty());

    const CubicMesh memory_mesh(std::span<const std::byte>(bytes.data(), bytes.size()));
    const auto      source_geometry = ops::geometry(source);
    const auto      memory_geometry = ops::geometry(memory_mesh);

    EXPECT_EQ(memory_mesh.getElementType(), ElementType::Cubic);
    EXPECT_DOUBLE_EQ(memory_mesh.getCubeSize(), 1.0);
    EXPECT_EQ(memory_geometry.numElementVertices, 8);
    expectVertexVectorsEqual(memory_geometry.vertices, source_geometry.vertices);
    EXPECT_EQ(memory_geometry.elements, source_geometry.elements);

    std::filesystem::remove(binary_path);
}

TEST(CoreSceneVolumetricMeshTest, CubicMeshBarycentricWeightsForInteriorPoint) {
    pgo::VolumetricMeshes::CubicMesh mesh = makeSingleCubeMesh();
    const pgo::Vec3d                 interior_point(0.5, 0.5, 0.5);

    double weights[8] = {};
    mesh.computeBarycentricWeights(0, interior_point, weights);

    expectWeightsNormalized(weights, 8);
    EXPECT_TRUE(mesh.containsVertex(0, interior_point));
}

TEST(CoreSceneVolumetricMeshTest, CubicMeshSubdivideExpandsElementsAndKeepsSets) {
    pgo::VolumetricMeshes::CubicMesh mesh = makeTwoCubeMeshWithDistinctMaterials();
    const int                        original_num_elements = mesh.getNumElements();
    const double                     original_cube_size = mesh.getCubeSize();

    mesh.subdivide();

    EXPECT_EQ(mesh.getNumElements(), original_num_elements * 8);
    EXPECT_DOUBLE_EQ(mesh.getCubeSize(), original_cube_size * 0.5);
    expectAllElementsSetCoversMesh(mesh);

    for (int element = 0; element < 8; ++element) {
        const auto* material = pgo::VolumetricMeshes::try_get_material<pgo::VolumetricMeshes::EnuMaterialData>(
            mesh.getElementMaterial(element));
        ASSERT_NE(material, nullptr);
        EXPECT_DOUBLE_EQ(material->E, 500.0);
        EXPECT_DOUBLE_EQ(material->nu, 0.25);
        EXPECT_DOUBLE_EQ(material->density, 3.0);
    }

    for (int element = 8; element < 16; ++element) {
        const auto* material = pgo::VolumetricMeshes::try_get_material<pgo::VolumetricMeshes::EnuMaterialData>(
            mesh.getElementMaterial(element));
        ASSERT_NE(material, nullptr);
        EXPECT_DOUBLE_EQ(material->E, 1800.0);
        EXPECT_DOUBLE_EQ(material->nu, 0.33);
        EXPECT_DOUBLE_EQ(material->density, 6.5);
    }
}

TEST(CoreSceneVolumetricMeshTest, CubicMeshNodeEleExportWritesEightNodeElements) {
    using namespace pgo::VolumetricMeshes;

    const CubicMesh source = makeSingleCubeMesh();

    const std::filesystem::path base_path = uniqueTempPath("");
    const std::filesystem::path ele_path  = base_path.string() + ".ele";
    const std::filesystem::path node_path = base_path.string() + ".node";

    ASSERT_EQ(io::save_to_node_ele(source, base_path), 0);
    EXPECT_TRUE(std::filesystem::exists(ele_path));
    EXPECT_TRUE(std::filesystem::exists(node_path));

    std::ifstream ele_stream(ele_path);
    ASSERT_TRUE(ele_stream.is_open());

    std::string header_line;
    std::getline(ele_stream, header_line);
    EXPECT_EQ(header_line, "1 8 0");

    std::filesystem::remove(ele_path);
    std::filesystem::remove(node_path);
}

}  // namespace pgo::Mesh::test
