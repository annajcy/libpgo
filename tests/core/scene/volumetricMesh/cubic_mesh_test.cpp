#include "cubicMesh.h"
#include "volumetricMeshENuMaterial.h"
#include "volumetricMeshExport.h"
#include "volumetricMeshIO.h"

#include "volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <filesystem>

namespace pgo::Mesh::test {

TEST(CoreSceneVolumetricMeshTest, CubicMeshBinaryRoundTripAndGeometryExportPreserveData) {
    using namespace pgo::VolumetricMeshes;

    const CubicMesh source = makeSingleCubeMesh();

    const auto source_geometry = exporting::geometry(source);
    EXPECT_EQ(source_geometry.numElementVertices, 8);
    expectVertexVectorsEqual(source_geometry.vertices, std::vector<pgo::Vec3d>(source.getVertices().begin(),
                                                                               source.getVertices().end()));
    ASSERT_EQ(source_geometry.elements.size(), 8u);
    EXPECT_EQ(source_geometry.elements[0], 0);
    EXPECT_EQ(source_geometry.elements[7], 7);

    const std::filesystem::path binary_path = uniqueTempPath(".vegb");
    ASSERT_EQ(io::save_to_binary(source, binary_path), 0);

    const CubicMesh binary_mesh(binary_path);
    const auto      binary_geometry = exporting::geometry(binary_mesh);

    EXPECT_EQ(binary_mesh.getElementType(), VolumetricMesh::ElementType::Cubic);
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
    const auto      source_geometry = exporting::geometry(source);
    const auto      memory_geometry = exporting::geometry(memory_mesh);

    EXPECT_EQ(memory_mesh.getElementType(), VolumetricMesh::ElementType::Cubic);
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
    using pgo::VolumetricMeshes::downcastENuMaterial;

    pgo::VolumetricMeshes::CubicMesh mesh = makeTwoCubeMeshWithDistinctMaterials();
    const int                        original_num_elements = mesh.getNumElements();
    const double                     original_cube_size = mesh.getCubeSize();

    mesh.subdivide();

    EXPECT_EQ(mesh.getNumElements(), original_num_elements * 8);
    EXPECT_DOUBLE_EQ(mesh.getCubeSize(), original_cube_size * 0.5);
    expectAllElementsSetCoversMesh(mesh);

    for (int element = 0; element < 8; ++element) {
        const auto* material = downcastENuMaterial(mesh.getElementMaterial(element));
        ASSERT_NE(material, nullptr);
        EXPECT_DOUBLE_EQ(material->getE(), 500.0);
        EXPECT_DOUBLE_EQ(material->getNu(), 0.25);
        EXPECT_DOUBLE_EQ(material->getDensity(), 3.0);
    }

    for (int element = 8; element < 16; ++element) {
        const auto* material = downcastENuMaterial(mesh.getElementMaterial(element));
        ASSERT_NE(material, nullptr);
        EXPECT_DOUBLE_EQ(material->getE(), 1800.0);
        EXPECT_DOUBLE_EQ(material->getNu(), 0.33);
        EXPECT_DOUBLE_EQ(material->getDensity(), 6.5);
    }
}

}  // namespace pgo::Mesh::test
