#include "cubicMesh.h"
#include "tetMesh.h"
#include "volumetricMeshIO.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>

namespace pgo::Mesh::test {

TEST(CoreSceneVolumetricMeshIoTest, TetBinaryRoundTripPreservesGeometryAndTopology) {
    using namespace pgo::VolumetricMeshes;

    const TetMesh source = makeSingleTetMesh();
    const std::filesystem::path binary_path = uniqueTempPath(".vegb");

    ASSERT_EQ(io::save_to_binary(source, binary_path), 0);
    const std::unique_ptr<TetMesh> reloaded = io::load_tet(binary_path, VolumetricMesh::FileFormatType::Binary, 0);

    ASSERT_NE(reloaded, nullptr);
    expectMeshGeometryEqual(*reloaded, source);

    std::filesystem::remove(binary_path);
}

TEST(CoreSceneVolumetricMeshIoTest, CubicBinaryRoundTripPreservesGeometryAndTopology) {
    using namespace pgo::VolumetricMeshes;

    const CubicMesh source = makeSingleCubeMesh();
    const std::filesystem::path binary_path = uniqueTempPath(".vegb");

    ASSERT_EQ(io::save_to_binary(source, binary_path), 0);
    const std::unique_ptr<CubicMesh> reloaded = io::load_cubic(binary_path, VolumetricMesh::FileFormatType::Binary, 0);

    ASSERT_NE(reloaded, nullptr);
    expectMeshGeometryEqual(*reloaded, source);
    EXPECT_DOUBLE_EQ(reloaded->getCubeSize(), source.getCubeSize());

    std::filesystem::remove(binary_path);
}

TEST(CoreSceneVolumetricMeshIoTest, BinaryLoadRejectsTruncatedPayload) {
    using namespace pgo::VolumetricMeshes;

    const std::filesystem::path truncated_binary_path = uniqueTempPath(".vegb");
    std::ofstream stream(truncated_binary_path, std::ios::binary);
    ASSERT_TRUE(stream.is_open());

    const double version = 1.0;
    const int element_type = static_cast<int>(VolumetricMesh::ElementType::Tet);
    stream.write(reinterpret_cast<const char*>(&version), sizeof(version));
    stream.write(reinterpret_cast<const char*>(&element_type), sizeof(element_type));
    ASSERT_TRUE(stream.good());
    stream.close();

    EXPECT_THROW(io::load_tet(truncated_binary_path, VolumetricMesh::FileFormatType::Binary, 0), std::runtime_error);

    std::filesystem::remove(truncated_binary_path);
}

}  // namespace pgo::Mesh::test
