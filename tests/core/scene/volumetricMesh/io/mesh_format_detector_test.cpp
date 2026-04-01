#include "cubicMesh.h"
#include "io/mesh_format_detector.h"
#include "io/mesh_save.h"
#include "tetMesh.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <vector>

namespace pgo::Mesh::test {
namespace {

void writeBinaryHeader(const std::filesystem::path& path, double version, int element_type) {
    std::ofstream stream(path, std::ios::binary);
    ASSERT_TRUE(stream.is_open());
    if (!stream.is_open()) {
        return;
    }

    stream.write(reinterpret_cast<const char*>(&version), sizeof(version));
    stream.write(reinterpret_cast<const char*>(&element_type), sizeof(element_type));
    ASSERT_TRUE(stream.good());
}

}  // namespace

TEST(CoreSceneVolumetricMeshIoTest, DetectFileFormatByExtension) {
    using FileFormatType = pgo::VolumetricMeshes::VolumetricMesh::FileFormatType;
    using namespace pgo::VolumetricMeshes;

    EXPECT_EQ(io::detect_file_format("mesh.veg"), FileFormatType::Ascii);
    EXPECT_EQ(io::detect_file_format("mesh.vegb"), FileFormatType::Binary);
    EXPECT_EQ(io::detect_file_format("mesh.unknown"), FileFormatType::Unknown);
}

TEST(CoreSceneVolumetricMeshIoTest, DetectElementTypeFromSavedFilesAndBinarySpans) {
    using ElementType = pgo::VolumetricMeshes::VolumetricMesh::ElementType;
    using namespace pgo::VolumetricMeshes;

    const TetMesh   tet_mesh = makeSingleTetMesh();
    const CubicMesh cube_mesh = makeSingleCubeMesh();

    const std::filesystem::path tet_ascii_path = uniqueTempPath(".veg");
    const std::filesystem::path tet_binary_path = uniqueTempPath(".vegb");
    const std::filesystem::path cube_ascii_path = uniqueTempPath(".veg");
    const std::filesystem::path cube_binary_path = uniqueTempPath(".vegb");

    ASSERT_EQ(io::save_to_ascii(tet_mesh, tet_ascii_path), 0);
    ASSERT_EQ(io::save_to_binary(tet_mesh, tet_binary_path), 0);
    ASSERT_EQ(io::save_to_ascii(cube_mesh, cube_ascii_path), 0);
    ASSERT_EQ(io::save_to_binary(cube_mesh, cube_binary_path), 0);

    EXPECT_EQ(io::detect_element_type(tet_ascii_path), ElementType::Tet);
    EXPECT_EQ(io::detect_element_type(tet_binary_path), ElementType::Tet);
    EXPECT_EQ(io::detect_element_type(cube_ascii_path), ElementType::Cubic);
    EXPECT_EQ(io::detect_element_type(cube_binary_path), ElementType::Cubic);

    const std::vector<std::byte> tet_bytes = readBinaryFile(tet_binary_path);
    const std::vector<std::byte> cube_bytes = readBinaryFile(cube_binary_path);
    ASSERT_FALSE(tet_bytes.empty());
    ASSERT_FALSE(cube_bytes.empty());

    EXPECT_EQ(io::detect_element_type(std::span<const std::byte>(tet_bytes.data(), tet_bytes.size())), ElementType::Tet);
    EXPECT_EQ(io::detect_element_type(std::span<const std::byte>(cube_bytes.data(), cube_bytes.size())),
              ElementType::Cubic);

    std::filesystem::remove(tet_ascii_path);
    std::filesystem::remove(tet_binary_path);
    std::filesystem::remove(cube_ascii_path);
    std::filesystem::remove(cube_binary_path);
}

TEST(CoreSceneVolumetricMeshIoTest, DetectElementTypeReturnsInvalidForUnknownBinaryHeader) {
    using ElementType = pgo::VolumetricMeshes::VolumetricMesh::ElementType;
    using FileFormatType = pgo::VolumetricMeshes::VolumetricMesh::FileFormatType;
    using namespace pgo::VolumetricMeshes;

    const std::filesystem::path invalid_binary_path = uniqueTempPath(".vegb");
    writeBinaryHeader(invalid_binary_path, 1.0, 99);

    EXPECT_EQ(io::detect_element_type(invalid_binary_path, FileFormatType::Binary), ElementType::Invalid);

    std::filesystem::remove(invalid_binary_path);
}

TEST(CoreSceneVolumetricMeshIoTest, DetectElementTypeRejectsTruncatedBinarySpan) {
    using namespace pgo::VolumetricMeshes;

    const std::vector<std::byte> truncated_bytes{std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
    EXPECT_THROW(
        io::detect_element_type(std::span<const std::byte>(truncated_bytes.data(), truncated_bytes.size())),
        std::runtime_error);
}

}  // namespace pgo::Mesh::test
