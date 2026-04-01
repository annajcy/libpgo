#include "cubicMesh.h"
#include "io/mesh_load.h"
#include "io/mesh_save.h"
#include "tetMesh.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>

namespace pgo::Mesh::test {

TEST(CoreSceneVolumetricMeshIoTest, TetAsciiRoundTripPreservesGeometryAndTopology) {
    using namespace pgo::VolumetricMeshes;

    const TetMesh source = makeSingleTetMesh();
    const std::filesystem::path ascii_path = uniqueTempPath(".veg");

    ASSERT_EQ(io::save_to_ascii(source, ascii_path), 0);
    const std::unique_ptr<TetMesh> reloaded = io::load_tet(ascii_path, FileFormatType::Ascii, 0);

    ASSERT_NE(reloaded, nullptr);
    expectMeshGeometryEqual(*reloaded, source);

    std::filesystem::remove(ascii_path);
}

TEST(CoreSceneVolumetricMeshIoTest, CubicAsciiRoundTripPreservesGeometryAndTopology) {
    using namespace pgo::VolumetricMeshes;

    const CubicMesh source = makeSingleCubeMesh();
    const std::filesystem::path ascii_path = uniqueTempPath(".veg");

    ASSERT_EQ(io::save_to_ascii(source, ascii_path), 0);
    const std::unique_ptr<CubicMesh> reloaded = io::load_cubic(ascii_path, FileFormatType::Ascii, 0);

    ASSERT_NE(reloaded, nullptr);
    expectMeshGeometryEqual(*reloaded, source);
    EXPECT_DOUBLE_EQ(reloaded->getCubeSize(), source.getCubeSize());

    std::filesystem::remove(ascii_path);
}

TEST(CoreSceneVolumetricMeshIoTest, AsciiLoadRejectsMalformedInput) {
    using namespace pgo::VolumetricMeshes;

    const std::filesystem::path malformed_ascii_path = uniqueTempPath(".veg");
    writeTextFile(malformed_ascii_path, "not a valid veg mesh\n");

    EXPECT_THROW(io::load_tet(malformed_ascii_path, FileFormatType::Ascii, 0), int);

    std::filesystem::remove(malformed_ascii_path);
}

}  // namespace pgo::Mesh::test
