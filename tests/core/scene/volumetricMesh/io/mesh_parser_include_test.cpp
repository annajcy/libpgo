#include "tetMesh.h"
#include "volumetricMeshENuMaterial.h"
#include "volumetricMeshIO.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>

namespace pgo::Mesh::test {
namespace {

constexpr const char* kMainMeshWithInclude = R"(*VERTICES
4 3 0 0
1 0 0 0
2 1 0 0
3 0 1 0
4 0 0 1
*INCLUDE child.veg
)";

constexpr const char* kIncludedTetSection = R"(*ELEMENTS
TET
1 4 0
1 1 2 3 4

*MATERIAL defaultMaterial
ENU, 7.5, 1234, 0.31

*REGION
allElements, defaultMaterial
)";

}  // namespace

TEST(CoreSceneVolumetricMeshIoTest, ParserIncludeLoadsRelativeChildFile) {
    using namespace pgo::VolumetricMeshes;

    const std::filesystem::path main_path = uniqueTempPath(".veg");
    const std::filesystem::path child_path = main_path.parent_path() / "child.veg";
    writeTextFile(main_path, kMainMeshWithInclude);
    writeTextFile(child_path, kIncludedTetSection);

    const TetMesh expected = makeSingleTetMesh();
    const std::unique_ptr<TetMesh> loaded = io::load_tet(main_path, VolumetricMesh::FileFormatType::Ascii, 0);

    ASSERT_NE(loaded, nullptr);
    expectMeshGeometryEqual(*loaded, expected);
    const auto* material = downcastENuMaterial(loaded->getMaterial(0));
    ASSERT_NE(material, nullptr);
    EXPECT_DOUBLE_EQ(material->getDensity(), 7.5);
    EXPECT_DOUBLE_EQ(material->getE(), 1234.0);
    EXPECT_DOUBLE_EQ(material->getNu(), 0.31);

    std::filesystem::remove(main_path);
    std::filesystem::remove(child_path);
}

TEST(CoreSceneVolumetricMeshIoTest, ParserIncludeMissingChildPreservesCurrentFailureMode) {
    using namespace pgo::VolumetricMeshes;

    const std::filesystem::path main_path = uniqueTempPath(".veg");
    writeTextFile(main_path, kMainMeshWithInclude);

    try {
        (void)io::load_tet(main_path, VolumetricMesh::FileFormatType::Ascii, 0);
        FAIL() << "Expected missing include to throw.";
    } catch (int error_code) {
        EXPECT_EQ(error_code, -1);
    }

    std::filesystem::remove(main_path);
}

}  // namespace pgo::Mesh::test
