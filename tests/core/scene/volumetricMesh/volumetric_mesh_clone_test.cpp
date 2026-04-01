#include "volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <memory>

namespace pgo::Mesh::test {

TEST(CoreSceneVolumetricMeshTest, VolumetricMeshCloneOwnsDeepCopiedMaterial) {
    using namespace pgo::VolumetricMeshes;

    TetMesh source = makeSingleTetMesh();

    std::unique_ptr<VolumetricMesh> clone = source.clone();
    auto* const source_material = try_get_material<EnuMaterialData>(source.getMaterial(0));
    auto* const clone_material  = try_get_material<EnuMaterialData>(clone->getMaterial(0));
    ASSERT_NE(source_material, nullptr);
    ASSERT_NE(clone_material, nullptr);
    ASSERT_NE(source_material, clone_material);

    clone_material->E = 4321.0;
    clone_material->nu = 0.21;
    clone_material->density = 9.25;

    EXPECT_DOUBLE_EQ(source_material->E, 1234.0);
    EXPECT_DOUBLE_EQ(source_material->nu, 0.31);
    EXPECT_DOUBLE_EQ(source_material->density, 7.5);
    EXPECT_DOUBLE_EQ(clone_material->E, 4321.0);
    EXPECT_DOUBLE_EQ(clone_material->nu, 0.21);
    EXPECT_DOUBLE_EQ(clone_material->density, 9.25);
}

}  // namespace pgo::Mesh::test
