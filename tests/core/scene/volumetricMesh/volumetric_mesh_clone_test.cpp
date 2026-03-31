#include "volumetricMeshENuMaterial.h"

#include "volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <memory>

namespace pgo::Mesh::test {

TEST(CoreSceneVolumetricMeshTest, VolumetricMeshCloneOwnsDeepCopiedMaterial) {
    using namespace pgo::VolumetricMeshes;

    TetMesh source = makeSingleTetMesh();

    std::unique_ptr<VolumetricMesh> clone = source.clone();
    auto* const source_material = downcastENuMaterial(source.getMaterial(0));
    auto* const clone_material  = downcastENuMaterial(clone->getMaterial(0));
    ASSERT_NE(source_material, nullptr);
    ASSERT_NE(clone_material, nullptr);
    ASSERT_NE(source_material, clone_material);

    clone_material->setE(4321.0);
    clone_material->setNu(0.21);
    clone_material->setDensity(9.25);

    EXPECT_DOUBLE_EQ(source_material->getE(), 1234.0);
    EXPECT_DOUBLE_EQ(source_material->getNu(), 0.31);
    EXPECT_DOUBLE_EQ(source_material->getDensity(), 7.5);
    EXPECT_DOUBLE_EQ(clone_material->getE(), 4321.0);
    EXPECT_DOUBLE_EQ(clone_material->getNu(), 0.21);
    EXPECT_DOUBLE_EQ(clone_material->getDensity(), 9.25);
}

}  // namespace pgo::Mesh::test
