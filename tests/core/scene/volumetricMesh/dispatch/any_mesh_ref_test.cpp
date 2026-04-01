#include "dispatch/any_mesh_ref.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

namespace pgo::Mesh::test {

TEST(CoreSceneVolumetricMeshTest, AnyMeshRefConvertsLegacyTetBaseToTypedAlternative) {
    pgo::VolumetricMeshes::TetMesh mesh = makeSingleTetMesh();

    const auto any_mesh = pgo::VolumetricMeshes::make_any_mesh_ref(mesh);

    EXPECT_TRUE((std::holds_alternative<std::reference_wrapper<const pgo::VolumetricMeshes::TetMesh>>(any_mesh)));
    EXPECT_EQ(&std::get<std::reference_wrapper<const pgo::VolumetricMeshes::TetMesh>>(any_mesh).get(), &mesh);
}

TEST(CoreSceneVolumetricMeshTest, AnyMutableMeshRefConvertsLegacyCubeBaseToTypedAlternative) {
    pgo::VolumetricMeshes::CubicMesh mesh = makeSingleCubeMesh();

    const auto any_mesh = pgo::VolumetricMeshes::make_any_mutable_mesh_ref(mesh);

    EXPECT_TRUE((std::holds_alternative<std::reference_wrapper<pgo::VolumetricMeshes::CubicMesh>>(any_mesh)));
    EXPECT_EQ(&std::get<std::reference_wrapper<pgo::VolumetricMeshes::CubicMesh>>(any_mesh).get(), &mesh);
}

}  // namespace pgo::Mesh::test
