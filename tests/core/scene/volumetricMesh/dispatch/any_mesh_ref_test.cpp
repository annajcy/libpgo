#include "dispatch/any_mesh_ref.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

namespace pgo::Mesh::test {

TEST(CoreSceneVolumetricMeshTest, AnyMeshRefConvertsLegacyTetBaseToTypedAlternative) {
    pgo::VolumetricMeshes::TetMesh mesh = makeSingleTetMesh();

    const pgo::VolumetricMeshes::VolumetricMesh& base_ref = mesh;
    const auto any_mesh = pgo::VolumetricMeshes::to_any_mesh_ref(base_ref);

    EXPECT_TRUE((std::holds_alternative<std::reference_wrapper<const pgo::VolumetricMeshes::TetMesh>>(any_mesh)));
    EXPECT_EQ(&std::get<std::reference_wrapper<const pgo::VolumetricMeshes::TetMesh>>(any_mesh).get(), &mesh);
}

TEST(CoreSceneVolumetricMeshTest, AnyMutableMeshRefConvertsLegacyCubeBaseToTypedAlternative) {
    pgo::VolumetricMeshes::CubicMesh mesh = makeSingleCubeMesh();

    pgo::VolumetricMeshes::VolumetricMesh& base_ref = mesh;
    const auto any_mesh = pgo::VolumetricMeshes::to_any_mesh_ref(base_ref);

    EXPECT_TRUE((std::holds_alternative<std::reference_wrapper<pgo::VolumetricMeshes::CubicMesh>>(any_mesh)));
    EXPECT_EQ(&std::get<std::reference_wrapper<pgo::VolumetricMeshes::CubicMesh>>(any_mesh).get(), &mesh);
}

}  // namespace pgo::Mesh::test
