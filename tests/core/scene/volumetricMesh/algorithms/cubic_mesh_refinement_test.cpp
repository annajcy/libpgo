#include "algorithms/cubic_mesh_refinement.h"

#include "../volumetric_mesh_test_fixtures.h"

#include "volumetricMeshENuMaterial.h"

#include <gtest/gtest.h>

namespace {

TEST(CubicMeshAlgorithmsTest, CubicMeshRefinementSubdividePreservesCurrentExpansionAndMaterialMapping) {
    using pgo::VolumetricMeshes::downcastENuMaterial;

    pgo::VolumetricMeshes::CubicMesh mesh = pgo::Mesh::test::makeTwoCubeMeshWithDistinctMaterials();
    const int original_num_elements = mesh.getNumElements();
    const double original_cube_size = mesh.getCubeSize();

    pgo::VolumetricMeshes::algorithms::subdivide_cubic_mesh(mesh);

    EXPECT_EQ(mesh.getNumElements(), original_num_elements * 8);
    EXPECT_DOUBLE_EQ(mesh.getCubeSize(), original_cube_size * 0.5);
    pgo::Mesh::test::expectAllElementsSetCoversMesh(mesh);

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

}  // namespace
