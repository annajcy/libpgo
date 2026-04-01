#include "internal/material_catalog.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <set>

namespace pgo::Mesh::test {

TEST(MaterialCatalogTest, SingleMaterialBuildsAllElementsCoverage) {
    pgo::VolumetricMeshes::internal::MaterialCatalog catalog(3, 500.0, 0.28, 7.0);

    ASSERT_EQ(catalog.num_materials(), 1);
    ASSERT_EQ(catalog.num_sets(), 1);
    ASSERT_EQ(catalog.num_regions(), 1);
    ASSERT_EQ(catalog.element_materials().size(), 3u);
    EXPECT_EQ(catalog.set(0).getName(), pgo::VolumetricMeshes::VolumetricMesh::allElementsSetName);

    const auto* material =
        pgo::VolumetricMeshes::try_get_material<pgo::VolumetricMeshes::EnuMaterialData>(catalog.element_material(2));
    ASSERT_NE(material, nullptr);
    EXPECT_DOUBLE_EQ(material->E, 500.0);
    EXPECT_DOUBLE_EQ(material->nu, 0.28);
    EXPECT_DOUBLE_EQ(material->density, 7.0);
    EXPECT_EQ(catalog.element_materials()[0], 0);
    EXPECT_EQ(catalog.element_materials()[1], 0);
    EXPECT_EQ(catalog.element_materials()[2], 0);
}

TEST(MaterialCatalogTest, AddMaterialReassignsSelectedElements) {
    pgo::VolumetricMeshes::internal::MaterialCatalog catalog(4, 500.0, 0.25, 3.0);
    pgo::VolumetricMeshes::ElementSet selected("selected");
    selected.insert(1);
    selected.insert(2);

    const auto new_material = pgo::Mesh::test::makeEnuMaterial("stiff", 6.5, 1800.0, 0.33);
    catalog.add_material(4, new_material, selected, true, true);

    ASSERT_EQ(catalog.num_materials(), 2);
    ASSERT_EQ(catalog.num_regions(), 2);
    ASSERT_EQ(catalog.element_materials().size(), 4u);
    EXPECT_EQ(catalog.element_materials()[0], 0);
    EXPECT_EQ(catalog.element_materials()[1], 1);
    EXPECT_EQ(catalog.element_materials()[2], 1);
    EXPECT_EQ(catalog.element_materials()[3], 0);

    const auto* original =
        pgo::VolumetricMeshes::try_get_material<pgo::VolumetricMeshes::EnuMaterialData>(catalog.element_material(0));
    const auto* reassigned =
        pgo::VolumetricMeshes::try_get_material<pgo::VolumetricMeshes::EnuMaterialData>(catalog.element_material(1));
    ASSERT_NE(original, nullptr);
    ASSERT_NE(reassigned, nullptr);
    EXPECT_DOUBLE_EQ(original->E, 500.0);
    EXPECT_DOUBLE_EQ(reassigned->E, 1800.0);

    std::set<int> region_elements;
    catalog.set(catalog.region(1).getSetIndex()).getElements(region_elements);
    EXPECT_EQ(region_elements, std::set<int>({1, 2}));
}

}  // namespace pgo::Mesh::test
