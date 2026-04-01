#include "materials/material_access.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

namespace {

TEST(MaterialAccessTest, MaterialKindIsDerivedFromVariantPayload) {
    using namespace pgo::VolumetricMeshes;

    EXPECT_EQ(material_kind(pgo::Mesh::test::makeEnuMaterial("enu", 7.5, 1234.0, 0.31)), MaterialKind::Enu);
    EXPECT_EQ(material_kind(pgo::Mesh::test::makeOrthotropicMaterial(
                  "ortho", 4.5, 100.0, 200.0, 300.0, 0.12, 0.23, 0.31, 10.0, 20.0, 30.0,
                  {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0})),
              MaterialKind::Orthotropic);
    EXPECT_EQ(material_kind(pgo::Mesh::test::makeMooneyRivlinMaterial("mr", 8.0, 0.12, 0.34, 0.56)),
              MaterialKind::MooneyRivlin);
}

TEST(MaterialAccessTest, MaterialCastReturnsExpectedPayload) {
    using namespace pgo::VolumetricMeshes;

    MaterialRecord enu = pgo::Mesh::test::makeEnuMaterial("enu", 7.5, 1234.0, 0.31);
    ASSERT_NE(try_get_material<EnuMaterialData>(enu), nullptr);
    EXPECT_EQ(try_get_material<OrthotropicMaterialData>(enu), nullptr);
    EXPECT_EQ(try_get_material<MooneyRivlinMaterialData>(enu), nullptr);

    MaterialRecord orthotropic = pgo::Mesh::test::makeOrthotropicMaterial(
        "ortho", 4.5, 100.0, 200.0, 300.0, 0.12, 0.23, 0.31, 10.0, 20.0, 30.0,
        {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    ASSERT_NE(try_get_material<OrthotropicMaterialData>(orthotropic), nullptr);
    EXPECT_EQ(try_get_material<EnuMaterialData>(orthotropic), nullptr);

    MaterialRecord mooney = pgo::Mesh::test::makeMooneyRivlinMaterial("mr", 8.0, 0.12, 0.34, 0.56);
    ASSERT_NE(try_get_material<MooneyRivlinMaterialData>(mooney), nullptr);
    EXPECT_EQ(try_get_material<EnuMaterialData>(mooney), nullptr);
}

TEST(MaterialAccessTest, RequireMaterialReturnsReferenceOrThrows) {
    using namespace pgo::VolumetricMeshes;

    MaterialRecord enu = pgo::Mesh::test::makeEnuMaterial("enu", 7.5, 1234.0, 0.31);
    const EnuMaterialData& enu_data = require_material<EnuMaterialData>(enu);
    EXPECT_DOUBLE_EQ(enu_data.E, 1234.0);
    EXPECT_DOUBLE_EQ(enu_data.nu, 0.31);

    MaterialRecord mooney = pgo::Mesh::test::makeMooneyRivlinMaterial("mr", 8.0, 0.12, 0.34, 0.56);
    EXPECT_THROW((void)require_material<EnuMaterialData>(mooney), std::invalid_argument);
}

}  // namespace
