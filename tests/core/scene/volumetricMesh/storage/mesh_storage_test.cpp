#include "storage/mesh_storage.h"

#include <gtest/gtest.h>

namespace {

TEST(MeshStorageTest, ValidatesGeometryAndMaterialCatalogAgreement) {
    using namespace pgo::VolumetricMeshes;

    std::vector<pgo::Vec3d> vertices{
        pgo::Vec3d(0.0, 0.0, 0.0),
        pgo::Vec3d(1.0, 0.0, 0.0),
        pgo::Vec3d(0.0, 1.0, 0.0),
        pgo::Vec3d(0.0, 0.0, 1.0),
    };
    std::vector<int> elements{0, 1, 2, 3};

    storage::MeshStorage storage(
        internal::VolumetricMeshData(4, std::move(vertices), std::move(elements)),
        internal::MaterialCatalog(1, 1200.0, 0.29, 8.5));

    storage.validate_invariants();

    EXPECT_EQ(storage.geometry().num_vertices(), 4);
    EXPECT_EQ(storage.geometry().num_elements(), 1);
    const auto* material = try_get_material<EnuMaterialData>(storage.material_catalog().element_material(0));
    ASSERT_NE(material, nullptr);
    EXPECT_DOUBLE_EQ(material->E, 1200.0);
    EXPECT_DOUBLE_EQ(material->nu, 0.29);
    EXPECT_DOUBLE_EQ(material->density, 8.5);
}

}  // namespace
