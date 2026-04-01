#include "ops/mesh_construction.h"

#include "../volumetric_mesh_test_fixtures.h"

#include "internal/material_catalog.h"
#include "internal/volumetric_mesh_data.h"

#include <gtest/gtest.h>

namespace {

TEST(MeshConstructionOpsTest, AssignCommonLoadedDataReplacesTetGeometryAndMaterialCatalog) {
    pgo::VolumetricMeshes::TetMesh mesh = pgo::Mesh::test::makeSingleTetMesh();

    std::vector<pgo::Vec3d> vertices{
        pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(2.0, 0.0, 0.0), pgo::Vec3d(0.0, 2.0, 0.0), pgo::Vec3d(0.0, 0.0, 2.0),
    };
    std::vector<int> elements{0, 1, 2, 3};

    pgo::VolumetricMeshes::io::detail::LoadedMeshData data;
    data.element_type = pgo::VolumetricMeshes::ElementType::Tet;
    data.geometry = pgo::VolumetricMeshes::internal::VolumetricMeshData(4, std::move(vertices), std::move(elements));
    data.material_catalog = pgo::VolumetricMeshes::internal::MaterialCatalog(1, 2500.0, 0.28, 9.0);

    pgo::VolumetricMeshes::ops::assign_common_loaded_data(mesh, std::move(data));

    EXPECT_EQ(mesh.getNumVertices(), 4);
    EXPECT_EQ(mesh.getNumElements(), 1);
    EXPECT_DOUBLE_EQ(mesh.getVertex(1)[0], 2.0);

    const auto* material =
        pgo::VolumetricMeshes::try_get_material<pgo::VolumetricMeshes::EnuMaterialData>(mesh.getElementMaterial(0));
    ASSERT_NE(material, nullptr);
    EXPECT_DOUBLE_EQ(material->E, 2500.0);
    EXPECT_DOUBLE_EQ(material->nu, 0.28);
    EXPECT_DOUBLE_EQ(material->density, 9.0);
}

}  // namespace
