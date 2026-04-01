#include "io/mesh_load.h"
#include "io/mesh_save.h"
#include "tetMesh.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <memory>

namespace pgo::Mesh::test {
namespace {

pgo::VolumetricMeshes::TetMesh makeSingleTetMeshWithMaterial(const pgo::VolumetricMeshes::MaterialRecord& material) {
    pgo::VolumetricMeshes::TetMesh mesh = makeSingleTetMesh();
    mesh.setMaterial(0, material);
    return mesh;
}

void expectENuMaterialMatches(const pgo::VolumetricMeshes::MaterialRecord& material, double density, double E,
                              double nu) {
    using namespace pgo::VolumetricMeshes;

    ASSERT_EQ(material_kind(material), MaterialKind::Enu);
    const auto* enu = try_get_material<EnuMaterialData>(material);
    ASSERT_NE(enu, nullptr);
    EXPECT_DOUBLE_EQ(enu->density, density);
    EXPECT_DOUBLE_EQ(enu->E, E);
    EXPECT_DOUBLE_EQ(enu->nu, nu);
}

void expectOrthotropicMaterialMatches(const pgo::VolumetricMeshes::MaterialRecord& material, double density,
                                      double E1, double E2, double E3, double nu12, double nu23, double nu31,
                                      double G12, double G23, double G31, const std::array<double, 9>& rotation) {
    using namespace pgo::VolumetricMeshes;

    ASSERT_EQ(material_kind(material), MaterialKind::Orthotropic);
    const auto* orthotropic = try_get_material<OrthotropicMaterialData>(material);
    ASSERT_NE(orthotropic, nullptr);
    EXPECT_DOUBLE_EQ(orthotropic->density, density);
    EXPECT_DOUBLE_EQ(orthotropic->E1, E1);
    EXPECT_DOUBLE_EQ(orthotropic->E2, E2);
    EXPECT_DOUBLE_EQ(orthotropic->E3, E3);
    EXPECT_DOUBLE_EQ(orthotropic->nu12, nu12);
    EXPECT_DOUBLE_EQ(orthotropic->nu23, nu23);
    EXPECT_DOUBLE_EQ(orthotropic->nu31, nu31);
    EXPECT_DOUBLE_EQ(orthotropic->G12, G12);
    EXPECT_DOUBLE_EQ(orthotropic->G23, G23);
    EXPECT_DOUBLE_EQ(orthotropic->G31, G31);
    for (int i = 0; i < 9; ++i) {
        EXPECT_DOUBLE_EQ(orthotropic->rotation[static_cast<size_t>(i)], rotation[static_cast<size_t>(i)]);
    }
}

void expectMooneyRivlinMaterialMatches(const pgo::VolumetricMeshes::MaterialRecord& material, double density,
                                       double mu01, double mu10, double v1) {
    using namespace pgo::VolumetricMeshes;

    ASSERT_EQ(material_kind(material), MaterialKind::MooneyRivlin);
    const auto* mooney = try_get_material<MooneyRivlinMaterialData>(material);
    ASSERT_NE(mooney, nullptr);
    EXPECT_DOUBLE_EQ(mooney->density, density);
    EXPECT_DOUBLE_EQ(mooney->mu01, mu01);
    EXPECT_DOUBLE_EQ(mooney->mu10, mu10);
    EXPECT_DOUBLE_EQ(mooney->v1, v1);
}

}  // namespace

TEST(CoreSceneVolumetricMeshIoTest, ENuMaterialAsciiAndBinaryRoundTrip) {
    using namespace pgo::VolumetricMeshes;

    const auto material = makeEnuMaterial("enuMaterial", 9.25, 4321.0, 0.21);
    const TetMesh source = makeSingleTetMeshWithMaterial(material);

    const std::filesystem::path ascii_path = uniqueTempPath(".veg");
    const std::filesystem::path binary_path = uniqueTempPath(".vegb");
    ASSERT_EQ(io::save_to_ascii(source, ascii_path), 0);
    ASSERT_EQ(io::save_to_binary(source, binary_path), 0);

    const std::unique_ptr<TetMesh> ascii_mesh = io::load_tet(ascii_path, FileFormatType::Ascii, 0);
    const std::unique_ptr<TetMesh> binary_mesh = io::load_tet(binary_path, FileFormatType::Binary, 0);

    ASSERT_NE(ascii_mesh, nullptr);
    ASSERT_NE(binary_mesh, nullptr);
    expectENuMaterialMatches(ascii_mesh->getMaterial(0), 9.25, 4321.0, 0.21);
    expectENuMaterialMatches(binary_mesh->getMaterial(0), 9.25, 4321.0, 0.21);

    std::filesystem::remove(ascii_path);
    std::filesystem::remove(binary_path);
}

TEST(CoreSceneVolumetricMeshIoTest, OrthotropicMaterialAsciiAndBinaryRoundTrip) {
    using namespace pgo::VolumetricMeshes;

    const std::array<double, 9> rotation = {0.0, 1.0, 0.0,
                                            1.0, 0.0, 0.0,
                                            0.0, 0.0, 1.0};
    const auto material =
        makeOrthotropicMaterial("orthotropicMaterial", 4.5, 100.0, 200.0, 300.0, 0.12, 0.23, 0.31, 10.0, 20.0, 30.0,
                                rotation);
    const TetMesh source = makeSingleTetMeshWithMaterial(material);

    const std::filesystem::path ascii_path = uniqueTempPath(".veg");
    const std::filesystem::path binary_path = uniqueTempPath(".vegb");
    ASSERT_EQ(io::save_to_ascii(source, ascii_path), 0);
    ASSERT_EQ(io::save_to_binary(source, binary_path), 0);

    const std::unique_ptr<TetMesh> ascii_mesh = io::load_tet(ascii_path, FileFormatType::Ascii, 0);
    const std::unique_ptr<TetMesh> binary_mesh = io::load_tet(binary_path, FileFormatType::Binary, 0);

    ASSERT_NE(ascii_mesh, nullptr);
    ASSERT_NE(binary_mesh, nullptr);
    expectOrthotropicMaterialMatches(ascii_mesh->getMaterial(0), 4.5, 100.0, 200.0, 300.0, 0.12, 0.23, 0.31, 10.0, 20.0,
                                     30.0, rotation);
    expectOrthotropicMaterialMatches(binary_mesh->getMaterial(0), 4.5, 100.0, 200.0, 300.0, 0.12, 0.23, 0.31, 10.0,
                                     20.0, 30.0, rotation);

    std::filesystem::remove(ascii_path);
    std::filesystem::remove(binary_path);
}

TEST(CoreSceneVolumetricMeshIoTest, MooneyRivlinMaterialAsciiAndBinaryRoundTrip) {
    using namespace pgo::VolumetricMeshes;

    const auto material = makeMooneyRivlinMaterial("mooneyMaterial", 8.0, 0.12, 0.34, 0.56);
    const TetMesh source = makeSingleTetMeshWithMaterial(material);

    const std::filesystem::path ascii_path = uniqueTempPath(".veg");
    const std::filesystem::path binary_path = uniqueTempPath(".vegb");
    ASSERT_EQ(io::save_to_ascii(source, ascii_path), 0);
    ASSERT_EQ(io::save_to_binary(source, binary_path), 0);

    const std::unique_ptr<TetMesh> ascii_mesh = io::load_tet(ascii_path, FileFormatType::Ascii, 0);
    const std::unique_ptr<TetMesh> binary_mesh = io::load_tet(binary_path, FileFormatType::Binary, 0);

    ASSERT_NE(ascii_mesh, nullptr);
    ASSERT_NE(binary_mesh, nullptr);
    expectMooneyRivlinMaterialMatches(ascii_mesh->getMaterial(0), 8.0, 0.12, 0.34, 0.56);
    expectMooneyRivlinMaterialMatches(binary_mesh->getMaterial(0), 8.0, 0.12, 0.34, 0.56);

    std::filesystem::remove(ascii_path);
    std::filesystem::remove(binary_path);
}

}  // namespace pgo::Mesh::test
