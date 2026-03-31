#include "sceneToSimulationMesh.h"
#include "simulationMesh.h"
#include "simulationMeshMaterial.h"
#include "tetMesh.h"
#include "volumetricMeshMooneyRivlinMaterial.h"

#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <set>
#include <stdexcept>
#include <vector>

namespace pgo::SolidDeformationModel::test {
namespace {

VolumetricMeshes::TetMesh makeTwoTetMesh() {
    const std::vector<Vec3d> vertices{
        Vec3d(0.0, 0.0, 0.0), Vec3d(1.0, 0.0, 0.0), Vec3d(0.0, 1.0, 0.0), Vec3d(0.0, 0.0, 1.0),
        Vec3d(3.0, 0.0, 0.0), Vec3d(4.0, 0.0, 0.0), Vec3d(3.0, 1.0, 0.0), Vec3d(3.0, 0.0, 1.0)};
    const std::vector<Vec4i> tets{Vec4i(0, 1, 2, 3), Vec4i(4, 5, 6, 7)};
    return VolumetricMeshes::TetMesh(vertices, tets, 1234.0, 0.31, 7.5);
}

VolumetricMeshes::TetMesh makeSingleTetMeshWithMooneyRivlinMaterial() {
    const std::vector<Vec3d> vertices{
        Vec3d(0.0, 0.0, 0.0), Vec3d(1.0, 0.0, 0.0), Vec3d(0.0, 1.0, 0.0), Vec3d(0.0, 0.0, 1.0)};
    const std::vector<Vec4i> tets{Vec4i(0, 1, 2, 3)};

    std::vector<std::unique_ptr<VolumetricMeshes::VolumetricMesh::Material>> materials;
    materials.push_back(std::make_unique<VolumetricMeshes::VolumetricMesh::MooneyRivlinMaterial>(
        "mooneyRivlin", 7.5, 0.2, 0.3, 0.4));

    std::vector<VolumetricMeshes::VolumetricMesh::Set> sets{
        VolumetricMeshes::VolumetricMesh::Set("allElements", std::set<int>{0})};
    std::vector<VolumetricMeshes::VolumetricMesh::Region> regions{
        VolumetricMeshes::VolumetricMesh::Region(0, 0)};

    return VolumetricMeshes::TetMesh(vertices, tets, std::move(materials), std::move(sets), std::move(regions));
}

void expectVertexMatches(const VolumetricMeshes::TetMesh& tet_mesh, const SimulationMesh& simulation_mesh, int vertex_id) {
    std::array<double, 3> actual_position = {};
    simulation_mesh.getVertex(vertex_id, actual_position.data());

    const Vec3d& expected_position = tet_mesh.getVertex(vertex_id);
    EXPECT_DOUBLE_EQ(actual_position[0], expected_position[0]);
    EXPECT_DOUBLE_EQ(actual_position[1], expected_position[1]);
    EXPECT_DOUBLE_EQ(actual_position[2], expected_position[2]);
}

}  // namespace

TEST(SceneToSimulationMeshTest, SceneToSimulationMeshPreservesTetTopology) {
    VolumetricMeshes::TetMesh tet_mesh = makeTwoTetMesh();

    std::unique_ptr<SimulationMesh> mesh = SceneToSimulationMesh::fromTetMesh(tet_mesh);

    ASSERT_NE(mesh, nullptr);
    EXPECT_EQ(mesh->getElementType(), SimulationMeshType::TET);
    EXPECT_EQ(mesh->getNumVertices(), tet_mesh.getNumVertices());
    EXPECT_EQ(mesh->getNumElements(), tet_mesh.getNumElements());
    EXPECT_EQ(mesh->getNumElementVertices(), tet_mesh.getNumElementVertices());

    for (int vertex = 0; vertex < tet_mesh.getNumVertices(); ++vertex) {
        expectVertexMatches(tet_mesh, *mesh, vertex);
    }

    for (int element = 0; element < tet_mesh.getNumElements(); ++element) {
        const int* indices = mesh->getVertexIndices(element);
        for (int local_vertex = 0; local_vertex < tet_mesh.getNumElementVertices(); ++local_vertex) {
            EXPECT_EQ(indices[local_vertex], tet_mesh.getVertexIndex(element, local_vertex));
        }
    }
}

TEST(SceneToSimulationMeshTest, SceneToSimulationMeshPreservesENuMaterialParams) {
    VolumetricMeshes::TetMesh tet_mesh = makeTwoTetMesh();

    std::unique_ptr<SimulationMesh> mesh = SceneToSimulationMesh::fromTetMesh(tet_mesh);

    ASSERT_NE(mesh, nullptr);
    for (int element = 0; element < tet_mesh.getNumElements(); ++element) {
        const auto* material = dynamic_cast<const SimulationMeshENuMaterial*>(mesh->getPrimaryMaterial(element));
        ASSERT_NE(material, nullptr);
        EXPECT_FALSE(mesh->hasSecondaryMaterial(element));
        EXPECT_DOUBLE_EQ(material->getE(), 1234.0);
        EXPECT_DOUBLE_EQ(material->getNu(), 0.31);
    }
}

TEST(SceneToSimulationMeshTest, SceneToSimulationMeshRejectsNonENuMaterial) {
    VolumetricMeshes::TetMesh tet_mesh = makeSingleTetMeshWithMooneyRivlinMaterial();

    EXPECT_THROW((void)SceneToSimulationMesh::fromTetMesh(tet_mesh), std::invalid_argument);
}

}  // namespace pgo::SolidDeformationModel::test
