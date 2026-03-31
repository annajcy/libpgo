#include "simulationMesh.h"
#include "sceneToSimulationMesh.h"
#include "deformationModelManager.h"
#include "tetMesh.h"
#include "triMeshGeo.h"
#include "pgoLogging.h"

#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <span>
#include <stdexcept>
#include <vector>

namespace pgo::SolidDeformationModel::test {
namespace {

std::unique_ptr<SimulationMesh> makeDirectTetSimulationMesh(const ElementMaterialBinding& binding,
                                                            const SimulationMeshMaterial* const* materials,
                                                            int numMaterials) {
    const std::array<SimulationMesh::Vertex, 4> vertices = {
        SimulationMesh::Vertex(0.0, 0.0, 0.0),
        SimulationMesh::Vertex(1.0, 0.0, 0.0),
        SimulationMesh::Vertex(0.0, 1.0, 0.0),
        SimulationMesh::Vertex(0.0, 0.0, 1.0),
    };
    const std::array<SimulationMesh::TetElement, 1> elements = {SimulationMesh::TetElement{0, 1, 2, 3}};
    const std::array<ElementMaterialBinding, 1> bindings = {binding};

    return SimulationMesh::createTet(vertices, elements, bindings,
                                     std::span<const SimulationMeshMaterial* const>(materials, numMaterials));
}

VolumetricMeshes::TetMesh makeSingleTetMesh() {
    return VolumetricMeshes::TetMesh(Vec3d(0.0, 0.0, 0.0), Vec3d(1.0, 0.0, 0.0), Vec3d(0.0, 1.0, 0.0),
                                     Vec3d(0.0, 0.0, 1.0));
}

Mesh::TriMeshGeo makeSingleTriangleMesh() {
    std::vector<Vec3d> vertices = {
        Vec3d(0.0, 0.0, 0.0),
        Vec3d(1.0, 0.0, 0.0),
        Vec3d(0.0, 1.0, 0.0),
    };
    std::vector<Vec3i> triangles = {Vec3i(0, 1, 2)};
    return Mesh::TriMeshGeo(std::move(vertices), std::move(triangles));
}

}  // namespace

TEST(SimulationMeshMaterialBindingTest, SingleMaterialBindingExposesPrimaryOnly) {
    const SimulationMeshENuMaterial primaryMaterial(1200.0, 0.42);
    const SimulationMeshMaterial*   materials[] = {&primaryMaterial};

    std::unique_ptr<SimulationMesh> mesh = makeDirectTetSimulationMesh({0, -1}, materials, 1);

    const auto* primary = dynamic_cast<const SimulationMeshENuMaterial*>(mesh->getPrimaryMaterial(0));
    ASSERT_NE(primary, nullptr);
    EXPECT_DOUBLE_EQ(primary->getE(), 1200.0);
    EXPECT_FALSE(mesh->hasSecondaryMaterial(0));
    EXPECT_THROW(mesh->getSecondaryMaterial(0), std::logic_error);
}

TEST(SimulationMeshMaterialBindingTest, DualMaterialBindingExposesPrimaryAndSecondary) {
    const SimulationMeshENuMaterial  primaryMaterial(1200.0, 0.42);
    const SimulationMeshHillMaterial secondaryMaterial(25.0, 1.3, 0.7);
    const SimulationMeshMaterial*    materials[] = {&primaryMaterial, &secondaryMaterial};

    std::unique_ptr<SimulationMesh> mesh = makeDirectTetSimulationMesh({0, 1}, materials, 2);

    const auto* primary = dynamic_cast<const SimulationMeshENuMaterial*>(mesh->getPrimaryMaterial(0));
    const auto* secondary = dynamic_cast<const SimulationMeshHillMaterial*>(mesh->getSecondaryMaterial(0));
    ASSERT_NE(primary, nullptr);
    ASSERT_NE(secondary, nullptr);
    EXPECT_TRUE(mesh->hasSecondaryMaterial(0));
    EXPECT_DOUBLE_EQ(primary->getNu(), 0.42);
    EXPECT_DOUBLE_EQ(secondary->getGamma(), 1.3);
}

TEST(SimulationMeshMaterialBindingTest, InvalidPrimaryBindingThrows) {
    const SimulationMeshENuMaterial primaryMaterial(1200.0, 0.42);
    const SimulationMeshMaterial*   materials[] = {&primaryMaterial};

    EXPECT_THROW((void)makeDirectTetSimulationMesh({-1, -1}, materials, 1), std::out_of_range);
    EXPECT_THROW((void)makeDirectTetSimulationMesh({1, -1}, materials, 1), std::out_of_range);
}

TEST(SimulationMeshMaterialBindingTest, CreateFromTetMeshBuildsPrimaryOnlyBinding) {
    VolumetricMeshes::TetMesh tetMesh = makeSingleTetMesh();

    std::unique_ptr<SimulationMesh> mesh = SceneToSimulationMesh::fromTetMesh(tetMesh);

    ASSERT_NE(mesh, nullptr);
    EXPECT_EQ(mesh->getElementType(), SimulationMeshType::TET);
    EXPECT_EQ(mesh->getNumElements(), 1);
    EXPECT_FALSE(mesh->hasSecondaryMaterial(0));
    EXPECT_NE(dynamic_cast<const SimulationMeshENuMaterial*>(mesh->getPrimaryMaterial(0)), nullptr);
}

TEST(SimulationMeshMaterialBindingTest, CreateTriangleFromTriMeshBuildsPrimaryOnlyBinding) {
    Mesh::TriMeshGeo            triMesh = makeSingleTriangleMesh();
    SimulationMeshENuhMaterial  material(900.0, 0.35, 0.01);

    std::unique_ptr<SimulationMesh> mesh = SceneToSimulationMesh::triangleFromTriMesh(triMesh, material);

    ASSERT_NE(mesh, nullptr);
    EXPECT_EQ(mesh->getElementType(), SimulationMeshType::TRIANGLE);
    EXPECT_EQ(mesh->getNumElements(), 1);
    EXPECT_FALSE(mesh->hasSecondaryMaterial(0));
    EXPECT_NE(dynamic_cast<const SimulationMeshENuhMaterial*>(mesh->getPrimaryMaterial(0)), nullptr);
}

TEST(SimulationMeshMaterialBindingTest, CreateShellFromTriMeshBuildsPrimaryOnlyBinding) {
    Mesh::TriMeshGeo            triMesh = makeSingleTriangleMesh();
    SimulationMeshENuhMaterial  material(900.0, 0.35, 0.01);

    std::unique_ptr<SimulationMesh> mesh = SceneToSimulationMesh::shellFromTriMesh(triMesh, material);

    ASSERT_NE(mesh, nullptr);
    EXPECT_EQ(mesh->getElementType(), SimulationMeshType::SHELL);
    EXPECT_EQ(mesh->getNumElements(), 1);
    EXPECT_FALSE(mesh->hasSecondaryMaterial(0));
}

TEST(SimulationMeshMaterialBindingTest, DeformationModelManagerStableNeoSmokeTest) {
    VolumetricMeshes::TetMesh tetMesh = makeSingleTetMesh();
    std::unique_ptr<SimulationMesh> mesh = SceneToSimulationMesh::fromTetMesh(tetMesh);
    Logging::init();

    DeformationModelManager dmm;
    dmm.setMesh(mesh.get());
    dmm.init(DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, DeformationModelElasticMaterial::STABLE_NEO, 0);

    EXPECT_NE(dmm.getDeformationModel(0), nullptr);
}

TEST(SimulationMeshMaterialBindingTest, DeformationModelManagerHillSmokeTest) {
    const SimulationMeshENuMaterial  primaryMaterial(1200.0, 0.42);
    const SimulationMeshHillMaterial secondaryMaterial(25.0, 1.3, 0.7);
    const SimulationMeshMaterial*    materials[] = {&primaryMaterial, &secondaryMaterial};
    std::unique_ptr<SimulationMesh>  mesh = makeDirectTetSimulationMesh({0, 1}, materials, 2);
    Logging::init();

    DeformationModelManager dmm;
    dmm.setMesh(mesh.get());
    dmm.init(DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, DeformationModelElasticMaterial::HILL_STABLE_NEO, 0);

    EXPECT_NE(dmm.getDeformationModel(0), nullptr);
}

TEST(SimulationMeshMaterialBindingTest, AssignElementUVsRejectsInvalidSpanSize) {
    const SimulationMeshENuMaterial primaryMaterial(1200.0, 0.42);
    const SimulationMeshMaterial* materials[] = {&primaryMaterial};
    std::unique_ptr<SimulationMesh> mesh = makeDirectTetSimulationMesh({0, -1}, materials, 1);
    const std::array<SimulationMesh::UV, 3> badUvs = {
        SimulationMesh::UV(0.0, 0.0),
        SimulationMesh::UV(1.0, 0.0),
        SimulationMesh::UV(0.0, 1.0),
    };

    EXPECT_THROW(mesh->assignElementUVs(badUvs), std::invalid_argument);
}

}  // namespace pgo::SolidDeformationModel::test
