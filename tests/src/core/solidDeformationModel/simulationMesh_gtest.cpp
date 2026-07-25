#include <gtest/gtest.h>

#include "simulation/simulationMesh.h"
#include "cubicMesh.h"
#include "tetMesh.h"
#include "triMeshGeo.h"
#include "volumetricMeshENuMaterial.h"
#include "volumetricMeshMooneyRivlinMaterial.h"
#include "volumetricMeshOrthotropicMaterial.h"

#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;
constexpr const char *kTorusVegPath = LIBPGO_TEST_TORUS_VEG;
}

TEST(SimulationMeshGTest, LoadsCubicMeshFromExampleFile)
{
  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<pgo::SolidDeformationModel::SimulationMesh> simMesh =
    pgo::SolidDeformationModel::loadCubicMesh(&cubicMesh);

  ASSERT_NE(simMesh, nullptr);
  EXPECT_EQ(simMesh->getElementType(), pgo::SolidDeformationModel::SimulationMeshType::CUBIC);
  EXPECT_EQ(simMesh->getNumVertices(), cubicMesh.getNumVertices());
  EXPECT_EQ(simMesh->getNumElements(), cubicMesh.getNumElements());
  EXPECT_EQ(simMesh->getNumElementVertices(), 8);

  for (int j = 0; j < 8; j++) {
    EXPECT_EQ(simMesh->getVertexIndex(0, j), cubicMesh.getVertexIndex(0, j));
  }

  double simPos[3];
  simMesh->getVertex(0, simPos);
  const pgo::Vec3d cubicPos = cubicMesh.getVertex(0);
  EXPECT_DOUBLE_EQ(simPos[0], cubicPos[0]);
  EXPECT_DOUBLE_EQ(simPos[1], cubicPos[1]);
  EXPECT_DOUBLE_EQ(simPos[2], cubicPos[2]);

  const auto &simMat = simMesh->requireElementField<
    pgo::SolidDeformationModel::SimulationMeshENuMaterial>().at(0);

  const auto *cubicMat = pgo::VolumetricMeshes::downcastENuMaterial(cubicMesh.getElementMaterial(0));
  ASSERT_NE(cubicMat, nullptr);
  EXPECT_DOUBLE_EQ(simMat.getE(), cubicMat->getE());
  EXPECT_DOUBLE_EQ(simMat.getNu(), cubicMat->getNu());
}

// Characterization: current tet ENu load path produces
// SimulationMeshENuMaterial payloads.
TEST(SimulationMeshGTest, TetLoadProducesENuMaterialPayloads)
{
  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  std::shared_ptr<pgo::SolidDeformationModel::SimulationMesh> simMesh =
    pgo::SolidDeformationModel::loadTetMesh(&tetMesh);

  ASSERT_NE(simMesh, nullptr);
  EXPECT_EQ(simMesh->getElementType(), pgo::SolidDeformationModel::SimulationMeshType::TET);

  // Every element must carry an ENu material payload.
  for (int ei = 0; ei < simMesh->getNumElements(); ei++) {
    const auto &simMat = simMesh->requireElementField<
      pgo::SolidDeformationModel::SimulationMeshENuMaterial>().at(ei);
    EXPECT_GT(simMat.getE(), 0.0);
    EXPECT_GT(simMat.getNu(), 0.0);
  }
}

TEST(SimulationMeshGTest, ShellLoadPreservesTriangleZeroAsNeighbor)
{
  using namespace pgo;
  using namespace pgo::SolidDeformationModel;

  std::vector<Vec3d> vertices{
    Vec3d(0.0, 0.0, 0.0),
    Vec3d(1.0, 0.0, 0.0),
    Vec3d(1.0, 1.0, 0.0),
    Vec3d(0.0, 1.0, 0.0),
  };
  std::vector<Vec3i> triangles{
    Vec3i(0, 1, 2),
    Vec3i(0, 2, 3),
  };
  Mesh::TriMeshGeo surface(
    std::move(vertices), std::move(triangles));
  SimulationMeshENuhMaterial material(1000.0, 0.4, 0.01);

  auto mesh = loadShellMesh(surface, material);

  ASSERT_NE(mesh, nullptr);
  ASSERT_EQ(mesh->getNumElements(), 2);
  ASSERT_EQ(mesh->getNumElementVertices(), 6);

  // Triangle 0 sees triangle 1 across local edge (2, 0), whose opposite
  // vertex is 3.
  EXPECT_EQ(mesh->getVertexIndex(0, 5), 3);
  // Triangle 1 sees triangle 0 across local edge (0, 2), whose opposite
  // vertex is 1. Triangle index 0 is a valid neighbor, not a boundary.
  EXPECT_EQ(mesh->getVertexIndex(1, 3), 1);
}

TEST(SimulationMeshGTest, TriangleLoadUsesOneMaterialPerTriangle)
{
  using namespace pgo;
  using namespace pgo::SolidDeformationModel;

  std::vector<Vec3d> vertices{
    Vec3d(0.0, 0.0, 0.0),
    Vec3d(1.0, 0.0, 0.0),
    Vec3d(1.0, 1.0, 0.0),
    Vec3d(0.0, 1.0, 0.0),
  };
  std::vector<Vec3i> triangles{
    Vec3i(0, 1, 2),
    Vec3i(0, 2, 3),
  };
  Mesh::TriMeshGeo surface(std::move(vertices), std::move(triangles));

  auto field = ElementField<SimulationMeshENuMaterial>::fromValues({
    SimulationMeshENuMaterial(1000.0, 0.4),
    SimulationMeshENuMaterial(2000.0, 0.35),
  });
  auto mesh = loadTriangleMesh(surface, std::move(field));

  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->getElementType(), SimulationMeshType::TRIANGLE);
  EXPECT_EQ(mesh->getNumElements(), 2);
  EXPECT_EQ(mesh->getNumElementVertices(), 3);
  EXPECT_DOUBLE_EQ(
    mesh->requireElementField<SimulationMeshENuMaterial>().at(1).getE(), 2000.0);
  EXPECT_EQ(mesh->getVertexIndex(1, 0), 0);
  EXPECT_EQ(mesh->getVertexIndex(1, 1), 2);
  EXPECT_EQ(mesh->getVertexIndex(1, 2), 3);
}

TEST(SimulationMeshGTest, EdgeQuadLoadAveragesSourceTriangleMaterials)
{
  using namespace pgo;
  using namespace pgo::SolidDeformationModel;

  std::vector<Vec3d> vertices{
    Vec3d(0.0, 0.0, 0.0),
    Vec3d(1.0, 0.0, 0.0),
    Vec3d(1.0, 1.0, 0.0),
    Vec3d(0.0, 1.0, 0.0),
  };
  std::vector<Vec3i> triangles{
    Vec3i(0, 1, 2),
    Vec3i(0, 2, 3),
  };
  Mesh::TriMeshGeo surface(std::move(vertices), std::move(triangles));

  auto field = ElementField<SimulationMeshENuhMaterial>::fromValues({
    SimulationMeshENuhMaterial(1000.0, 0.4, 0.01),
    SimulationMeshENuhMaterial(2000.0, 0.35, 0.03),
  });
  auto mesh = loadEdgeQuadMesh(surface, std::move(field));

  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->getElementType(), SimulationMeshType::EDGE_QUAD);
  EXPECT_EQ(mesh->getNumElements(), 1);
  EXPECT_EQ(mesh->getNumElementVertices(), 4);
  EXPECT_EQ(mesh->getVertexIndex(0, 0), 1);
  EXPECT_EQ(mesh->getVertexIndex(0, 1), 0);
  EXPECT_EQ(mesh->getVertexIndex(0, 2), 2);
  EXPECT_EQ(mesh->getVertexIndex(0, 3), 3);

  const auto &material =
    mesh->requireElementField<SimulationMeshENuhMaterial>().at(0);
  EXPECT_DOUBLE_EQ(material.getE(), 1500.0);
  EXPECT_DOUBLE_EQ(material.getNu(), 0.375);
  EXPECT_DOUBLE_EQ(material.geth(), 0.02);
}

TEST(SimulationMeshGTest, ShellMaterialFieldRejectsInvalidPaletteAndSize)
{
  using namespace pgo::SolidDeformationModel;

  std::vector<std::shared_ptr<const SimulationMeshENuhMaterial>> palette;
  palette.emplace_back(std::make_shared<const SimulationMeshENuhMaterial>(
    1000.0, 0.4, 0.01));
  EXPECT_THROW(
    ElementField<SimulationMeshENuhMaterial>::fromPalette(
      palette, std::vector<int>{1}),
    std::invalid_argument);

  std::vector<pgo::Vec3d> vertices{
    pgo::Vec3d(0.0, 0.0, 0.0),
    pgo::Vec3d(1.0, 0.0, 0.0),
    pgo::Vec3d(1.0, 1.0, 0.0),
    pgo::Vec3d(0.0, 1.0, 0.0),
  };
  std::vector<pgo::Vec3i> triangles{
    pgo::Vec3i(0, 1, 2),
    pgo::Vec3i(0, 2, 3),
  };
  pgo::Mesh::TriMeshGeo surface(std::move(vertices), std::move(triangles));
  auto field = ElementField<SimulationMeshENuhMaterial>::uniform(
    1, SimulationMeshENuhMaterial(1000.0, 0.4, 0.01));
  EXPECT_THROW(loadShellMesh(surface, std::move(field)), std::invalid_argument);
}

TEST(SimulationMeshGTest, ShellLoadAcceptsTypedMaterialPalette)
{
  using namespace pgo;
  using namespace pgo::SolidDeformationModel;

  std::vector<Vec3d> vertices{
    Vec3d(0.0, 0.0, 0.0),
    Vec3d(1.0, 0.0, 0.0),
    Vec3d(1.0, 1.0, 0.0),
    Vec3d(0.0, 1.0, 0.0),
  };
  std::vector<Vec3i> triangles{
    Vec3i(0, 1, 2),
    Vec3i(0, 2, 3),
  };
  Mesh::TriMeshGeo surface(std::move(vertices), std::move(triangles));
  std::vector<std::shared_ptr<const SimulationMeshENuhMaterial>> palette{
    std::make_shared<const SimulationMeshENuhMaterial>(1000.0, 0.4, 0.01),
    std::make_shared<const SimulationMeshENuhMaterial>(2000.0, 0.35, 0.02),
  };
  auto field = ElementField<SimulationMeshENuhMaterial>::fromPalette(
    std::move(palette), std::vector<int>{0, 1});

  auto mesh = loadShellMesh(surface, std::move(field));
  ASSERT_NE(mesh, nullptr);
  EXPECT_DOUBLE_EQ(
    mesh->requireElementField<SimulationMeshENuhMaterial>().at(0).getE(), 1000.0);
  EXPECT_DOUBLE_EQ(
    mesh->requireElementField<SimulationMeshENuhMaterial>().at(1).getE(), 2000.0);
}

TEST(SimulationMeshGTest, LoadsMooneyRivlinElementField)
{
  // Build a minimal tet mesh with one element and Mooney-Rivlin material.
  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
  const int elements[] = { 0, 1, 2, 3 };

  pgo::VolumetricMeshes::VolumetricMesh::MooneyRivlinMaterial mrMat(
    "mr_test", 1000.0, 0.5, 0.3, 0.1);
  const pgo::VolumetricMeshes::VolumetricMesh::Material *materials[] = { &mrMat };

  pgo::VolumetricMeshes::VolumetricMesh::Set set("all", std::set<int>{ 0 });
  pgo::VolumetricMeshes::VolumetricMesh::Region region(0, 0);

  pgo::VolumetricMeshes::TetMesh tetMesh(
    4, vertices, 1, elements, 1, materials, 1, &set, 1, &region);

  // Material exists at Vega level with correct type.
  const auto *mat = tetMesh.getElementMaterial(0);
  ASSERT_NE(mat, nullptr);
  EXPECT_EQ(mat->getType(), pgo::VolumetricMeshes::VolumetricMesh::Material::MOONEYRIVLIN);

  const auto *mrDowncast = pgo::VolumetricMeshes::downcastMooneyRivlinMaterial(
    tetMesh.getElementMaterial(0));
  ASSERT_NE(mrDowncast, nullptr);
  EXPECT_DOUBLE_EQ(mrDowncast->getmu01(), 0.5);
  EXPECT_DOUBLE_EQ(mrDowncast->getmu10(), 0.3);
  EXPECT_DOUBLE_EQ(mrDowncast->getv1(), 0.1);

  auto simMesh = pgo::SolidDeformationModel::loadTetMesh(&tetMesh);
  ASSERT_NE(simMesh, nullptr);
  const auto &simMaterial = simMesh->requireElementField<
    pgo::SolidDeformationModel::SimulationMeshMooneyRivlinMaterial>().at(0);
  EXPECT_DOUBLE_EQ(simMaterial.mu01(), 0.5);
  EXPECT_DOUBLE_EQ(simMaterial.mu10(), 0.3);
  EXPECT_DOUBLE_EQ(simMaterial.v1(), 0.1);
}

TEST(SimulationMeshGTest, RejectsInvalidMooneyRivlinParametersAtConversion)
{
  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  const int elements[] = {0, 1, 2, 3};
  pgo::VolumetricMeshes::VolumetricMesh::MooneyRivlinMaterial invalidMaterial(
    "invalid_mr", 1000.0, 0.0, 0.0, 0.1);
  const pgo::VolumetricMeshes::VolumetricMesh::Material *materials[] = {
    &invalidMaterial};
  pgo::VolumetricMeshes::VolumetricMesh::Set set("all", std::set<int>{0});
  pgo::VolumetricMeshes::VolumetricMesh::Region region(0, 0);
  pgo::VolumetricMeshes::TetMesh tetMesh(
    4, vertices, 1, elements, 1, materials, 1, &set, 1, &region);

  EXPECT_THROW(pgo::SolidDeformationModel::loadTetMesh(&tetMesh), std::invalid_argument);
}

// Characterization: Orthotropic material payload can be read from Vega
// but there is no corresponding deformation ElasticModel yet (no
// ElasticModel3DOrthotropicStVK exists).
TEST(SimulationMeshGTest, OrthotropicPayloadExistsAtVegaLevelButNoElasticModel)
{
  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
  const int elements[] = { 0, 1, 2, 3 };

  double R[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
  pgo::VolumetricMeshes::VolumetricMesh::OrthotropicMaterial orthoMat(
    "ortho_test", 1000.0, 1e6, 1e6, 1e6, 0.3, 0.3, 0.3, 0.4e6, 0.4e6, 0.4e6, R);
  const pgo::VolumetricMeshes::VolumetricMesh::Material *materials[] = { &orthoMat };

  pgo::VolumetricMeshes::VolumetricMesh::Set set("all", std::set<int>{ 0 });
  pgo::VolumetricMeshes::VolumetricMesh::Region region(0, 0);

  pgo::VolumetricMeshes::TetMesh tetMesh(
    4, vertices, 1, elements, 1, materials, 1, &set, 1, &region);

  const auto *mat = tetMesh.getElementMaterial(0);
  ASSERT_NE(mat, nullptr);
  EXPECT_EQ(mat->getType(), pgo::VolumetricMeshes::VolumetricMesh::Material::ORTHOTROPIC);

  const auto *orthoDowncast = pgo::VolumetricMeshes::downcastOrthotropicMaterial(
    tetMesh.getElementMaterial(0));
  ASSERT_NE(orthoDowncast, nullptr);
  EXPECT_DOUBLE_EQ(orthoDowncast->getE1(), 1e6);
  EXPECT_DOUBLE_EQ(orthoDowncast->getNu12(), 0.3);
  EXPECT_DOUBLE_EQ(orthoDowncast->getG12(), 0.4e6);

  // loadTetMesh only handles ENu; it will fail on Orthotropic.
  const auto *enuDowncast = pgo::VolumetricMeshes::downcastENuMaterial(
    tetMesh.getElementMaterial(0));
  EXPECT_EQ(enuDowncast, nullptr);

  // There is no ElasticModelConfig implementation for
  // Orthotropic, and no ElasticModel3DOrthotropicStVK class exists yet.
  // This test documents that Orthotropic is payload-only at the Vega
  // level and has not yet reached the solver deformation energy path.
}

// Hill and base material data are independent per-element fields. Orientation
// is supplied separately by an immutable MaterialFrameField.
TEST(SimulationMeshGTest, HillAndBaseMaterialFieldsAreIndependent)
{
  using namespace pgo::SolidDeformationModel;

  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    1.0, 1.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
    1.0, 0.0, 1.0,
    1.0, 1.0, 1.0,
    0.0, 1.0, 1.0,
  };
  const int elementVertices[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
  SimulationMeshENuMaterial baseMaterial(1200.0, 0.45);

  ElementFieldStore fields;
  fields.add(ElementField<SimulationMeshENuMaterial>::uniform(1, baseMaterial));
  fields.add(ElementField<SimulationMeshHillMaterial>::uniform(
    1, SimulationMeshHillMaterial(2500.0, 0.35, 1.0)));
  auto mesh = std::make_unique<SimulationMesh>(
    8, vertices, 1, 8, elementVertices, std::move(fields), SimulationMeshType::CUBIC);

  ASSERT_NE(mesh, nullptr);
  EXPECT_DOUBLE_EQ(mesh->requireElementField<SimulationMeshHillMaterial>().at(0).getEact(), 2500.0);
  EXPECT_DOUBLE_EQ(mesh->requireElementField<SimulationMeshENuMaterial>().at(0).getE(), 1200.0);

  // Orientation is not stored in SimulationMesh. The deformation builder
  // supplies a MaterialFrameField (GlobalAxes by default).
}

TEST(SimulationMeshGTest, HillFieldSupportsSpatiallyVaryingMaterialData)
{
  using namespace pgo::SolidDeformationModel;
  auto hill0 = std::make_shared<const SimulationMeshHillMaterial>(1000.0, 0.2, 0.8);
  auto hill1 = std::make_shared<const SimulationMeshHillMaterial>(2000.0, 0.4, 1.1);
  auto field = ElementField<SimulationMeshHillMaterial>::fromShared(
    4, std::vector<std::shared_ptr<const SimulationMeshHillMaterial>>{hill0, hill0, hill1, hill0});

  ASSERT_EQ(field.size(), 4);
  EXPECT_DOUBLE_EQ(field.at(0).getEact(), 1000.0);
  EXPECT_DOUBLE_EQ(field.at(1).getEact(), 1000.0);
  EXPECT_DOUBLE_EQ(field.at(2).getEact(), 2000.0);
  EXPECT_DOUBLE_EQ(field.at(3).getEact(), 1000.0);
}

TEST(ElementFieldStoreGTest, UsesExactTypeAndValidatesShape)
{
  using namespace pgo::SolidDeformationModel;

  ElementFieldStore store;
  store.add(ElementField<SimulationMeshENuMaterial>::uniform(
    2, SimulationMeshENuMaterial(1000.0, 0.4)));

  EXPECT_TRUE(store.contains<SimulationMeshENuMaterial>());
  EXPECT_FALSE(store.contains<SimulationMeshHillMaterial>());
  EXPECT_THROW(store.require<SimulationMeshHillMaterial>(), std::invalid_argument);
  EXPECT_THROW(
    store.add(ElementField<SimulationMeshENuMaterial>::uniform(
      2, SimulationMeshENuMaterial(2000.0, 0.3))),
    std::invalid_argument);
  EXPECT_THROW(
    store.add(ElementField<SimulationMeshHillMaterial>::uniform(
      3, SimulationMeshHillMaterial())),
    std::invalid_argument);
}

TEST(ElementFieldStoreGTest, SharedHandlesPreserveSpatialValues)
{
  using namespace pgo::SolidDeformationModel;
  auto first = std::make_shared<const SimulationMeshHillMaterial>(1000.0, 0.2, 0.8);
  auto second = std::make_shared<const SimulationMeshHillMaterial>(2000.0, 0.4, 1.1);
  auto field = ElementField<SimulationMeshHillMaterial>::fromShared(
    3, { first, first, second });

  ElementFieldStore store;
  store.add(std::move(field));
  EXPECT_DOUBLE_EQ(store.require<SimulationMeshHillMaterial>().at(0).getEact(), 1000.0);
  EXPECT_DOUBLE_EQ(store.require<SimulationMeshHillMaterial>().at(1).getEact(), 1000.0);
  EXPECT_DOUBLE_EQ(store.require<SimulationMeshHillMaterial>().at(2).getEact(), 2000.0);
}
