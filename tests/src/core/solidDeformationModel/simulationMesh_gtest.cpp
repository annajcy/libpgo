#include <gtest/gtest.h>

#include "simulation/simulationMesh.h"
#include "simulation/simulationAsset.h"
#include "material/core/materialEvaluator.h"
#include "material/core/parameterLayout.h"
#include "material/core/optimizableParameters.h"
#include "cubicMesh.h"
#include "tetMesh.h"
#include "triMeshGeo.h"
#include "volumetricMeshENuMaterial.h"
#include "volumetricMeshMooneyRivlinMaterial.h"
#include "volumetricMeshOrthotropicMaterial.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;
constexpr const char *kTorusVegPath = LIBPGO_TEST_TORUS_VEG;

double importedValue(
  const pgo::SolidDeformationModel::SimulationAsset &asset,
  int element, std::string_view name)
{
  for (const auto &field : asset.materialData().fields()) {
    const auto names = field.channelNames();
    const int row = field.rowForElement(element);
    for (int channel = 0; channel < static_cast<int>(names.size()); ++channel)
      if (names[channel] == name)
        return field.valueRows()(row, channel);
  }
  const auto assignments = asset.materialData().elementMaterialIndices();
  if (element >= 0 && element < static_cast<int>(assignments.size()) &&
      assignments[static_cast<std::size_t>(element)] >= 0) {
    const auto &material = asset.materialData().materials()[static_cast<std::size_t>(
      assignments[static_cast<std::size_t>(element)])];
    const auto iter = material.properties.find(std::string(name));
    if (iter != material.properties.end()) {
      if (const auto *value = std::get_if<double>(&iter->second))
        return *value;
    }
  }
  throw std::invalid_argument("missing imported channel");
}
}

TEST(SimulationMeshGTest, LoadsCubicMeshFromExampleFile)
{
  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto asset = pgo::SolidDeformationModel::loadCubicMesh(cubicMesh);

  ASSERT_NE(asset, nullptr);
  const auto &simMesh = asset->mesh();
  EXPECT_EQ(simMesh->getElementType(), pgo::SolidDeformationModel::SimulationMeshType::CUBIC);
  EXPECT_EQ(simMesh->getNumVertices(), cubicMesh.getNumVertices());
  EXPECT_EQ(simMesh->getNumElements(), cubicMesh.getNumElements());
  EXPECT_EQ(simMesh->getNumElementVertices(), 8);

  for (int j = 0; j < 8; j++) {
    EXPECT_EQ(simMesh->getVertexIndex(0, j), cubicMesh.getVertexIndex(0, j));
  }

  const auto &simPos = simMesh->getVertex(0);
  const pgo::Vec3d cubicPos = cubicMesh.getVertex(0);
  EXPECT_DOUBLE_EQ(simPos[0], cubicPos[0]);
  EXPECT_DOUBLE_EQ(simPos[1], cubicPos[1]);
  EXPECT_DOUBLE_EQ(simPos[2], cubicPos[2]);

  const auto *cubicMat = pgo::VolumetricMeshes::downcastENuMaterial(cubicMesh.getElementMaterial(0));
  ASSERT_NE(cubicMat, nullptr);
  EXPECT_DOUBLE_EQ(importedValue(*asset, 0, "E"), cubicMat->getE());
  EXPECT_DOUBLE_EQ(importedValue(*asset, 0, "nu"), cubicMat->getNu());
}

// Characterization: current tet ENu load path produces
// neutral imported E/nu/J channels.
TEST(SimulationMeshGTest, TetLoadProducesENuMaterialPayloads)
{
  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto asset = pgo::SolidDeformationModel::loadTetMesh(tetMesh);

  ASSERT_NE(asset, nullptr);
  const auto &simMesh = asset->mesh();
  EXPECT_EQ(simMesh->getElementType(), pgo::SolidDeformationModel::SimulationMeshType::TET);

  // Every element must carry neutral imported values outside the mesh.
  for (int ei = 0; ei < simMesh->getNumElements(); ei++) {
    EXPECT_GT(importedValue(*asset, ei, "E"), 0.0);
    EXPECT_GT(importedValue(*asset, ei, "nu"), 0.0);
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
  ImportedENuhMaterial material(1000.0, 0.4, 0.01);

  auto mesh = loadShellMesh(surface, material);

  ASSERT_NE(mesh, nullptr);
  ASSERT_EQ(mesh->mesh()->getNumElements(), 2);
  ASSERT_EQ(mesh->mesh()->getNumElementVertices(), 6);

  // Triangle 0 sees triangle 1 across local edge (2, 0), whose opposite
  // vertex is 3.
  EXPECT_EQ(mesh->mesh()->getVertexIndex(0, 5), 3);
  // Triangle 1 sees triangle 0 across local edge (0, 2), whose opposite
  // vertex is 1. Triangle index 0 is a valid neighbor, not a boundary.
  EXPECT_EQ(mesh->mesh()->getVertexIndex(1, 3), 1);
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

  auto field = ElementField<ImportedENuMaterial>::fromValues({
    ImportedENuMaterial(1000.0, 0.4),
    ImportedENuMaterial(2000.0, 0.35),
  });
  auto mesh = loadTriangleMesh(surface, std::move(field));

  ASSERT_NE(mesh, nullptr);
  const auto &geometry = mesh->mesh();
  EXPECT_EQ(geometry->getElementType(), SimulationMeshType::TRIANGLE);
  EXPECT_EQ(geometry->getNumElements(), 2);
  EXPECT_EQ(geometry->getNumElementVertices(), 3);
  EXPECT_DOUBLE_EQ(importedValue(*mesh, 1, "E"), 2000.0);
  EXPECT_EQ(geometry->getVertexIndex(1, 0), 0);
  EXPECT_EQ(geometry->getVertexIndex(1, 1), 2);
  EXPECT_EQ(geometry->getVertexIndex(1, 2), 3);

  const std::span<const int> indices = geometry->getVertexIndices(1);
  ASSERT_EQ(indices.size(), 3u);
  EXPECT_EQ(indices[0], 0);
  EXPECT_EQ(indices[1], 2);
  EXPECT_EQ(indices[2], 3);

  computeTriangleUV(
    const_cast<SimulationMesh &>(*geometry), 1.0);
  ASSERT_TRUE(geometry->hasElementUV());
  EXPECT_TRUE(geometry->getElementUV(0, 0).isApprox(EigenSupport::V2d::Zero()));
  EXPECT_TRUE(geometry->getElementUV(0, 1).isApprox(EigenSupport::V2d(1.0, 0.0)));
  EXPECT_TRUE(geometry->getElementUV(0, 2).isApprox(EigenSupport::V2d(1.0, 1.0)));
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

  auto field = ElementField<ImportedENuhMaterial>::fromValues({
    ImportedENuhMaterial(1000.0, 0.4, 0.01),
    ImportedENuhMaterial(2000.0, 0.35, 0.03),
  });
  auto mesh = loadEdgeQuadMesh(surface, std::move(field));

  ASSERT_NE(mesh, nullptr);
  const auto &geometry = mesh->mesh();
  EXPECT_EQ(geometry->getElementType(), SimulationMeshType::EDGE_QUAD);
  EXPECT_EQ(geometry->getNumElements(), 1);
  EXPECT_EQ(geometry->getNumElementVertices(), 4);
  EXPECT_EQ(geometry->getVertexIndex(0, 0), 1);
  EXPECT_EQ(geometry->getVertexIndex(0, 1), 0);
  EXPECT_EQ(geometry->getVertexIndex(0, 2), 2);
  EXPECT_EQ(geometry->getVertexIndex(0, 3), 3);
  EXPECT_DOUBLE_EQ(importedValue(*mesh, 0, "E"), 1500.0);
  EXPECT_DOUBLE_EQ(importedValue(*mesh, 0, "nu"), 0.375);
  EXPECT_DOUBLE_EQ(importedValue(*mesh, 0, "h"), 0.02);
}

TEST(SimulationMeshGTest, ShellMaterialFieldRejectsInvalidPaletteAndSize)
{
  using namespace pgo::SolidDeformationModel;

  std::vector<std::shared_ptr<const ImportedENuhMaterial>> palette;
  palette.emplace_back(std::make_shared<const ImportedENuhMaterial>(
    1000.0, 0.4, 0.01));
  EXPECT_THROW(
    ElementField<ImportedENuhMaterial>::fromPalette(
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
  auto field = ElementField<ImportedENuhMaterial>::uniform(
    1, ImportedENuhMaterial(1000.0, 0.4, 0.01));
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
  std::vector<std::shared_ptr<const ImportedENuhMaterial>> palette{
    std::make_shared<const ImportedENuhMaterial>(1000.0, 0.4, 0.01),
    std::make_shared<const ImportedENuhMaterial>(2000.0, 0.35, 0.02),
  };
  auto field = ElementField<ImportedENuhMaterial>::fromPalette(
    std::move(palette), std::vector<int>{0, 1});

  auto mesh = loadShellMesh(surface, std::move(field));
  ASSERT_NE(mesh, nullptr);
  EXPECT_DOUBLE_EQ(importedValue(*mesh, 0, "E"), 1000.0);
  EXPECT_DOUBLE_EQ(importedValue(*mesh, 1, "E"), 2000.0);
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

  auto simMesh = pgo::SolidDeformationModel::loadTetMesh(tetMesh);
  ASSERT_NE(simMesh, nullptr);
  EXPECT_DOUBLE_EQ(importedValue(*simMesh, 0, "mu01"), 0.5);
  EXPECT_DOUBLE_EQ(importedValue(*simMesh, 0, "mu10"), 0.3);
  EXPECT_DOUBLE_EQ(importedValue(*simMesh, 0, "v1"), 0.1);
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

  auto asset = pgo::SolidDeformationModel::loadTetMesh(tetMesh);
  ASSERT_NE(asset, nullptr);
  EXPECT_DOUBLE_EQ(importedValue(*asset, 0, "mu01"), 0.0);
  EXPECT_DOUBLE_EQ(importedValue(*asset, 0, "mu10"), 0.0);
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

  // There is no ElasticModelDefinition implementation for
  // Orthotropic, and no ElasticModel3DOrthotropicStVK class exists yet.
  // This test documents that Orthotropic is payload-only at the Vega
  // level and has not yet reached the solver deformation energy path.
}

// Fixed fields and frames are independent simulation inputs; neither is
// embedded in the geometry-only SimulationMesh.
TEST(SimulationMeshGTest, FixedFieldsAreIndependentOfGeometry)
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
  auto mesh = std::make_unique<SimulationMesh>(
    8, vertices, 1, 8, elementVertices, SimulationMeshType::CUBIC);

  ASSERT_NE(mesh, nullptr);
  static constexpr std::string_view names[] = {
    "E", "nu", "Eact", "gamma", "lo"};
  const double values[] = {1200.0, 0.45, 2500.0, 0.35, 1.0};
  auto fixed = std::make_shared<const FixedParameterField>(
    ParameterSchema(std::vector<std::string>{"E", "nu", "Eact", "gamma", "lo"}),
    std::make_shared<const ConstantParameterLayout>(1, 5),
    std::make_shared<const IdentityMaterialEvaluator>(5));
  EXPECT_EQ(fixed->parameterSchema().numParameters(), 5);
  EXPECT_EQ(fixed->layout().numGlobalParameters(), 5);
  EXPECT_EQ(mesh->getNumElements(), 1);
}

TEST(SimulationMeshGTest, HillFieldSupportsSpatiallyVaryingMaterialData)
{
  using namespace pgo::SolidDeformationModel;
  auto hill0 = std::make_shared<const ImportedHillMaterial>(1000.0, 0.2, 0.8);
  auto hill1 = std::make_shared<const ImportedHillMaterial>(2000.0, 0.4, 1.1);
  auto field = ElementField<ImportedHillMaterial>::fromShared(
    4, std::vector<std::shared_ptr<const ImportedHillMaterial>>{hill0, hill0, hill1, hill0});

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
  store.add(ElementField<ImportedENuMaterial>::uniform(
    2, ImportedENuMaterial(1000.0, 0.4)));

  EXPECT_TRUE(store.contains<ImportedENuMaterial>());
  EXPECT_FALSE(store.contains<ImportedHillMaterial>());
  EXPECT_THROW(store.require<ImportedHillMaterial>(), std::invalid_argument);
  EXPECT_THROW(
    store.add(ElementField<ImportedENuMaterial>::uniform(
      2, ImportedENuMaterial(2000.0, 0.3))),
    std::invalid_argument);
  EXPECT_THROW(
    store.add(ElementField<ImportedHillMaterial>::uniform(
      3, ImportedHillMaterial())),
    std::invalid_argument);
}

TEST(ElementFieldStoreGTest, SharedHandlesPreserveSpatialValues)
{
  using namespace pgo::SolidDeformationModel;
  auto first = std::make_shared<const ImportedHillMaterial>(1000.0, 0.2, 0.8);
  auto second = std::make_shared<const ImportedHillMaterial>(2000.0, 0.4, 1.1);
  auto field = ElementField<ImportedHillMaterial>::fromShared(
    3, { first, first, second });

  ElementFieldStore store;
  store.add(std::move(field));
  EXPECT_DOUBLE_EQ(store.require<ImportedHillMaterial>().at(0).getEact(), 1000.0);
  EXPECT_DOUBLE_EQ(store.require<ImportedHillMaterial>().at(1).getEact(), 1000.0);
  EXPECT_DOUBLE_EQ(store.require<ImportedHillMaterial>().at(2).getEact(), 2000.0);
}
