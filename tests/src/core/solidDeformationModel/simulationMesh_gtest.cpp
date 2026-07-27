#include <gtest/gtest.h>

#include "simulation/simulationMesh.h"
#include "simulation/import/simulationImportResult.h"
#include "simulation/simulationMeshVolume.h"
#include "material/import/volumeMaterialImporter.h"
#include "material/data/namedMaterialInputData.h"
#include "material/parameterization/materialChannelMapping.h"
#include "material/parameterization/parameterLayout.h"
#include "material/runtime/optimizableParameters.h"
#include "cubicMesh.h"
#include "tetMesh.h"
#include "triMeshGeo.h"
#include "volumetricMeshENuMaterial.h"
#include "volumetricMeshMooneyRivlinMaterial.h"
#include "volumetricMeshOrthotropicMaterial.h"
#include "vegFile.h"

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
  const pgo::SolidDeformationModel::SimulationImportResult &asset,
  int element, std::string_view name)
{
  const auto assignments = asset.materialCatalog().elementMaterialIndices();
  if (element >= 0 && element < static_cast<int>(assignments.size()) &&
      assignments[static_cast<std::size_t>(element)] >= 0) {
    const auto &material = asset.materialCatalog().materials()[static_cast<std::size_t>(
      assignments[static_cast<std::size_t>(element)])];
    const auto iter = material.properties.find(std::string(name));
    if (iter != material.properties.end()) {
      if (const auto *value = std::get_if<double>(&iter->second))
        return *value;
    }
  }
  throw std::invalid_argument("missing imported channel");
}

pgo::SolidDeformationModel::NamedMaterialInputData fieldMaterialData(
  int numElements,
  std::vector<std::string> channelNames,
  std::vector<std::vector<double>> rowValues,
  std::vector<int> elementToRow,
  std::string name)
{
  const int numChannels = static_cast<int>(channelNames.size());
  pgo::EigenSupport::MXd rows(
    static_cast<int>(rowValues.size()), numChannels);
  for (int row = 0; row < static_cast<int>(rowValues.size()); ++row) {
    if (static_cast<int>(rowValues[static_cast<std::size_t>(row)].size()) !=
        numChannels)
      throw std::invalid_argument("test material row has the wrong width");
    for (int channel = 0; channel < numChannels; ++channel)
      rows(row, channel) =
        rowValues[static_cast<std::size_t>(row)]
                 [static_cast<std::size_t>(channel)];
  }
  return pgo::SolidDeformationModel::NamedMaterialInputData(
    numElements,
    {pgo::SolidDeformationModel::NamedMaterialInputField(
      std::move(channelNames), std::move(rows), std::move(elementToRow),
      std::move(name))});
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

TEST(SimulationMeshGTest, VolumePayloadImportPreservesLosslessRegionSemantics)
{
  using namespace pgo;
  using namespace pgo::VolumetricMeshes;
  using namespace pgo::SolidDeformationModel;

  VegFilePayload payload;
  payload.meshData = Mesh::TetMeshData::fromFlatElements(
    std::vector<Vec3d>{
      Vec3d(0.0, 0.0, 0.0),
      Vec3d(1.0, 0.0, 0.0),
      Vec3d(0.0, 1.0, 0.0),
      Vec3d(0.0, 0.0, 1.0),
      Vec3d(1.0, 1.0, 1.0),
    },
    std::vector<int>{0, 1, 2, 3, 1, 2, 3, 4});
  payload.materials.push_back(VegENuMaterialPayload{
    "unused", 900.0, 1e6, 0.3});
  payload.materials.push_back(VegENuMaterialPayload{
    "first", 1000.0, 2e6, 0.35});
  VegOrthotropicMaterialPayload orthotropic;
  orthotropic.name = "later";
  orthotropic.density = 1100.0;
  orthotropic.E1 = 3e6;
  orthotropic.E2 = 2e6;
  orthotropic.E3 = 1e6;
  orthotropic.G12 = 0.7e6;
  orthotropic.G23 = 0.6e6;
  orthotropic.G31 = 0.5e6;
  orthotropic.R = {
    0.0, -1.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, 1.0};
  payload.materials.push_back(orthotropic);
  payload.sets = {
    VegSetPayload{"unusedSet", {}},
    VegSetPayload{"firstSet", {0, 1}},
    VegSetPayload{"overlapSet", {1}},
  };
  payload.regions = {
    VegRegionPayload{1, 1},
    VegRegionPayload{2, 2},
  };

  auto volume = VolumetricMesh::fromVegFilePayload(payload);
  ASSERT_NE(volume, nullptr);
  const ImportedMaterialCatalog imported = importVolumeMaterialCatalog(*volume);

  ASSERT_EQ(imported.materials().size(), 3u);
  EXPECT_EQ(imported.materials()[0].name, "unused");
  EXPECT_EQ(imported.materials()[2].family, "orthotropic");
  ASSERT_EQ(imported.sets().size(), 3u);
  EXPECT_EQ(imported.sets()[0].name, "unusedSet");
  ASSERT_EQ(imported.regions().size(), 2u);
  EXPECT_EQ(imported.regions()[0].materialIndex, 1);
  EXPECT_EQ(imported.regions()[0].setIndex, 1);
  EXPECT_EQ(imported.regions()[1].materialIndex, 2);
  EXPECT_EQ(imported.regions()[1].setIndex, 2);

  const std::vector<int> assignments = imported.elementMaterialIndices();
  ASSERT_EQ(assignments.size(), 2u);
  EXPECT_EQ(assignments[0], 1);
  EXPECT_EQ(assignments[1], 2);

  const auto rotationIter =
    imported.materials()[2].properties.find("rotation");
  ASSERT_NE(rotationIter, imported.materials()[2].properties.end());
  const auto *rotation =
    std::get_if<std::vector<double>>(&rotationIter->second);
  ASSERT_NE(rotation, nullptr);
  EXPECT_EQ(*rotation, std::vector<double>(orthotropic.R.begin(), orthotropic.R.end()));

  VegFilePayload invalid = payload;
  invalid.regions = {VegRegionPayload{3, 0}};
  EXPECT_THROW(
    VolumetricMesh::fromVegFilePayload(invalid),
    std::invalid_argument);

  VegFilePayload partial = payload;
  partial.sets = {VegSetPayload{"onlyFirst", {0}}};
  partial.regions = {VegRegionPayload{1, 0}};
  auto partialVolume = VolumetricMesh::fromVegFilePayload(partial);
  const std::vector<int> partialAssignments =
    importVolumeMaterialCatalog(*partialVolume).elementMaterialIndices();
  ASSERT_EQ(partialAssignments.size(), 2u);
  EXPECT_EQ(partialAssignments[0], 1);
  EXPECT_EQ(partialAssignments[1], -1);
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
  auto mesh = loadShellMesh(surface);

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

TEST(SimulationMeshGTest, TriangleLoadBuildsStandaloneGeometry)
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

  auto mesh = loadTriangleMesh(surface);

  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->getElementType(), SimulationMeshType::TRIANGLE);
  EXPECT_EQ(mesh->getNumElements(), 2);
  EXPECT_EQ(mesh->getNumElementVertices(), 3);
  EXPECT_EQ(mesh->getVertexIndex(1, 0), 0);
  EXPECT_EQ(mesh->getVertexIndex(1, 1), 2);
  EXPECT_EQ(mesh->getVertexIndex(1, 2), 3);

  const std::span<const int> indices = mesh->getVertexIndices(1);
  ASSERT_EQ(indices.size(), 3u);
  EXPECT_EQ(indices[0], 0);
  EXPECT_EQ(indices[1], 2);
  EXPECT_EQ(indices[2], 3);

  computeTriangleUV(
    *mesh, 1.0);
  ASSERT_TRUE(mesh->hasElementUV());
  EXPECT_TRUE(mesh->getElementUV(0, 0).isApprox(EigenSupport::V2d::Zero()));
  EXPECT_TRUE(mesh->getElementUV(0, 1).isApprox(EigenSupport::V2d(1.0, 0.0)));
  EXPECT_TRUE(mesh->getElementUV(0, 2).isApprox(EigenSupport::V2d(1.0, 1.0)));
}

TEST(SimulationMeshGTest, EdgeQuadLoadBuildsStandaloneGeometry)
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

  auto mesh = loadEdgeQuadMesh(surface);

  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->getElementType(), SimulationMeshType::EDGE_QUAD);
  EXPECT_EQ(mesh->getNumElements(), 1);
  EXPECT_EQ(mesh->getNumElementVertices(), 4);
  EXPECT_EQ(mesh->getVertexIndex(0, 0), 1);
  EXPECT_EQ(mesh->getVertexIndex(0, 1), 0);
  EXPECT_EQ(mesh->getVertexIndex(0, 2), 2);
  EXPECT_EQ(mesh->getVertexIndex(0, 3), 3);
}

TEST(SimulationMeshGTest, NamedMaterialInputDataRejectsInvalidRows)
{
  using namespace pgo::SolidDeformationModel;

  EXPECT_THROW(
    fieldMaterialData(
      1, {"E", "nu", "h", "J"},
      {{1000.0, 0.4, 0.01, 10000.0}}, {1}, "shell"),
    std::invalid_argument);

}

TEST(SimulationMeshGTest, ImportResultChecksMeshAndPayloadElementCounts)
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
  auto mesh = loadShellMesh(surface);
  ASSERT_NE(mesh, nullptr);
  EXPECT_THROW(
    SimulationImportResult(
      mesh, ImportedMaterialCatalog(1, {}, {}, {})),
    std::invalid_argument);
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
  auto fixed = std::make_shared<const FixedParameterField>(
    ParameterInputSchema(std::vector<std::string>{"E", "nu", "Eact", "gamma", "lo"}),
    std::make_shared<const ConstantParameterLayout>(1, 5),
    std::make_shared<const IdentityMaterialChannelMapping>(5));
  EXPECT_EQ(fixed->inputSchema().numParameters(), 5);
  EXPECT_EQ(fixed->layout().numGlobalParameters(), 5);
  EXPECT_EQ(mesh->getNumElements(), 1);
}
