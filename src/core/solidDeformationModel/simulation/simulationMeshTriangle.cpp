#include "simulation/simulationMeshEdgeQuad.h"
#include "simulation/simulationMeshTriangle.h"

#include "EigenSupport.h"
#include "pgoLogging.h"
#include "triMeshGeo.h"

#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

using namespace pgo::SolidDeformationModel;

namespace
{
struct EdgeQuadTopology
{
  std::vector<int> elementVertexIndices;
  std::vector<std::pair<int, int>> sourceTriangles;
};

EdgeQuadTopology buildEdgeQuadTopology(const pgo::Mesh::TriMeshGeo &triMeshGeo)
{
  using EdgeIndex = std::pair<int, int>;
  std::map<EdgeIndex, std::array<int, 3>> edgeTriangles;

  for (int tri = 0; tri < triMeshGeo.numTriangles(); tri++) {
    for (int edge = 0; edge < 3; edge++) {
      int v0 = triMeshGeo.tri(tri)[edge];
      int v1 = triMeshGeo.tri(tri)[(edge + 1) % 3];
      if (v0 > v1)
        std::swap(v0, v1);

      auto iter = edgeTriangles.find({v0, v1});
      if (iter != edgeTriangles.end()) {
        PGO_ALOG(iter->second[2] < 2);
        iter->second[1] = tri;
        iter->second[2]++;
      }
      else {
        edgeTriangles.emplace(
          std::make_pair(v0, v1), std::array<int, 3>{tri, -1, 1});
      }
    }
  }

  EdgeQuadTopology topology;
  topology.elementVertexIndices.reserve(edgeTriangles.size() * 4);
  topology.sourceTriangles.reserve(edgeTriangles.size());

  for (const auto &entry : edgeTriangles) {
    const int v0 = entry.first.first;
    const int v1 = entry.first.second;
    const auto &triangles = entry.second;
    if (triangles[2] < 2)
      continue;

    int firstVertex = 0;
    for (; firstVertex < 3; firstVertex++) {
      if (triMeshGeo.tri(triangles[0])[firstVertex] == v0)
        break;
    }
    PGO_ALOG(firstVertex < 3);

    int nextVertex = (firstVertex + 1) % 3;
    int opposite0 = -1;
    int opposite1 = -1;
    if (triMeshGeo.tri(triangles[0])[nextVertex] == v1) {
      opposite0 = triMeshGeo.tri(triangles[0])[(nextVertex + 1) % 3];
      opposite1 = pgo::Mesh::getTriangleVertexOppositeEdge(
        triMeshGeo.tri(triangles[1]), v0, v1);
    }
    else {
      nextVertex = 0;
      for (; nextVertex < 3; nextVertex++) {
        if (triMeshGeo.tri(triangles[1])[nextVertex] == v1)
          break;
      }
      PGO_ALOG(nextVertex < 3);
      opposite1 = triMeshGeo.tri(triangles[1])[(nextVertex + 1) % 3];
      opposite0 = pgo::Mesh::getTriangleVertexOppositeEdge(
        triMeshGeo.tri(triangles[0]), v0, v1);
    }
    PGO_ALOG(opposite0 >= 0 && opposite1 >= 0);

    topology.elementVertexIndices.emplace_back(opposite0);
    topology.elementVertexIndices.emplace_back(v0);
    topology.elementVertexIndices.emplace_back(v1);
    topology.elementVertexIndices.emplace_back(opposite1);
    topology.sourceTriangles.emplace_back(triangles[0], triangles[1]);
  }
  return topology;
}

std::vector<double> copyTriMeshVertices(const pgo::Mesh::TriMeshGeo &triMeshGeo)
{
  std::vector<double> vertices;
  vertices.reserve(static_cast<std::size_t>(triMeshGeo.numVertices()) * 3);
  for (int vi = 0; vi < triMeshGeo.numVertices(); vi++) {
    vertices.emplace_back(triMeshGeo.pos(vi)[0]);
    vertices.emplace_back(triMeshGeo.pos(vi)[1]);
    vertices.emplace_back(triMeshGeo.pos(vi)[2]);
  }
  return vertices;
}

ElementField<ImportedENuMaterial> averageEdgeQuadMaterials(
  const EdgeQuadTopology &topology,
  const ElementField<ImportedENuMaterial> &triangleMaterials)
{
  std::vector<ImportedENuMaterial> materials;
  materials.reserve(topology.sourceTriangles.size());
  for (const auto &source : topology.sourceTriangles) {
    const auto &mat0 = triangleMaterials.at(source.first);
    const auto &mat1 = triangleMaterials.at(source.second);
    materials.emplace_back(
      0.5 * (mat0.getE() + mat1.getE()),
      0.5 * (mat0.getNu() + mat1.getNu()),
      0.5 * (mat0.getCompressionRatio() + mat1.getCompressionRatio()));
  }
  return ElementField<ImportedENuMaterial>::fromValues(std::move(materials));
}

ElementField<ImportedENuhMaterial> averageEdgeQuadMaterials(
  const EdgeQuadTopology &topology,
  const ElementField<ImportedENuhMaterial> &triangleMaterials)
{
  std::vector<ImportedENuhMaterial> materials;
  materials.reserve(topology.sourceTriangles.size());
  for (const auto &source : topology.sourceTriangles) {
    const auto &mat0 = triangleMaterials.at(source.first);
    const auto &mat1 = triangleMaterials.at(source.second);
    materials.emplace_back(
      0.5 * (mat0.getE() + mat1.getE()),
      0.5 * (mat0.getNu() + mat1.getNu()),
      0.5 * (mat0.geth() + mat1.geth()),
      0.5 * (mat0.getCompressionRatio() + mat1.getCompressionRatio()));
  }
  return ElementField<ImportedENuhMaterial>::fromValues(std::move(materials));
}

template<class Material>
std::unique_ptr<SimulationAsset> buildEdgeQuadSimulationMesh(
  const pgo::Mesh::TriMeshGeo &triMeshGeo,
  ElementField<Material> triangleMaterials)
{
  if (triangleMaterials.size() != triMeshGeo.numTriangles())
    throw std::invalid_argument(
      "edge-quad material field size must match the triangle count");

  EdgeQuadTopology topology = buildEdgeQuadTopology(triMeshGeo);
  std::vector<double> vertices = copyTriMeshVertices(triMeshGeo);
  auto averaged = averageEdgeQuadMaterials(topology, triangleMaterials);
  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(averaged.size()) * (std::is_same_v<Material, ImportedENuhMaterial> ? 4 : 3));
  for (int element = 0; element < static_cast<int>(averaged.size()); ++element) {
    const auto &material = averaged.at(element);
    values.push_back(material.getE());
    values.push_back(material.getNu());
    if constexpr (std::is_same_v<Material, ImportedENuhMaterial>)
      values.push_back(material.geth());
    values.push_back(material.getCompressionRatio());
  }
  static constexpr std::string_view enuNames[] = {"E", "nu", "J"};
  static constexpr std::string_view enuhNames[] = {"E", "nu", "h", "J"};
  auto mesh = std::make_shared<SimulationMesh>(
    triMeshGeo.numVertices(), vertices,
    static_cast<int>(topology.sourceTriangles.size()), 4,
    topology.elementVertexIndices,
    SimulationMeshType::EDGE_QUAD);
  const int numElements = static_cast<int>(averaged.size());
  const int numChannels = std::is_same_v<Material, ImportedENuhMaterial> ? 4 : 3;
  pgo::EigenSupport::MXd rows(numElements, numChannels);
  for (int element = 0; element < numElements; ++element)
    for (int channel = 0; channel < numChannels; ++channel)
      rows(element, channel) = values[static_cast<std::size_t>(element) * numChannels + channel];
  const auto sourceNames = std::is_same_v<Material, ImportedENuhMaterial> ?
    std::span<const std::string_view>(enuhNames, 4) :
    std::span<const std::string_view>(enuNames, 3);
  std::vector<std::string> channelNames;
  channelNames.reserve(sourceNames.size());
  for (const auto name : sourceNames)
    channelNames.emplace_back(name);
  std::vector<int> elementToRow(static_cast<std::size_t>(numElements));
  std::iota(elementToRow.begin(), elementToRow.end(), 0);
  auto materialData = ImportedMaterialData(
    numElements, {}, {}, {},
    {ImportedMaterialField(
      std::move(channelNames), std::move(rows), std::move(elementToRow), "edge_quad")});
  return std::make_unique<SimulationAsset>(std::move(mesh), std::move(materialData));
}

}

std::unique_ptr<SimulationAsset> pgo::SolidDeformationModel::loadTriangleMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  const ImportedENuMaterial &mat)
{
  return loadTriangleMesh(
    triMeshGeo, ElementField<ImportedENuMaterial>::uniform(
      triMeshGeo.numTriangles(), mat));
}

std::unique_ptr<SimulationAsset> pgo::SolidDeformationModel::loadTriangleMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  ElementField<ImportedENuMaterial> materials)
{
  if (materials.size() != triMeshGeo.numTriangles())
    throw std::invalid_argument(
      "triangle material field size must match the triangle count");

  std::vector<int> triangles;
  triangles.reserve(static_cast<std::size_t>(triMeshGeo.numTriangles()) * 3);
  for (int tri = 0; tri < triMeshGeo.numTriangles(); tri++) {
    const pgo::Vec3i indices = triMeshGeo.tri(tri);
    triangles.emplace_back(indices[0]);
    triangles.emplace_back(indices[1]);
    triangles.emplace_back(indices[2]);
  }

  std::vector<double> vertices = copyTriMeshVertices(triMeshGeo);
  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(materials.size()) * 3);
  for (int element = 0; element < static_cast<int>(materials.size()); ++element) {
    values.push_back(materials.at(element).getE());
    values.push_back(materials.at(element).getNu());
    values.push_back(materials.at(element).getCompressionRatio());
  }
  static constexpr std::string_view names[] = {"E", "nu", "J"};
  auto mesh = std::make_shared<SimulationMesh>(
    triMeshGeo.numVertices(), vertices,
    triMeshGeo.numTriangles(), 3, triangles,
    SimulationMeshType::TRIANGLE);
  const int numElements = static_cast<int>(materials.size());
  pgo::EigenSupport::MXd rows(numElements, 3);
  for (int element = 0; element < numElements; ++element)
    for (int channel = 0; channel < 3; ++channel)
      rows(element, channel) = values[static_cast<std::size_t>(element) * 3 + channel];
  std::vector<std::string> channelNames;
  for (const auto name : names)
    channelNames.emplace_back(name);
  std::vector<int> elementToRow(static_cast<std::size_t>(numElements));
  std::iota(elementToRow.begin(), elementToRow.end(), 0);
  auto materialData = ImportedMaterialData(
    numElements, {}, {}, {},
    {ImportedMaterialField(
      std::move(channelNames), std::move(rows), std::move(elementToRow), "triangle")});
  return std::make_unique<SimulationAsset>(std::move(mesh), std::move(materialData));
}

std::unique_ptr<SimulationAsset> pgo::SolidDeformationModel::loadEdgeQuadMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  const ImportedENuMaterial &mat)
{
  return loadEdgeQuadMesh(
    triMeshGeo, ElementField<ImportedENuMaterial>::uniform(
      triMeshGeo.numTriangles(), mat));
}

std::unique_ptr<SimulationAsset> pgo::SolidDeformationModel::loadEdgeQuadMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  ElementField<ImportedENuMaterial> triangleMaterials)
{
  return buildEdgeQuadSimulationMesh(
    triMeshGeo, std::move(triangleMaterials));
}

std::unique_ptr<SimulationAsset> pgo::SolidDeformationModel::loadEdgeQuadMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  const ImportedENuhMaterial &mat)
{
  return loadEdgeQuadMesh(
    triMeshGeo, ElementField<ImportedENuhMaterial>::uniform(
      triMeshGeo.numTriangles(), mat));
}

std::unique_ptr<SimulationAsset> pgo::SolidDeformationModel::loadEdgeQuadMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  ElementField<ImportedENuhMaterial> triangleMaterials)
{
  return buildEdgeQuadSimulationMesh(
    triMeshGeo, std::move(triangleMaterials));
}

void pgo::SolidDeformationModel::computeTriangleUV(
  SimulationMesh &mesh, double scaleFactor)
{
  PGO_ALOG(mesh.getElementType() == SimulationMeshType::TRIANGLE);

  std::vector<double> uvs(mesh.getNumElements() * 3 * 2);
  for (int trii = 0; trii < mesh.getNumElements(); trii++) {
    std::array<EigenSupport::V3d, 3> restX;
    for (int j = 0; j < 3; j++)
      restX[j] = mesh.getVertex(trii, j);

    std::array<EigenSupport::V2d, 3> restUV;
    EigenSupport::V3d edge0 = restX[1] - restX[0];
    EigenSupport::V3d edge1 = restX[2] - restX[0];
    restUV[0] = EigenSupport::V2d(0, 0);
    restUV[1] = EigenSupport::V2d(edge0.norm(), 0);

    EigenSupport::V3d norm0 = edge0.normalized();
    restUV[2](0) = edge1.dot(norm0);
    restUV[2](1) = sqrt(edge1.squaredNorm() - restUV[2](0) * restUV[2](0));
    for (int j = 0; j < 3; j++)
      restUV[j] *= scaleFactor;
    for (int j = 0; j < 3; j++) {
      uvs[trii * 3 * 2 + j * 2] = restUV[j][0];
      uvs[trii * 3 * 2 + j * 2 + 1] = restUV[j][1];
    }
  }
  mesh.assignElementUVs(uvs);
}
