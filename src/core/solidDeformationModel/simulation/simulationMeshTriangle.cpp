#include "simulation/simulationMeshEdgeQuad.h"
#include "simulation/simulationMeshTriangle.h"

#include "EigenSupport.h"
#include "pgoLogging.h"
#include "triMeshGeo.h"

#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
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

ElementField<SimulationMeshENuMaterial> averageEdgeQuadMaterials(
  const EdgeQuadTopology &topology,
  const ElementField<SimulationMeshENuMaterial> &triangleMaterials)
{
  std::vector<SimulationMeshENuMaterial> materials;
  materials.reserve(topology.sourceTriangles.size());
  for (const auto &source : topology.sourceTriangles) {
    const auto &mat0 = triangleMaterials.at(source.first);
    const auto &mat1 = triangleMaterials.at(source.second);
    materials.emplace_back(
      0.5 * (mat0.getE() + mat1.getE()),
      0.5 * (mat0.getNu() + mat1.getNu()),
      0.5 * (mat0.getCompressionRatio() + mat1.getCompressionRatio()));
  }
  return ElementField<SimulationMeshENuMaterial>::fromValues(std::move(materials));
}

ElementField<SimulationMeshENuhMaterial> averageEdgeQuadMaterials(
  const EdgeQuadTopology &topology,
  const ElementField<SimulationMeshENuhMaterial> &triangleMaterials)
{
  std::vector<SimulationMeshENuhMaterial> materials;
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
  return ElementField<SimulationMeshENuhMaterial>::fromValues(std::move(materials));
}

template<class Material>
std::unique_ptr<SimulationMesh> buildEdgeQuadSimulationMesh(
  const pgo::Mesh::TriMeshGeo &triMeshGeo,
  ElementField<Material> triangleMaterials)
{
  if (triangleMaterials.size() != triMeshGeo.numTriangles())
    throw std::invalid_argument(
      "edge-quad material field size must match the triangle count");

  EdgeQuadTopology topology = buildEdgeQuadTopology(triMeshGeo);
  std::vector<double> vertices = copyTriMeshVertices(triMeshGeo);
  ElementFieldStore fields;
  fields.add(averageEdgeQuadMaterials(topology, triangleMaterials));
  return std::make_unique<SimulationMesh>(
    triMeshGeo.numVertices(), vertices,
    static_cast<int>(topology.sourceTriangles.size()), 4,
    topology.elementVertexIndices, std::move(fields),
    SimulationMeshType::EDGE_QUAD);
}

}

std::unique_ptr<SimulationMesh> pgo::SolidDeformationModel::loadTriangleMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  const SimulationMeshENuMaterial &mat)
{
  return loadTriangleMesh(
    triMeshGeo, ElementField<SimulationMeshENuMaterial>::uniform(
      triMeshGeo.numTriangles(), mat));
}

std::unique_ptr<SimulationMesh> pgo::SolidDeformationModel::loadTriangleMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  ElementField<SimulationMeshENuMaterial> materials)
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
  ElementFieldStore fields;
  fields.add(std::move(materials));
  return std::make_unique<SimulationMesh>(
    triMeshGeo.numVertices(), vertices,
    triMeshGeo.numTriangles(), 3, triangles, std::move(fields),
    SimulationMeshType::TRIANGLE);
}

std::unique_ptr<SimulationMesh> pgo::SolidDeformationModel::loadEdgeQuadMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  const SimulationMeshENuMaterial &mat)
{
  return loadEdgeQuadMesh(
    triMeshGeo, ElementField<SimulationMeshENuMaterial>::uniform(
      triMeshGeo.numTriangles(), mat));
}

std::unique_ptr<SimulationMesh> pgo::SolidDeformationModel::loadEdgeQuadMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  ElementField<SimulationMeshENuMaterial> triangleMaterials)
{
  return buildEdgeQuadSimulationMesh(
    triMeshGeo, std::move(triangleMaterials));
}

std::unique_ptr<SimulationMesh> pgo::SolidDeformationModel::loadEdgeQuadMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  const SimulationMeshENuhMaterial &mat)
{
  return loadEdgeQuadMesh(
    triMeshGeo, ElementField<SimulationMeshENuhMaterial>::uniform(
      triMeshGeo.numTriangles(), mat));
}

std::unique_ptr<SimulationMesh> pgo::SolidDeformationModel::loadEdgeQuadMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  ElementField<SimulationMeshENuhMaterial> triangleMaterials)
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
