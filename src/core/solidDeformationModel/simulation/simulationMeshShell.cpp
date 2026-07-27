#include "simulation/simulationMeshShell.h"

#include "triMeshGeo.h"
#include "triMeshNeighbor.h"

#include <memory>
#include <stdexcept>
#include <vector>

using namespace pgo::SolidDeformationModel;

namespace
{
std::shared_ptr<SimulationMesh> buildShellSimulationMesh(
  const pgo::Mesh::TriMeshGeo &triMeshGeo)
{
  const int numElements = triMeshGeo.numTriangles();

  std::vector<double> vertices;
  vertices.reserve(static_cast<std::size_t>(triMeshGeo.numVertices()) * 3);
  for (int vi = 0; vi < triMeshGeo.numVertices(); vi++) {
    vertices.emplace_back(triMeshGeo.pos(vi)[0]);
    vertices.emplace_back(triMeshGeo.pos(vi)[1]);
    vertices.emplace_back(triMeshGeo.pos(vi)[2]);
  }

  std::vector<int> elementVertexIndices;
  elementVertexIndices.reserve(static_cast<std::size_t>(numElements) * 6);
  pgo::Mesh::TriMeshNeighbor triNeighbor(triMeshGeo);
  for (int element = 0; element < numElements; element++) {
    pgo::Vec3i triangle = triMeshGeo.tri(element);
    pgo::Vec3i neighborList = triNeighbor.getTriangleNeighbors(element);

    for (int j = 0; j < 3; j++)
      elementVertexIndices.emplace_back(triangle[j]);

    for (int j = 0; j < 3; j++) {
      int e0 = triangle[j];
      int e1 = triangle[(j + 1) % 3];
      if (neighborList[j] >= 0) {
        pgo::Vec3i neighbor = triMeshGeo.tri(neighborList[j]);
        elementVertexIndices.emplace_back(
          pgo::Mesh::getTriangleVertexOppositeEdge(neighbor, e0, e1));
      }
      else {
        elementVertexIndices.emplace_back(-1);
      }
    }
  }

  return std::make_shared<SimulationMesh>(
    triMeshGeo.numVertices(), vertices, numElements, 6,
    elementVertexIndices, SimulationMeshType::SHELL);
}
}

std::shared_ptr<SimulationMesh> pgo::SolidDeformationModel::loadShellMesh(
  const Mesh::TriMeshGeo &triMeshGeo)
{
  return buildShellSimulationMesh(triMeshGeo);
}
