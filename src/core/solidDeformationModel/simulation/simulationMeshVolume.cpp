#include "simulation/simulationMeshVolume.h"

#include "cubicMesh.h"
#include "tetMesh.h"

#include <memory>
#include <vector>

using namespace pgo::SolidDeformationModel;

std::shared_ptr<SimulationMesh> pgo::SolidDeformationModel::loadTetMesh(
  const VolumetricMeshes::TetMesh &tetMesh)
{
  std::vector<double> vertices;
  vertices.reserve(static_cast<std::size_t>(tetMesh.getNumVertices()) * 3);
  for (int vi = 0; vi < tetMesh.getNumVertices(); vi++) {
    Vec3d p = tetMesh.getVertex(vi);
    vertices.emplace_back(p[0]);
    vertices.emplace_back(p[1]);
    vertices.emplace_back(p[2]);
  }

  std::vector<int> elementVertices;
  elementVertices.reserve(static_cast<std::size_t>(tetMesh.getNumElements()) * 4);
  for (int ei = 0; ei < tetMesh.getNumElements(); ei++) {
    for (int j = 0; j < 4; j++)
      elementVertices.emplace_back(tetMesh.getVertexIndex(ei, j));
  }

  return std::make_shared<SimulationMesh>(
    tetMesh.getNumVertices(), vertices,
    tetMesh.getNumElements(), 4, elementVertices,
    SimulationMeshType::TET);
}

std::shared_ptr<SimulationMesh> pgo::SolidDeformationModel::loadCubicMesh(
  const VolumetricMeshes::CubicMesh &cubicMesh)
{
  std::vector<double> vertices;
  vertices.reserve(static_cast<std::size_t>(cubicMesh.getNumVertices()) * 3);
  for (int vi = 0; vi < cubicMesh.getNumVertices(); vi++) {
    Vec3d p = cubicMesh.getVertex(vi);
    vertices.emplace_back(p[0]);
    vertices.emplace_back(p[1]);
    vertices.emplace_back(p[2]);
  }

  std::vector<int> elementVertices;
  elementVertices.reserve(static_cast<std::size_t>(cubicMesh.getNumElements()) * 8);
  for (int ei = 0; ei < cubicMesh.getNumElements(); ei++) {
    for (int j = 0; j < 8; j++)
      elementVertices.emplace_back(cubicMesh.getVertexIndex(ei, j));
  }

  return std::make_shared<SimulationMesh>(
    cubicMesh.getNumVertices(), vertices,
    cubicMesh.getNumElements(), 8, elementVertices,
    SimulationMeshType::CUBIC);
}
