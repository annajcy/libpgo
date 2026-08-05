/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "simulation/simulationMesh.h"

#include <memory>
#include <stdexcept>
#include <vector>

namespace pgo
{
namespace ES = EigenSupport;

namespace SolidDeformationModel
{
class SimulationMeshImpl
{
public:
  SimulationMeshImpl(int numVertices, std::span<const double> vertexPositions,
    int numElements, int numElementVertices, std::span<const int> elementVertexIndices,
    SimulationMeshType meshType);

  std::vector<ES::V3d> vertices;
  std::vector<std::vector<int>> elements;
  std::vector<std::vector<ES::V2d>> elementUVs;
  SimulationMeshType meshType;
};

}  // namespace SolidDeformationModel
}  // namespace pgo

using namespace pgo::SolidDeformationModel;

SimulationMesh::SimulationMesh(int numVertices, std::span<const double> vertexPositions,
  int numElements, int numElementVertices, std::span<const int> elementVertexIndices,
  SimulationMeshType meshType)
{
  impl = std::make_unique<SimulationMeshImpl>(numVertices, vertexPositions,
    numElements, numElementVertices, elementVertexIndices,
    meshType);
}

SimulationMesh::~SimulationMesh() = default;

int SimulationMesh::getNumElements() const
{
  return static_cast<int>(impl->elements.size());
}

int SimulationMesh::getNumElementVertices() const
{
  return impl->elements.empty() ? 0 : static_cast<int>(impl->elements[0].size());
}

int SimulationMesh::getNumVertices() const
{
  return static_cast<int>(impl->vertices.size());
}

int SimulationMesh::getVertexIndex(int ele, int j) const
{
  return impl->elements[ele][j];
}

std::span<const int> SimulationMesh::getVertexIndices(int ele) const
{
  const std::vector<int> &indices = impl->elements[ele];
  return std::span<const int>(indices.data(), indices.size());
}

const pgo::EigenSupport::V3d &SimulationMesh::getVertex(int ele, int j) const
{
  return impl->vertices[impl->elements[ele][j]];
}

const pgo::EigenSupport::V3d &SimulationMesh::getVertex(int vi) const
{
  return impl->vertices[vi];
}

SimulationMeshType SimulationMesh::getElementType() const
{
  return impl->meshType;
}

void SimulationMesh::assignElementUVs(std::span<const double> uvs)
{
  const std::size_t expectedSize = static_cast<std::size_t>(
    getNumElements()) * static_cast<std::size_t>(getNumElementVertices()) * 2;
  if (uvs.size() < expectedSize)
    throw std::invalid_argument("SimulationMesh UV data is smaller than the mesh topology.");

  impl->elementUVs.assign(
    impl->elements.size(), std::vector<ES::V2d>(getNumElementVertices()));
  for (size_t ei = 0; ei < impl->elements.size(); ei++) {
    for (int j = 0; j < getNumElementVertices(); j++) {
      const std::size_t offset = (ei * static_cast<std::size_t>(getNumElementVertices()) + j) * 2;
      impl->elementUVs[ei][j] = ES::V2d(uvs[offset], uvs[offset + 1]);
    }
  }
}

bool SimulationMesh::hasElementUV() const
{
  return !impl->elementUVs.empty();
}

const pgo::EigenSupport::V2d &SimulationMesh::getElementUV(int ele, int j) const
{
  return impl->elementUVs[ele][j];
}

SimulationMeshImpl::SimulationMeshImpl(
  int numVertices, std::span<const double> vertexPositions,
  int numElements, int numElementVertices, std::span<const int> elementVertexIndices,
  SimulationMeshType mt)
{
  if (numVertices < 0 || numElements < 0 || numElementVertices < 0)
    throw std::invalid_argument("SimulationMesh dimensions must be nonnegative.");
  const std::size_t requiredVertexValues = static_cast<std::size_t>(numVertices) * 3;
  const std::size_t requiredElementIndices = static_cast<std::size_t>(numElements) *
    static_cast<std::size_t>(numElementVertices);
  if (vertexPositions.size() < requiredVertexValues)
    throw std::invalid_argument("SimulationMesh vertex data is smaller than numVertices * 3.");
  if (elementVertexIndices.size() < requiredElementIndices)
    throw std::invalid_argument("SimulationMesh connectivity is smaller than the topology.");

  vertices.assign(numVertices, ES::V3d::Zero());
  for (int vi = 0; vi < numVertices; vi++)
    vertices[vi] = ES::V3d(
      vertexPositions[static_cast<std::size_t>(vi) * 3 + 0],
      vertexPositions[static_cast<std::size_t>(vi) * 3 + 1],
      vertexPositions[static_cast<std::size_t>(vi) * 3 + 2]);

  elements.assign(numElements, std::vector<int>(numElementVertices, 0));
  for (int ei = 0; ei < numElements; ei++) {
    for (int j = 0; j < numElementVertices; j++) {
      elements[ei][j] = elementVertexIndices[
        static_cast<std::size_t>(ei) * numElementVertices + j];
    }
  }

  meshType = mt;
}

const char *pgo::SolidDeformationModel::meshTypeName(SimulationMeshType meshType)
{
  switch (meshType) {
  case SimulationMeshType::TET: return "TET";
  case SimulationMeshType::CUBIC: return "CUBIC";
  case SimulationMeshType::TRIANGLE: return "TRIANGLE";
  case SimulationMeshType::EDGE_QUAD: return "EDGE_QUAD";
  case SimulationMeshType::SHELL: return "SHELL";
  default: return "UNKNOWN";
  }
}
