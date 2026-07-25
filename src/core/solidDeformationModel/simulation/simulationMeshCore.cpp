/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "simulation/simulationMeshBase.h"

#include "EigenSupport.h"
#include "meshLinearAlgebra.h"

#include <cstring>
#include <memory>
#include <vector>

namespace pgo
{
namespace ES = EigenSupport;

namespace SolidDeformationModel
{
class SimulationMeshImpl
{
public:
  SimulationMeshImpl(int numVertices, const double *vertexPositions,
    int numElements, int numElementVertices, const int *elementVertexIndices,
    ElementFieldStore elementFields,
    SimulationMeshType meshType);

  std::vector<ES::V3d> vertices;
  std::vector<std::vector<int>> elements;
  std::vector<std::vector<ES::V2d>> elementUVs;
  ElementFieldStore elementFields;

  SimulationMeshType meshType;
};

}  // namespace SolidDeformationModel
}  // namespace pgo

using namespace pgo::SolidDeformationModel;

SimulationMesh::SimulationMesh(int numVertices, const double *vertexPositions,
  int numElements, int numElementVertices, const int *elementVertexIndices,
  ElementFieldStore elementFields,
  SimulationMeshType meshType)
{
  impl = std::make_unique<SimulationMeshImpl>(numVertices, vertexPositions,
    numElements, numElementVertices, elementVertexIndices,
    std::move(elementFields), meshType);
}

SimulationMesh::~SimulationMesh() = default;

int SimulationMesh::getNumElements() const
{
  return static_cast<int>(impl->elements.size());
}

int SimulationMesh::getNumElementVertices() const
{
  return static_cast<int>(impl->elements[0].size());
}

int SimulationMesh::getNumVertices() const
{
  return static_cast<int>(impl->vertices.size());
}

int SimulationMesh::getVertexIndex(int ele, int j) const
{
  return impl->elements[ele][j];
}

const int *SimulationMesh::getVertexIndices(int ele) const
{
  return impl->elements[ele].data();
}

void SimulationMesh::getVertex(int ele, int j, double pos[3]) const
{
  (ES::Mp<ES::V3d>(pos)) = impl->vertices[impl->elements[ele][j]];
}

void SimulationMesh::getVertex(int vi, double pos[3]) const
{
  (ES::Mp<ES::V3d>(pos)) = impl->vertices[vi];
}

SimulationMeshType SimulationMesh::getElementType() const
{
  return impl->meshType;
}

const ElementFieldStore &SimulationMesh::implElementFields() const
{
  return impl->elementFields;
}

void SimulationMesh::assignElementUVs(const double *uvs)
{
  impl->elementUVs.assign(
    impl->elements.size(), std::vector<Vec2d>(getNumElementVertices()));
  for (size_t ei = 0; ei < impl->elements.size(); ei++) {
    for (int j = 0; j < getNumElementVertices(); j++) {
      const double *uv = uvs + ei * getNumElementVertices() * 2 + j * 2;
      impl->elementUVs[ei][j] = Vec2d(uv[0], uv[1]);
    }
  }
}

bool SimulationMesh::hasElementUV() const
{
  return !impl->elementUVs.empty();
}

void SimulationMesh::getElementUV(int ele, int j, double uv[2]) const
{
  uv[0] = impl->elementUVs[ele][j][0];
  uv[1] = impl->elementUVs[ele][j][1];
}

SimulationMeshImpl::SimulationMeshImpl(
  int numVertices, const double *vertexPositions,
  int numElements, int numElementVertices, const int *elementVertexIndices,
  ElementFieldStore elementFields_, SimulationMeshType mt)
{
  vertices.assign(numVertices, ES::V3d::Zero());
  for (int vi = 0; vi < numVertices; vi++)
    vertices[vi] = asVec3d(vertexPositions + vi * 3);

  elements.assign(numElements, std::vector<int>(numElementVertices, 0));
  for (int ei = 0; ei < numElements; ei++) {
    memcpy(elements[ei].data(),
      elementVertexIndices + ei * numElementVertices,
      sizeof(int) * numElementVertices);
  }

  elementFields_.validateSize(numElements);
  elementFields = std::move(elementFields_);
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
