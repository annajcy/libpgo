/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include <memory>
#include <span>

#include "EigenSupport.h"
#include "simulation/elementField.h"

namespace pgo
{
namespace SolidDeformationModel
{

enum class SimulationMeshType { TET, CUBIC, TRIANGLE, EDGE_QUAD, SHELL };

class SimulationMeshImpl;

class SimulationMesh
{
public:
  SimulationMesh(int numVertices, std::span<const double> vertexPositions,
    int numElements, int numElementVertices, std::span<const int> elementVertexIndices,
    ElementFieldStore elementFields, SimulationMeshType meshType);

  ~SimulationMesh();

  int getNumElements() const;
  int getNumVertices() const;
  int getNumElementVertices() const;
  int getVertexIndex(int ele, int j) const;
  std::span<const int> getVertexIndices(int ele) const;
  const EigenSupport::V3d &getVertex(int vi) const;
  const EigenSupport::V3d &getVertex(int ele, int j) const;
  void assignElementUVs(std::span<const double> uvs);
  bool hasElementUV() const;
  const EigenSupport::V2d &getElementUV(int ele, int j) const;
  SimulationMeshType getElementType() const;

  template<class T>
  const ElementField<T> &requireElementField() const
  {
    return implElementFields().require<T>();
  }

private:
  const ElementFieldStore &implElementFields() const;
  std::unique_ptr<SimulationMeshImpl> impl;
};

const char *meshTypeName(SimulationMeshType meshType);

template<class T>
ElementFieldStore makeUniformSimulationMeshElementFieldStore(int numElements, const T &value)
{
  ElementFieldStore store;
  store.add(ElementField<T>::uniform(numElements, value));
  return store;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
