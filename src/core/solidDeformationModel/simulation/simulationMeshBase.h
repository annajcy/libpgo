/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include <memory>

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
  SimulationMesh(int numVertices, const double *vertexPositions,
    int numElements, int numElementVertices, const int *elementVertexIndices,
    ElementFieldStore elementFields, SimulationMeshType meshType);

  ~SimulationMesh();

  int getNumElements() const;
  int getNumVertices() const;
  int getNumElementVertices() const;
  int getVertexIndex(int ele, int j) const;
  const int *getVertexIndices(int ele) const;
  void getVertex(int vi, double pos[3]) const;
  void getVertex(int ele, int j, double pos[3]) const;
  void assignElementUVs(const double *uvs);
  bool hasElementUV() const;
  void getElementUV(int ele, int j, double uv[2]) const;
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
