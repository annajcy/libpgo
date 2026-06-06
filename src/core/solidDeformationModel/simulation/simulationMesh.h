/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include <memory>

#include "simulation/simulationMeshMaterial.h"

namespace pgo
{

namespace Mesh
{
class TriMeshGeo;
class TetMeshGeo;
}  // namespace Mesh

namespace VolumetricMeshes
{
class CubicMesh;
class TetMesh;
}

namespace SolidDeformationModel
{

// Forward declarations for enum types (definitions in deformationModelManager.h).
enum class DeformationModelElasticMaterial : int;
enum class DeformationModelPlasticMaterial : int;

enum class SimulationMeshType
{
  TET,
  CUBIC,
  TRIANGLE,
  EDGE_QUAD,
  SHELL,
};

class SimulationMeshImpl;

class SimulationMesh
{
public:
  SimulationMesh(int numVertices, const double *vertexPositions,
    int numElements, int numElementVertices, const int *elementVertexIndices,
    const int *elementMaterialIndices, int numMaterials, const SimulationMeshMaterial *const *materials,
    SimulationMeshType meshType);
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

  const SimulationMeshMaterial *getElementMaterial(int ele, int j) const;
  SimulationMeshMaterial *getElementMaterial(int ele, int j);
  int getElementNumMaterials(int ele) const;

  void setMaterial(int matID, const SimulationMeshMaterial *mat);
  void appendMaterialToAllElements(const SimulationMeshMaterial *mat);

protected:
  std::unique_ptr<SimulationMeshImpl> impl;
};

std::unique_ptr<SimulationMesh> loadTetMesh(const VolumetricMeshes::TetMesh *tetmesh);
std::unique_ptr<SimulationMesh> loadCubicMesh(const VolumetricMeshes::CubicMesh *cubicMesh);

std::unique_ptr<SimulationMesh> loadTriMesh(const Mesh::TriMeshGeo &triMeshGeo, const SimulationMeshMaterial *mat, int toTriangle);
std::unique_ptr<SimulationMesh> loadTriMesh(const Mesh::TriMeshGeo &triMeshGeo, int numMaterials, const SimulationMeshMaterial *const *const mat, const int *materialIndices, int toTriangle);

std::unique_ptr<SimulationMesh> loadShellMesh(const Mesh::TriMeshGeo &triMeshGeo, const SimulationMeshMaterial *mat);
std::unique_ptr<SimulationMesh> loadShellMesh(const Mesh::TriMeshGeo &triMeshGeo, const int *elementMaterialIndices, const SimulationMeshMaterial *const *mat);

void computeTriangleUV(SimulationMesh *mesh, double scaleFactor);
}  // namespace SolidDeformationModel
}  // namespace pgo
