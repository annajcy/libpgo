#include "cubicMeshGeo.h"
#include "cellMeshGeo.h"
#include <cstring>

namespace pgo::Mesh
{

CubicMeshGeo::CubicMeshGeo(int numVertices, const double *vertices, int numCubes, const int *cubes)
{
  positions_.resize(numVertices);
  for (int i = 0; i < numVertices; i++) {
    positions_[i] = Vec3d(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]);
  }
  cubes_.resize(numCubes);
  std::memcpy(cubes_.data(), cubes, numCubes * sizeof(Vec8i));
}

CubicMeshGeo::CubicMeshGeo(std::vector<Vec3d> positions, std::vector<Vec8i> cubes):
  positions_(std::move(positions)), cubes_(std::move(cubes))
{
}

CubicMeshGeo::CubicMeshGeo(const CellMeshGeo<8>& cellMesh)
{
  positions_ = cellMesh.positions();
  cubes_.resize(cellMesh.numCells());
  std::memcpy(cubes_.data(), cellMesh.cellsFlat().data(), cellMesh.cellsFlat().size() * sizeof(int));
}

CellMeshGeo<8> CubicMeshGeo::toCellMesh() const
{
  std::vector<int> flatCells(cubes_.size() * 8);
  std::memcpy(flatCells.data(), cubes_.data(), cubes_.size() * sizeof(Vec8i));
  return CellMeshGeo<8>(positions_, std::move(flatCells));
}

}  // namespace pgo::Mesh
