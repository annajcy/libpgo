#include "cubicMeshGeo.h"
#include "meshData.h"
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

CubicMeshGeo::CubicMeshGeo(const MeshData<8>& meshData)
{
  positions_ = meshData.positions();
  cubes_.resize(meshData.numElements());
  std::memcpy(cubes_.data(), meshData.elementsFlat().data(), meshData.elementsFlat().size() * sizeof(int));
}

MeshData<8> CubicMeshGeo::toMeshData() const
{
  std::vector<int> flatElements(cubes_.size() * 8);
  std::memcpy(flatElements.data(), cubes_.data(), cubes_.size() * sizeof(Vec8i));
  return MeshData<8>(positions_, std::move(flatElements));
}

}  // namespace pgo::Mesh
