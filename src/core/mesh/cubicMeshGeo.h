#pragma once

#include "meshLinearAlgebra.h"
#include <vector>

namespace pgo::Mesh
{

template<int K>
class MeshData;

class CubicMeshGeo
{
public:
  CubicMeshGeo() = default;
  CubicMeshGeo(int numVertices, const double *vertices, int numCubes, const int *cubes);
  CubicMeshGeo(std::vector<Vec3d> positions, std::vector<Vec8i> cubes);

  explicit CubicMeshGeo(const MeshData<8>& meshData);

  int numVertices() const { return static_cast<int>(positions_.size()); }
  int numCubes() const { return static_cast<int>(cubes_.size()); }

  const Vec3d &pos(int vtxID) const { return positions_[vtxID]; }
  Vec3d &pos(int vtxID) { return positions_[vtxID]; }

  Vec8i cube(int cubeID) const { return cubes_[cubeID]; }
  int cubeVtxID(int cubeID, int i) const { return cubes_[cubeID][i]; }

  const std::vector<Vec3d> &positions() const { return positions_; }
  std::vector<Vec3d> &positions() { return positions_; }

  const std::vector<Vec8i> &cubes() const { return cubes_; }
  std::vector<Vec8i> &cubes() { return cubes_; }

  // 桥接转换
  MeshData<8> toMeshData() const;

private:
  std::vector<Vec3d> positions_;
  std::vector<Vec8i> cubes_;
};

}  // namespace pgo::Mesh
