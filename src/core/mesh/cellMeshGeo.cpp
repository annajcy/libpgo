#include "cellMeshGeo.h"

#include <algorithm>
#include <stdexcept>

namespace pgo::Mesh
{

template<int K>
CellMeshGeo<K>::CellMeshGeo(std::vector<Vec3d> positions, std::vector<Cell> cells):
  positions_(std::move(positions))
{
  cells_.reserve(cells.size() * K);
  for (const auto &cell : cells) {
    for (int i = 0; i < K; i++)
      cells_.push_back(cell[i]);
  }
  validateCells();
}

template<int K>
CellMeshGeo<K>::CellMeshGeo(std::vector<Vec3d> positions, std::vector<int> cells):
  positions_(std::move(positions)), cells_(std::move(cells))
{
  validateCells();
}

template<int K>
CellMeshGeo<K> CellMeshGeo<K>::fromCells(std::vector<Vec3d> positions, std::vector<Cell> cells)
{
  return CellMeshGeo(std::move(positions), std::move(cells));
}

template<int K>
CellMeshGeo<K> CellMeshGeo<K>::fromFlatCells(std::vector<Vec3d> positions, std::vector<int> cells)
{
  return CellMeshGeo(std::move(positions), std::move(cells));
}

template<int K>
void CellMeshGeo<K>::validateCells() const
{
  if (cells_.size() % static_cast<size_t>(K) != 0)
    throw std::invalid_argument("cell array size is not a multiple of vertices per cell");

  for (int cellVtxID : cells_) {
    if (cellVtxID < 0 || cellVtxID >= numVertices())
      throw std::invalid_argument("cell index out of range");
  }
}

template class CellMeshGeo<3>;
template class CellMeshGeo<4>;
template class CellMeshGeo<8>;

}  // namespace pgo::Mesh
