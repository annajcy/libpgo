#pragma once

#include "meshLinearAlgebra.h"

#include <array>
#include <stdexcept>
#include <vector>
#include <cassert>

namespace pgo::Mesh
{

enum class CellMeshType
{
  Triangle,
  Tet,
  Cubic,
};

template<int K>
constexpr CellMeshType cellTypeFor();

template<>
constexpr CellMeshType cellTypeFor<3>()
{
  return CellMeshType::Triangle;
}

template<>
constexpr CellMeshType cellTypeFor<4>()
{
  return CellMeshType::Tet;
}

template<>
constexpr CellMeshType cellTypeFor<8>()
{
  return CellMeshType::Cubic;
}

template<int K>
class CellElementView
{
public:
  CellElementView() = default;
  explicit CellElementView(const int *data): data_(data) {}

  int operator[](int localID) const { 
    assert(localID >= 0 && localID < K);
    return data_[localID]; 
  }

  const int *data() const { return data_; }

private:
  const int *data_ = nullptr;
};

template<int K>
class CellMeshView
{
public:
  CellMeshView() = default;
  CellMeshView(const int *data, int numCells): data_(data), numCells_(numCells) {}

  int size() const { return numCells_; }
  const int *data() const { return data_; }
  int operator()(int cellID, int localID) const { return data_[cellID * K + localID]; }
  CellElementView<K> operator[](int cellID) const { return CellElementView<K>(data_ + cellID * K); }

private:
  const int *data_ = nullptr;
  int numCells_ = 0;
};

template<int K>
class CellMeshGeo
{
public:
  using Cell = std::array<int, K>;
  using CellsView = CellMeshView<K>;

  CellMeshGeo() = default;
  CellMeshGeo(std::vector<Vec3d> positions, std::vector<Cell> cells);
  CellMeshGeo(std::vector<Vec3d> positions, std::vector<int> cells);

  static CellMeshGeo fromCells(std::vector<Vec3d> positions, std::vector<Cell> cells);
  static CellMeshGeo fromFlatCells(std::vector<Vec3d> positions, std::vector<int> cells);

  CellMeshType cellType() const { return cellTypeFor<K>(); }
  int verticesPerCell() const { return K; }
  int numVertices() const { return static_cast<int>(positions_.size()); }
  int numCells() const { return static_cast<int>(cells_.size() / static_cast<size_t>(K)); }

  const std::vector<Vec3d> &positions() const { return positions_; }
  std::vector<Vec3d> &positions() { return positions_; }

  CellsView cells() const { return CellsView(cells_.data(), numCells()); }
  const std::vector<int> &cellsFlat() const { return cells_; }
  std::vector<int> &cellsFlat() { return cells_; }

  CellElementView<K> cell(int cellID) const { return cells()[cellID]; }

  int cellVtxID(int cellID, int localID) const { return cells_[cellID * K + localID]; }

private:
  void validateCells() const;

private:
  std::vector<Vec3d> positions_;
  std::vector<int> cells_;
};

using TriCellMeshGeo = CellMeshGeo<3>;
using TetCellMeshGeo = CellMeshGeo<4>;
using CubicCellMeshGeo = CellMeshGeo<8>;

}  // namespace pgo::Mesh
