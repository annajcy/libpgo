#pragma once

#include "meshLinearAlgebra.h"

#include <array>
#include <stdexcept>
#include <vector>
#include <cassert>

namespace pgo::Mesh
{

enum class MeshDataType
{
  Triangle,
  Tet,
  Cubic,
};

template<int K>
constexpr MeshDataType meshDataTypeFor();

template<>
constexpr MeshDataType meshDataTypeFor<3>()
{
  return MeshDataType::Triangle;
}

template<>
constexpr MeshDataType meshDataTypeFor<4>()
{
  return MeshDataType::Tet;
}

template<>
constexpr MeshDataType meshDataTypeFor<8>()
{
  return MeshDataType::Cubic;
}

template<int K>
class ElementView
{
public:
  ElementView() = default;
  explicit ElementView(const int *data): data_(data) {}

  int operator[](int localVertexID) const { 
    assert(localVertexID >= 0 && localVertexID < K);
    return data_[localVertexID]; 
  }

  const int *data() const { return data_; }

private:
  const int *data_ = nullptr;
};

template<int K>
class ElementsView
{
public:
  ElementsView() = default;
  ElementsView(const int *data, int numElements): data_(data), numElements_(numElements) {}

  int size() const { return numElements_; }
  const int *data() const { return data_; }
  int operator()(int elementID, int localVertexID) const { return data_[elementID * K + localVertexID]; }
  ElementView<K> operator[](int elementID) const { return ElementView<K>(data_ + elementID * K); }

private:
  const int *data_ = nullptr;
  int numElements_ = 0;
};

template<int K>
class MeshData
{
public:
  using Element = std::array<int, K>;
  using View = ElementsView<K>;

  MeshData() = default;
  MeshData(std::vector<Vec3d> positions, std::vector<Element> elements);
  MeshData(std::vector<Vec3d> positions, std::vector<int> elements);

  static MeshData fromElements(std::vector<Vec3d> positions, std::vector<Element> elements);
  static MeshData fromFlatElements(std::vector<Vec3d> positions, std::vector<int> elements);

  MeshDataType meshType() const { return meshDataTypeFor<K>(); }
  int verticesPerElement() const { return K; }
  int numVertices() const { return static_cast<int>(positions_.size()); }
  int numElements() const { return static_cast<int>(elements_.size() / static_cast<size_t>(K)); }

  const std::vector<Vec3d> &positions() const { return positions_; }
  std::vector<Vec3d> &positions() { return positions_; }

  View elements() const { return View(elements_.data(), numElements()); }
  const std::vector<int> &elementsFlat() const { return elements_; }
  std::vector<int> &elementsFlat() { return elements_; }

  ElementView<K> element(int elementID) const { return elements()[elementID]; }

  int elementVtxID(int elementID, int localVertexID) const { return elements_[elementID * K + localVertexID]; }

private:
  void validateElements() const;

private:
  std::vector<Vec3d> positions_;
  std::vector<int> elements_;
};

using TriMeshData = MeshData<3>;
using TetMeshData = MeshData<4>;
using CubicMeshData = MeshData<8>;

TriMeshData filterMeshComponentsByFace(const TriMeshData &mesh, int minElements, int keepLargest);
TetMeshData filterMeshComponentsByFace(const TetMeshData &mesh, int minElements, int keepLargest);
CubicMeshData filterMeshComponentsByFace(const CubicMeshData &mesh, int minElements, int keepLargest);

}  // namespace pgo::Mesh
