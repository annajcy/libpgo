#include "meshData.h"

#include <algorithm>
#include <stdexcept>

namespace pgo::Mesh
{

template<int K>
MeshData<K>::MeshData(std::vector<Vec3d> positions, std::vector<Element> elements):
  positions_(std::move(positions))
{
  elements_.reserve(elements.size() * K);
  for (const auto &element : elements) {
    for (int i = 0; i < K; i++)
      elements_.push_back(element[i]);
  }
  validateElements();
}

template<int K>
MeshData<K>::MeshData(std::vector<Vec3d> positions, std::vector<int> elements):
  positions_(std::move(positions)), elements_(std::move(elements))
{
  validateElements();
}

template<int K>
MeshData<K> MeshData<K>::fromElements(std::vector<Vec3d> positions, std::vector<Element> elements)
{
  return MeshData(std::move(positions), std::move(elements));
}

template<int K>
MeshData<K> MeshData<K>::fromFlatElements(std::vector<Vec3d> positions, std::vector<int> elements)
{
  return MeshData(std::move(positions), std::move(elements));
}

template<int K>
void MeshData<K>::validateElements() const
{
  if (elements_.size() % static_cast<size_t>(K) != 0)
    throw std::invalid_argument("element array size is not a multiple of vertices per element");

  for (int elementVtxID : elements_) {
    if (elementVtxID < 0 || elementVtxID >= numVertices())
      throw std::invalid_argument("element index out of range");
  }
}

template class MeshData<3>;
template class MeshData<4>;
template class MeshData<8>;

}  // namespace pgo::Mesh
