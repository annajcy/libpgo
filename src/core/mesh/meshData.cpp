#include "meshData.h"

#include <algorithm>
#include <map>
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

namespace
{

template<int K>
const std::vector<std::vector<int>> &elementFaces();

template<>
const std::vector<std::vector<int>> &elementFaces<3>()
{
  static const std::vector<std::vector<int>> faces = { { 0, 1 }, { 1, 2 }, { 2, 0 } };
  return faces;
}

template<>
const std::vector<std::vector<int>> &elementFaces<4>()
{
  static const std::vector<std::vector<int>> faces = {
    { 0, 1, 2 }, { 0, 3, 1 }, { 1, 3, 2 }, { 2, 3, 0 }
  };
  return faces;
}

template<>
const std::vector<std::vector<int>> &elementFaces<8>()
{
  static const std::vector<std::vector<int>> faces = {
    { 0, 1, 2, 3 }, { 4, 5, 6, 7 }, { 0, 1, 5, 4 },
    { 1, 2, 6, 5 }, { 2, 3, 7, 6 }, { 3, 0, 4, 7 }
  };
  return faces;
}

template<int K>
MeshData<K> filterMeshComponents(const MeshData<K> &mesh, int minElements, int keepLargest)
{
  if (minElements < 0)
    throw std::invalid_argument("min_elements must be non-negative");
  if (keepLargest == 0 || keepLargest < -1)
    throw std::invalid_argument("keep_largest must be -1 or positive");

  const int numElements = mesh.numElements();
  std::vector<std::vector<int>> neighbors(numElements);
  std::map<std::vector<int>, int> faceOwners;
  for (int elementID = 0; elementID < numElements; ++elementID) {
    for (const auto &localFace : elementFaces<K>()) {
      std::vector<int> face;
      face.reserve(localFace.size());
      for (int localVertexID : localFace)
        face.push_back(mesh.elementVtxID(elementID, localVertexID));
      std::sort(face.begin(), face.end());

      auto [iter, inserted] = faceOwners.emplace(std::move(face), elementID);
      if (!inserted) {
        neighbors[elementID].push_back(iter->second);
        neighbors[iter->second].push_back(elementID);
      }
    }
  }

  std::vector<char> visited(numElements, false);
  std::vector<std::vector<int>> components;
  for (int seed = 0; seed < numElements; ++seed) {
    if (visited[seed])
      continue;
    visited[seed] = true;
    components.push_back({});
    std::vector<int> pending = { seed };
    while (!pending.empty()) {
      const int elementID = pending.back();
      pending.pop_back();
      components.back().push_back(elementID);
      for (int neighbor : neighbors[elementID]) {
        if (!visited[neighbor]) {
          visited[neighbor] = true;
          pending.push_back(neighbor);
        }
      }
    }
  }

  components.erase(std::remove_if(components.begin(), components.end(), [minElements](const auto &component) {
    return static_cast<int>(component.size()) < minElements;
  }), components.end());
  std::stable_sort(components.begin(), components.end(), [](const auto &a, const auto &b) {
    return a.size() > b.size();
  });
  if (keepLargest > 0 && static_cast<int>(components.size()) > keepLargest)
    components.resize(keepLargest);
  if (components.empty())
    throw std::runtime_error("Component filtering removed every mesh element");

  std::vector<char> keepElement(numElements, false);
  for (const auto &component : components)
    for (int elementID : component)
      keepElement[elementID] = true;

  std::vector<int> oldToNew(mesh.numVertices(), -1);
  std::vector<Vec3d> positions;
  std::vector<int> elements;
  for (int elementID = 0; elementID < numElements; ++elementID) {
    if (!keepElement[elementID])
      continue;
    for (int localVertexID = 0; localVertexID < K; ++localVertexID) {
      const int oldVertexID = mesh.elementVtxID(elementID, localVertexID);
      if (oldToNew[oldVertexID] < 0) {
        oldToNew[oldVertexID] = static_cast<int>(positions.size());
        positions.push_back(mesh.positions()[oldVertexID]);
      }
      elements.push_back(oldToNew[oldVertexID]);
    }
  }
  return MeshData<K>(std::move(positions), std::move(elements));
}

}  // namespace

TriMeshData filterMeshComponentsByFace(const TriMeshData &mesh, int minElements, int keepLargest)
{
  return filterMeshComponents(mesh, minElements, keepLargest);
}

TetMeshData filterMeshComponentsByFace(const TetMeshData &mesh, int minElements, int keepLargest)
{
  return filterMeshComponents(mesh, minElements, keepLargest);
}

CubicMeshData filterMeshComponentsByFace(const CubicMeshData &mesh, int minElements, int keepLargest)
{
  return filterMeshComponents(mesh, minElements, keepLargest);
}

}  // namespace pgo::Mesh
