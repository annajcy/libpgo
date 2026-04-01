#pragma once

#include "concepts/mesh_concepts.h"

#include "rectKey.h"
#include "triKey.h"

#include <array>
#include <vector>

namespace pgo::VolumetricMeshes {
class CubicMesh;
class TetMesh;
}

namespace pgo::VolumetricMeshes::ops {

template <concepts::TetMeshLike MeshT>
int face_degree(const MeshT&) {
    return 3;
}

template <concepts::CubicMeshLike MeshT>
int face_degree(const MeshT&) {
    return 4;
}

std::array<Mesh::OTriKey, 4> tet_element_faces(const TetMesh& mesh, int element);
std::array<Mesh::ORectKey, 6> cubic_element_faces(const CubicMesh& mesh, int element);

void append_face(const Mesh::OTriKey& face, std::vector<std::vector<int>>& faces);
void append_face(const Mesh::OTriKey& face, bool triangulate, std::vector<std::vector<int>>& faces);
void append_face(const Mesh::ORectKey& face, bool triangulate, std::vector<std::vector<int>>& faces);

}  // namespace pgo::VolumetricMeshes::ops
