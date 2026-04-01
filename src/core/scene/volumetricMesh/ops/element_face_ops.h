#pragma once

#include "volumetricMesh.h"

#include "rectKey.h"
#include "triKey.h"

#include <array>
#include <vector>

namespace pgo::VolumetricMeshes {
class CubicMesh;
class TetMesh;
}

namespace pgo::VolumetricMeshes::ops {

int face_degree(const VolumetricMesh& mesh);

std::array<Mesh::OTriKey, 4> tet_element_faces(const TetMesh& mesh, int element);
std::array<Mesh::ORectKey, 6> cubic_element_faces(const CubicMesh& mesh, int element);

void append_face(const Mesh::OTriKey& face, std::vector<std::vector<int>>& faces);
void append_face(const Mesh::OTriKey& face, bool triangulate, std::vector<std::vector<int>>& faces);
void append_face(const Mesh::ORectKey& face, bool triangulate, std::vector<std::vector<int>>& faces);

}  // namespace pgo::VolumetricMeshes::ops
