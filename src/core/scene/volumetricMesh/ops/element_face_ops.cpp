#include "ops/element_face_ops.h"

#include "cubicMesh.h"
#include "tetMesh.h"

namespace pgo::VolumetricMeshes::ops {

std::array<Mesh::OTriKey, 4> tet_element_faces(const TetMesh& mesh, int element) {
    if (mesh.getTetDeterminant(element) >= 0.0) {
        return {
            Mesh::OTriKey(mesh.getVertexIndex(element, 1), mesh.getVertexIndex(element, 2), mesh.getVertexIndex(element, 3)),
            Mesh::OTriKey(mesh.getVertexIndex(element, 2), mesh.getVertexIndex(element, 0), mesh.getVertexIndex(element, 3)),
            Mesh::OTriKey(mesh.getVertexIndex(element, 3), mesh.getVertexIndex(element, 0), mesh.getVertexIndex(element, 1)),
            Mesh::OTriKey(mesh.getVertexIndex(element, 1), mesh.getVertexIndex(element, 0), mesh.getVertexIndex(element, 2)),
        };
    }

    return {
        Mesh::OTriKey(mesh.getVertexIndex(element, 3), mesh.getVertexIndex(element, 2), mesh.getVertexIndex(element, 1)),
        Mesh::OTriKey(mesh.getVertexIndex(element, 3), mesh.getVertexIndex(element, 0), mesh.getVertexIndex(element, 2)),
        Mesh::OTriKey(mesh.getVertexIndex(element, 1), mesh.getVertexIndex(element, 0), mesh.getVertexIndex(element, 3)),
        Mesh::OTriKey(mesh.getVertexIndex(element, 2), mesh.getVertexIndex(element, 0), mesh.getVertexIndex(element, 1)),
    };
}

std::array<Mesh::ORectKey, 6> cubic_element_faces(const CubicMesh& mesh, int element) {
    return {
        Mesh::ORectKey(mesh.getVertexIndex(element, 0), mesh.getVertexIndex(element, 3), mesh.getVertexIndex(element, 2),
                       mesh.getVertexIndex(element, 1)),
        Mesh::ORectKey(mesh.getVertexIndex(element, 4), mesh.getVertexIndex(element, 5), mesh.getVertexIndex(element, 6),
                       mesh.getVertexIndex(element, 7)),
        Mesh::ORectKey(mesh.getVertexIndex(element, 0), mesh.getVertexIndex(element, 1), mesh.getVertexIndex(element, 5),
                       mesh.getVertexIndex(element, 4)),
        Mesh::ORectKey(mesh.getVertexIndex(element, 3), mesh.getVertexIndex(element, 7), mesh.getVertexIndex(element, 6),
                       mesh.getVertexIndex(element, 2)),
        Mesh::ORectKey(mesh.getVertexIndex(element, 1), mesh.getVertexIndex(element, 2), mesh.getVertexIndex(element, 6),
                       mesh.getVertexIndex(element, 5)),
        Mesh::ORectKey(mesh.getVertexIndex(element, 0), mesh.getVertexIndex(element, 4), mesh.getVertexIndex(element, 7),
                       mesh.getVertexIndex(element, 3)),
    };
}

void append_face(const Mesh::OTriKey& face, std::vector<std::vector<int>>& faces) {
    faces.emplace_back(std::vector<int>{face[0], face[1], face[2]});
}

void append_face(const Mesh::OTriKey& face, bool, std::vector<std::vector<int>>& faces) {
    append_face(face, faces);
}

void append_face(const Mesh::ORectKey& face, bool triangulate, std::vector<std::vector<int>>& faces) {
    if (triangulate) {
        faces.emplace_back(std::vector<int>{face[0], face[1], face[2]});
        faces.emplace_back(std::vector<int>{face[2], face[3], face[0]});
        return;
    }

    faces.emplace_back(std::vector<int>{face[0], face[1], face[2], face[3]});
}

}  // namespace pgo::VolumetricMeshes::ops
