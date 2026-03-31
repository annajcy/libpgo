#include "internal/mesh_transforms.h"

#include "internal/mesh_mutation.h"

#include <cstring>

namespace pgo::VolumetricMeshes::internal::mesh_transforms {

void compute_gravity(const VolumetricMesh& mesh, double* gravity_force, double g, bool add_force) {
    if (!add_force) {
        std::memset(gravity_force, 0, sizeof(double) * 3 * mesh.getNumVertices());
    }

    const double inv_num_element_vertices = 1.0 / mesh.getNumElementVertices();

    for (int element = 0; element < mesh.getNumElements(); element++) {
        const double mass = mesh.getElementDensity(element) * mesh.getElementVolume(element);
        for (int vertex = 0; vertex < mesh.getNumElementVertices(); vertex++) {
            gravity_force[3 * mesh.getVertexIndex(element, vertex) + 1] -= inv_num_element_vertices * mass * g;
        }
    }
}

void apply_deformation(VolumetricMesh& mesh, const double* u) {
    for (int i = 0; i < mesh.getNumVertices(); i++) {
        Vec3d& vertex = mesh.getVertex(i);
        vertex[0] += u[3 * i + 0];
        vertex[1] += u[3 * i + 1];
        vertex[2] += u[3 * i + 2];
    }
}

void apply_linear_transformation(VolumetricMesh& mesh, double* pos, double* R) {
    for (int i = 0; i < mesh.getNumVertices(); i++) {
        Vec3d& vertex = mesh.getVertex(i);

        double new_pos[3];
        for (int j = 0; j < 3; j++) {
            new_pos[j] = pos[j];
            for (int k = 0; k < 3; k++) {
                new_pos[j] += R[3 * j + k] * vertex[k];
            }
        }

        vertex[0] = new_pos[0];
        vertex[1] = new_pos[1];
        vertex[2] = new_pos[2];
    }
}

void renumber_vertices(VolumetricMesh& mesh, const std::vector<int>& permutation) {
    MeshMutation::geometry(mesh).renumber_vertices(permutation);
}

}  // namespace pgo::VolumetricMeshes::internal::mesh_transforms
