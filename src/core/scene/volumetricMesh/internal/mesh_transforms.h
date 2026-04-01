#pragma once

#include "concepts/mesh_concepts.h"
#include "internal/mesh_mutation.h"
#include "meshLinearAlgebra.h"

#include <cstring>

namespace pgo::VolumetricMeshes::internal::mesh_transforms {

namespace detail {

template <class MeshT>
void compute_gravity_impl(const MeshT& mesh, double* gravity_force, double g, bool add_force) {
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

template <class MeshT>
void apply_deformation_impl(MeshT& mesh, const double* u) {
    for (int i = 0; i < mesh.getNumVertices(); i++) {
        Vec3d& vertex = mesh.getVertex(i);
        vertex[0] += u[3 * i + 0];
        vertex[1] += u[3 * i + 1];
        vertex[2] += u[3 * i + 2];
    }
}

template <class MeshT>
void apply_linear_transformation_impl(MeshT& mesh, double* pos, double* R) {
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

template <class MeshT>
void renumber_vertices_impl(MeshT& mesh, const std::vector<int>& permutation) {
    MeshMutation::geometry(mesh).renumber_vertices(permutation);
}

}  // namespace detail

template <concepts::VolumetricMeshLike MeshT>
void compute_gravity(const MeshT& mesh, double* gravity_force, double g, bool add_force) {
    detail::compute_gravity_impl(mesh, gravity_force, g, add_force);
}

template <concepts::VolumetricMeshLike MeshT>
void apply_deformation(MeshT& mesh, const double* u) {
    detail::apply_deformation_impl(mesh, u);
}

template <concepts::VolumetricMeshLike MeshT>
void apply_linear_transformation(MeshT& mesh, double* pos, double* R) {
    detail::apply_linear_transformation_impl(mesh, pos, R);
}

template <concepts::VolumetricMeshLike MeshT>
void renumber_vertices(MeshT& mesh, const std::vector<int>& permutation) {
    detail::renumber_vertices_impl(mesh, permutation);
}

}  // namespace pgo::VolumetricMeshes::internal::mesh_transforms
