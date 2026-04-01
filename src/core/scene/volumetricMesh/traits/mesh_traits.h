#pragma once

#include "ops/cubic_mesh_ops.h"
#include "ops/tet_mesh_ops.h"
#include "traits/mesh_tags.h"

#include <type_traits>

namespace pgo::VolumetricMeshes {

class TetMesh;
class CubicMesh;

namespace traits {

template <class MeshT>
struct mesh_traits;

template <class MeshT>
using mesh_tag_t = typename mesh_traits<std::remove_cvref_t<MeshT>>::tag;

template <>
struct mesh_traits<TetMesh> {
    using tag = TetTag;

    static constexpr ElementType kElementType     = ElementType::Tet;
    static constexpr int         kElementVertices = 4;

    static constexpr ElementType element_type() { return kElementType; }
    static constexpr int         element_vertices() { return kElementVertices; }

    static double element_volume(const TetMesh& mesh, int element) {
        return ops::element_volume(mesh, element);
    }

    static void element_inertia_tensor(const TetMesh& mesh, int element, Mat3d& inertia_tensor) {
        ops::element_inertia_tensor(mesh, element, inertia_tensor);
    }

    static void compute_element_mass_matrix(const TetMesh& mesh, int element, double* mass_matrix) {
        ops::compute_element_mass_matrix(mesh, element, mass_matrix);
    }

    static bool contains_vertex(const TetMesh& mesh, int element, Vec3d pos) {
        return ops::contains_vertex(mesh, element, pos);
    }

    static void compute_barycentric_weights(const TetMesh& mesh, int element, const Vec3d& pos, double* weights) {
        ops::compute_barycentric_weights(mesh, element, pos, weights);
    }

    static void interpolate_gradient(const TetMesh& mesh, int element, const double* U, int num_fields, Vec3d pos,
                                     double* grad) {
        (void)pos;
        ops::compute_gradient(mesh, element, U, num_fields, grad);
    }

    static int closest_element(const TetMesh& mesh, const Vec3d& pos) { return ops::closest_element(mesh, pos); }

    static int num_element_edges(const TetMesh& mesh) { return ops::num_element_edges(mesh); }

    static void fill_element_edges(const TetMesh& mesh, int element, int* edge_buffer) {
        ops::fill_element_edges(mesh, element, edge_buffer);
    }
};

template <>
struct mesh_traits<CubicMesh> {
    using tag = CubicTag;

    static constexpr ElementType kElementType     = ElementType::Cubic;
    static constexpr int         kElementVertices = 8;

    static constexpr ElementType element_type() { return kElementType; }
    static constexpr int         element_vertices() { return kElementVertices; }

    static double element_volume(const CubicMesh& mesh, int element) {
        return ops::element_volume(mesh, element);
    }

    static void element_inertia_tensor(const CubicMesh& mesh, int element, Mat3d& inertia_tensor) {
        ops::element_inertia_tensor(mesh, element, inertia_tensor);
    }

    static void compute_element_mass_matrix(const CubicMesh& mesh, int element, double* mass_matrix) {
        ops::compute_element_mass_matrix(mesh, element, mass_matrix);
    }

    static bool contains_vertex(const CubicMesh& mesh, int element, Vec3d pos) {
        return ops::contains_vertex(mesh, element, pos);
    }

    static void compute_barycentric_weights(const CubicMesh& mesh, int element, const Vec3d& pos, double* weights) {
        ops::compute_barycentric_weights(mesh, element, pos, weights);
    }

    static void interpolate_gradient(const CubicMesh& mesh, int element, const double* U, int num_fields, Vec3d pos,
                                     double* grad) {
        ops::interpolate_gradient(mesh, element, U, num_fields, pos, grad);
    }

    static int num_element_edges(const CubicMesh& mesh) { return ops::num_element_edges(mesh); }

    static void fill_element_edges(const CubicMesh& mesh, int element, int* edge_buffer) {
        ops::fill_element_edges(mesh, element, edge_buffer);
    }
};

}  // namespace traits
}  // namespace pgo::VolumetricMeshes
