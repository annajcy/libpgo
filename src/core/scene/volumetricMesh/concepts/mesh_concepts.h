#pragma once

#include "traits/mesh_traits.h"

#include <concepts>
#include <type_traits>

namespace pgo::VolumetricMeshes::concepts {

template <class MeshT>
concept HasMeshTraits = requires {
    typename traits::mesh_tag_t<MeshT>;
};

template <class MeshT>
concept VolumetricMeshLike =
    HasMeshTraits<MeshT> &&
    requires(const std::remove_cvref_t<MeshT>& mesh, int element, Vec3d pos, double* weights, double* mass_matrix,
             double* grad, Mat3d inertia_tensor) {
        { mesh.getNumVertices() } -> std::convertible_to<int>;
        { mesh.getNumElements() } -> std::convertible_to<int>;
        { mesh.getNumElementVertices() } -> std::convertible_to<int>;
        { mesh.getVertexIndex(element, 0) } -> std::convertible_to<int>;
        { mesh.getVertex(element, 0) } -> std::same_as<const Vec3d&>;
        { traits::mesh_traits<std::remove_cvref_t<MeshT>>::element_type() } -> std::same_as<ElementType>;
        { traits::mesh_traits<std::remove_cvref_t<MeshT>>::element_vertices() } -> std::convertible_to<int>;
        { traits::mesh_traits<std::remove_cvref_t<MeshT>>::contains_vertex(mesh, element, pos) } -> std::same_as<bool>;
        { traits::mesh_traits<std::remove_cvref_t<MeshT>>::element_volume(mesh, element) } -> std::convertible_to<double>;
        traits::mesh_traits<std::remove_cvref_t<MeshT>>::compute_barycentric_weights(mesh, element, pos, weights);
        traits::mesh_traits<std::remove_cvref_t<MeshT>>::compute_element_mass_matrix(mesh, element, mass_matrix);
        traits::mesh_traits<std::remove_cvref_t<MeshT>>::element_inertia_tensor(mesh, element, inertia_tensor);
        traits::mesh_traits<std::remove_cvref_t<MeshT>>::interpolate_gradient(mesh, element, nullptr, 0, pos, grad);
    };

template <class MeshT>
concept TetMeshLike = VolumetricMeshLike<MeshT> && std::same_as<traits::mesh_tag_t<MeshT>, traits::TetTag>;

template <class MeshT>
concept CubicMeshLike = VolumetricMeshLike<MeshT> && std::same_as<traits::mesh_tag_t<MeshT>, traits::CubicTag>;

}  // namespace pgo::VolumetricMeshes::concepts
