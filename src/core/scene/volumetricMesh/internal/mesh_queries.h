#pragma once

#include "concepts/mesh_concepts.h"
#include "meshLinearAlgebra.h"

#include <algorithm>
#include <cfloat>
#include <vector>

namespace pgo::VolumetricMeshes::internal::mesh_queries {

namespace detail {

template <class MeshT>
void get_vertices_in_elements_impl(const MeshT& mesh, const std::vector<int>& elements, std::vector<int>& vertices) {
    vertices.clear();
    for (unsigned int i = 0; i < elements.size(); i++) {
        for (int j = 0; j < mesh.getNumElementVertices(); j++) {
            vertices.push_back(mesh.getVertexIndex(elements[i], j));
        }
    }

    std::sort(vertices.begin(), vertices.end());
    const auto new_end = std::unique(vertices.begin(), vertices.end());
    vertices.resize(std::distance(vertices.begin(), new_end));
}

template <class MeshT>
void get_elements_touching_vertices_impl(const MeshT& mesh, const std::vector<int>& vertices,
                                         std::vector<int>& elements) {
    elements.clear();
    for (int i = 0; i < mesh.getNumElements(); i++) {
        for (int j = 0; j < mesh.getNumElementVertices(); j++) {
            if (std::binary_search(vertices.begin(), vertices.end(), mesh.getVertexIndex(i, j))) {
                elements.push_back(i);
                break;
            }
        }
    }
}

template <class MeshT>
void get_elements_with_only_vertices_impl(const MeshT& mesh, const std::vector<int>& vertices,
                                          std::vector<int>& elements) {
    elements.clear();
    for (int i = 0; i < mesh.getNumElements(); i++) {
        bool with_only_vertices = true;
        for (int j = 0; j < mesh.getNumElementVertices(); j++) {
            if (!std::binary_search(vertices.begin(), vertices.end(), mesh.getVertexIndex(i, j))) {
                with_only_vertices = false;
                break;
            }
        }
        if (with_only_vertices) {
            elements.push_back(i);
        }
    }
}

template <class MeshT>
void get_vertex_neighborhood_impl(const MeshT& mesh, const std::vector<int>& vertices, std::vector<int>& neighborhood) {
    std::vector<int> elements;
    get_elements_touching_vertices_impl(mesh, vertices, elements);
    get_vertices_in_elements_impl(mesh, elements, neighborhood);
}

template <class MeshT>
int get_closest_vertex_impl(const MeshT& mesh, Vec3d pos) {
    double closest_dist   = DBL_MAX;
    int    closest_vertex = -1;

    for (int i = 0; i < mesh.getNumVertices(); i++) {
        const Vec3d& vertex_position = mesh.getVertex(i);
        const double dist            = (pos - vertex_position).norm();
        if (dist < closest_dist) {
            closest_dist   = dist;
            closest_vertex = i;
        }
    }

    return closest_vertex;
}

template <class MeshT>
int get_closest_element_impl(const MeshT& mesh, const Vec3d& pos) {
    double closest_dist    = DBL_MAX;
    int    closest_element = 0;
    for (int element = 0; element < mesh.getNumElements(); element++) {
        const Vec3d center = mesh.getElementCenter(element);
        const double dist  = (pos - center).norm();
        if (dist < closest_dist) {
            closest_dist    = dist;
            closest_element = element;
        }
    }

    return closest_element;
}

template <class MeshT>
int get_containing_element_impl(const MeshT& mesh, Vec3d pos) {
    for (int element = 0; element < mesh.getNumElements(); element++) {
        if (mesh.containsVertex(element, pos)) {
            return element;
        }
    }
    return -1;
}

}  // namespace detail

template <concepts::VolumetricMeshLike MeshT>
void get_vertices_in_elements(const MeshT& mesh, const std::vector<int>& elements, std::vector<int>& vertices) {
    detail::get_vertices_in_elements_impl(mesh, elements, vertices);
}

template <concepts::VolumetricMeshLike MeshT>
void get_elements_touching_vertices(const MeshT& mesh, const std::vector<int>& vertices, std::vector<int>& elements) {
    detail::get_elements_touching_vertices_impl(mesh, vertices, elements);
}

template <concepts::VolumetricMeshLike MeshT>
void get_elements_with_only_vertices(const MeshT& mesh, const std::vector<int>& vertices,
                                     std::vector<int>& elements) {
    detail::get_elements_with_only_vertices_impl(mesh, vertices, elements);
}

template <concepts::VolumetricMeshLike MeshT>
void get_vertex_neighborhood(const MeshT& mesh, const std::vector<int>& vertices, std::vector<int>& neighborhood) {
    detail::get_vertex_neighborhood_impl(mesh, vertices, neighborhood);
}

template <concepts::VolumetricMeshLike MeshT>
int get_closest_vertex(const MeshT& mesh, Vec3d pos) {
    return detail::get_closest_vertex_impl(mesh, pos);
}

template <concepts::VolumetricMeshLike MeshT>
int get_closest_element(const MeshT& mesh, const Vec3d& pos) {
    return detail::get_closest_element_impl(mesh, pos);
}

template <concepts::VolumetricMeshLike MeshT>
int get_containing_element(const MeshT& mesh, Vec3d pos) {
    return detail::get_containing_element_impl(mesh, pos);
}

}  // namespace pgo::VolumetricMeshes::internal::mesh_queries
