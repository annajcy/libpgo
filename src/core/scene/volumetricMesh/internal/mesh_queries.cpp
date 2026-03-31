#include "internal/mesh_queries.h"

#include <algorithm>
#include <cfloat>

namespace pgo::VolumetricMeshes::internal::mesh_queries {

void get_vertices_in_elements(const VolumetricMesh& mesh, const std::vector<int>& elements, std::vector<int>& vertices) {
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

void get_elements_touching_vertices(const VolumetricMesh& mesh, const std::vector<int>& vertices,
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

void get_elements_with_only_vertices(const VolumetricMesh& mesh, const std::vector<int>& vertices,
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

void get_vertex_neighborhood(const VolumetricMesh& mesh, const std::vector<int>& vertices, std::vector<int>& neighborhood) {
    std::vector<int> elements;
    get_elements_touching_vertices(mesh, vertices, elements);
    get_vertices_in_elements(mesh, elements, neighborhood);
}

int get_closest_vertex(const VolumetricMesh& mesh, Vec3d pos) {
    double closest_dist = DBL_MAX;
    int    closest_vertex = -1;

    for (int i = 0; i < mesh.getNumVertices(); i++) {
        const Vec3d& vertex_position = mesh.getVertex(i);
        const double dist = (pos - vertex_position).norm();
        if (dist < closest_dist) {
            closest_dist = dist;
            closest_vertex = i;
        }
    }

    return closest_vertex;
}

int get_closest_element(const VolumetricMesh& mesh, const Vec3d& pos) {
    double closest_dist = DBL_MAX;
    int    closest_element = 0;
    for (int element = 0; element < mesh.getNumElements(); element++) {
        const Vec3d center = mesh.getElementCenter(element);
        const double dist = (pos - center).norm();
        if (dist < closest_dist) {
            closest_dist = dist;
            closest_element = element;
        }
    }

    return closest_element;
}

int get_containing_element(const VolumetricMesh& mesh, Vec3d pos) {
    for (int element = 0; element < mesh.getNumElements(); element++) {
        if (mesh.containsVertex(element, pos)) {
            return element;
        }
    }
    return -1;
}

}  // namespace pgo::VolumetricMeshes::internal::mesh_queries
