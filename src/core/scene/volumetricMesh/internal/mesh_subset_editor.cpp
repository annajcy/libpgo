#include "internal/mesh_subset_editor.h"

#include "internal/mesh_mutation.h"

#include "pgoLogging.h"

#include <algorithm>
#include <cstring>
#include <span>
#include <vector>

namespace pgo::VolumetricMeshes::internal {

void MeshSubsetEditor::subset_in_place(VolumetricMesh& mesh, const std::set<int>& subset_elements,
                                       bool remove_isolated_vertices, std::map<int, int>* old_to_new_vertex_id_map) {
    VolumetricMeshData& geometry = MeshMutation::geometry(mesh);
    std::span<int>      elements = geometry.elements();
    const int           num_elements = geometry.num_elements();
    const int           num_element_vertices = geometry.num_element_vertices();

    std::vector<int> old_to_new_element(static_cast<size_t>(num_elements), -1);

    int head = 0;
    for (int tail = 0; tail < num_elements; ++tail) {
        if (subset_elements.find(tail) != subset_elements.end()) {
            if (head != tail) {
                std::memcpy(&elements[static_cast<size_t>(head * num_element_vertices)],
                            &elements[static_cast<size_t>(tail * num_element_vertices)],
                            sizeof(int) * num_element_vertices);
            }
            old_to_new_element[static_cast<size_t>(tail)] = head;
            ++head;
        }
    }
    PGO_ALOG(head == static_cast<int>(subset_elements.size()));

    geometry.resize_elements_for_num_elements(static_cast<int>(subset_elements.size()));
    MeshMutation::material_catalog(mesh).remap_elements(old_to_new_element, geometry.num_elements());

    if (remove_isolated_vertices) {
        MeshSubsetEditor::remove_isolated_vertices(mesh, old_to_new_vertex_id_map);
    }
}

void MeshSubsetEditor::remove_isolated_vertices(VolumetricMesh& mesh, std::map<int, int>* old_to_new_vertex_id_map) {
    VolumetricMeshData& geometry = MeshMutation::geometry(mesh);
    std::span<Vec3d>    vertices = geometry.vertices();
    std::span<int>      elements = geometry.elements();
    const int           num_elements = geometry.num_elements();
    const int           num_element_vertices = geometry.num_element_vertices();
    const int           num_vertices = geometry.num_vertices();

    std::vector<int> retained_vertices;
    for (int element = 0; element < num_elements; ++element) {
        for (int vertex = 0; vertex < num_element_vertices; ++vertex) {
            retained_vertices.push_back(elements[static_cast<size_t>(element * num_element_vertices + vertex)]);
        }
    }

    std::sort(retained_vertices.begin(), retained_vertices.end());
    retained_vertices.erase(std::unique(retained_vertices.begin(), retained_vertices.end()), retained_vertices.end());

    std::vector<int> old_to_new_vertex(static_cast<size_t>(num_vertices), -1);
    if (old_to_new_vertex_id_map != nullptr) {
        old_to_new_vertex_id_map->clear();
    }

    int head = 0;
    for (int tail = 0; tail < num_vertices; ++tail) {
        if (std::binary_search(retained_vertices.begin(), retained_vertices.end(), tail)) {
            old_to_new_vertex[static_cast<size_t>(tail)] = head;
            if (old_to_new_vertex_id_map != nullptr) {
                old_to_new_vertex_id_map->emplace(tail, head);
            }
            if (head != tail) {
                vertices[static_cast<size_t>(head)] = vertices[static_cast<size_t>(tail)];
            }
            ++head;
        }
    }
    PGO_ALOG(head == static_cast<int>(retained_vertices.size()));

    for (int index = 0, size = num_elements * num_element_vertices; index < size; ++index) {
        elements[static_cast<size_t>(index)] =
            old_to_new_vertex[static_cast<size_t>(elements[static_cast<size_t>(index)])];
    }

    geometry.resize_vertices(static_cast<int>(retained_vertices.size()));
}

}  // namespace pgo::VolumetricMeshes::internal
