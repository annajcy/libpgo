#include "volumetricMeshEdit.h"

#include "internal/material_catalog.h"
#include "pgoLogging.h"

#include <algorithm>
#include <cstring>

namespace pgo::VolumetricMeshes {
namespace editing {

void subset_in_place(VolumetricMesh& mesh, const std::set<int>& subsetElements, int removeIsolatedVertices,
                     std::map<int, int>* old2NewVertexIDMap) {
    internal::VolumetricMeshData& geometry = mesh.geometry_data();
    std::span<int>                elements = geometry.elements();
    const int                     num_elements = geometry.num_elements();
    const int                     num_element_vertices = geometry.num_element_vertices();
    std::vector<int>&             element_materials = mesh.material_catalog().mutable_element_materials();
    std::vector<VolumetricMesh::Set>& sets = mesh.material_catalog().mutable_sets();

    std::vector<int> lookupTable(static_cast<size_t>(num_elements), -1);

    int head = 0;
    for (int tail = 0; tail < num_elements; tail++) {
        if (subsetElements.find(tail) != subsetElements.end()) {
            if (head != tail) {
                memcpy(&elements[static_cast<size_t>(head * num_element_vertices)],
                       &elements[static_cast<size_t>(tail * num_element_vertices)], sizeof(int) * num_element_vertices);
                element_materials[static_cast<size_t>(head)] = element_materials[static_cast<size_t>(tail)];
            }
            lookupTable[static_cast<size_t>(tail)] = head;
            head++;
        }
    }
    PGO_ALOG(head == static_cast<int>(subsetElements.size()));

    geometry.resize_elements_for_num_elements(static_cast<int>(subsetElements.size()));
    elements = geometry.elements();
    element_materials.resize(static_cast<size_t>(geometry.num_elements()));

    for (int setIndex = 0; setIndex < mesh.getNumSets(); setIndex++) {
        std::set<int> oldSet = std::move(sets[static_cast<size_t>(setIndex)].getElements());
        sets[static_cast<size_t>(setIndex)].clear();
        for (int eleID : oldSet) {
            if (subsetElements.find(eleID) == subsetElements.end())
                continue;
            sets[static_cast<size_t>(setIndex)].insert(lookupTable[static_cast<size_t>(eleID)]);
        }
    }

    if (removeIsolatedVertices)
        remove_isolated_vertices(mesh, old2NewVertexIDMap);
}

void remove_isolated_vertices(VolumetricMesh& mesh, std::map<int, int>* old2NewVertexIDMap) {
    internal::VolumetricMeshData& geometry = mesh.geometry_data();
    std::span<Vec3d>              vertices = geometry.vertices();
    std::span<int>                elements = geometry.elements();
    const int                     num_elements = geometry.num_elements();
    const int                     num_element_vertices = geometry.num_element_vertices();
    const int                     num_vertices = geometry.num_vertices();

    std::vector<int> retainedVertices;
    for (int el = 0; el < num_elements; el++) {
        for (int j = 0; j < num_element_vertices; j++)
            retainedVertices.push_back(mesh.getVertexIndex(el, j));
    }

    std::sort(retainedVertices.begin(), retainedVertices.end());
    retainedVertices.erase(std::unique(retainedVertices.begin(), retainedVertices.end()), retainedVertices.end());

    std::vector<int> old2newVtxID(static_cast<size_t>(num_vertices), -1);
    if (old2NewVertexIDMap != nullptr)
        old2NewVertexIDMap->clear();

    int head = 0;
    for (int tail = 0; tail < num_vertices; tail++) {
        if (std::binary_search(retainedVertices.begin(), retainedVertices.end(), tail)) {
            old2newVtxID[static_cast<size_t>(tail)] = head;
            if (old2NewVertexIDMap != nullptr)
                old2NewVertexIDMap->emplace(tail, head);
            if (head != tail)
                vertices[static_cast<size_t>(head)] = vertices[static_cast<size_t>(tail)];
            head++;
        }
    }
    PGO_ALOG(head == static_cast<int>(retainedVertices.size()));

    for (int idx = 0, size = num_elements * num_element_vertices; idx < size; idx++) {
        elements[static_cast<size_t>(idx)] = old2newVtxID[static_cast<size_t>(elements[static_cast<size_t>(idx)])];
    }

    geometry.resize_vertices(static_cast<int>(retainedVertices.size()));
}

}  // namespace editing
}  // namespace pgo::VolumetricMeshes
