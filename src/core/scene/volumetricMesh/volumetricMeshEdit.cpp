#include "volumetricMeshEdit.h"

#include "pgoLogging.h"

#include <algorithm>
#include <cstring>

namespace pgo::VolumetricMeshes {
namespace editing {

void subset_in_place(VolumetricMesh& mesh, const std::set<int>& subsetElements, int removeIsolatedVertices,
                     std::map<int, int>* old2NewVertexIDMap) {
    std::vector<int> lookupTable(static_cast<size_t>(mesh.numElements), -1);

    int head = 0;
    for (int tail = 0; tail < mesh.numElements; tail++) {
        if (subsetElements.find(tail) != subsetElements.end()) {
            if (head != tail) {
                memcpy(&mesh.elements[static_cast<size_t>(head * mesh.numElementVertices)],
                       &mesh.elements[static_cast<size_t>(tail * mesh.numElementVertices)],
                       sizeof(int) * mesh.numElementVertices);
                mesh.elementMaterial[static_cast<size_t>(head)] = mesh.elementMaterial[static_cast<size_t>(tail)];
            }
            lookupTable[static_cast<size_t>(tail)] = head;
            head++;
        }
    }
    PGO_ALOG(head == static_cast<int>(subsetElements.size()));

    mesh.numElements = static_cast<int>(subsetElements.size());
    mesh.elements.resize(static_cast<size_t>(mesh.numElements * mesh.numElementVertices));
    mesh.elementMaterial.resize(static_cast<size_t>(mesh.numElements));

    for (int setIndex = 0; setIndex < mesh.numSets; setIndex++) {
        std::set<int> oldSet = std::move(mesh.sets[static_cast<size_t>(setIndex)].getElements());
        mesh.sets[static_cast<size_t>(setIndex)].clear();
        for (int eleID : oldSet) {
            if (subsetElements.find(eleID) == subsetElements.end())
                continue;
            mesh.sets[static_cast<size_t>(setIndex)].insert(lookupTable[static_cast<size_t>(eleID)]);
        }
    }

    if (removeIsolatedVertices)
        remove_isolated_vertices(mesh, old2NewVertexIDMap);
}

void remove_isolated_vertices(VolumetricMesh& mesh, std::map<int, int>* old2NewVertexIDMap) {
    std::vector<int> retainedVertices;
    for (int el = 0; el < mesh.numElements; el++) {
        for (int j = 0; j < mesh.numElementVertices; j++)
            retainedVertices.push_back(mesh.getVertexIndex(el, j));
    }

    std::sort(retainedVertices.begin(), retainedVertices.end());
    retainedVertices.erase(std::unique(retainedVertices.begin(), retainedVertices.end()), retainedVertices.end());

    std::vector<int> old2newVtxID(static_cast<size_t>(mesh.numVertices), -1);
    if (old2NewVertexIDMap != nullptr)
        old2NewVertexIDMap->clear();

    int head = 0;
    for (int tail = 0; tail < mesh.numVertices; tail++) {
        if (std::binary_search(retainedVertices.begin(), retainedVertices.end(), tail)) {
            old2newVtxID[static_cast<size_t>(tail)] = head;
            if (old2NewVertexIDMap != nullptr)
                old2NewVertexIDMap->emplace(tail, head);
            if (head != tail)
                mesh.vertices[static_cast<size_t>(head)] = mesh.vertices[static_cast<size_t>(tail)];
            head++;
        }
    }
    PGO_ALOG(head == static_cast<int>(retainedVertices.size()));

    for (int idx = 0, size = mesh.numElements * mesh.numElementVertices; idx < size; idx++)
        mesh.elements[static_cast<size_t>(idx)] =
            old2newVtxID[static_cast<size_t>(mesh.elements[static_cast<size_t>(idx)])];

    mesh.numVertices = static_cast<int>(retainedVertices.size());
    mesh.vertices.resize(static_cast<size_t>(mesh.numVertices));
}

}  // namespace editing
}  // namespace pgo::VolumetricMeshes
