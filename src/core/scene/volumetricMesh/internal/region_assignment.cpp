#include "internal/region_assignment.h"

#include "volumetricMeshENuMaterial.h"

#include <cstdio>
#include <set>

namespace pgo::VolumetricMeshes::internal {

void propagate_regions_to_elements(const std::vector<VolumetricMesh::Set>& sets,
                                   const std::vector<VolumetricMesh::Region>& regions,
                                   std::vector<int>& element_materials) {
    for (const VolumetricMesh::Region& region : regions) {
        const int material_index = region.getMaterialIndex();
        const std::set<int>& set_elements = sets[static_cast<size_t>(region.getSetIndex())].getElements();
        for (const int element : set_elements) {
            element_materials[static_cast<size_t>(element)] = material_index;
        }
    }
}

void assign_materials_to_elements_with_default(
    std::vector<std::unique_ptr<VolumetricMesh::Material>>& materials, std::vector<VolumetricMesh::Set>& sets,
    std::vector<VolumetricMesh::Region>& regions, std::vector<int>& element_materials, int num_elements, int verbose) {
    element_materials.assign(static_cast<size_t>(num_elements), static_cast<int>(materials.size()));

    propagate_regions_to_elements(sets, regions, element_materials);

    std::set<int> unassigned_elements;
    for (int element = 0; element < num_elements; ++element) {
        if (element_materials[static_cast<size_t>(element)] == static_cast<int>(materials.size())) {
            unassigned_elements.insert(element);
        }
    }

    if (unassigned_elements.empty()) {
        return;
    }

    if (materials.empty()) {
        materials.push_back(std::make_unique<VolumetricMesh::ENuMaterial>(
            "defaultMaterial", VolumetricMesh::density_default, VolumetricMesh::E_default, VolumetricMesh::nu_default));
    }

    sets.emplace_back("unassignedSet");
    VolumetricMesh::Set& unassigned_set = sets.back();
    for (const int element : unassigned_elements) {
        unassigned_set.insert(element);
    }

    regions.emplace_back(static_cast<int>(materials.size()) - 1, static_cast<int>(sets.size()) - 1);
    for (const int element : unassigned_elements) {
        element_materials[static_cast<size_t>(element)] = static_cast<int>(materials.size()) - 1;
    }

    if (verbose != 0) {
        std::printf(
            "Warning: %d elements were not found in any of the regions. Using default material parameters for these "
            "elements.\n",
            static_cast<int>(unassigned_elements.size()));
    }
}

}  // namespace pgo::VolumetricMeshes::internal
