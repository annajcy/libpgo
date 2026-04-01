#pragma once

#include "volumetricMesh.h"

#include <vector>

namespace pgo::VolumetricMeshes::internal {

void propagate_regions_to_elements(const std::vector<ElementSet>& sets,
                                   const std::vector<MaterialRegion>& regions,
                                   std::vector<int>& element_materials);

void assign_materials_to_elements_with_default(
    std::vector<MaterialRecord>& materials, std::vector<ElementSet>& sets, std::vector<MaterialRegion>& regions,
    std::vector<int>& element_materials, int num_elements, int verbose);

}  // namespace pgo::VolumetricMeshes::internal
