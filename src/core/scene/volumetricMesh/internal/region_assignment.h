#pragma once

#include "volumetricMesh.h"

#include <memory>
#include <vector>

namespace pgo::VolumetricMeshes::internal {

void propagate_regions_to_elements(const std::vector<VolumetricMesh::Set>& sets,
                                   const std::vector<VolumetricMesh::Region>& regions,
                                   std::vector<int>& element_materials);

void assign_materials_to_elements_with_default(
    std::vector<std::unique_ptr<VolumetricMesh::Material>>& materials, std::vector<VolumetricMesh::Set>& sets,
    std::vector<VolumetricMesh::Region>& regions, std::vector<int>& element_materials, int num_elements, int verbose);

}  // namespace pgo::VolumetricMeshes::internal
