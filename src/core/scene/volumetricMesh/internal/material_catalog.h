#pragma once

#include "volumetricMesh.h"

#include <vector>

namespace pgo::VolumetricMeshes::internal {

class MaterialCatalog {
public:
    MaterialCatalog() = default;
    MaterialCatalog(int num_elements, double E, double nu, double density);
    MaterialCatalog(std::vector<MaterialRecord> materials, std::vector<ElementSet> sets,
                    std::vector<MaterialRegion> regions, int num_elements, int verbose);
    MaterialCatalog(const MaterialCatalog& other) = default;
    MaterialCatalog& operator=(const MaterialCatalog& other) = default;
    MaterialCatalog(MaterialCatalog&&) noexcept = default;
    MaterialCatalog& operator=(MaterialCatalog&&) noexcept = default;
    ~MaterialCatalog() = default;

    int num_materials() const;
    int num_sets() const;
    int num_regions() const;

    const MaterialRecord& material(int index) const;
    MaterialRecord& material(int index);
    const MaterialRecord& element_material(int element) const;
    MaterialRecord& element_material(int element);

    const ElementSet& set(int index) const;
    ElementSet& set(int index);
    const MaterialRegion& region(int index) const;
    MaterialRegion& region(int index);

    void set_material(int index, const MaterialRecord& material);
    void set_single_material(int num_elements, double E, double nu, double density);
    void assign_materials_to_elements(int num_elements, int verbose);
    void propagate_regions_to_elements();
    void remap_elements(std::span<const int> old_to_new_element, int new_num_elements);
    void expand_elements(int factor);
    void add_material(int num_elements, const MaterialRecord& material, const ElementSet& new_set,
                      bool remove_empty_sets, bool remove_empty_materials);
    void validate_against_num_elements(int num_elements) const;

    const std::vector<int>& element_materials() const;
    std::vector<int>& mutable_element_materials();
    const std::vector<ElementSet>& sets() const;
    std::vector<ElementSet>& mutable_sets();
    const std::vector<MaterialRegion>& regions() const;
    std::vector<MaterialRegion>& mutable_regions();

private:
    std::vector<MaterialRecord> m_materials;
    std::vector<ElementSet> m_sets;
    std::vector<MaterialRegion> m_regions;
    std::vector<int> m_element_materials;
};

}  // namespace pgo::VolumetricMeshes::internal
