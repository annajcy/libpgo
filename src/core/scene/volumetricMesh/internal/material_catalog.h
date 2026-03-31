#pragma once

#include "volumetricMesh.h"

#include <memory>
#include <vector>

namespace pgo::VolumetricMeshes::internal {

class MaterialCatalog {
public:
    MaterialCatalog() = default;
    MaterialCatalog(int num_elements, double E, double nu, double density);
    MaterialCatalog(std::vector<std::unique_ptr<VolumetricMesh::Material>> materials,
                    std::vector<VolumetricMesh::Set> sets, std::vector<VolumetricMesh::Region> regions,
                    int num_elements, int verbose);
    MaterialCatalog(const MaterialCatalog& other);
    MaterialCatalog& operator=(const MaterialCatalog& other);
    MaterialCatalog(MaterialCatalog&&) noexcept = default;
    MaterialCatalog& operator=(MaterialCatalog&&) noexcept = default;
    ~MaterialCatalog() = default;

    int num_materials() const;
    int num_sets() const;
    int num_regions() const;

    const VolumetricMesh::Material* material(int index) const;
    VolumetricMesh::Material* material(int index);
    const VolumetricMesh::Material* element_material(int element) const;
    VolumetricMesh::Material* element_material(int element);

    const VolumetricMesh::Set& set(int index) const;
    VolumetricMesh::Set& set(int index);
    const VolumetricMesh::Region& region(int index) const;
    VolumetricMesh::Region& region(int index);

    void set_material(int index, const VolumetricMesh::Material* material);
    void set_single_material(int num_elements, double E, double nu, double density);
    void assign_materials_to_elements(int num_elements, int verbose);
    void propagate_regions_to_elements();
    void add_material(int num_elements, const VolumetricMesh::Material* material, const VolumetricMesh::Set& new_set,
                      bool remove_empty_sets, bool remove_empty_materials);
    void validate_against_num_elements(int num_elements) const;

    const std::vector<int>& element_materials() const;
    std::vector<int>& mutable_element_materials();
    const std::vector<VolumetricMesh::Set>& sets() const;
    std::vector<VolumetricMesh::Set>& mutable_sets();
    const std::vector<VolumetricMesh::Region>& regions() const;
    std::vector<VolumetricMesh::Region>& mutable_regions();

private:
    std::vector<std::unique_ptr<VolumetricMesh::Material>> m_materials;
    std::vector<VolumetricMesh::Set>                       m_sets;
    std::vector<VolumetricMesh::Region>                    m_regions;
    std::vector<int>                                       m_element_materials;
};

}  // namespace pgo::VolumetricMeshes::internal
