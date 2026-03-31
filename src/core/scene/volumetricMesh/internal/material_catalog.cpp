#include "internal/material_catalog.h"

#include "internal/region_assignment.h"
#include "volumetricMeshENuMaterial.h"

#include "pgoLogging.h"

#include <set>
#include <utility>

namespace pgo::VolumetricMeshes::internal {
namespace {

std::vector<std::unique_ptr<VolumetricMesh::Material>> clone_materials(
    const std::vector<std::unique_ptr<VolumetricMesh::Material>>& materials) {
    std::vector<std::unique_ptr<VolumetricMesh::Material>> clones(materials.size());
    for (size_t index = 0; index < materials.size(); ++index) {
        clones[index] = materials[index]->clone();
    }
    return clones;
}

}  // namespace

MaterialCatalog::MaterialCatalog(int num_elements, double E, double nu, double density) {
    set_single_material(num_elements, E, nu, density);
}

MaterialCatalog::MaterialCatalog(std::vector<std::unique_ptr<VolumetricMesh::Material>> materials,
                                 std::vector<VolumetricMesh::Set> sets,
                                 std::vector<VolumetricMesh::Region> regions, int num_elements, int verbose)
    : m_materials(std::move(materials)),
      m_sets(std::move(sets)),
      m_regions(std::move(regions)) {
    assign_materials_to_elements(num_elements, verbose);
}

MaterialCatalog::MaterialCatalog(const MaterialCatalog& other)
    : m_materials(clone_materials(other.m_materials)),
      m_sets(other.m_sets),
      m_regions(other.m_regions),
      m_element_materials(other.m_element_materials) {}

MaterialCatalog& MaterialCatalog::operator=(const MaterialCatalog& other) {
    if (this == &other) {
        return *this;
    }
    m_materials = clone_materials(other.m_materials);
    m_sets = other.m_sets;
    m_regions = other.m_regions;
    m_element_materials = other.m_element_materials;
    return *this;
}

int MaterialCatalog::num_materials() const {
    return static_cast<int>(m_materials.size());
}

int MaterialCatalog::num_sets() const {
    return static_cast<int>(m_sets.size());
}

int MaterialCatalog::num_regions() const {
    return static_cast<int>(m_regions.size());
}

const VolumetricMesh::Material* MaterialCatalog::material(int index) const {
    return m_materials[static_cast<size_t>(index)].get();
}

VolumetricMesh::Material* MaterialCatalog::material(int index) {
    return m_materials[static_cast<size_t>(index)].get();
}

const VolumetricMesh::Material* MaterialCatalog::element_material(int element) const {
    return m_materials[static_cast<size_t>(m_element_materials[static_cast<size_t>(element)])].get();
}

VolumetricMesh::Material* MaterialCatalog::element_material(int element) {
    return m_materials[static_cast<size_t>(m_element_materials[static_cast<size_t>(element)])].get();
}

const VolumetricMesh::Set& MaterialCatalog::set(int index) const {
    return m_sets[static_cast<size_t>(index)];
}

VolumetricMesh::Set& MaterialCatalog::set(int index) {
    return m_sets[static_cast<size_t>(index)];
}

const VolumetricMesh::Region& MaterialCatalog::region(int index) const {
    return m_regions[static_cast<size_t>(index)];
}

VolumetricMesh::Region& MaterialCatalog::region(int index) {
    return m_regions[static_cast<size_t>(index)];
}

void MaterialCatalog::set_material(int index, const VolumetricMesh::Material* material_value) {
    m_materials[static_cast<size_t>(index)] = material_value->clone();
}

void MaterialCatalog::set_single_material(int num_elements, double E, double nu, double density) {
    m_materials.clear();
    m_sets.clear();
    m_regions.clear();
    m_element_materials.assign(static_cast<size_t>(num_elements), 0);

    m_materials.push_back(std::make_unique<VolumetricMesh::ENuMaterial>("defaultMaterial", density, E, nu));
    m_sets.push_back(VolumetricMesh::generateAllElementsSet(num_elements));
    m_regions.emplace_back(0, 0);
}

void MaterialCatalog::assign_materials_to_elements(int num_elements, int verbose) {
    assign_materials_to_elements_with_default(m_materials, m_sets, m_regions, m_element_materials, num_elements,
                                              verbose);
}

void MaterialCatalog::propagate_regions_to_elements() {
    ::pgo::VolumetricMeshes::internal::propagate_regions_to_elements(m_sets, m_regions, m_element_materials);
}

void MaterialCatalog::add_material(int num_elements, const VolumetricMesh::Material* material_value,
                                   const VolumetricMesh::Set& new_set, bool remove_empty_sets,
                                   bool remove_empty_materials) {
    m_materials.push_back(material_value->clone());

    const std::set<int>& new_elements = new_set.getElements();
    std::vector<bool>    element_covered(static_cast<size_t>(num_elements), false);
    for (int set_index = 1; set_index < num_sets(); ++set_index) {
        std::set<int>& current_set = m_sets[static_cast<size_t>(set_index)].getElements();
        for (std::set<int>::iterator it = current_set.begin(); it != current_set.end();) {
            const int element = *it;
            element_covered[static_cast<size_t>(element)] = true;
            if (new_elements.find(element) != new_elements.end()) {
                std::set<int>::iterator to_erase = it;
                ++it;
                current_set.erase(to_erase);
            } else {
                ++it;
            }
        }
    }

    std::set<int> rest_set;
    for (int element = 0; element < num_elements; ++element) {
        if (!element_covered[static_cast<size_t>(element)]) {
            rest_set.insert(element);
        }
    }
    if (!rest_set.empty()) {
        m_sets.emplace_back("restElements", rest_set);
    }

    m_sets.emplace_back(new_set);
    m_regions.emplace_back(num_materials() - 1, num_sets() - 1);

    if (!rest_set.empty()) {
        for (int region_index = 0; region_index < num_regions() - 1; ++region_index) {
            if (m_regions[static_cast<size_t>(region_index)].getSetIndex() == 0) {
                m_regions[static_cast<size_t>(region_index)].setSetIndex(num_sets() - 2);
            }
        }
    }

    for (const int element : new_elements) {
        PGO_ALOG(element >= 0 && element < num_elements);
        m_element_materials[static_cast<size_t>(element)] = num_materials() - 1;
    }

    if (remove_empty_sets) {
        bool             has_empty_set = false;
        std::vector<int> set_index_change(static_cast<size_t>(num_sets()), 0);
        int              new_index = 0;
        for (int set_index = 0; set_index < num_sets(); ++set_index) {
            if (m_sets[static_cast<size_t>(set_index)].getNumElements() == 0) {
                set_index_change[static_cast<size_t>(set_index)] = -1;
                has_empty_set = true;
            } else {
                set_index_change[static_cast<size_t>(set_index)] = new_index;
                if (new_index != set_index) {
                    PGO_ALOG(new_index < set_index);
                    m_sets[static_cast<size_t>(new_index)] = m_sets[static_cast<size_t>(set_index)];
                }
                ++new_index;
            }
        }

        if (has_empty_set) {
            PGO_ALOG(new_index < num_sets());
            m_sets.resize(static_cast<size_t>(new_index));

            int new_region_index = 0;
            for (int region_index = 0; region_index < num_regions(); ++region_index) {
                const int old_set_index = m_regions[static_cast<size_t>(region_index)].getSetIndex();
                PGO_ALOG(static_cast<size_t>(old_set_index) < set_index_change.size());
                if (set_index_change[static_cast<size_t>(old_set_index)] != -1) {
                    m_regions[static_cast<size_t>(region_index)].setSetIndex(
                        set_index_change[static_cast<size_t>(old_set_index)]);
                    if (new_region_index != region_index) {
                        m_regions[static_cast<size_t>(new_region_index)] = m_regions[static_cast<size_t>(region_index)];
                    }
                    ++new_region_index;
                }
            }
            m_regions.resize(static_cast<size_t>(new_region_index));
        } else {
            PGO_ALOG(new_index == num_sets());
        }
    }

    if (remove_empty_materials) {
        std::vector<int> elements_with_material(static_cast<size_t>(num_materials()), 0);
        for (int element = 0; element < num_elements; ++element) {
            const int material_index = m_element_materials[static_cast<size_t>(element)];
            PGO_ALOG(material_index >= 0 && material_index < num_materials());
            ++elements_with_material[static_cast<size_t>(material_index)];
        }

        int              new_material_index = 0;
        std::vector<int> material_index_change(static_cast<size_t>(num_materials()), 0);
        bool             has_empty_material = false;
        for (int material_index = 0; material_index < num_materials(); ++material_index) {
            if (elements_with_material[static_cast<size_t>(material_index)] == 0) {
                material_index_change[static_cast<size_t>(material_index)] = -1;
                m_materials[static_cast<size_t>(material_index)] = nullptr;
                has_empty_material = true;
            } else {
                material_index_change[static_cast<size_t>(material_index)] = new_material_index;
                if (new_material_index != material_index) {
                    m_materials[static_cast<size_t>(new_material_index)] =
                        std::move(m_materials[static_cast<size_t>(material_index)]);
                }
                ++new_material_index;
            }
        }

        if (has_empty_material) {
            m_materials.resize(static_cast<size_t>(new_material_index));

            bool has_invalid_region = false;
            int  new_region_index = 0;
            for (int region_index = 0; region_index < num_regions(); ++region_index) {
                const int old_material_index = m_regions[static_cast<size_t>(region_index)].getMaterialIndex();
                if (material_index_change[static_cast<size_t>(old_material_index)] < 0) {
                    has_invalid_region = true;
                } else {
                    m_regions[static_cast<size_t>(region_index)].setMaterialIndex(
                        material_index_change[static_cast<size_t>(old_material_index)]);
                    if (new_region_index != region_index) {
                        m_regions[static_cast<size_t>(new_region_index)] = m_regions[static_cast<size_t>(region_index)];
                    }
                    ++new_region_index;
                }
            }

            if (has_invalid_region) {
                m_regions.resize(static_cast<size_t>(new_region_index));
            }

            std::fill(m_element_materials.begin(), m_element_materials.end(), 0);
            propagate_regions_to_elements();
        }
    }
}

void MaterialCatalog::validate_against_num_elements(int num_elements) const {
    PGO_ALOG(static_cast<int>(m_element_materials.size()) == num_elements);
    PGO_ALOG(static_cast<int>(m_sets.size()) == num_sets());
    PGO_ALOG(static_cast<int>(m_regions.size()) == num_regions());
    if (!m_sets.empty()) {
        PGO_ALOG(m_sets[0].getName() == VolumetricMesh::allElementsSetName);
    }
}

const std::vector<int>& MaterialCatalog::element_materials() const {
    return m_element_materials;
}

std::vector<int>& MaterialCatalog::mutable_element_materials() {
    return m_element_materials;
}

const std::vector<VolumetricMesh::Set>& MaterialCatalog::sets() const {
    return m_sets;
}

std::vector<VolumetricMesh::Set>& MaterialCatalog::mutable_sets() {
    return m_sets;
}

const std::vector<VolumetricMesh::Region>& MaterialCatalog::regions() const {
    return m_regions;
}

std::vector<VolumetricMesh::Region>& MaterialCatalog::mutable_regions() {
    return m_regions;
}

}  // namespace pgo::VolumetricMeshes::internal
