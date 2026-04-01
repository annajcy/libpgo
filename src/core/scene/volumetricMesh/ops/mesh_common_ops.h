#pragma once

#include "internal/material_catalog.h"
#include "internal/mesh_mass_properties.h"
#include "internal/mesh_queries.h"
#include "internal/mesh_transforms.h"
#include "storage/mesh_storage.h"

#include <cstdlib>
#include <map>
#include <set>
#include <span>
#include <vector>

namespace pgo::VolumetricMeshes::ops::common {

template <class MeshT>
int num_materials(const MeshT& mesh) {
    return mesh.material_catalog().num_materials();
}

template <class MeshT>
const MaterialRecord& material(const MeshT& mesh, int index) {
    return mesh.material_catalog().material(index);
}

template <class MeshT>
MaterialRecord& material(MeshT& mesh, int index) {
    return mesh.material_catalog().material(index);
}

template <class MeshT>
const MaterialRecord& element_material(const MeshT& mesh, int element) {
    return mesh.material_catalog().element_material(element);
}

template <class MeshT>
MaterialRecord& element_material(MeshT& mesh, int element) {
    return mesh.material_catalog().element_material(element);
}

template <class MeshT>
int num_sets(const MeshT& mesh) {
    return mesh.material_catalog().num_sets();
}

template <class MeshT>
const ElementSet& set(const MeshT& mesh, int index) {
    return mesh.material_catalog().set(index);
}

template <class MeshT>
int num_regions(const MeshT& mesh) {
    return mesh.material_catalog().num_regions();
}

template <class MeshT>
const MaterialRegion& region(const MeshT& mesh, int index) {
    return mesh.material_catalog().region(index);
}

template <class MeshT>
double element_density(const MeshT& mesh, int element) {
    return material_density(mesh.getElementMaterial(element));
}

template <class MeshT>
void assign_materials_to_elements(MeshT& mesh, int verbose) {
    mesh.material_catalog().assign_materials_to_elements(mesh.getNumElements(), verbose);
    mesh.sync_storage_from_legacy_state_for_transition();
}

template <class MeshT>
void reset_material_catalog(MeshT& mesh, std::vector<MaterialRecord> materials, std::vector<ElementSet> sets,
                            std::vector<MaterialRegion> regions, int verbose) {
    mesh.material_catalog() = internal::MaterialCatalog(std::move(materials), std::move(sets), std::move(regions),
                                                        mesh.getNumElements(), verbose);
    mesh.sync_storage_from_legacy_state_for_transition();
}

template <class MeshT>
void set_single_material(MeshT& mesh, double E, double nu, double density) {
    mesh.material_catalog().set_single_material(mesh.getNumElements(), E, nu, density);
    mesh.sync_storage_from_legacy_state_for_transition();
}

template <class MeshT>
void propagate_regions_to_elements(MeshT& mesh) {
    mesh.material_catalog().propagate_regions_to_elements();
    mesh.sync_storage_from_legacy_state_for_transition();
}

template <class MeshT>
void set_material(MeshT& mesh, int index, const MaterialRecord& value) {
    mesh.material_catalog().set_material(index, value);
    mesh.sync_storage_from_legacy_state_for_transition();
}

template <class MeshT>
void add_material(MeshT& mesh, const MaterialRecord& material_record, const ElementSet& new_set,
                  bool remove_empty_sets, bool remove_empty_materials) {
    mesh.material_catalog().add_material(mesh.getNumElements(), material_record, new_set, remove_empty_sets,
                                         remove_empty_materials);
    mesh.sync_storage_from_legacy_state_for_transition();
}

template <class MeshT>
double volume(const MeshT& mesh) {
    return internal::mesh_mass_properties::get_volume(mesh);
}

template <class MeshT>
void vertex_volumes(const MeshT& mesh, double* vertex_volumes) {
    internal::mesh_mass_properties::get_vertex_volumes(mesh, vertex_volumes);
}

template <class MeshT>
double mass(const MeshT& mesh) {
    return internal::mesh_mass_properties::get_mass(mesh);
}

template <class MeshT>
void inertia_parameters(const MeshT& mesh, double& mass_out, Vec3d& center_of_mass, Mat3d& inertia_tensor) {
    internal::mesh_mass_properties::get_inertia_parameters(mesh, mass_out, center_of_mass, inertia_tensor);
}

template <class MeshT>
void mesh_geometric_parameters(const MeshT& mesh, Vec3d& centroid, double* radius) {
    internal::mesh_mass_properties::get_mesh_geometric_parameters(mesh, centroid, radius);
}

template <class MeshT>
Mesh::BoundingBox bounding_box(const MeshT& mesh) {
    return internal::mesh_mass_properties::get_bounding_box(mesh);
}

template <class MeshT>
Vec3d element_center(const MeshT& mesh, int element) {
    Vec3d pos(0, 0, 0);
    for (int i = 0; i < mesh.getNumElementVertices(); i++) {
        pos += mesh.getVertex(element, i);
    }
    pos *= 1.0 / mesh.getNumElementVertices();
    return pos;
}

template <class MeshT>
void vertices_in_elements(const MeshT& mesh, const std::vector<int>& elements, std::vector<int>& vertices) {
    internal::mesh_queries::get_vertices_in_elements(mesh, elements, vertices);
}

template <class MeshT>
void elements_touching_vertices(const MeshT& mesh, const std::vector<int>& vertices, std::vector<int>& elements) {
    internal::mesh_queries::get_elements_touching_vertices(mesh, vertices, elements);
}

template <class MeshT>
void elements_with_only_vertices(const MeshT& mesh, const std::vector<int>& vertices, std::vector<int>& elements) {
    internal::mesh_queries::get_elements_with_only_vertices(mesh, vertices, elements);
}

template <class MeshT>
void vertex_neighborhood(const MeshT& mesh, const std::vector<int>& vertices, std::vector<int>& neighborhood) {
    internal::mesh_queries::get_vertex_neighborhood(mesh, vertices, neighborhood);
}

template <class MeshT>
int closest_vertex(const MeshT& mesh, Vec3d pos) {
    return internal::mesh_queries::get_closest_vertex(mesh, pos);
}

template <class MeshT>
int containing_element(const MeshT& mesh, Vec3d pos) {
    return internal::mesh_queries::get_containing_element(mesh, pos);
}

template <class MeshT>
int interpolate_gradient(const MeshT& mesh, const double* U, int num_fields, Vec3d pos, double* grad) {
    int external_vertex = 0;
    int element         = mesh.getContainingElement(pos);
    if (element < 0) {
        element         = mesh.getClosestElement(pos);
        external_vertex = 1;
    }

    mesh.interpolateGradient(element, U, num_fields, pos, grad);
    return external_vertex;
}

template <class MeshT>
void compute_gravity(const MeshT& mesh, double* gravity_force, double g, bool add_force) {
    internal::mesh_transforms::compute_gravity(mesh, gravity_force, g, add_force);
}

template <class MeshT>
void apply_deformation(MeshT& mesh, const double* deformation) {
    internal::mesh_transforms::apply_deformation(mesh, deformation);
    mesh.sync_storage_from_legacy_state_for_transition();
}

template <class MeshT>
void apply_linear_transformation(MeshT& mesh, double* pos, double* R) {
    internal::mesh_transforms::apply_linear_transformation(mesh, pos, R);
    mesh.sync_storage_from_legacy_state_for_transition();
}

template <class MeshT>
void renumber_vertices(MeshT& mesh, const std::vector<int>& permutation) {
    internal::mesh_transforms::renumber_vertices(mesh, permutation);
    mesh.sync_storage_from_legacy_state_for_transition();
}

template <class MeshT>
storage::MeshStorage make_subset_storage(const MeshT& mesh, std::span<const int> elements,
                                         std::map<int, int>* vertex_map) {
    const int num_element_vertices = mesh.getNumElementVertices();
    std::set<int> vertex_set;
    for (int i = 0; i < static_cast<int>(elements.size()); i++) {
        for (int j = 0; j < num_element_vertices; j++) {
            vertex_set.insert(mesh.getVertexIndex(elements[i], j));
        }
    }

    std::vector<Vec3d> subset_vertices(vertex_set.size());
    int vertex_no = 0;
    std::map<int, int> local_vertex_map;
    for (int old_vertex : vertex_set) {
        subset_vertices[static_cast<size_t>(vertex_no)] = mesh.getVertex(old_vertex);
        local_vertex_map.insert(std::make_pair(old_vertex, vertex_no));
        vertex_no++;
    }

    if (vertex_map != nullptr) {
        *vertex_map = local_vertex_map;
    }

    const int num_elements = static_cast<int>(elements.size());
    std::vector<int> subset_elements(static_cast<size_t>(num_elements * num_element_vertices));
    std::map<int, int> element_map;
    for (int i = 0; i < num_elements; i++) {
        for (int j = 0; j < num_element_vertices; j++) {
            auto iter = local_vertex_map.find(mesh.getVertexIndex(elements[i], j));
            if (iter == local_vertex_map.end()) {
                printf("Internal error 1.\n");
                throw 1;
            }
            subset_elements[static_cast<size_t>(i * num_element_vertices + j)] = iter->second;
        }
        element_map.insert(std::make_pair(elements[i], i));
    }

    std::vector<MaterialRecord> subset_materials(static_cast<size_t>(mesh.getNumMaterials()));
    for (int i = 0; i < mesh.getNumMaterials(); i++) {
        subset_materials[static_cast<size_t>(i)] = mesh.getMaterial(i);
    }

    std::vector<ElementSet> new_sets;
    std::map<int, int> old_to_new_set_index;
    for (int old_set_index = 0; old_set_index < mesh.getNumSets(); old_set_index++) {
        const ElementSet& old_set = mesh.getSet(old_set_index);
        std::set<int> old_elements;
        old_set.getElements(old_elements);

        for (int old_element : old_elements) {
            if (old_element < 0) {
                printf("Internal error 2.\n");
                std::exit(1);
            }
        }

        std::vector<int> new_elements;
        for (int old_element : old_elements) {
            auto iter = element_map.find(old_element);
            if (iter != element_map.end()) {
                new_elements.push_back(iter->second);
            }
        }

        if (!new_elements.empty()) {
            ElementSet new_set(old_set.getName());
            for (int new_element : new_elements) {
                if (new_element < 0) {
                    printf("Internal error 3.\n");
                    std::exit(1);
                }
                new_set.insert(new_element);
            }
            new_sets.emplace_back(std::move(new_set));
            old_to_new_set_index.insert(std::make_pair(old_set_index, static_cast<int>(new_sets.size()) - 1));
        }
    }

    std::vector<MaterialRegion> new_regions;
    for (int i = 0; i < mesh.getNumRegions(); i++) {
        const MaterialRegion& source_region = mesh.getRegion(i);
        auto iter = old_to_new_set_index.find(source_region.getSetIndex());
        if (iter != old_to_new_set_index.end()) {
            new_regions.emplace_back(source_region.getMaterialIndex(), iter->second);
        }
    }

    storage::MeshStorage storage(
        internal::VolumetricMeshData(num_element_vertices, std::move(subset_vertices), std::move(subset_elements)),
        internal::MaterialCatalog(std::move(subset_materials), std::move(new_sets), std::move(new_regions), num_elements,
                                  0));

    for (int element = 0; element < num_elements; element++) {
        int found = 0;
        for (int region_index = 0; region_index < storage.material_catalog().num_regions(); region_index++) {
            const MaterialRegion& stored_region = storage.material_catalog().region(region_index);
            const int element_set = stored_region.getSetIndex();
            if (storage.material_catalog().set(element_set).isMember(element)) {
                if (found != 0) {
                    printf("Warning: element %d (1-indexed) is in more than one region.\n", element + 1);
                } else {
                    found = 1;
                }
            }
        }
        if (found == 0) {
            printf("Warning: element %d (1-indexed) is not in any of the regions.\n", element + 1);
        }
    }

    for (int i = 0; i < storage.material_catalog().num_sets(); i++) {
        std::set<int> set_elements;
        storage.material_catalog().set(i).getElements(set_elements);
        for (int element : set_elements) {
            if (element < 0) {
                printf("Warning: encountered negative element index in element set %d.\n", i);
            }
            if (element >= num_elements) {
                printf("Warning: encountered too large element index in element set %d.\n", i);
            }
        }
    }

    storage.validate_invariants();
    return storage;
}

}  // namespace pgo::VolumetricMeshes::ops::common
