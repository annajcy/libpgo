/*************************************************************************
 *                                                                       *
 * Vega FEM Simulation Library Version 4.0                               *
 *                                                                       *
 * "volumetricMesh" library , Copyright (C) 2007 CMU, 2009 MIT, 2018 USC *
 * All rights reserved.                                                  *
 *                                                                       *
 * Code authors: Jernej Barbic, Yijing Li                                *
 * http://www.jernejbarbic.com/vega                                      *
 *                                                                       *
 * Research: Jernej Barbic, Hongyi Xu, Yijing Li,                        *
 *           Danyong Zhao, Bohan Wang,                                   *
 *           Fun Shing Sin, Daniel Schroeder,                            *
 *           Doug L. James, Jovan Popovic                                *
 *                                                                       *
 * Funding: National Science Foundation, Link Foundation,                *
 *          Singapore-MIT GAMBIT Game Lab,                               *
 *          Zumberge Research and Innovation Fund at USC,                *
 *          Sloan Foundation, Okawa Foundation,                          *
 *          USC Annenberg Foundation                                     *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of the BSD-style license that is            *
 * included with this library in the file LICENSE.txt                    *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the file     *
 * LICENSE.TXT for more details.                                         *
 *                                                                       *
 *************************************************************************/

#include "volumetricMeshParser.h"
#include "volumetricMesh.h"
#include "internal/material_catalog.h"
#include "internal/mesh_mass_properties.h"
#include "internal/mesh_queries.h"
#include "internal/mesh_transforms.h"
#include "volumetricMeshIO.h"
#include "volumetricMeshENuMaterial.h"
#include "volumetricMeshOrthotropicMaterial.h"
#include "volumetricMeshMooneyRivlinMaterial.h"

#include "range.h"
#include "stringHelper.h"
#include "pgoLogging.h"

#include <tbb/parallel_for.h>

#include <cfloat>
#include <cstring>
#include <cassert>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <stdexcept>

namespace pgo {
namespace VolumetricMeshes {
namespace {
using ElementType    = VolumetricMesh::ElementType;
using FileFormatType = VolumetricMesh::FileFormatType;
using MaterialType   = VolumetricMesh::Material::MaterialType;

constexpr ElementType    INVALID = ElementType::Invalid;
constexpr ElementType    TET     = ElementType::Tet;
constexpr ElementType    CUBIC   = ElementType::Cubic;
constexpr FileFormatType ASCII   = FileFormatType::Ascii;
constexpr FileFormatType BINARY  = FileFormatType::Binary;
constexpr FileFormatType BY_EXT  = FileFormatType::ByExtension;
constexpr FileFormatType NUM_FILE_FORMATS = FileFormatType::Unknown;

constexpr int ENU_DENSITY = static_cast<int>(VolumetricMesh::Material::ENuMaterialProperty::Density);
constexpr int ENU_E = static_cast<int>(VolumetricMesh::Material::ENuMaterialProperty::E);
constexpr int ENU_NU = static_cast<int>(VolumetricMesh::Material::ENuMaterialProperty::Nu);
constexpr int ENU_NUM_PROPERTIES = static_cast<int>(VolumetricMesh::Material::ENuMaterialProperty::Count);

constexpr int ORTHOTROPIC_DENSITY = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::Density);
constexpr int ORTHOTROPIC_E1 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::E1);
constexpr int ORTHOTROPIC_E2 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::E2);
constexpr int ORTHOTROPIC_E3 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::E3);
constexpr int ORTHOTROPIC_NU12 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::Nu12);
constexpr int ORTHOTROPIC_NU23 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::Nu23);
constexpr int ORTHOTROPIC_NU31 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::Nu31);
constexpr int ORTHOTROPIC_G12 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::G12);
constexpr int ORTHOTROPIC_G23 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::G23);
constexpr int ORTHOTROPIC_G31 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::G31);
constexpr int ORTHOTROPIC_NUM_PROPERTIES =
    static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::Count);

constexpr int MOONEYRIVLIN_DENSITY = static_cast<int>(VolumetricMesh::Material::MooneyRivlinMaterialProperty::Density);
constexpr int MOONEYRIVLIN_MU01 = static_cast<int>(VolumetricMesh::Material::MooneyRivlinMaterialProperty::Mu01);
constexpr int MOONEYRIVLIN_MU10 = static_cast<int>(VolumetricMesh::Material::MooneyRivlinMaterialProperty::Mu10);
constexpr int MOONEYRIVLIN_V1 = static_cast<int>(VolumetricMesh::Material::MooneyRivlinMaterialProperty::V1);
constexpr int MOONEYRIVLIN_NUM_PROPERTIES =
    static_cast<int>(VolumetricMesh::Material::MooneyRivlinMaterialProperty::Count);
}  // namespace

VolumetricMesh::~VolumetricMesh() = default;

VolumetricMesh::VolumetricMesh(int numElementVertices_)
    : m_geometry(numElementVertices_, {}, {}),
      m_material_catalog(std::make_unique<internal::MaterialCatalog>()) {}

int VolumetricMesh::getNumMaterials() const {
    return m_material_catalog->num_materials();
}

const VolumetricMesh::Material* VolumetricMesh::getMaterial(int i) const {
    return m_material_catalog->material(i);
}

VolumetricMesh::Material* VolumetricMesh::getMaterial(int i) {
    return m_material_catalog->material(i);
}

const VolumetricMesh::Material* VolumetricMesh::getElementMaterial(int el) const {
    return m_material_catalog->element_material(el);
}

VolumetricMesh::Material* VolumetricMesh::getElementMaterial(int el) {
    return m_material_catalog->element_material(el);
}

int VolumetricMesh::getNumSets() const {
    return m_material_catalog->num_sets();
}

const VolumetricMesh::Set& VolumetricMesh::getSet(int i) const {
    return m_material_catalog->set(i);
}

int VolumetricMesh::getNumRegions() const {
    return m_material_catalog->num_regions();
}

const VolumetricMesh::Region& VolumetricMesh::getRegion(int i) const {
    return m_material_catalog->region(i);
}

double VolumetricMesh::getElementDensity(int el) const {
    return getElementMaterial(el)->getDensity();
}

internal::MaterialCatalog& VolumetricMesh::material_catalog() {
    return *m_material_catalog;
}

const internal::MaterialCatalog& VolumetricMesh::material_catalog() const {
    return *m_material_catalog;
}

void VolumetricMesh::assignMaterialsToElements(int verbose) {
    m_material_catalog->assign_materials_to_elements(getNumElements(), verbose);
}

void VolumetricMesh::reset_material_catalog(std::vector<std::unique_ptr<Material>> materials, std::vector<Set> sets,
                                            std::vector<Region> regions, int verbose) {
    m_material_catalog =
        std::make_unique<internal::MaterialCatalog>(std::move(materials), std::move(sets), std::move(regions),
                                                    getNumElements(), verbose);
}


VolumetricMesh::VolumetricMesh(std::span<const Vec3d> vertices_, int numElementVertices_, std::span<const int> elements_,
                               double E, double nu, double density)
    : m_geometry(numElementVertices_, std::vector<Vec3d>(vertices_.begin(), vertices_.end()),
                 std::vector<int>(elements_.begin(), elements_.end())),
      m_material_catalog(std::make_unique<internal::MaterialCatalog>(getNumElements(), E, nu, density)) {
}

VolumetricMesh::VolumetricMesh(std::span<const Vec3d> vertices_, int numElementVertices_, std::span<const int> elements_,
                               std::vector<std::unique_ptr<Material>> materials_, std::vector<Set> sets_,
                               std::vector<Region> regions_)
    : m_geometry(numElementVertices_, std::vector<Vec3d>(vertices_.begin(), vertices_.end()),
                 std::vector<int>(elements_.begin(), elements_.end())),
      m_material_catalog(std::make_unique<internal::MaterialCatalog>(std::move(materials_), std::move(sets_),
                                                                     std::move(regions_), getNumElements(), 0)) {
}


VolumetricMesh::VolumetricMesh(const VolumetricMesh& volumetricMesh)
    : m_geometry(volumetricMesh.m_geometry),
      m_material_catalog(std::make_unique<internal::MaterialCatalog>(*volumetricMesh.m_material_catalog)) {}

double VolumetricMesh::getVolume() const {
    return internal::mesh_mass_properties::get_volume(*this);
}

void VolumetricMesh::getVertexVolumes(double* vertexVolumes) const {
    internal::mesh_mass_properties::get_vertex_volumes(*this, vertexVolumes);
}

Vec3d VolumetricMesh::getElementCenter(int el) const {
    Vec3d pos(0, 0, 0);
    for (int i = 0; i < getNumElementVertices(); i++)
        pos += getVertex(el, i);

    pos *= 1.0 / getNumElementVertices();

    return pos;
}

void VolumetricMesh::getVerticesInElements(const std::vector<int>& elements_, std::vector<int>& vertices_) const {
    internal::mesh_queries::get_vertices_in_elements(*this, elements_, vertices_);
}

void VolumetricMesh::getElementsTouchingVertices(const std::vector<int>& vertices_, std::vector<int>& elements_) const {
    internal::mesh_queries::get_elements_touching_vertices(*this, vertices_, elements_);
}

void VolumetricMesh::getElementsWithOnlyVertices(const std::vector<int>& vertices_, std::vector<int>& elements_) const {
    internal::mesh_queries::get_elements_with_only_vertices(*this, vertices_, elements_);
}

void VolumetricMesh::getVertexNeighborhood(const std::vector<int>& vertices_, std::vector<int>& neighborhood) const {
    internal::mesh_queries::get_vertex_neighborhood(*this, vertices_, neighborhood);
}

double VolumetricMesh::getMass() const {
    return internal::mesh_mass_properties::get_mass(*this);
}

void VolumetricMesh::getInertiaParameters(double& mass, Vec3d& centerOfMass, Mat3d& inertiaTensor) const {
    internal::mesh_mass_properties::get_inertia_parameters(*this, mass, centerOfMass, inertiaTensor);
}

void VolumetricMesh::getMeshGeometricParameters(Vec3d& centroid, double* radius) const {
    internal::mesh_mass_properties::get_mesh_geometric_parameters(*this, centroid, radius);
}

Mesh::BoundingBox VolumetricMesh::getBoundingBox() const {
    return internal::mesh_mass_properties::get_bounding_box(*this);
}

int VolumetricMesh::getClosestVertex(Vec3d pos) const {
    return internal::mesh_queries::get_closest_vertex(*this, pos);
}

int VolumetricMesh::getClosestElement(const Vec3d& pos) const {
    return internal::mesh_queries::get_closest_element(*this, pos);
}

int VolumetricMesh::getContainingElement(Vec3d pos) const {
    return internal::mesh_queries::get_containing_element(*this, pos);
}

void VolumetricMesh::setSingleMaterial(double E, double nu, double density) {
    m_material_catalog->set_single_material(getNumElements(), E, nu, density);
}

void VolumetricMesh::propagateRegionsToElements() {
    m_material_catalog->propagate_regions_to_elements();
}

int VolumetricMesh::interpolateGradient(const double* U, int numFields, Vec3d pos, double* grad) const {
    // find the element containing "pos"
    int externalVertex = 0;
    int element        = getContainingElement(pos);
    if (element < 0) {
        element        = getClosestElement(pos);
        externalVertex = 1;
    }

    interpolateGradient(element, U, numFields, pos, grad);

    return externalVertex;
}

void VolumetricMesh::computeGravity(double* gravityForce, double g, bool addForce) const {
    internal::mesh_transforms::compute_gravity(*this, gravityForce, g, addForce);
}

void VolumetricMesh::applyDeformation(const double* u) {
    internal::mesh_transforms::apply_deformation(*this, u);
}

// transforms every vertex as X |--> pos + R * X
void VolumetricMesh::applyLinearTransformation(double* pos, double* R) {
    internal::mesh_transforms::apply_linear_transformation(*this, pos, R);
}

void VolumetricMesh::setMaterial(int i, const Material* material) {
    m_material_catalog->set_material(i, material);
}

VolumetricMesh::VolumetricMesh(const VolumetricMesh& volumetricMesh, std::span<const int> elements_,
                               std::map<int, int>* vertexMap_) {
    // determine vertices in the submesh
    const int num_element_vertices = volumetricMesh.getNumElementVertices();
    std::set<int> vertexSet;
    for (int i = 0; i < static_cast<int>(elements_.size()); i++)
        for (int j = 0; j < num_element_vertices; j++)
            vertexSet.insert(volumetricMesh.getVertexIndex(elements_[i], j));

    // copy vertices into place and also into vertexMap
    std::vector<Vec3d> subset_vertices(vertexSet.size());
    std::set<int>::iterator iter;
    int                     vertexNo = 0;
    std::map<int, int>      vertexMap;
    for (iter = vertexSet.begin(); iter != vertexSet.end(); iter++) {
        subset_vertices[static_cast<size_t>(vertexNo)] = volumetricMesh.getVertex(*iter);
        vertexMap.insert(std::make_pair(*iter, vertexNo));
        vertexNo++;
    }

    if (vertexMap_ != nullptr)
        *vertexMap_ = vertexMap;

    // copy elements
    const int num_elements = static_cast<int>(elements_.size());
    std::vector<int> subset_elements(num_elements * num_element_vertices);
    std::map<int, int> elementMap;
    for (int i = 0; i < num_elements; i++) {
        for (int j = 0; j < num_element_vertices; j++) {
            std::map<int, int>::iterator iter2 = vertexMap.find(volumetricMesh.getVertexIndex(elements_[i], j));
            if (iter2 == vertexMap.end()) {
                printf("Internal error 1.\n");
                throw 1;
            }
            subset_elements[static_cast<size_t>(i * num_element_vertices + j)] = iter2->second;
        }
        elementMap.insert(std::make_pair(elements_[i], i));
    }

    m_geometry = internal::VolumetricMeshData(num_element_vertices, std::move(subset_vertices), std::move(subset_elements));

    // copy materials
    std::vector<std::unique_ptr<Material>> subset_materials(static_cast<size_t>(volumetricMesh.getNumMaterials()));
    for (int i = 0; i < volumetricMesh.getNumMaterials(); i++)
        subset_materials[static_cast<size_t>(i)] = volumetricMesh.getMaterial(i)->clone();

    // copy element sets; restrict element sets to the new mesh, also rename vertices to reflect new vertex indices
    std::vector<Set>   newSets;
    std::map<int, int> oldToNewSetIndex;
    for (int oldSetIndex = 0; oldSetIndex < volumetricMesh.getNumSets(); oldSetIndex++) {
        const Set&    oldSet = volumetricMesh.getSet(oldSetIndex);
        std::set<int> oldElements;
        oldSet.getElements(oldElements);

        for (std::set<int>::iterator iter = oldElements.begin(); iter != oldElements.end(); iter++) {
            if (*iter < 0) {
                printf("Internal error 2.\n");
                exit(1);
            }
        }

        // construct the element list
        std::vector<int> newElements;
        for (std::set<int>::iterator iter = oldElements.begin(); iter != oldElements.end(); iter++) {
            std::map<int, int>::iterator iter2 = elementMap.find(*iter);
            if (iter2 != elementMap.end())
                newElements.push_back(iter2->second);
        }

        // if there is at least one element in the new set, create a set for it
        if (newElements.size() > 0) {
            Set newSet(oldSet.getName());
            for (unsigned int j = 0; j < newElements.size(); j++) {
                if (newElements[j] < 0) {
                    printf("Internal error 3.\n");
                    exit(1);
                }
                newSet.insert(newElements[j]);
            }
            newSets.emplace_back(std::move(newSet));
            oldToNewSetIndex.insert(std::make_pair(oldSetIndex, (int)newSets.size() - 1));
        }
    }

    // copy regions; remove empty ones
    std::vector<Region> vregions;
    for (int i = 0; i < volumetricMesh.getNumRegions(); i++) {
        const Region&                sregion = volumetricMesh.getRegion(i);
        std::map<int, int>::iterator iter    = oldToNewSetIndex.find(sregion.getSetIndex());
        if (iter != oldToNewSetIndex.end()) {
            vregions.emplace_back(sregion.getMaterialIndex(), iter->second);
        }
    }

    m_material_catalog = std::make_unique<internal::MaterialCatalog>(std::move(subset_materials), std::move(newSets),
                                                                     std::move(vregions), num_elements, 0);

    // sanity check
    // seek each element in all the regions
    for (int el = 0; el < num_elements; el++) {
        int found = 0;
        for (int region = 0; region < getNumRegions(); region++) {
            int elementSet = getRegion(region).getSetIndex();

            // seek for element in elementSet
            if (getSet(elementSet).isMember(el)) {
                if (found != 0)
                    printf("Warning: element %d (1-indexed) is in more than one region.\n", el + 1);
                else
                    found = 1;
            }
        }
        if (found == 0)
            printf("Warning: element %d (1-indexed) is not in any of the regions.\n", el + 1);
    }

    // sanity check: make sure all elements are between bounds
    for (int i = 0; i < getNumSets(); i++) {
        std::set<int> elts;
        getSet(i).getElements(elts);
        for (std::set<int>::iterator iter = elts.begin(); iter != elts.end(); iter++) {
            if (*iter < 0)
                printf("Warning: encountered negative element index in element set %d.\n", i);
            if (*iter >= num_elements)
                printf("Warning: encountered too large element index in element set %d.\n", i);
        }
    }
}

void VolumetricMesh::renumberVertices(const std::vector<int>& permutation) {
    internal::mesh_transforms::renumber_vertices(*this, permutation);
}

void VolumetricMesh::addMaterial(const Material* material, const Set& newSet, bool removeEmptySets,
                                 bool removeEmptyMaterials) {
    m_material_catalog->add_material(getNumElements(), material, newSet, removeEmptySets, removeEmptyMaterials);
}

VolumetricMesh::Set VolumetricMesh::generateAllElementsSet(int numElements) {
    Set set(allElementsSetName);
    for (int i = 0; i < numElements; i++)
        set.insert(i);
    return set;
}

}  // namespace VolumetricMeshes
}  // namespace pgo
