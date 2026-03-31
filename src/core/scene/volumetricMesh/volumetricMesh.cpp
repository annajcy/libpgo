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
    double vol = 0.0;
    for (int el = 0; el < getNumElements(); el++)
        vol += getElementVolume(el);
    return vol;
}

void VolumetricMesh::getVertexVolumes(double* vertexVolumes) const {
    memset(vertexVolumes, 0, sizeof(double) * getNumVertices());
    double factor = 1.0 / getNumElementVertices();
    for (int el = 0; el < getNumElements(); el++) {
        double volume = getElementVolume(el);
        for (int j = 0; j < getNumElementVertices(); j++)
            vertexVolumes[getVertexIndex(el, j)] += factor * volume;
    }
}

Vec3d VolumetricMesh::getElementCenter(int el) const {
    Vec3d pos(0, 0, 0);
    for (int i = 0; i < getNumElementVertices(); i++)
        pos += getVertex(el, i);

    pos *= 1.0 / getNumElementVertices();

    return pos;
}

void VolumetricMesh::getVerticesInElements(const std::vector<int>& elements_, std::vector<int>& vertices_) const {
    vertices_.clear();
    for (unsigned int i = 0; i < elements_.size(); i++)
        for (int j = 0; j < getNumElementVertices(); j++)
            vertices_.push_back(getVertexIndex(elements_[i], j));

    // sort and deduplicate vertices_
    std::sort(vertices_.begin(), vertices_.end());
    auto newEnd = std::unique(vertices_.begin(), vertices_.end());
    vertices_.resize(std::distance(vertices_.begin(), newEnd));
}

void VolumetricMesh::getElementsTouchingVertices(const std::vector<int>& vertices_, std::vector<int>& elements_) const {
    elements_.clear();
    for (int i = 0; i < getNumElements(); i++) {
        for (int j = 0; j < getNumElementVertices(); j++) {
            // assumes input vector vertices_ is sorted
            if (std::binary_search(vertices_.begin(), vertices_.end(), getVertexIndex(i, j))) {
                elements_.push_back(i);
                break;
            }
        }
    }
}

void VolumetricMesh::getElementsWithOnlyVertices(const std::vector<int>& vertices_, std::vector<int>& elements_) const {
    elements_.clear();
    for (int i = 0; i < getNumElements(); i++) {
        bool withOnlyVertices = true;
        for (int j = 0; j < getNumElementVertices(); j++) {
            // assumes input vector vertices_ is sorted
            if (std::binary_search(vertices_.begin(), vertices_.end(), getVertexIndex(i, j)) == false) {
                withOnlyVertices = false;
                break;
            }
        }
        if (withOnlyVertices)
            elements_.push_back(i);
    }
}

void VolumetricMesh::getVertexNeighborhood(const std::vector<int>& vertices_, std::vector<int>& neighborhood) const {
    std::vector<int> elements_;
    getElementsTouchingVertices(vertices_, elements_);
    getVerticesInElements(elements_, neighborhood);
}

double VolumetricMesh::getMass() const {
    double mass = 0.0;
    for (int i = 0; i < getNumRegions(); i++) {
        const Region& region  = getRegion(i);
        double        density = getMaterial(region.getMaterialIndex())->getDensity();
        std::set<int> setElements;  // elements in the region
        getSet(region.getSetIndex()).getElements(setElements);

        // over all elements in the region
        for (std::set<int>::iterator iter = setElements.begin(); iter != setElements.end(); iter++) {
            int    element       = *iter;
            double elementVolume = getElementVolume(element);
            double elementMass   = elementVolume * density;
            mass += elementMass;
        }
    }

    return mass;
}

void VolumetricMesh::getInertiaParameters(double& mass, Vec3d& centerOfMass, Mat3d& inertiaTensor) const {
    mass = 0.0;
    centerOfMass.setZero();
    inertiaTensor.setZero();

    // compute mass, center of mass, inertia tensor
    for (int i = 0; i < getNumElements(); i++) {
        double density       = getElementDensity(i);
        double elementVolume = getElementVolume(i);
        double elementMass   = elementVolume * density;

        mass += elementMass;
        Vec3d elementCenter = getElementCenter(i);
        centerOfMass += elementMass * elementCenter;

        Mat3d elementITUnitDensity;
        getElementInertiaTensor(i, elementITUnitDensity);

        double a = elementCenter[0];
        double b = elementCenter[1];
        double c = elementCenter[2];

        Mat3d elementITCorrection =
            asMat3d(b * b + c * c, -a * b, -a * c, -a * b, a * a + c * c, -b * c, -a * c, -b * c, a * a + b * b);

        Mat3d elementIT = density * elementITUnitDensity + elementMass * elementITCorrection;

        inertiaTensor += elementIT;
    }

    // printf("final mass: %G\n",mass);
    centerOfMass /= mass;

    // correct inertia tensor so it's around the center of mass
    double a = centerOfMass[0];
    double b = centerOfMass[1];
    double c = centerOfMass[2];

    Mat3d correction =
        asMat3d(b * b + c * c, -a * b, -a * c, -a * b, a * a + c * c, -b * c, -a * c, -b * c, a * a + b * b);

    inertiaTensor -= mass * correction;
}

void VolumetricMesh::getMeshGeometricParameters(Vec3d& centroid, double* radius) const {
    // compute centroid
    centroid = Vec3d(0, 0, 0);
    for (int i = 0; i < getNumVertices(); i++)
        centroid += getVertex(i);

    centroid /= getNumVertices();

    // compute radius
    *radius = 0;
    for (int i = 0; i < getNumVertices(); i++) {
        Vec3d  vertex = getVertex(i);
        double dist   = (vertex - centroid).norm();
        if (dist > *radius)
            *radius = dist;
    }
}

Mesh::BoundingBox VolumetricMesh::getBoundingBox() const {
    const auto vertices = getVertices();
    return Mesh::BoundingBox(std::vector<Vec3d>(vertices.begin(), vertices.end()));
}

int VolumetricMesh::getClosestVertex(Vec3d pos) const {
    // linear scan
    double closestDist   = DBL_MAX;
    int    closestVertex = -1;

    for (int i = 0; i < getNumVertices(); i++) {
        const Vec3d& vertexPosition = getVertex(i);
        double       dist           = (pos - vertexPosition).norm();
        if (dist < closestDist) {
            closestDist   = dist;
            closestVertex = i;
        }
    }

    return closestVertex;
}

int VolumetricMesh::getClosestElement(const Vec3d& pos) const {
    // linear scan
    double closestDist    = DBL_MAX;
    int    closestElement = 0;
    for (int element = 0; element < getNumElements(); element++) {
        Vec3d  center = getElementCenter(element);
        double dist   = (pos - center).norm();
        if (dist < closestDist) {
            closestDist    = dist;
            closestElement = element;
        }
    }

    return closestElement;
}

int VolumetricMesh::getContainingElement(Vec3d pos) const {
    // linear scan
    for (int element = 0; element < getNumElements(); element++) {
        if (containsVertex(element, pos))
            return element;
    }

    return -1;
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
    if (!addForce)
        memset(gravityForce, 0, sizeof(double) * 3 * getNumVertices());

    double invNumElementVertices = 1.0 / getNumElementVertices();

    for (int el = 0; el < getNumElements(); el++) {
        double volume  = getElementVolume(el);
        double density = getElementDensity(el);
        double mass    = density * volume;
        for (int j = 0; j < getNumElementVertices(); j++)
            gravityForce[3 * getVertexIndex(el, j) + 1] -=
                invNumElementVertices * mass * g;  // gravity assumed to act in negative y-direction
    }
}

void VolumetricMesh::applyDeformation(const double* u) {
    for (int i = 0; i < getNumVertices(); i++) {
        Vec3d& v = getVertex(i);
        v[0] += u[3 * i + 0];
        v[1] += u[3 * i + 1];
        v[2] += u[3 * i + 2];
    }
}

// transforms every vertex as X |--> pos + R * X
void VolumetricMesh::applyLinearTransformation(double* pos, double* R) {
    for (int i = 0; i < getNumVertices(); i++) {
        Vec3d& v = getVertex(i);

        double newPos[3];
        for (int j = 0; j < 3; j++) {
            newPos[j] = pos[j];
            for (int k = 0; k < 3; k++)
                newPos[j] += R[3 * j + k] * v[k];
        }

        v[0] = newPos[0];
        v[1] = newPos[1];
        v[2] = newPos[2];
    }
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
    m_geometry.renumber_vertices(permutation);
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
