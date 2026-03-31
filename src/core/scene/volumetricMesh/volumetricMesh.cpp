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

void VolumetricMesh::assignMaterialsToElements(int verbose) {
    elementMaterial.assign(numElements, numMaterials);

    propagateRegionsToElements();

    // seek for unassigned elements
    std::set<int> unassignedElements;
    for (int el = 0; el < numElements; el++) {
        if (elementMaterial[el] == numMaterials)
            unassignedElements.insert(el);
    }

    if (unassignedElements.size() > 0) {
        // assign set and region to the unassigned elements

        // create a material if none exists
        if (numMaterials == 0) {
            numMaterials++;
            materials.clear();
            materials.resize(1);
            materials[0] = std::make_unique<ENuMaterial>("defaultMaterial", density_default, E_default, nu_default);
        }

        numSets++;
        sets.resize(numSets, Set("unassignedSet"));
        for (std::set<int>::iterator iter = unassignedElements.begin(); iter != unassignedElements.end(); iter++)
            sets[numSets - 1].insert(*iter);  // elements in sets are 0-indexed

        // create a new region for the unassigned elements
        numRegions++;
        regions.resize(numRegions, Region(numMaterials - 1, numSets - 1));

        for (std::set<int>::iterator iter = unassignedElements.begin(); iter != unassignedElements.end(); iter++)
            elementMaterial[*iter] = numMaterials - 1;

        if (verbose)
            printf(
                "Warning: %d elements were not found in any of the regions. Using default material parameters for "
                "these elements.\n",
                (int)unassignedElements.size());
    }
}


VolumetricMesh::VolumetricMesh(std::span<const Vec3d> vertices_, int numElementVertices_, std::span<const int> elements_,
                               double E, double nu, double density)
    : numElementVertices(numElementVertices_) {
    numElements = static_cast<int>(elements_.size()) / numElementVertices_;
    numVertices = static_cast<int>(vertices_.size());

    numMaterials = 1;
    numSets      = 1;
    numRegions   = 1;

    vertices.assign(vertices_.begin(), vertices_.end());
    elements.assign(elements_.begin(), elements_.end());
    elementMaterial.assign(numElements, 0);
    materials.resize(numMaterials);
    sets.resize(numSets);
    regions.assign(numRegions, Region(0, 0));

    materials[0] = std::make_unique<ENuMaterial>("defaultMaterial", density, E, nu);

    sets[0] = generateAllElementsSet(numElements);
    // we don't need to call propagateRegionsToElements here because elementMaterial has been set
}

VolumetricMesh::VolumetricMesh(std::span<const Vec3d> vertices_, int numElementVertices_, std::span<const int> elements_,
                               std::vector<std::unique_ptr<Material>> materials_, std::vector<Set> sets_,
                               std::vector<Region> regions_)
    : numElementVertices(numElementVertices_) {
    numElements = static_cast<int>(elements_.size()) / numElementVertices_;
    numVertices = static_cast<int>(vertices_.size());

    numMaterials = static_cast<int>(materials_.size());
    numSets      = static_cast<int>(sets_.size());
    numRegions   = static_cast<int>(regions_.size());

    vertices.assign(vertices_.begin(), vertices_.end());
    elements.assign(elements_.begin(), elements_.end());
    elementMaterial.assign(numElements, 0);
    materials    = std::move(materials_);
    sets         = std::move(sets_);
    regions      = std::move(regions_);

    // set elementMaterial:
    propagateRegionsToElements();
}


VolumetricMesh::VolumetricMesh(const VolumetricMesh& volumetricMesh) {
    numVertices        = volumetricMesh.numVertices;
    vertices           = volumetricMesh.vertices;
    numElementVertices = volumetricMesh.numElementVertices;
    numElements        = volumetricMesh.numElements;
    elements           = volumetricMesh.elements;
    numMaterials       = volumetricMesh.numMaterials;
    numSets            = volumetricMesh.numSets;
    numRegions         = volumetricMesh.numRegions;

    materials.resize(numMaterials);
    for (int i = 0; i < numMaterials; i++)
        materials[i] = (volumetricMesh.materials)[i]->clone();

    sets    = volumetricMesh.sets;
    regions = volumetricMesh.regions;

    elementMaterial = volumetricMesh.elementMaterial;
}

double VolumetricMesh::getVolume() const {
    double vol = 0.0;
    for (int el = 0; el < numElements; el++)
        vol += getElementVolume(el);
    return vol;
}

void VolumetricMesh::getVertexVolumes(double* vertexVolumes) const {
    memset(vertexVolumes, 0, sizeof(double) * numVertices);
    double factor = 1.0 / numElementVertices;
    for (int el = 0; el < numElements; el++) {
        double volume = getElementVolume(el);
        for (int j = 0; j < numElementVertices; j++)
            vertexVolumes[getVertexIndex(el, j)] += factor * volume;
    }
}

Vec3d VolumetricMesh::getElementCenter(int el) const {
    Vec3d pos(0, 0, 0);
    for (int i = 0; i < numElementVertices; i++)
        pos += getVertex(el, i);

    pos *= 1.0 / numElementVertices;

    return pos;
}

void VolumetricMesh::getVerticesInElements(const std::vector<int>& elements_, std::vector<int>& vertices_) const {
    vertices_.clear();
    for (unsigned int i = 0; i < elements_.size(); i++)
        for (int j = 0; j < numElementVertices; j++)
            vertices_.push_back(getVertexIndex(elements_[i], j));

    // sort and deduplicate vertices_
    std::sort(vertices_.begin(), vertices_.end());
    auto newEnd = std::unique(vertices_.begin(), vertices_.end());
    vertices_.resize(std::distance(vertices_.begin(), newEnd));
}

void VolumetricMesh::getElementsTouchingVertices(const std::vector<int>& vertices_, std::vector<int>& elements_) const {
    elements_.clear();
    for (int i = 0; i < numElements; i++) {
        for (int j = 0; j < numElementVertices; j++) {
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
    for (int i = 0; i < numElements; i++) {
        bool withOnlyVertices = true;
        for (int j = 0; j < numElementVertices; j++) {
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
    for (int i = 0; i < numElements; i++) {
        double density       = materials[elementMaterial[i]]->getDensity();
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
    for (int i = 0; i < numVertices; i++)
        centroid += getVertex(i);

    centroid /= numVertices;

    // compute radius
    *radius = 0;
    for (int i = 0; i < numVertices; i++) {
        Vec3d  vertex = getVertex(i);
        double dist   = (vertex - centroid).norm();
        if (dist > *radius)
            *radius = dist;
    }
}

Mesh::BoundingBox VolumetricMesh::getBoundingBox() const {
    return Mesh::BoundingBox(vertices);
}

int VolumetricMesh::getClosestVertex(Vec3d pos) const {
    // linear scan
    double closestDist   = DBL_MAX;
    int    closestVertex = -1;

    for (int i = 0; i < numVertices; i++) {
        const Vec3d& vertexPosition = vertices[i];
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
    for (int element = 0; element < numElements; element++) {
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
    for (int element = 0; element < numElements; element++) {
        if (containsVertex(element, pos))
            return element;
    }

    return -1;
}

void VolumetricMesh::setSingleMaterial(double E, double nu, double density) {
    // add a single material
    numMaterials = 1;
    numSets      = 1;
    numRegions   = 1;

    materials.clear();
    materials.resize(numMaterials);
    sets.assign(numSets, generateAllElementsSet(numElements));
    regions.assign(numRegions, Region(0, 0));

    materials[0] = std::make_unique<ENuMaterial>("defaultMaterial", density, E, nu);

    elementMaterial.assign(numElements, 0);
}

void VolumetricMesh::propagateRegionsToElements() {
    for (int regionIndex = 0; regionIndex < numRegions; regionIndex++) {
        const Region& region        = regions[regionIndex];
        int           materialIndex = region.getMaterialIndex();

        const std::set<int>& setElements = sets[region.getSetIndex()].getElements();
        for (const auto& elt : setElements)
            elementMaterial[elt] = materialIndex;
    }
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
        memset(gravityForce, 0, sizeof(double) * 3 * numVertices);

    double invNumElementVertices = 1.0 / getNumElementVertices();

    for (int el = 0; el < numElements; el++) {
        double volume  = getElementVolume(el);
        double density = getElementDensity(el);
        double mass    = density * volume;
        for (int j = 0; j < getNumElementVertices(); j++)
            gravityForce[3 * getVertexIndex(el, j) + 1] -=
                invNumElementVertices * mass * g;  // gravity assumed to act in negative y-direction
    }
}

void VolumetricMesh::applyDeformation(const double* u) {
    for (int i = 0; i < numVertices; i++) {
        Vec3d& v = getVertex(i);
        v[0] += u[3 * i + 0];
        v[1] += u[3 * i + 1];
        v[2] += u[3 * i + 2];
    }
}

// transforms every vertex as X |--> pos + R * X
void VolumetricMesh::applyLinearTransformation(double* pos, double* R) {
    for (int i = 0; i < numVertices; i++) {
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
    materials[i] = material->clone();
}

VolumetricMesh::VolumetricMesh(const VolumetricMesh& volumetricMesh, std::span<const int> elements_,
                               std::map<int, int>* vertexMap_) {
    // determine vertices in the submesh
    numElementVertices = volumetricMesh.getNumElementVertices();
    std::set<int> vertexSet;
    for (int i = 0; i < static_cast<int>(elements_.size()); i++)
        for (int j = 0; j < numElementVertices; j++)
            vertexSet.insert(volumetricMesh.getVertexIndex(elements_[i], j));

    // copy vertices into place and also into vertexMap
    numVertices = vertexSet.size();
    vertices.resize(numVertices);
    std::set<int>::iterator iter;
    int                     vertexNo = 0;
    std::map<int, int>      vertexMap;
    for (iter = vertexSet.begin(); iter != vertexSet.end(); iter++) {
        vertices[vertexNo] = volumetricMesh.getVertex(*iter);
        vertexMap.insert(std::make_pair(*iter, vertexNo));
        vertexNo++;
    }

    if (vertexMap_ != nullptr)
        *vertexMap_ = vertexMap;

    // copy elements
    numElements = static_cast<int>(elements_.size());
    elements.resize(numElements * numElementVertices);
    elementMaterial.resize(numElements);
    std::map<int, int> elementMap;
    for (int i = 0; i < numElements; i++) {
        for (int j = 0; j < numElementVertices; j++) {
            std::map<int, int>::iterator iter2 =
                vertexMap.find(volumetricMesh.elements[elements_[i] * numElementVertices + j]);
            if (iter2 == vertexMap.end()) {
                printf("Internal error 1.\n");
                throw 1;
            }
            elements[i * numElementVertices + j] = iter2->second;
        }

        elementMaterial[i] = (volumetricMesh.elementMaterial)[elements_[i]];
        elementMap.insert(std::make_pair(elements_[i], i));
    }

    // copy materials
    numMaterials = volumetricMesh.getNumMaterials();
    numSets      = volumetricMesh.getNumSets();
    numRegions   = volumetricMesh.getNumRegions();

    materials.resize(numMaterials);
    for (int i = 0; i < numMaterials; i++)
        materials[i] = volumetricMesh.getMaterial(i)->clone();

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

    numSets = newSets.size();
    sets    = std::move(newSets);

    // printf("numSets: %d\n", numSets);

    // copy regions; remove empty ones
    std::vector<Region> vregions;
    for (int i = 0; i < numRegions; i++) {
        const Region&                sregion = volumetricMesh.getRegion(i);
        std::map<int, int>::iterator iter    = oldToNewSetIndex.find(sregion.getSetIndex());
        if (iter != oldToNewSetIndex.end()) {
            vregions.emplace_back(sregion.getMaterialIndex(), iter->second);
        }
    }

    numRegions = vregions.size();
    regions    = std::move(vregions);

    // sanity check
    // seek each element in all the regions
    for (int el = 0; el < numElements; el++) {
        int found = 0;
        for (int region = 0; region < numRegions; region++) {
            int elementSet = regions[region].getSetIndex();

            // seek for element in elementSet
            if (sets[elementSet].isMember(el)) {
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
    for (int i = 0; i < numSets; i++) {
        std::set<int> elts;
        sets[i].getElements(elts);
        for (std::set<int>::iterator iter = elts.begin(); iter != elts.end(); iter++) {
            if (*iter < 0)
                printf("Warning: encountered negative element index in element set %d.\n", i);
            if (*iter >= numElements)
                printf("Warning: encountered too large element index in element set %d.\n", i);
        }
    }
}

void VolumetricMesh::renumberVertices(const std::vector<int>& permutation) {
    // renumber vertices
    std::vector<Vec3d> newVertices(numVertices);
    for (int i = 0; i < numVertices; i++)
        newVertices[permutation[i]] = vertices[i];
    vertices = std::move(newVertices);

    // renumber tets
    for (int& vtxID : elements)
        vtxID = permutation[vtxID];
}

void VolumetricMesh::addMaterial(const Material* material, const Set& newSet, bool removeEmptySets,
                                 bool removeEmptyMaterials) {
    // add new material to materials
    numMaterials++;
    materials.resize(numMaterials);
    materials[numMaterials - 1] = material->clone();

    // remove indices in the sets that belong to the newSet
    const std::set<int>& newElements = newSet.getElements();
    std::vector<bool>    eleCovered(numElements, false);
    for (int i = 1; i < numSets; i++)  // skip the first set, which is always allElements
    {
        std::set<int>& s = sets[i].getElements();

        for (std::set<int>::iterator it = s.begin(); it != s.end();) {
            int ele         = *it;
            eleCovered[ele] = true;
            if (newElements.find(ele) != newElements.end()) {
                std::set<int>::iterator it2 = it;
                it++;
                s.erase(it2);
            } else
                it++;
        }
    }

    // we have to be careful here: if previously the entire mesh is in default set: allElements,
    // adding a new set will invalidate the elements that are previously in allElements
    // a newSet is needed to cover those elements
    std::set<int> restSet;
    for (int i = 0; i < numElements; i++)
        if (eleCovered[i] == false)  // this element is covered only by allElements
            restSet.insert(i);
    if (restSet.size() > 0)
        numSets++;

    // add the new Set
    numSets++;
    sets.resize(numSets);
    sets[numSets - 1] = Set(newSet);
    if (restSet.size() > 0)
        sets[numSets - 2] = Set("restElements", restSet);

    // create a new Region
    numRegions++;
    regions.resize(numRegions);
    regions[numRegions - 1] = Region(numMaterials - 1, numSets - 1);

    if (restSet.size() > 0) {
        // first find the material that used for allElements
        for (int i = 0; i < numRegions - 1; i++)
            if (regions[i].getSetIndex() == 0)  // this is the index of the allElements set
                regions[i].setSetIndex(numSets - 2);
    }

    // modify elementMaterial
    for (std::set<int>::const_iterator it = newElements.begin(); it != newElements.end(); it++) {
        int el = *it;
        PGO_ALOG(el >= 0 && el < numElements);
        elementMaterial[el] = numMaterials - 1;
    }

    if (removeEmptySets) {
        bool             hasEmptySet = false;
        std::vector<int> setIndexChange(numSets, 0);  // old set index -> new set index
        int              newIndex = 0;                // store the next available set index
        for (int i = 0; i < numSets; i++) {
            if (sets[i].getNumElements() == 0) {
                setIndexChange[i] = -1;  // this set will be deleted, so its new set index is -1
                hasEmptySet       = true;
            } else {
                setIndexChange[i] = newIndex;  // this set is remained, its new index is the next available index
                if (newIndex != i)             // this means there's already at least one set deleted
                {
                    PGO_ALOG(newIndex < i);
                    sets[newIndex] = sets[i];  // assign the pointer to the current set to the location at newIndex
                }
                newIndex++;
            }
        }

        if (hasEmptySet) {
            PGO_ALOG(newIndex < numSets);
            numSets = newIndex;
            sets.resize(numSets);

            int newRegionIdx = 0;
            for (int i = 0; i < numRegions; i++) {
                int oldSetIdx = regions[i].getSetIndex();
                PGO_ALOG((size_t)oldSetIdx < setIndexChange.size());
                if (setIndexChange[oldSetIdx] != -1)  // this set has been deleted
                {
                    regions[i].setSetIndex(setIndexChange[oldSetIdx]);
                    if (newRegionIdx != i)
                        regions[newRegionIdx] = regions[i];
                    newRegionIdx++;
                }
            }
            numRegions = newRegionIdx;
            regions.resize(numRegions);
        }  // end if (hasEmptySet)
        else {
            PGO_ALOG(newIndex == numSets);
        }
    }  // end if (removeEmptySets)

    // remove material
    if (removeEmptyMaterials) {
        // count #Elements for each material
        std::vector<int> elementsWithMaterial(numMaterials, 0);

        for (int i = 0; i < numElements; i++) {
            int matIdx = elementMaterial[i];
            PGO_ALOG(matIdx >= 0 && matIdx < numMaterials);
            elementsWithMaterial[matIdx]++;
        }

        int              newMatIdx = 0;
        std::vector<int> matIndexChange(numMaterials, 0);  // old material index -> new material index
        bool             hasEmptyMat = false;
        for (int i = 0; i < numMaterials; i++) {
            if (elementsWithMaterial[i] == 0) {
                matIndexChange[i] = -1;
                materials[i] = nullptr;
                hasEmptyMat  = true;
            } else {
                matIndexChange[i] = newMatIdx;
                if (newMatIdx != i)
                    materials[newMatIdx] = std::move(materials[i]);
                newMatIdx++;
            }
        }

        if (hasEmptyMat) {
            numMaterials = newMatIdx;
            materials.resize(numMaterials);

            // we also need to modify and delete invalid regions
            bool hasInvalidRegion = false;
            int  newRegionIdx     = 0;
            for (int i = 0; i < numRegions; i++) {
                int oldMatIndex = regions[i].getMaterialIndex();
                if (matIndexChange[oldMatIndex] < 0) {
                    hasInvalidRegion = true;
                } else {
                    regions[i].setMaterialIndex(matIndexChange[oldMatIndex]);
                    if (newRegionIdx != i)
                        regions[newRegionIdx] = regions[i];
                    newRegionIdx++;
                }
            }

            if (hasInvalidRegion) {
                numRegions = newRegionIdx;
                regions.resize(numRegions);
            }

            // reassign the correct material index to each element
            propagateRegionsToElements();
        }
    }  // end if (removeEmptyMaterials)
}

VolumetricMesh::Set VolumetricMesh::generateAllElementsSet(int numElements) {
    Set set(allElementsSetName);
    for (int i = 0; i < numElements; i++)
        set.insert(i);
    return set;
}

}  // namespace VolumetricMeshes
}  // namespace pgo
