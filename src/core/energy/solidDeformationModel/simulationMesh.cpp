/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "simulationMesh.h"

#include "pgoLogging.h"
#include "EigenSupport.h"

#include <vector>
#include <array>
#include <cstring>
#include <memory>
#include <span>
#include <stdexcept>

namespace pgo {
namespace ES = EigenSupport;
}  // namespace pgo

using namespace pgo::SolidDeformationModel;

namespace {
int validateMaterialIndex(int materialIndex, int numMaterials, const char* slotName) {
    if (materialIndex < 0 || materialIndex >= numMaterials) {
        throw std::out_of_range(std::string("invalid ") + slotName + " material index");
    }
    return materialIndex;
}

int expectedElementVertices(SimulationMeshType meshType) {
    switch (meshType) {
    case SimulationMeshType::TET:
        return 4;
    case SimulationMeshType::CUBIC:
        return 8;
    case SimulationMeshType::TRIANGLE:
        return 3;
    case SimulationMeshType::EDGE_QUAD:
        return 4;
    case SimulationMeshType::SHELL:
        return 6;
    }

    throw std::invalid_argument("unknown simulation mesh type");
}

}  // namespace

SimulationMesh::~SimulationMesh() = default;

template <size_t N>
std::unique_ptr<SimulationMesh> SimulationMesh::createTyped(
    SimulationMeshType meshType, std::span<const Vertex> vertices, std::span<const std::array<int, N>> elements,
    std::span<const ElementMaterialBinding> elementMaterialBindings,
    std::span<const SimulationMeshMaterial* const> materials) {
    if (expectedElementVertices(meshType) != static_cast<int>(N)) {
        throw std::invalid_argument("mesh type and typed element arity do not match");
    }
    if (elements.size() != elementMaterialBindings.size()) {
        throw std::invalid_argument("element/material binding size mismatch");
    }

    auto mesh = std::unique_ptr<SimulationMesh>(new SimulationMesh());
    mesh->vertices_.assign(vertices.begin(), vertices.end());

    mesh->elements_.assign(elements.size(), std::vector<int>(N, 0));
    for (size_t ei = 0; ei < elements.size(); ei++) {
        memcpy(mesh->elements_[ei].data(), elements[ei].data(), sizeof(int) * N);
    }

    mesh->materials_.clear();
    mesh->materials_.reserve(materials.size());
    for (size_t mi = 0; mi < materials.size(); mi++) {
        mesh->materials_.push_back(materials[mi]->clone());
    }

    mesh->elementMaterialBindings_.assign(elementMaterialBindings.begin(), elementMaterialBindings.end());
    for (const ElementMaterialBinding& binding : mesh->elementMaterialBindings_) {
        validateMaterialIndex(binding.primary, static_cast<int>(mesh->materials_.size()), "primary");
        if (binding.secondary >= 0) {
            validateMaterialIndex(binding.secondary, static_cast<int>(mesh->materials_.size()), "secondary");
        }
    }

    mesh->meshType_ = meshType;
    return mesh;
}

std::unique_ptr<SimulationMesh> SimulationMesh::createTet(
    std::span<const Vertex> vertices, std::span<const TetElement> elements,
    std::span<const ElementMaterialBinding> elementMaterialBindings,
    std::span<const SimulationMeshMaterial* const> materials) {
    return createTyped<4>(SimulationMeshType::TET, vertices, elements, elementMaterialBindings, materials);
}

std::unique_ptr<SimulationMesh> SimulationMesh::createCubic(
    std::span<const Vertex> vertices, std::span<const CubicElement> elements,
    std::span<const ElementMaterialBinding> elementMaterialBindings,
    std::span<const SimulationMeshMaterial* const> materials) {
    return createTyped<8>(SimulationMeshType::CUBIC, vertices, elements, elementMaterialBindings, materials);
}

std::unique_ptr<SimulationMesh> SimulationMesh::createTriangle(
    std::span<const Vertex> vertices, std::span<const TriangleElement> elements,
    std::span<const ElementMaterialBinding> elementMaterialBindings,
    std::span<const SimulationMeshMaterial* const> materials) {
    return createTyped<3>(SimulationMeshType::TRIANGLE, vertices, elements, elementMaterialBindings, materials);
}

std::unique_ptr<SimulationMesh> SimulationMesh::createEdgeQuad(
    std::span<const Vertex> vertices, std::span<const EdgeQuadElement> elements,
    std::span<const ElementMaterialBinding> elementMaterialBindings,
    std::span<const SimulationMeshMaterial* const> materials) {
    return createTyped<4>(SimulationMeshType::EDGE_QUAD, vertices, elements, elementMaterialBindings, materials);
}

std::unique_ptr<SimulationMesh> SimulationMesh::createShell(
    std::span<const Vertex> vertices, std::span<const ShellElement> elements,
    std::span<const ElementMaterialBinding> elementMaterialBindings,
    std::span<const SimulationMeshMaterial* const> materials) {
    return createTyped<6>(SimulationMeshType::SHELL, vertices, elements, elementMaterialBindings, materials);
}

int SimulationMesh::getNumElements() const {
    return (int)elements_.size();
}

int SimulationMesh::getNumElementVertices() const {
    return (int)elements_[0].size();
}

int SimulationMesh::getNumVertices() const {
    return (int)vertices_.size();
}

int SimulationMesh::getVertexIndex(int ele, int j) const {
    return elements_[ele][j];
}

const int* SimulationMesh::getVertexIndices(int ele) const {
    return elements_[ele].data();
}

void SimulationMesh::getVertex(int ele, int j, double pos[3]) const {
    (ES::Mp<ES::V3d>(pos)) = vertices_[elements_[ele][j]];
}

void SimulationMesh::getVertex(int vi, double pos[3]) const {
    (ES::Mp<ES::V3d>(pos)) = vertices_[vi];
}

SimulationMeshType SimulationMesh::getElementType() const {
    return meshType_;
}

const SimulationMeshMaterial* SimulationMesh::getPrimaryMaterial(int ele) const {
    int materialIndex = validateMaterialIndex(elementMaterialBindings_[ele].primary, static_cast<int>(materials_.size()),
                                              "primary");
    return materials_[materialIndex].get();
}

SimulationMeshMaterial* SimulationMesh::getPrimaryMaterial(int ele) {
    int materialIndex = validateMaterialIndex(elementMaterialBindings_[ele].primary, static_cast<int>(materials_.size()),
                                              "primary");
    return materials_[materialIndex].get();
}

const SimulationMeshMaterial* SimulationMesh::getSecondaryMaterial(int ele) const {
    if (!hasSecondaryMaterial(ele)) {
        throw std::logic_error("secondary material binding is not available");
    }
    int materialIndex = validateMaterialIndex(elementMaterialBindings_[ele].secondary,
                                              static_cast<int>(materials_.size()), "secondary");
    return materials_[materialIndex].get();
}

SimulationMeshMaterial* SimulationMesh::getSecondaryMaterial(int ele) {
    if (!hasSecondaryMaterial(ele)) {
        throw std::logic_error("secondary material binding is not available");
    }
    int materialIndex = validateMaterialIndex(elementMaterialBindings_[ele].secondary,
                                              static_cast<int>(materials_.size()), "secondary");
    return materials_[materialIndex].get();
}

bool SimulationMesh::hasSecondaryMaterial(int ele) const {
    int materialIndex = elementMaterialBindings_[ele].secondary;
    if (materialIndex < 0) {
        return false;
    }
    validateMaterialIndex(materialIndex, static_cast<int>(materials_.size()), "secondary");
    return true;
}

void SimulationMesh::assignElementUVs(std::span<const UV> uvs) {
    const size_t expectedSize = elements_.size() * (elements_.empty() ? 0 : elements_[0].size());
    if (uvs.size() != expectedSize) {
        throw std::invalid_argument("element UV span size mismatch");
    }

    elementUVs_.assign(elements_.size(), std::vector<UV>(getNumElementVertices()));
    for (size_t ei = 0; ei < elements_.size(); ei++) {
        for (int j = 0; j < getNumElementVertices(); j++) {
            elementUVs_[ei][j] = uvs[ei * getNumElementVertices() + j];
        }
    }
}

bool SimulationMesh::hasElementUV() const {
    return elementUVs_.size() != 0;
}

void SimulationMesh::getElementUV(int ele, int j, double uv[2]) const {
    uv[0] = elementUVs_[ele][j][0];
    uv[1] = elementUVs_[ele][j][1];
}

void SimulationMesh::setMaterial(int matID, const SimulationMeshMaterial* mat) {
    if (matID >= 0 && matID < (int)materials_.size()) {
        materials_[matID] = mat->clone();
    } else {
        for (int mi = 0; mi < (int)materials_.size(); mi++) {
            materials_[mi] = mat->clone();
        }
    }
}

void pgo::SolidDeformationModel::computeTriangleUV(SimulationMesh* mesh, double scaleFactor) {
    PGO_ALOG(mesh->getElementType() == SimulationMeshType::TRIANGLE);

    std::vector<SimulationMesh::UV> uvs(mesh->getNumElements() * 3);
    for (int trii = 0; trii < mesh->getNumElements(); trii++) {
        ES::V3d restX[3];
        for (int j = 0; j < 3; j++) {
            mesh->getVertex(trii, j, restX[j].data());
        }

        ES::V2d restUV[3];

        ES::V3d edge0 = restX[1] - restX[0];
        ES::V3d edge1 = restX[2] - restX[0];

        restUV[0] = ES::V2d(0, 0);
        restUV[1] = ES::V2d(edge0.norm(), 0);

        ES::V3d norm0 = edge0.normalized();
        restUV[2](0)  = edge1.dot(norm0);
        restUV[2](1)  = sqrt(edge1.squaredNorm() - restUV[2](0) * restUV[2](0));

        for (int j = 0; j < 3; j++) {
            restUV[j] *= scaleFactor;
        }

        for (int j = 0; j < 3; j++) {
            uvs[trii * 3 + j] = restUV[j];
        }
    }
    mesh->assignElementUVs(uvs);
}
