/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "simulationMesh.h"

#include "tetMesh.h"
#include "cubicMesh.h"
#include "triMeshGeo.h"
#include "volumetricMeshENuMaterial.h"
#include "pgoLogging.h"
#include "EigenSupport.h"
#include "triMeshNeighbor.h"

#include <vector>
#include <array>
#include <cstring>
#include <map>
#include <memory>
#include <numeric>
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

std::vector<SimulationMesh::Vertex> collectTriMeshVertices(const pgo::Mesh::TriMeshGeo& triMeshGeo) {
    std::vector<SimulationMesh::Vertex> vertices;
    vertices.reserve(triMeshGeo.numVertices());
    for (int vi = 0; vi < triMeshGeo.numVertices(); vi++) {
        vertices.push_back(triMeshGeo.pos(vi));
    }
    return vertices;
}

std::vector<SimulationMesh::ShellElement> buildShellElements(const pgo::Mesh::TriMeshGeo& triMeshGeo) {
    std::vector<SimulationMesh::ShellElement> elements;
    elements.reserve(triMeshGeo.numTriangles());

    pgo::Mesh::TriMeshNeighbor triNeighbor(triMeshGeo);
    for (int i = 0; i < triMeshGeo.numTriangles(); i++) {
        pgo::Vec3i triangle = triMeshGeo.tri(i);
        pgo::Vec3i neighborList = triNeighbor.getTriangleNeighbors(i);

        SimulationMesh::ShellElement element = {triangle[0], triangle[1], triangle[2], -1, -1, -1};
        for (int j = 0; j < 3; j++) {
            int e0 = triangle[j];
            int e1 = triangle[(j + 1) % 3];

            if (neighborList[j] > 0) {
                pgo::Vec3i neighbor = triMeshGeo.tri(neighborList[j]);
                element[j + 3] = pgo::Mesh::getTriangleVertexOppositeEdge(neighbor, e0, e1);
            }
        }

        elements.push_back(element);
    }

    return elements;
}

struct EdgeQuadData {
    std::vector<SimulationMesh::EdgeQuadElement> elements;
    std::vector<std::pair<int, int>> trianglePairs;
};

EdgeQuadData buildEdgeQuadData(const pgo::Mesh::TriMeshGeo& triMeshGeo) {
    using EdgeIndex = std::pair<int, int>;

    std::map<EdgeIndex, std::array<int, 3>> edgeTriangles;

    for (int tri = 0; tri < triMeshGeo.numTriangles(); tri++) {
        for (int ei = 0; ei < 3; ei++) {
            int v0 = triMeshGeo.tri(tri)[ei];
            int v1 = triMeshGeo.tri(tri)[(ei + 1) % 3];

            if (v0 > v1)
                std::swap(v0, v1);

            EdgeIndex eidx(v0, v1);
            auto iter = edgeTriangles.find(eidx);
            if (iter != edgeTriangles.end()) {
                PGO_ALOG(iter->second[2] < 2);
                iter->second[1] = tri;
                iter->second[2]++;
            } else {
                edgeTriangles.emplace(eidx, std::array<int, 3>{tri, -1, 1});
            }
        }
    }

    EdgeQuadData data;
    data.elements.reserve(edgeTriangles.size());
    data.trianglePairs.reserve(edgeTriangles.size());

    for (const auto& pr : edgeTriangles) {
        int v0 = pr.first.first;
        int v1 = pr.first.second;

        if (pr.second[2] < 2)
            continue;

        int vidx = 0;
        for (; vidx < 3; vidx++) {
            if (triMeshGeo.tri(pr.second[0])[vidx] == v0)
                break;
        }
        PGO_ALOG(vidx < 3);

        int vidxNext = (vidx + 1) % 3;
        int v2 = -1, v3 = -1;

        if (triMeshGeo.tri(pr.second[0])[vidxNext] == v1) {
            v2 = triMeshGeo.tri(pr.second[0])[(vidxNext + 1) % 3];
            uint64_t vAll = (uint64_t)triMeshGeo.tri(pr.second[1])[0] ^ (uint64_t)triMeshGeo.tri(pr.second[1])[1] ^
                            (uint64_t)triMeshGeo.tri(pr.second[1])[2];
            v3 = (int)(vAll ^ (uint64_t)v0 ^ (uint64_t)v1);
        } else {
            vidxNext = 0;
            for (; vidxNext < 3; vidxNext++) {
                if (triMeshGeo.tri(pr.second[1])[vidxNext] == v1)
                    break;
            }
            PGO_ALOG(vidxNext < 3);

            v2 = triMeshGeo.tri(pr.second[1])[(vidxNext + 1) % 3];
            uint64_t vAll = (uint64_t)triMeshGeo.tri(pr.second[0])[0] ^ (uint64_t)triMeshGeo.tri(pr.second[0])[1] ^
                            (uint64_t)triMeshGeo.tri(pr.second[0])[2];
            v3 = (int)(vAll ^ (uint64_t)v0 ^ (uint64_t)v1);
        }
        PGO_ALOG(v2 >= 0 && v3 >= 0);

        data.elements.push_back({v2, v0, v1, v3});
        data.trianglePairs.emplace_back(pr.second[0], pr.second[1]);
    }

    return data;
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

    elementUVs_.assign(elements_.size(), std::vector<Vec2d>(getNumElementVertices()));
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

std::unique_ptr<SimulationMesh> SimulationMesh::createFromTetMesh(const VolumetricMeshes::TetMesh& tetMesh) {
    std::vector<Vertex> vertices;
    vertices.reserve(tetMesh.getNumVertices());
    for (int vi = 0; vi < tetMesh.getNumVertices(); vi++) {
        vertices.push_back(tetMesh.getVertex(vi));
    }

    std::vector<TetElement> elements;
    std::vector<const SimulationMeshMaterial*> materials;
    std::vector<std::unique_ptr<SimulationMeshMaterial>> materialOwners;
    std::vector<ElementMaterialBinding> elementMaterialBindings;

    elements.reserve(tetMesh.getNumElements());
    materials.reserve(tetMesh.getNumElements());
    materialOwners.reserve(tetMesh.getNumElements());
    elementMaterialBindings.reserve(tetMesh.getNumElements());

    for (int ei = 0; ei < tetMesh.getNumElements(); ei++) {
        elements.push_back(
            {tetMesh.getVertexIndex(ei, 0), tetMesh.getVertexIndex(ei, 1), tetMesh.getVertexIndex(ei, 2),
             tetMesh.getVertexIndex(ei, 3)});

        const VolumetricMeshes::VolumetricMesh::ENuMaterial* mat = downcastENuMaterial(tetMesh.getElementMaterial(ei));
        materialOwners.push_back(std::make_unique<SimulationMeshENuMaterial>(mat->getE(), mat->getNu()));
        materials.push_back(materialOwners.back().get());
        elementMaterialBindings.push_back({ei, -1});
    }

    return createTet(vertices, elements, elementMaterialBindings, materials);
}

std::unique_ptr<SimulationMesh> SimulationMesh::createFromCubicMesh(const VolumetricMeshes::CubicMesh& cubicMesh) {
    std::vector<Vertex> vertices;
    vertices.reserve(cubicMesh.getNumVertices());
    for (int vi = 0; vi < cubicMesh.getNumVertices(); vi++) {
        vertices.push_back(cubicMesh.getVertex(vi));
    }

    std::vector<CubicElement> elements;
    std::vector<const SimulationMeshMaterial*> materials;
    std::vector<std::unique_ptr<SimulationMeshMaterial>> materialOwners;
    std::vector<ElementMaterialBinding> elementMaterialBindings;

    elements.reserve(cubicMesh.getNumElements());
    materials.reserve(cubicMesh.getNumElements());
    materialOwners.reserve(cubicMesh.getNumElements());
    elementMaterialBindings.reserve(cubicMesh.getNumElements());

    for (int ei = 0; ei < cubicMesh.getNumElements(); ei++) {
        elements.push_back(
            {cubicMesh.getVertexIndex(ei, 0), cubicMesh.getVertexIndex(ei, 1), cubicMesh.getVertexIndex(ei, 2),
             cubicMesh.getVertexIndex(ei, 3), cubicMesh.getVertexIndex(ei, 4), cubicMesh.getVertexIndex(ei, 5),
             cubicMesh.getVertexIndex(ei, 6), cubicMesh.getVertexIndex(ei, 7)});

        const VolumetricMeshes::VolumetricMesh::ENuMaterial* mat =
            downcastENuMaterial(cubicMesh.getElementMaterial(ei));
        materialOwners.push_back(std::make_unique<SimulationMeshENuMaterial>(mat->getE(), mat->getNu()));
        materials.push_back(materialOwners.back().get());
        elementMaterialBindings.push_back({ei, -1});
    }

    return createCubic(vertices, elements, elementMaterialBindings, materials);
}

std::unique_ptr<SimulationMesh> SimulationMesh::createShellFromTriMesh(const Mesh::TriMeshGeo& triMeshGeo,
                                                                       const SimulationMeshMaterial& mat) {
    std::vector<Vertex> vertices = collectTriMeshVertices(triMeshGeo);
    std::vector<ShellElement> elements = buildShellElements(triMeshGeo);
    std::vector<ElementMaterialBinding> bindings(elements.size(), {0, -1});
    const SimulationMeshMaterial* materials[] = {&mat};
    return createShell(vertices, elements, bindings, materials);
}

std::unique_ptr<SimulationMesh> SimulationMesh::createShellFromTriMesh(
    const Mesh::TriMeshGeo& triMeshGeo, std::span<const int> elementMaterialIndices,
    std::span<const SimulationMeshMaterial* const> materials) {
    std::vector<Vertex> vertices = collectTriMeshVertices(triMeshGeo);
    std::vector<ShellElement> elements = buildShellElements(triMeshGeo);
    if (elementMaterialIndices.size() != elements.size()) {
        throw std::invalid_argument("shell material index count must match shell element count");
    }

    std::vector<ElementMaterialBinding> bindings(elements.size());
    for (size_t ei = 0; ei < elements.size(); ei++) {
        bindings[ei] = {elementMaterialIndices[ei], -1};
    }
    return createShell(vertices, elements, bindings, materials);
}

std::unique_ptr<SimulationMesh> SimulationMesh::createTriangleFromTriMesh(const Mesh::TriMeshGeo& triMeshGeo,
                                                                          const SimulationMeshMaterial& mat) {
    std::vector<Vertex> vertices = collectTriMeshVertices(triMeshGeo);
    std::vector<TriangleElement> elements;
    elements.reserve(triMeshGeo.numTriangles());
    for (int i = 0; i < triMeshGeo.numTriangles(); i++) {
        Vec3i tri = triMeshGeo.tri(i);
        elements.push_back({tri[0], tri[1], tri[2]});
    }

    std::vector<ElementMaterialBinding> bindings(elements.size(), {0, -1});
    const SimulationMeshMaterial* materials[] = {&mat};
    return createTriangle(vertices, elements, bindings, materials);
}

std::unique_ptr<SimulationMesh> SimulationMesh::createTriangleFromTriMesh(
    const Mesh::TriMeshGeo& triMeshGeo, std::span<const SimulationMeshMaterial* const> materials,
    std::span<const int> materialIndices) {
    std::vector<Vertex> vertices = collectTriMeshVertices(triMeshGeo);
    std::vector<TriangleElement> elements;
    elements.reserve(triMeshGeo.numTriangles());
    for (int i = 0; i < triMeshGeo.numTriangles(); i++) {
        Vec3i tri = triMeshGeo.tri(i);
        elements.push_back({tri[0], tri[1], tri[2]});
    }
    if (materialIndices.size() != elements.size()) {
        throw std::invalid_argument("triangle material index count must match triangle count");
    }

    std::vector<ElementMaterialBinding> bindings(elements.size());
    for (size_t tri = 0; tri < elements.size(); tri++) {
        bindings[tri] = {materialIndices[tri], -1};
    }
    return createTriangle(vertices, elements, bindings, materials);
}

std::unique_ptr<SimulationMesh> SimulationMesh::createEdgeQuadFromTriMesh(const Mesh::TriMeshGeo& triMeshGeo,
                                                                          const SimulationMeshMaterial& mat) {
    std::vector<Vertex> vertices = collectTriMeshVertices(triMeshGeo);
    EdgeQuadData edgeQuadData = buildEdgeQuadData(triMeshGeo);
    std::vector<ElementMaterialBinding> bindings(edgeQuadData.elements.size(), {0, -1});
    const SimulationMeshMaterial* materials[] = {&mat};
    return createEdgeQuad(vertices, edgeQuadData.elements, bindings, materials);
}

std::unique_ptr<SimulationMesh> SimulationMesh::createEdgeQuadFromTriMesh(
    const Mesh::TriMeshGeo& triMeshGeo, std::span<const SimulationMeshMaterial* const> materials,
    std::span<const int> materialIndices) {
    EdgeQuadData edgeQuadData = buildEdgeQuadData(triMeshGeo);
    if (materialIndices.size() != static_cast<size_t>(triMeshGeo.numTriangles())) {
        throw std::invalid_argument("triangle material index count must match source triangle count");
    }

    std::vector<Vertex> vertices = collectTriMeshVertices(triMeshGeo);
    std::vector<std::unique_ptr<SimulationMeshMaterial>> materialOwners(edgeQuadData.elements.size());
    std::vector<const SimulationMeshMaterial*> edgeMaterials(edgeQuadData.elements.size(), nullptr);
    std::vector<ElementMaterialBinding> bindings(edgeQuadData.elements.size());

    for (size_t edgei = 0; edgei < edgeQuadData.elements.size(); edgei++) {
        int triIdx[2] = {edgeQuadData.trianglePairs[edgei].first, edgeQuadData.trianglePairs[edgei].second};
        double E = 0, nu = 0, h = 0;
        for (int trii = 0; trii < 2; trii++) {
            const SimulationMeshENuhMaterial* m =
                dynamic_cast<const SimulationMeshENuhMaterial*>(materials[materialIndices[triIdx[trii]]]);
            E += m->getE();
            nu += m->getNu();
            h += m->geth();
        }

        materialOwners[edgei] = std::make_unique<SimulationMeshENuhMaterial>(E * 0.5, nu * 0.5, h * 0.5);
        edgeMaterials[edgei] = materialOwners[edgei].get();
        bindings[edgei] = {static_cast<int>(edgei), -1};
    }

    return createEdgeQuad(vertices, edgeQuadData.elements, bindings, edgeMaterials);
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
