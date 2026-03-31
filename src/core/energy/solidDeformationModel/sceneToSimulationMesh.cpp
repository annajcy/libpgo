/*
author: GPT-5
*/

#include "sceneToSimulationMesh.h"

#include "tetMesh.h"
#include "triMeshGeo.h"
#include "triMeshNeighbor.h"
#include "volumetricMeshENuMaterial.h"
#include "pgoLogging.h"

#include <array>
#include <map>
#include <memory>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace pgo {
namespace ES = EigenSupport;
}

namespace {

using pgo::SolidDeformationModel::ElementMaterialBinding;
using pgo::SolidDeformationModel::SceneToSimulationMesh;
using pgo::SolidDeformationModel::SimulationMesh;
using pgo::SolidDeformationModel::SimulationMeshENuMaterial;
using pgo::SolidDeformationModel::SimulationMeshENuhMaterial;
using pgo::SolidDeformationModel::SimulationMeshMaterial;

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
        pgo::Vec3i triangle     = triMeshGeo.tri(i);
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

    for (const auto& [edge, triData] : edgeTriangles) {
        if (triData[1] < 0)
            continue;

        pgo::Vec3i tri0 = triMeshGeo.tri(triData[0]);
        pgo::Vec3i tri1 = triMeshGeo.tri(triData[1]);

        data.elements.push_back({edge.first, edge.second, pgo::Mesh::getTriangleVertexOppositeEdge(tri0, edge.first, edge.second),
                                 pgo::Mesh::getTriangleVertexOppositeEdge(tri1, edge.first, edge.second)});
        data.trianglePairs.emplace_back(triData[0], triData[1]);
    }

    return data;
}

struct ENuMaterialParams {
    double E;
    double nu;
};

ENuMaterialParams toENuParams(const pgo::VolumetricMeshes::VolumetricMesh::Material& material) {
    const auto* enu = pgo::VolumetricMeshes::downcastENuMaterial(&material);
    if (enu == nullptr) {
        throw std::invalid_argument("TetMesh to SimulationMesh currently requires ENu material");
    }
    return {enu->getE(), enu->getNu()};
}

}  // namespace

namespace pgo::SolidDeformationModel {

std::unique_ptr<SimulationMesh> SceneToSimulationMesh::fromTetMesh(const VolumetricMeshes::TetMesh& tetMesh) {
    std::vector<SimulationMesh::Vertex> vertices;
    vertices.reserve(tetMesh.getNumVertices());
    for (int vi = 0; vi < tetMesh.getNumVertices(); vi++) {
        vertices.push_back(tetMesh.getVertex(vi));
    }

    std::vector<SimulationMesh::TetElement> elements;
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

        const ENuMaterialParams params = toENuParams(*tetMesh.getElementMaterial(ei));
        materialOwners.push_back(std::make_unique<SimulationMeshENuMaterial>(params.E, params.nu));
        materials.push_back(materialOwners.back().get());
        elementMaterialBindings.push_back({ei, -1});
    }

    return SimulationMesh::createTet(vertices, elements, elementMaterialBindings, materials);
}

std::unique_ptr<SimulationMesh> SceneToSimulationMesh::shellFromTriMesh(const Mesh::TriMeshGeo& triMeshGeo,
                                                                        const SimulationMeshMaterial& mat) {
    std::vector<SimulationMesh::Vertex> vertices = collectTriMeshVertices(triMeshGeo);
    std::vector<SimulationMesh::ShellElement> elements = buildShellElements(triMeshGeo);
    std::vector<ElementMaterialBinding> bindings(elements.size(), {0, -1});
    const SimulationMeshMaterial* materials[] = {&mat};
    return SimulationMesh::createShell(vertices, elements, bindings, materials);
}

std::unique_ptr<SimulationMesh> SceneToSimulationMesh::shellFromTriMesh(
    const Mesh::TriMeshGeo& triMeshGeo, std::span<const int> elementMaterialIndices,
    std::span<const SimulationMeshMaterial* const> materials) {
    std::vector<SimulationMesh::Vertex> vertices = collectTriMeshVertices(triMeshGeo);
    std::vector<SimulationMesh::ShellElement> elements = buildShellElements(triMeshGeo);
    if (elementMaterialIndices.size() != elements.size()) {
        throw std::invalid_argument("shell material index count must match shell element count");
    }

    std::vector<ElementMaterialBinding> bindings(elements.size());
    for (size_t ei = 0; ei < elements.size(); ei++) {
        bindings[ei] = {elementMaterialIndices[ei], -1};
    }
    return SimulationMesh::createShell(vertices, elements, bindings, materials);
}

std::unique_ptr<SimulationMesh> SceneToSimulationMesh::triangleFromTriMesh(const Mesh::TriMeshGeo& triMeshGeo,
                                                                           const SimulationMeshMaterial& mat) {
    std::vector<SimulationMesh::Vertex> vertices = collectTriMeshVertices(triMeshGeo);
    std::vector<SimulationMesh::TriangleElement> elements;
    elements.reserve(triMeshGeo.numTriangles());
    for (int i = 0; i < triMeshGeo.numTriangles(); i++) {
        Vec3i tri = triMeshGeo.tri(i);
        elements.push_back({tri[0], tri[1], tri[2]});
    }

    std::vector<ElementMaterialBinding> bindings(elements.size(), {0, -1});
    const SimulationMeshMaterial* materials[] = {&mat};
    return SimulationMesh::createTriangle(vertices, elements, bindings, materials);
}

std::unique_ptr<SimulationMesh> SceneToSimulationMesh::triangleFromTriMesh(
    const Mesh::TriMeshGeo& triMeshGeo, std::span<const SimulationMeshMaterial* const> materials,
    std::span<const int> materialIndices) {
    std::vector<SimulationMesh::Vertex> vertices = collectTriMeshVertices(triMeshGeo);
    std::vector<SimulationMesh::TriangleElement> elements;
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
    return SimulationMesh::createTriangle(vertices, elements, bindings, materials);
}

std::unique_ptr<SimulationMesh> SceneToSimulationMesh::edgeQuadFromTriMesh(const Mesh::TriMeshGeo& triMeshGeo,
                                                                           const SimulationMeshMaterial& mat) {
    std::vector<SimulationMesh::Vertex> vertices = collectTriMeshVertices(triMeshGeo);
    EdgeQuadData edgeQuadData = buildEdgeQuadData(triMeshGeo);
    std::vector<ElementMaterialBinding> bindings(edgeQuadData.elements.size(), {0, -1});
    const SimulationMeshMaterial* materials[] = {&mat};
    return SimulationMesh::createEdgeQuad(vertices, edgeQuadData.elements, bindings, materials);
}

std::unique_ptr<SimulationMesh> SceneToSimulationMesh::edgeQuadFromTriMesh(
    const Mesh::TriMeshGeo& triMeshGeo, std::span<const SimulationMeshMaterial* const> materials,
    std::span<const int> materialIndices) {
    EdgeQuadData edgeQuadData = buildEdgeQuadData(triMeshGeo);
    if (materialIndices.size() != static_cast<size_t>(triMeshGeo.numTriangles())) {
        throw std::invalid_argument("triangle material index count must match source triangle count");
    }

    std::vector<SimulationMesh::Vertex> vertices = collectTriMeshVertices(triMeshGeo);
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

    return SimulationMesh::createEdgeQuad(vertices, edgeQuadData.elements, bindings, edgeMaterials);
}

}  // namespace pgo::SolidDeformationModel
