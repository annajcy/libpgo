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
#include <cstring>
#include <numeric>
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
}  // namespace

SimulationMesh::SimulationMesh(int numVertices, const double* vertexPositions, int numElements, int numElementVertices,
                               const int* elementVertexIndices,
                               const ElementMaterialBinding* elementMaterialBindings, int numMaterials,
                               const SimulationMeshMaterial* const* materials, SimulationMeshType meshType) {
    vertices_.assign(numVertices, ES::V3d::Zero());
    for (int vi = 0; vi < numVertices; vi++) {
        vertices_[vi] = asVec3d(vertexPositions + vi * 3);
    }

    elements_.assign(numElements, std::vector<int>(numElementVertices, 0));
    for (int ei = 0; ei < numElements; ei++) {
        memcpy(elements_[ei].data(), elementVertexIndices + ei * numElementVertices, sizeof(int) * numElementVertices);
    }

    materials_.assign(numMaterials, nullptr);
    for (int mi = 0; mi < numMaterials; mi++) {
        materials_[mi] = materials[mi]->clone();
    }

    elementMaterialBindings_.assign(numElements, ElementMaterialBinding{});
    for (int ei = 0; ei < numElements; ei++) {
        const ElementMaterialBinding& binding = elementMaterialBindings[ei];
        validateMaterialIndex(binding.primary, numMaterials, "primary");
        if (binding.secondary >= 0) {
            validateMaterialIndex(binding.secondary, numMaterials, "secondary");
        }
        elementMaterialBindings_[ei] = binding;
    }

    meshType_ = meshType;
}

SimulationMesh::~SimulationMesh() {
    for (SimulationMeshMaterial* material : materials_) {
        delete material;
    }
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
    return materials_[materialIndex];
}

SimulationMeshMaterial* SimulationMesh::getPrimaryMaterial(int ele) {
    int materialIndex = validateMaterialIndex(elementMaterialBindings_[ele].primary, static_cast<int>(materials_.size()),
                                              "primary");
    return materials_[materialIndex];
}

const SimulationMeshMaterial* SimulationMesh::getSecondaryMaterial(int ele) const {
    if (!hasSecondaryMaterial(ele)) {
        throw std::logic_error("secondary material binding is not available");
    }
    int materialIndex = validateMaterialIndex(elementMaterialBindings_[ele].secondary,
                                              static_cast<int>(materials_.size()), "secondary");
    return materials_[materialIndex];
}

SimulationMeshMaterial* SimulationMesh::getSecondaryMaterial(int ele) {
    if (!hasSecondaryMaterial(ele)) {
        throw std::logic_error("secondary material binding is not available");
    }
    int materialIndex = validateMaterialIndex(elementMaterialBindings_[ele].secondary,
                                              static_cast<int>(materials_.size()), "secondary");
    return materials_[materialIndex];
}

bool SimulationMesh::hasSecondaryMaterial(int ele) const {
    int materialIndex = elementMaterialBindings_[ele].secondary;
    if (materialIndex < 0) {
        return false;
    }
    validateMaterialIndex(materialIndex, static_cast<int>(materials_.size()), "secondary");
    return true;
}

void SimulationMesh::assignElementUVs(const double* uvs) {
    elementUVs_.assign(elements_.size(), std::vector<Vec2d>(getNumElementVertices()));
    for (size_t ei = 0; ei < elements_.size(); ei++) {
        for (int j = 0; j < getNumElementVertices(); j++) {
            const double* uv        = uvs + ei * getNumElementVertices() * 2 + j * 2;
            elementUVs_[ei][j] = Vec2d(uv[0], uv[1]);
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
        delete materials_[matID];
        materials_[matID] = mat->clone();
    } else {
        for (int mi = 0; mi < (int)materials_.size(); mi++) {
            delete materials_[mi];
            materials_[mi] = mat->clone();
        }
    }
}

SimulationMesh* pgo::SolidDeformationModel::loadTetMesh(const VolumetricMeshes::TetMesh* tetMesh) {
    std::vector<double> vtx;
    for (int vi = 0; vi < tetMesh->getNumVertices(); vi++) {
        Vec3d p = tetMesh->getVertex(vi);
        vtx.push_back(p[0]);
        vtx.push_back(p[1]);
        vtx.push_back(p[2]);
    }

    std::vector<int>                    elementVertices;
    std::vector<SimulationMeshMaterial*> materials;
    std::vector<ElementMaterialBinding> elementMaterialBindings;

    for (int ei = 0; ei < tetMesh->getNumElements(); ei++) {
        elementVertices.push_back(tetMesh->getVertexIndex(ei, 0));
        elementVertices.push_back(tetMesh->getVertexIndex(ei, 1));
        elementVertices.push_back(tetMesh->getVertexIndex(ei, 2));
        elementVertices.push_back(tetMesh->getVertexIndex(ei, 3));

        const VolumetricMeshes::VolumetricMesh::ENuMaterial* mat = downcastENuMaterial(tetMesh->getElementMaterial(ei));
        SimulationMeshENuMaterial* mat1 = new SimulationMeshENuMaterial(mat->getE(), mat->getNu());

        materials.push_back(mat1);
        elementMaterialBindings.push_back({ei, -1});
    }

    SimulationMesh* mesh = new SimulationMesh(tetMesh->getNumVertices(), vtx.data(), tetMesh->getNumElements(), 4,
                                              elementVertices.data(), elementMaterialBindings.data(),
                                              tetMesh->getNumElements(), materials.data(), SimulationMeshType::TET);

    for (auto ptr : materials)
        delete ptr;

    return mesh;
}

SimulationMesh* pgo::SolidDeformationModel::loadShellMesh(const Mesh::TriMeshGeo&       triMeshGeo,
                                                          const SimulationMeshMaterial* mat) {
    std::vector<double> vertices;
    for (int i = 0; i < triMeshGeo.numVertices(); i++) {
        vertices.push_back(triMeshGeo.pos(i)[0]);
        vertices.push_back(triMeshGeo.pos(i)[1]);
        vertices.push_back(triMeshGeo.pos(i)[2]);
    }

    std::vector<int>      elementVertexIndices;
    Mesh::TriMeshNeighbor triNeighbor(triMeshGeo);
    for (int i = 0; i < triMeshGeo.numTriangles(); i++) {
        Vec3i triangle     = triMeshGeo.tri(i);
        Vec3i neighborList = triNeighbor.getTriangleNeighbors(i);

        for (int j = 0; j < 3; j++) {
            elementVertexIndices.emplace_back(triangle[j]);
        }

        for (int j = 0; j < 3; j++) {
            int e0 = triangle[j];
            int e1 = triangle[(j + 1) % 3];

            if (neighborList[j] > 0) {
                Vec3i neighbor    = triMeshGeo.tri(neighborList[j]);
                int   neighborIdx = Mesh::getTriangleVertexOppositeEdge(neighbor, e0, e1);
                elementVertexIndices.emplace_back(neighborIdx);
            } else {
                elementVertexIndices.emplace_back(-1);
            }
        }
    }

    int                                 numElements = (int)elementVertexIndices.size() / 6;
    std::vector<ElementMaterialBinding> elementMaterialBindings(numElements, {0, -1});

    SimulationMesh* mesh =
        new SimulationMesh(triMeshGeo.numVertices(), vertices.data(), numElements, 6, elementVertexIndices.data(),
                           elementMaterialBindings.data(), 1, &mat, SimulationMeshType::SHELL);

    return mesh;
}

SimulationMesh* pgo::SolidDeformationModel::loadShellMesh(const Mesh::TriMeshGeo&              triMeshGeo,
                                                          const int*                           elementMaterialIndices_,
                                                          const SimulationMeshMaterial* const* mat) {
    std::vector<double> vertices;
    for (int i = 0; i < triMeshGeo.numVertices(); i++) {
        vertices.push_back(triMeshGeo.pos(i)[0]);
        vertices.push_back(triMeshGeo.pos(i)[1]);
        vertices.push_back(triMeshGeo.pos(i)[2]);
    }

    std::vector<int>      elementVertexIndices;
    Mesh::TriMeshNeighbor triNeighbor(triMeshGeo);
    for (int i = 0; i < triMeshGeo.numTriangles(); i++) {
        Vec3i triangle     = triMeshGeo.tri(i);
        Vec3i neighborList = triNeighbor.getTriangleNeighbors(i);

        for (int j = 0; j < 3; j++) {
            elementVertexIndices.emplace_back(triangle[j]);
        }

        for (int j = 0; j < 3; j++) {
            int e0 = triangle[j];
            int e1 = triangle[(j + 1) % 3];

            if (neighborList[j] > 0) {
                Vec3i neighbor    = triMeshGeo.tri(neighborList[j]);
                int   neighborIdx = Mesh::getTriangleVertexOppositeEdge(neighbor, e0, e1);
                elementVertexIndices.emplace_back(neighborIdx);
            } else {
                elementVertexIndices.emplace_back(-1);
            }
        }
    }

    int                                 numElements = (int)elementVertexIndices.size() / 6;
    std::vector<ElementMaterialBinding> elementMaterialBindings(numElements);
    for (int ei = 0; ei < numElements; ei++) {
        elementMaterialBindings[ei] = {elementMaterialIndices_[ei], -1};
    }

    SimulationMesh* mesh =
        new SimulationMesh(triMeshGeo.numVertices(), vertices.data(), numElements, 6, elementVertexIndices.data(),
                           elementMaterialBindings.data(), numElements, mat, SimulationMeshType::SHELL);

    return mesh;
}

SimulationMesh* pgo::SolidDeformationModel::loadTriMesh(const Mesh::TriMeshGeo&       triMeshGeo,
                                                        const SimulationMeshMaterial* mat, int toTriangle) {
    if (toTriangle == 1) {
        std::vector<double> vertices(triMeshGeo.numVertices() * 3);
        std::vector<int>    triangles(triMeshGeo.numTriangles() * 3);

        for (int i = 0; i < triMeshGeo.numVertices(); i++) {
            (ES::Mp<ES::V3d>(vertices.data() + i * 3)) = triMeshGeo.pos(i);
        }

        for (int i = 0; i < triMeshGeo.numTriangles(); i++) {
            (ES::Mp<ES::V3i>(triangles.data() + i * 3)) = triMeshGeo.tri(i);
        }

        std::vector<ElementMaterialBinding> elementMaterialBindings(triMeshGeo.numTriangles(), {0, -1});
        SimulationMesh*  mesh =
            new SimulationMesh(triMeshGeo.numVertices(), vertices.data(), triMeshGeo.numTriangles(), 3,
                               triangles.data(), elementMaterialBindings.data(), 1, &mat, SimulationMeshType::TRIANGLE);

        return mesh;
    } else {
        using EdgeIndex = std::pair<int, int>;
        std::map<EdgeIndex, std::array<int, 3>> edgeTriangles;

        for (int tri = 0; tri < triMeshGeo.numTriangles(); tri++) {
            for (int ei = 0; ei < 3; ei++) {
                int v0 = triMeshGeo.tri(tri)[ei];
                int v1 = triMeshGeo.tri(tri)[(ei + 1) % 3];

                if (v0 > v1)
                    std::swap(v0, v1);

                EdgeIndex eidx(v0, v1);
                auto      iter = edgeTriangles.find(eidx);
                // if it exist
                if (iter != edgeTriangles.end()) {
                    PGO_ALOG(iter->second[2] < 2);

                    iter->second[1] = tri;
                    iter->second[2]++;
                } else {
                    std::array<int, 3> idx = {tri, -1, 1};
                    edgeTriangles.emplace(eidx, idx);
                }
            }
        }

        std::vector<int> elementVertexIndices;

        for (const auto& pr : edgeTriangles) {
            int v0 = pr.first.first;
            int v1 = pr.first.second;

            // if the edge is not shared by two triangles, we skip
            if (pr.second[2] < 2)
                continue;

            // find v0 in the first triangle
            int vidx = 0;
            for (; vidx < 3; vidx++) {
                if (triMeshGeo.tri(pr.second[0])[vidx] == v0)
                    break;
            }
            PGO_ALOG(vidx < 3);

            int vidxNext = (vidx + 1) % 3;
            int v2 = -1, v3 = -1;

            // v2 belongs to tri0
            if (triMeshGeo.tri(pr.second[0])[vidxNext] == v1) {
                v2 = triMeshGeo.tri(pr.second[0])[(vidxNext + 1) % 3];
                // find the index for v3
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
                // find the index for v3
                uint64_t vAll = (uint64_t)triMeshGeo.tri(pr.second[0])[0] ^ (uint64_t)triMeshGeo.tri(pr.second[0])[1] ^
                                (uint64_t)triMeshGeo.tri(pr.second[0])[2];
                v3 = (int)(vAll ^ (uint64_t)v0 ^ (uint64_t)v1);
            }
            PGO_ALOG(v2 >= 0 && v3 >= 0);

            std::array<int, 4> quad = {v2, v0, v1, v3};

            // LGI << fmt::format("{},{},{},{}->{},{},{}={},{},{}", v0, v1, v2, v3,
            //   surface.tri(pr.second[0])[0], surface.tri(pr.second[0])[1], surface.tri(pr.second[0])[2],
            //   surface.tri(pr.second[1])[0], surface.tri(pr.second[1])[1], surface.tri(pr.second[1])[2]);
            for (int vi = 0; vi < 4; vi++)
                elementVertexIndices.emplace_back(quad[vi]);
        }

        int                                 nEdges = (int)elementVertexIndices.size() / 4;
        std::vector<ElementMaterialBinding> elementMaterialBindings(nEdges, {0, -1});

        std::vector<double> vertices;
        for (int vi = 0; vi < triMeshGeo.numVertices(); vi++) {
            vertices.push_back(triMeshGeo.pos(vi)[0]);
            vertices.push_back(triMeshGeo.pos(vi)[1]);
            vertices.push_back(triMeshGeo.pos(vi)[2]);
        }

        SimulationMesh* mesh =
            new SimulationMesh(triMeshGeo.numVertices(), vertices.data(), nEdges, 4, elementVertexIndices.data(),
                               elementMaterialBindings.data(), 1, &mat, SimulationMeshType::EDGE_QUAD);

        return mesh;
    }
}

SimulationMesh* pgo::SolidDeformationModel::loadTriMesh(const Mesh::TriMeshGeo& triMeshGeo, int numMaterials,
                                                        const SimulationMeshMaterial* const* const mat,
                                                        const int* materialIndices, int toTriangle) {
    if (toTriangle == 1) {
        std::vector<double> vertices(triMeshGeo.numVertices() * 3);
        std::vector<int>    triangles(triMeshGeo.numTriangles() * 3);

        for (int i = 0; i < triMeshGeo.numVertices(); i++) {
            (ES::Mp<ES::V3d>(vertices.data() + i * 3)) = triMeshGeo.pos(i);
        }

        for (int i = 0; i < triMeshGeo.numTriangles(); i++) {
            (ES::Mp<ES::V3i>(triangles.data() + i * 3)) = triMeshGeo.tri(i);
        }

        std::vector<ElementMaterialBinding> elementMaterialBindings(triMeshGeo.numTriangles());
        for (int tri = 0; tri < triMeshGeo.numTriangles(); tri++) {
            elementMaterialBindings[tri] = {materialIndices[tri], -1};
        }
        SimulationMesh*  mesh = new SimulationMesh(triMeshGeo.numVertices(), vertices.data(), triMeshGeo.numTriangles(),
                                                   3, triangles.data(), elementMaterialBindings.data(), numMaterials, mat,
                                                   SimulationMeshType::TRIANGLE);

        return mesh;
    } else {
        using EdgeIndex = std::pair<int, int>;
        std::map<EdgeIndex, std::array<int, 3>> edgeTriangles;

        for (int tri = 0; tri < triMeshGeo.numTriangles(); tri++) {
            for (int ei = 0; ei < 3; ei++) {
                int v0 = triMeshGeo.tri(tri)[ei];
                int v1 = triMeshGeo.tri(tri)[(ei + 1) % 3];

                if (v0 > v1)
                    std::swap(v0, v1);

                EdgeIndex eidx(v0, v1);
                auto      iter = edgeTriangles.find(eidx);
                // if it exist
                if (iter != edgeTriangles.end()) {
                    PGO_ALOG(iter->second[2] < 2);

                    iter->second[1] = tri;
                    iter->second[2]++;
                } else {
                    std::array<int, 3> idx = {tri, -1, 1};
                    edgeTriangles.emplace(eidx, idx);
                }
            }
        }

        std::vector<int>                 elementVertexIndices;
        std::vector<std::pair<int, int>> elementTriangles;

        for (const auto& pr : edgeTriangles) {
            int v0 = pr.first.first;
            int v1 = pr.first.second;

            // if the edge is not shared by two triangles, we skip
            if (pr.second[2] < 2)
                continue;

            // find v0 in the first triangle
            int vidx = 0;
            for (; vidx < 3; vidx++) {
                if (triMeshGeo.tri(pr.second[0])[vidx] == v0)
                    break;
            }
            PGO_ALOG(vidx < 3);

            int vidxNext = (vidx + 1) % 3;
            int v2 = -1, v3 = -1;

            // v2 belongs to tri0
            if (triMeshGeo.tri(pr.second[0])[vidxNext] == v1) {
                v2 = triMeshGeo.tri(pr.second[0])[(vidxNext + 1) % 3];
                // find the index for v3
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
                // find the index for v3
                uint64_t vAll = (uint64_t)triMeshGeo.tri(pr.second[0])[0] ^ (uint64_t)triMeshGeo.tri(pr.second[0])[1] ^
                                (uint64_t)triMeshGeo.tri(pr.second[0])[2];
                v3 = (int)(vAll ^ (uint64_t)v0 ^ (uint64_t)v1);
            }
            PGO_ALOG(v2 >= 0 && v3 >= 0);

            std::array<int, 4> quad = {v2, v0, v1, v3};

            // LGI << fmt::format("{},{},{},{}->{},{},{}={},{},{}", v0, v1, v2, v3,
            //   surface.tri(pr.second[0])[0], surface.tri(pr.second[0])[1], surface.tri(pr.second[0])[2],
            //   surface.tri(pr.second[1])[0], surface.tri(pr.second[1])[1], surface.tri(pr.second[1])[2]);
            for (int vi = 0; vi < 4; vi++)
                elementVertexIndices.emplace_back(quad[vi]);

            elementTriangles.emplace_back(pr.second[0], pr.second[1]);
        }

        int                 nEdges = (int)elementVertexIndices.size() / 4;
        std::vector<double> vertices;
        for (int vi = 0; vi < triMeshGeo.numVertices(); vi++) {
            vertices.push_back(triMeshGeo.pos(vi)[0]);
            vertices.push_back(triMeshGeo.pos(vi)[1]);
            vertices.push_back(triMeshGeo.pos(vi)[2]);
        }

        std::vector<SimulationMeshMaterial*> materials(nEdges, nullptr);
        std::vector<ElementMaterialBinding> elementMaterialBindings(nEdges);

        for (int edgei = 0; edgei < nEdges; edgei++) {
            int    triIdx[2] = {elementTriangles[edgei].first, elementTriangles[edgei].second};
            double E = 0, nu = 0, h = 0;
            for (int trii = 0; trii < 2; trii++) {
                const SimulationMeshENuhMaterial* m =
                    dynamic_cast<const SimulationMeshENuhMaterial*>(mat[materialIndices[triIdx[trii]]]);
                E += m->getE();
                nu += m->getNu();
                h += m->geth();
            }

            E *= 0.5;
            nu *= 0.5;
            h *= 0.5;

            materials[edgei] = new SimulationMeshENuhMaterial(E, nu, h);
            elementMaterialBindings[edgei] = {edgei, -1};
        }

        SimulationMesh* mesh = new SimulationMesh(
            triMeshGeo.numVertices(), vertices.data(), nEdges, 4, elementVertexIndices.data(),
            elementMaterialBindings.data(), (int)materials.size(), materials.data(), SimulationMeshType::EDGE_QUAD);

        for (auto ptr : materials)
            delete ptr;

        return mesh;
    }
}

void pgo::SolidDeformationModel::computeTriangleUV(SimulationMesh* mesh, double scaleFactor) {
    PGO_ALOG(mesh->getElementType() == SimulationMeshType::TRIANGLE);

    std::vector<double> uvs(mesh->getNumElements() * 3 * 2);
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
            uvs[trii * 3 * 2 + j * 2]     = restUV[j][0];
            uvs[trii * 3 * 2 + j * 2 + 1] = restUV[j][1];
        }
    }
    mesh->assignElementUVs(uvs.data());
}
