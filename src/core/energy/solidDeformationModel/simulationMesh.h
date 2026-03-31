/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "EigenSupport.h"
#include "simulationMeshMaterial.h"

#include <array>
#include <memory>
#include <span>
#include <vector>

namespace pgo {
namespace SolidDeformationModel {
enum class SimulationMeshType {
    TET,
    CUBIC,
    TRIANGLE,
    EDGE_QUAD,
    SHELL,
};

struct ElementMaterialBinding {
    int primary = -1;
    int secondary = -1;
};

class SimulationMesh {
public:
    using Vertex = EigenSupport::V3d;
    using UV = EigenSupport::V2d;
    using TetElement = std::array<int, 4>;
    using CubicElement = std::array<int, 8>;
    using TriangleElement = std::array<int, 3>;
    using EdgeQuadElement = std::array<int, 4>;
    using ShellElement = std::array<int, 6>;

    SimulationMesh(const SimulationMesh&) = delete;
    SimulationMesh& operator=(const SimulationMesh&) = delete;
    ~SimulationMesh();

    static std::unique_ptr<SimulationMesh> createTet(std::span<const Vertex> vertices,
                                                     std::span<const TetElement> elements,
                                                     std::span<const ElementMaterialBinding> elementMaterialBindings,
                                                     std::span<const SimulationMeshMaterial* const> materials);
    static std::unique_ptr<SimulationMesh> createCubic(std::span<const Vertex> vertices,
                                                       std::span<const CubicElement> elements,
                                                       std::span<const ElementMaterialBinding> elementMaterialBindings,
                                                       std::span<const SimulationMeshMaterial* const> materials);
    static std::unique_ptr<SimulationMesh> createTriangle(std::span<const Vertex> vertices,
                                                          std::span<const TriangleElement> elements,
                                                          std::span<const ElementMaterialBinding> elementMaterialBindings,
                                                          std::span<const SimulationMeshMaterial* const> materials);
    static std::unique_ptr<SimulationMesh> createEdgeQuad(std::span<const Vertex> vertices,
                                                          std::span<const EdgeQuadElement> elements,
                                                          std::span<const ElementMaterialBinding> elementMaterialBindings,
                                                          std::span<const SimulationMeshMaterial* const> materials);
    static std::unique_ptr<SimulationMesh> createShell(std::span<const Vertex> vertices,
                                                       std::span<const ShellElement> elements,
                                                       std::span<const ElementMaterialBinding> elementMaterialBindings,
                                                       std::span<const SimulationMeshMaterial* const> materials);

    int        getNumElements() const;
    int        getNumVertices() const;
    int        getNumElementVertices() const;
    int        getVertexIndex(int ele, int j) const;
    const int* getVertexIndices(int ele) const;

    void getVertex(int vi, double pos[3]) const;
    void getVertex(int ele, int j, double pos[3]) const;

    void assignElementUVs(std::span<const UV> uvs);
    bool hasElementUV() const;
    void getElementUV(int ele, int j, double uv[2]) const;

    SimulationMeshType getElementType() const;

    const SimulationMeshMaterial* getPrimaryMaterial(int ele) const;
    SimulationMeshMaterial*       getPrimaryMaterial(int ele);
    const SimulationMeshMaterial* getSecondaryMaterial(int ele) const;
    SimulationMeshMaterial*       getSecondaryMaterial(int ele);
    bool                          hasSecondaryMaterial(int ele) const;

    void setMaterial(int matID, const SimulationMeshMaterial* mat);

protected:
    SimulationMesh() = default;

    template <size_t N>
    static std::unique_ptr<SimulationMesh> createTyped(
        SimulationMeshType meshType, std::span<const Vertex> vertices, std::span<const std::array<int, N>> elements,
        std::span<const ElementMaterialBinding> elementMaterialBindings,
        std::span<const SimulationMeshMaterial* const> materials);

    std::vector<EigenSupport::V3d>                 vertices_;
    std::vector<std::vector<int>>                  elements_;
    std::vector<std::vector<EigenSupport::V2d>>    elementUVs_;
    std::vector<ElementMaterialBinding>            elementMaterialBindings_;
    std::vector<std::unique_ptr<SimulationMeshMaterial>> materials_;
    SimulationMeshType                             meshType_;
};

void computeTriangleUV(SimulationMesh* mesh, double scaleFactor);
}  // namespace SolidDeformationModel
}  // namespace pgo
