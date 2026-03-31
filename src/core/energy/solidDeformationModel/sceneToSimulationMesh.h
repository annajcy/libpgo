#pragma once

#include "simulationMesh.h"

namespace pgo::Mesh {
class TriMeshGeo;
}

namespace pgo::VolumetricMeshes {
class TetMesh;
}

namespace pgo::SolidDeformationModel {

class SceneToSimulationMesh {
public:
    static std::unique_ptr<SimulationMesh> fromTetMesh(const VolumetricMeshes::TetMesh& tetMesh);

    static std::unique_ptr<SimulationMesh> triangleFromTriMesh(const Mesh::TriMeshGeo& triMeshGeo,
                                                               const SimulationMeshMaterial& mat);
    static std::unique_ptr<SimulationMesh> triangleFromTriMesh(
        const Mesh::TriMeshGeo& triMeshGeo, std::span<const SimulationMeshMaterial* const> materials,
        std::span<const int> materialIndices);

    static std::unique_ptr<SimulationMesh> edgeQuadFromTriMesh(const Mesh::TriMeshGeo& triMeshGeo,
                                                               const SimulationMeshMaterial& mat);
    static std::unique_ptr<SimulationMesh> edgeQuadFromTriMesh(
        const Mesh::TriMeshGeo& triMeshGeo, std::span<const SimulationMeshMaterial* const> materials,
        std::span<const int> materialIndices);

    static std::unique_ptr<SimulationMesh> shellFromTriMesh(const Mesh::TriMeshGeo& triMeshGeo,
                                                            const SimulationMeshMaterial& mat);
    static std::unique_ptr<SimulationMesh> shellFromTriMesh(
        const Mesh::TriMeshGeo& triMeshGeo, std::span<const int> elementMaterialIndices,
        std::span<const SimulationMeshMaterial* const> materials);
};

}  // namespace pgo::SolidDeformationModel
