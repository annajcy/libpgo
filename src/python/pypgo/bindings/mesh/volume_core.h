#pragma once

#include "volumetricMesh.h"
#include "tetMesh.h"
#include "cubicMesh.h"

#include <memory>

namespace pgo
{

class PyVolumeMesh {
public:
    enum class MeshType { Tet, Cubic };

    PyVolumeMesh(std::unique_ptr<VolumetricMeshes::TetMesh> tetMesh)
        : type_(MeshType::Tet), tetMesh_(std::move(tetMesh)) {}

    PyVolumeMesh(std::unique_ptr<VolumetricMeshes::CubicMesh> cubicMesh)
        : type_(MeshType::Cubic), cubicMesh_(std::move(cubicMesh)) {}

    MeshType meshType() const { return type_; }

    int numVertices() const { return getVM()->getNumVertices(); }
    int numElements() const { return getVM()->getNumElements(); }

    const VolumetricMeshes::VolumetricMesh* getVM() const {
        return type_ == MeshType::Tet
            ? static_cast<const VolumetricMeshes::VolumetricMesh*>(tetMesh_.get())
            : static_cast<const VolumetricMeshes::VolumetricMesh*>(cubicMesh_.get());
    }

private:
    MeshType type_;
    std::unique_ptr<VolumetricMeshes::TetMesh> tetMesh_;
    std::unique_ptr<VolumetricMeshes::CubicMesh> cubicMesh_;
};

}  // namespace pgo
