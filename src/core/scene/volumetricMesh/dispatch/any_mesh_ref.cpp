#include "dispatch/any_mesh_ref.h"

#include "cubicMesh.h"
#include "tetMesh.h"
#include "volumetricMesh.h"

#include <stdexcept>

namespace pgo::VolumetricMeshes {

AnyMeshRef to_any_mesh_ref(const VolumetricMesh& mesh) {
    switch (mesh.getElementType()) {
    case ElementType::Tet: {
        const auto* tet_mesh = dynamic_cast<const TetMesh*>(&mesh);
        if (tet_mesh == nullptr) {
            throw std::invalid_argument("VolumetricMesh advertised Tet element type but is not a TetMesh.");
        }
        return std::cref(*tet_mesh);
    }
    case ElementType::Cubic: {
        const auto* cubic_mesh = dynamic_cast<const CubicMesh*>(&mesh);
        if (cubic_mesh == nullptr) {
            throw std::invalid_argument("VolumetricMesh advertised Cubic element type but is not a CubicMesh.");
        }
        return std::cref(*cubic_mesh);
    }
    case ElementType::Invalid:
        break;
    }

    throw std::invalid_argument("Unsupported VolumetricMesh element type for AnyMeshRef conversion.");
}

AnyMutableMeshRef to_any_mesh_ref(VolumetricMesh& mesh) {
    switch (mesh.getElementType()) {
    case ElementType::Tet: {
        auto* tet_mesh = dynamic_cast<TetMesh*>(&mesh);
        if (tet_mesh == nullptr) {
            throw std::invalid_argument("VolumetricMesh advertised Tet element type but is not a TetMesh.");
        }
        return std::ref(*tet_mesh);
    }
    case ElementType::Cubic: {
        auto* cubic_mesh = dynamic_cast<CubicMesh*>(&mesh);
        if (cubic_mesh == nullptr) {
            throw std::invalid_argument("VolumetricMesh advertised Cubic element type but is not a CubicMesh.");
        }
        return std::ref(*cubic_mesh);
    }
    case ElementType::Invalid:
        break;
    }

    throw std::invalid_argument("Unsupported VolumetricMesh element type for AnyMutableMeshRef conversion.");
}

}  // namespace pgo::VolumetricMeshes
