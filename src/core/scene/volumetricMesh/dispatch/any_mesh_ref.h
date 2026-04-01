#pragma once

#include <functional>
#include <variant>

namespace pgo::VolumetricMeshes {

class TetMesh;
class CubicMesh;

using AnyMeshRef = std::variant<std::reference_wrapper<const TetMesh>, std::reference_wrapper<const CubicMesh>>;
using AnyMutableMeshRef = std::variant<std::reference_wrapper<TetMesh>, std::reference_wrapper<CubicMesh>>;

inline AnyMeshRef make_any_mesh_ref(const TetMesh& mesh) {
    return std::cref(mesh);
}

inline AnyMeshRef make_any_mesh_ref(const CubicMesh& mesh) {
    return std::cref(mesh);
}

inline AnyMutableMeshRef make_any_mutable_mesh_ref(TetMesh& mesh) {
    return std::ref(mesh);
}

inline AnyMutableMeshRef make_any_mutable_mesh_ref(CubicMesh& mesh) {
    return std::ref(mesh);
}

}  // namespace pgo::VolumetricMeshes
