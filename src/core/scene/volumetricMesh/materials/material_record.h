#pragma once

#include <array>
#include <string>
#include <variant>

namespace pgo::VolumetricMeshes {

enum class MaterialKind { Enu, Orthotropic, MooneyRivlin };

struct EnuMaterialData {
    double density = 0.0;
    double E = 0.0;
    double nu = 0.0;
};

struct OrthotropicMaterialData {
    double density = 0.0;
    double E1 = 0.0;
    double E2 = 0.0;
    double E3 = 0.0;
    double nu12 = 0.0;
    double nu23 = 0.0;
    double nu31 = 0.0;
    double G12 = 0.0;
    double G23 = 0.0;
    double G31 = 0.0;
    std::array<double, 9> rotation = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
};

struct MooneyRivlinMaterialData {
    double density = 0.0;
    double mu01 = 0.0;
    double mu10 = 0.0;
    double v1 = 0.0;
};

using MaterialPayload = std::variant<EnuMaterialData, OrthotropicMaterialData, MooneyRivlinMaterialData>;

struct MaterialRecord {
    std::string     name;
    MaterialPayload data;
};

}  // namespace pgo::VolumetricMeshes
