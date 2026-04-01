#pragma once

#include "materials/material_record.h"

#include <stdexcept>
#include <utility>
#include <variant>

namespace pgo::VolumetricMeshes {

inline MaterialKind material_kind(const MaterialRecord& record) {
    return std::visit(
        [](const auto& material) -> MaterialKind {
            using T = std::decay_t<decltype(material)>;
            if constexpr (std::is_same_v<T, EnuMaterialData>) {
                return MaterialKind::Enu;
            } else if constexpr (std::is_same_v<T, OrthotropicMaterialData>) {
                return MaterialKind::Orthotropic;
            } else {
                return MaterialKind::MooneyRivlin;
            }
        },
        record.data);
}

template <class T>
const T* try_get_material(const MaterialRecord& record) {
    return std::get_if<T>(&record.data);
}

template <class T>
T* try_get_material(MaterialRecord& record) {
    return std::get_if<T>(&record.data);
}

template <class Visitor>
decltype(auto) visit_material(const MaterialRecord& record, Visitor&& visitor) {
    return std::visit(std::forward<Visitor>(visitor), record.data);
}

template <class Visitor>
decltype(auto) visit_material(MaterialRecord& record, Visitor&& visitor) {
    return std::visit(std::forward<Visitor>(visitor), record.data);
}

template <class T>
const T& require_material(const MaterialRecord& record) {
    const T* material = try_get_material<T>(record);
    if (material == nullptr) {
        throw std::invalid_argument("material payload type mismatch");
    }
    return *material;
}

template <class T>
T& require_material(MaterialRecord& record) {
    T* material = try_get_material<T>(record);
    if (material == nullptr) {
        throw std::invalid_argument("material payload type mismatch");
    }
    return *material;
}

inline double material_density(const MaterialRecord& record) {
    return visit_material(record, [](const auto& material) { return material.density; });
}

}  // namespace pgo::VolumetricMeshes
