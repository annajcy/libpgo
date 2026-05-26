#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/shared_ptr.h>
#include <memory>
#include <stdexcept>

#include "cellMeshGeo.h"
#include "volumetricMesh.h"
#include "tetMesh.h"
#include "cubicMesh.h"
#include "volumetricMeshENuMaterial.h"
#include "common.h"

namespace nb = nanobind;
using namespace pgo;

class VolumeMeshCore {
public:
    enum class MeshType { Tet, Cubic };

    VolumeMeshCore(std::unique_ptr<VolumetricMeshes::TetMesh> tetMesh)
        : type_(MeshType::Tet), tetMesh_(std::move(tetMesh)) {}

    VolumeMeshCore(std::unique_ptr<VolumetricMeshes::CubicMesh> cubicMesh)
        : type_(MeshType::Cubic), cubicMesh_(std::move(cubicMesh)) {}

    MeshType meshType() const { return type_; }

    int numVertices() const {
        return getVM()->getNumVertices();
    }

    int numElements() const {
        return getVM()->getNumElements();
    }

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

// --- create from CellMeshGeo + MaterialSpec ---

std::shared_ptr<VolumeMeshCore> create_volume_mesh(const nb::object& cellMeshObj, const MaterialSpecCore& mat) {
    if (nb::isinstance<Mesh::CellMeshGeo<4>>(cellMeshObj)) {
        const auto& cellMesh = nb::cast<const Mesh::CellMeshGeo<4>&>(cellMeshObj);
        auto tetMesh = std::make_unique<VolumetricMeshes::TetMesh>(cellMesh, mat.E(), mat.nu(), mat.density());
        return std::make_shared<VolumeMeshCore>(std::move(tetMesh));
    } else if (nb::isinstance<Mesh::CellMeshGeo<8>>(cellMeshObj)) {
        const auto& cellMesh = nb::cast<const Mesh::CellMeshGeo<8>&>(cellMeshObj);
        auto cubicMesh = std::make_unique<VolumetricMeshes::CubicMesh>(cellMesh, mat.E(), mat.nu(), mat.density());
        return std::make_shared<VolumeMeshCore>(std::move(cubicMesh));
    } else {
        throw std::runtime_error("Unsupported cell mesh type for create_volume_mesh");
    }
}

// --- load directly from .veg file (zero redundant construction) ---

std::shared_ptr<VolumeMeshCore> load_volume_mesh(const std::string& path) {
    auto type = VolumetricMeshes::VolumetricMesh::getElementType(path.c_str());
    if (type == VolumetricMeshes::VolumetricMesh::TET) {
        auto mesh = std::make_unique<VolumetricMeshes::TetMesh>(path.c_str());
        return std::make_shared<VolumeMeshCore>(std::move(mesh));
    } else if (type == VolumetricMeshes::VolumetricMesh::CUBIC) {
        auto mesh = std::make_unique<VolumetricMeshes::CubicMesh>(path.c_str());
        return std::make_shared<VolumeMeshCore>(std::move(mesh));
    } else {
        throw std::runtime_error("Unsupported or invalid volumetric mesh type in file: " + path);
    }
}

// --- save to .veg file ---

void save_volume_mesh(const std::string& path, const VolumeMeshCore& vm) {
    int result = vm.getVM()->saveToAscii(path.c_str());
    if (result != 0) {
        throw std::runtime_error("Failed to save volume mesh to " + path);
    }
}

// --- lazy export: geometry ---

nb::object export_geometry(const VolumeMeshCore& vm) {
    const auto* vMesh = vm.getVM();
    std::vector<Vec3d> vertices;
    std::vector<int> elements;
    vMesh->exportMeshGeometry(vertices, elements);

    if (vm.meshType() == VolumeMeshCore::MeshType::Tet) {
        auto cellMesh = Mesh::CellMeshGeo<4>::fromFlatCells(std::move(vertices), std::move(elements));
        return nb::cast(std::move(cellMesh));
    } else {
        auto cellMesh = Mesh::CellMeshGeo<8>::fromFlatCells(std::move(vertices), std::move(elements));
        return nb::cast(std::move(cellMesh));
    }
}

// --- lazy export: material ---

MaterialSpecCore export_material(const VolumeMeshCore& vm) {
    const auto* vMesh = vm.getVM();
    if (vMesh->getNumMaterials() > 0) {
        auto* enumMat = VolumetricMeshes::downcastENuMaterial(vMesh->getMaterial(0));
        if (enumMat) {
            return MaterialSpecCore(enumMat->getE(), enumMat->getNu(), enumMat->getDensity());
        }
        return MaterialSpecCore(1e9, 0.45, vMesh->getMaterial(0)->getDensity());
    }
    return MaterialSpecCore();
}

void init_mesh_bindings(nb::module_ &m) {
    nb::enum_<VolumeMeshCore::MeshType>(m, "MeshType")
        .value("Tet", VolumeMeshCore::MeshType::Tet)
        .value("Cubic", VolumeMeshCore::MeshType::Cubic);

    nb::class_<VolumeMeshCore>(m, "VolumeMeshCore")
        .def("mesh_type", &VolumeMeshCore::meshType)
        .def("num_vertices", &VolumeMeshCore::numVertices)
        .def("num_elements", &VolumeMeshCore::numElements)
        .def("export_geometry", [](const VolumeMeshCore& self) {
            return export_geometry(self);
        })
        .def("export_material", [](const VolumeMeshCore& self) {
            return export_material(self);
        });

    m.def("create_volume_mesh", &create_volume_mesh);
    m.def("load_volume_mesh", &load_volume_mesh);
    m.def("save_volume_mesh", &save_volume_mesh);
}
