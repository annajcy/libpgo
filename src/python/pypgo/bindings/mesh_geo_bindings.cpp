#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/pair.h>
#include <stdexcept>

#include "triMeshGeo.h"
#include "cellMeshGeo.h"
#include "volumetricMesh.h"
#include "tetMesh.h"
#include "cubicMesh.h"
#include "volumetricMeshENuMaterial.h"
#include "common.h"

namespace nb = nanobind;
using namespace pgo;


// Creator functions
Mesh::CellMeshGeo<3> create_tricellmesh_geo(const std::vector<double>& vertices, const std::vector<int>& cells) {
    std::vector<Vec3d> pts(vertices.size() / 3);
    for (size_t i = 0; i < pts.size(); ++i) {
        pts[i] = Vec3d(vertices[i * 3 + 0], vertices[i * 3 + 1], vertices[i * 3 + 2]);
    }
    return Mesh::CellMeshGeo<3>::fromFlatCells(std::move(pts), cells);
}

Mesh::CellMeshGeo<4> create_tetcellmesh_geo(const std::vector<double>& vertices, const std::vector<int>& cells) {
    std::vector<Vec3d> pts(vertices.size() / 3);
    for (size_t i = 0; i < pts.size(); ++i) {
        pts[i] = Vec3d(vertices[i * 3 + 0], vertices[i * 3 + 1], vertices[i * 3 + 2]);
    }
    return Mesh::CellMeshGeo<4>::fromFlatCells(std::move(pts), cells);
}

Mesh::CellMeshGeo<8> create_cubiccellmesh_geo(const std::vector<double>& vertices, const std::vector<int>& cells) {
    std::vector<Vec3d> pts(vertices.size() / 3);
    for (size_t i = 0; i < pts.size(); ++i) {
        pts[i] = Vec3d(vertices[i * 3 + 0], vertices[i * 3 + 1], vertices[i * 3 + 2]);
    }
    return Mesh::CellMeshGeo<8>::fromFlatCells(std::move(pts), cells);
}

// IO Adapters
std::pair<nb::object, MaterialSpecCore> read_veg_geo(const std::string& path) {
    auto type = VolumetricMeshes::VolumetricMesh::getElementType(path.c_str());
    if (type == VolumetricMeshes::VolumetricMesh::TET) {
        VolumetricMeshes::TetMesh tetMesh(path.c_str());
        MaterialSpecCore mat;
        if (tetMesh.getNumMaterials() > 0) {
            auto* enumMat = VolumetricMeshes::downcastENuMaterial(tetMesh.getMaterial(0));
            if (enumMat) {
                mat = MaterialSpecCore(enumMat->getE(), enumMat->getNu(), enumMat->getDensity());
            } else {
                mat = MaterialSpecCore(1e9, 0.45, tetMesh.getMaterial(0)->getDensity());
            }
        }
        std::vector<Vec3d> vertices;
        std::vector<Vec4i> tets;
        tetMesh.exportMeshGeometry(vertices, tets);
        const int* flatPtr = reinterpret_cast<const int*>(tets.data());
        std::vector<int> flatCells(flatPtr, flatPtr + tets.size() * 4);
        Mesh::CellMeshGeo<4> cellMesh = Mesh::CellMeshGeo<4>::fromFlatCells(std::move(vertices), std::move(flatCells));
        return { nb::cast(std::move(cellMesh)), mat };
    } else if (type == VolumetricMeshes::VolumetricMesh::CUBIC) {
        VolumetricMeshes::CubicMesh cubicMesh(path.c_str());
        MaterialSpecCore mat;
        if (cubicMesh.getNumMaterials() > 0) {
            auto* enumMat = VolumetricMeshes::downcastENuMaterial(cubicMesh.getMaterial(0));
            if (enumMat) {
                mat = MaterialSpecCore(enumMat->getE(), enumMat->getNu(), enumMat->getDensity());
            } else {
                mat = MaterialSpecCore(1e9, 0.45, cubicMesh.getMaterial(0)->getDensity());
            }
        }
        std::vector<Vec3d> vertices;
        std::vector<int> elements;
        cubicMesh.exportMeshGeometry(vertices, elements);
        Mesh::CellMeshGeo<8> cellMesh = Mesh::CellMeshGeo<8>::fromFlatCells(std::move(vertices), std::move(elements));
        return { nb::cast(std::move(cellMesh)), mat };
    } else {
        throw std::runtime_error("Unsupported or invalid volumetric mesh type in veg file: " + path);
    }
}

void write_veg_geo(const std::string& path, const nb::object& cellMeshObj, const MaterialSpecCore& mat) {
    if (nb::isinstance<Mesh::CellMeshGeo<4>>(cellMeshObj)) {
        const auto& cellMesh = nb::cast<const Mesh::CellMeshGeo<4>&>(cellMeshObj);
        VolumetricMeshes::TetMesh tetMesh(cellMesh, mat.E(), mat.nu(), mat.density());
        if (tetMesh.saveToAscii(path.c_str()) != 0) {
            throw std::runtime_error("Failed to write TetMesh to " + path);
        }
    } else if (nb::isinstance<Mesh::CellMeshGeo<8>>(cellMeshObj)) {
        const auto& cellMesh = nb::cast<const Mesh::CellMeshGeo<8>&>(cellMeshObj);
        VolumetricMeshes::CubicMesh cubicMesh(cellMesh, mat.E(), mat.nu(), mat.density());
        if (cubicMesh.saveToAscii(path.c_str()) != 0) {
            throw std::runtime_error("Failed to write CubicMesh to " + path);
        }
    } else {
        throw std::runtime_error("Unsupported cell mesh type for write_veg_geo");
    }
}

Mesh::CellMeshGeo<3> read_obj_geo(const std::string& path) {
    Mesh::TriMeshGeo mesh;
    if (!mesh.load(path)) {
        throw std::runtime_error("Failed to load TriMeshGeo from " + path);
    }
    return mesh.toCellMesh();
}

void write_obj_geo(const std::string& path, const Mesh::CellMeshGeo<3>& cellMesh) {
    Mesh::TriMeshGeo triMesh(cellMesh);
    if (!triMesh.save(path)) {
        throw std::runtime_error("Failed to save TriMeshGeo to " + path);
    }
}

void init_mesh_geo_bindings(nb::module_ &m) {
    nb::enum_<Mesh::CellMeshType>(m, "CellMeshType")
        .value("Triangle", Mesh::CellMeshType::Triangle)
        .value("Tet", Mesh::CellMeshType::Tet)
        .value("Cubic", Mesh::CellMeshType::Cubic);

    nb::class_<Mesh::CellMeshGeo<3>>(m, "TriCellMeshGeoCore")
        .def("num_vertices", &Mesh::CellMeshGeo<3>::numVertices)
        .def("num_cells", &Mesh::CellMeshGeo<3>::numCells)
        .def("cell_type", &Mesh::CellMeshGeo<3>::cellType)
        .def("vertices", [](const Mesh::CellMeshGeo<3>& self) {
            const auto& pos = self.positions();
            std::vector<double> res(pos.size() * 3);
            for (size_t i = 0; i < pos.size(); ++i) {
                res[i * 3 + 0] = pos[i][0];
                res[i * 3 + 1] = pos[i][1];
                res[i * 3 + 2] = pos[i][2];
            }
            return res;
        })
        .def("cells", [](const Mesh::CellMeshGeo<3>& self) {
            return self.cellsFlat();
        });

    nb::class_<Mesh::CellMeshGeo<4>>(m, "TetCellMeshGeoCore")
        .def("num_vertices", &Mesh::CellMeshGeo<4>::numVertices)
        .def("num_cells", &Mesh::CellMeshGeo<4>::numCells)
        .def("cell_type", &Mesh::CellMeshGeo<4>::cellType)
        .def("vertices", [](const Mesh::CellMeshGeo<4>& self) {
            const auto& pos = self.positions();
            std::vector<double> res(pos.size() * 3);
            for (size_t i = 0; i < pos.size(); ++i) {
                res[i * 3 + 0] = pos[i][0];
                res[i * 3 + 1] = pos[i][1];
                res[i * 3 + 2] = pos[i][2];
            }
            return res;
        })
        .def("cells", [](const Mesh::CellMeshGeo<4>& self) {
            return self.cellsFlat();
        });

    nb::class_<Mesh::CellMeshGeo<8>>(m, "CubicCellMeshGeoCore")
        .def("num_vertices", &Mesh::CellMeshGeo<8>::numVertices)
        .def("num_cells", &Mesh::CellMeshGeo<8>::numCells)
        .def("cell_type", &Mesh::CellMeshGeo<8>::cellType)
        .def("vertices", [](const Mesh::CellMeshGeo<8>& self) {
            const auto& pos = self.positions();
            std::vector<double> res(pos.size() * 3);
            for (size_t i = 0; i < pos.size(); ++i) {
                res[i * 3 + 0] = pos[i][0];
                res[i * 3 + 1] = pos[i][1];
                res[i * 3 + 2] = pos[i][2];
            }
            return res;
        })
        .def("cells", [](const Mesh::CellMeshGeo<8>& self) {
            return self.cellsFlat();
        });

    nb::class_<MaterialSpecCore>(m, "MaterialSpecCore")
        .def(nb::init<double, double, double>(), nb::arg("E") = 1e9, nb::arg("nu") = 0.45, nb::arg("density") = 1000.0)
        .def("E", &MaterialSpecCore::E)
        .def("nu", &MaterialSpecCore::nu)
        .def("density", &MaterialSpecCore::density);

    m.def("create_tricellmesh_geo", &create_tricellmesh_geo);
    m.def("create_tetcellmesh_geo", &create_tetcellmesh_geo);
    m.def("create_cubiccellmesh_geo", &create_cubiccellmesh_geo);
    m.def("read_veg_geo", &read_veg_geo);
    m.def("write_veg_geo", &write_veg_geo);
    m.def("read_obj_geo", &read_obj_geo);
    m.def("write_obj_geo", &write_obj_geo);
}
