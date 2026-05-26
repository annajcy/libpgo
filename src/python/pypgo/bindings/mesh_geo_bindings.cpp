#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <stdexcept>
#include <vector>

#include "common.h"
#include "cubicMesh.h"
#include "cubicMeshGeo.h"
#include "meshData.h"
#include "tetMesh.h"
#include "tetMeshGeo.h"
#include "triMeshGeo.h"
#include "volumetricMesh.h"
#include "volumetricMeshENuMaterial.h"

namespace nb = nanobind;
using namespace pgo;

namespace
{
std::vector<Vec3d> unpackVertices(const std::vector<double> &vertices)
{
  std::vector<Vec3d> pts(vertices.size() / 3);
  for (size_t i = 0; i < pts.size(); ++i) {
    pts[i] = Vec3d(vertices[i * 3 + 0], vertices[i * 3 + 1], vertices[i * 3 + 2]);
  }
  return pts;
}

template<class VecT>
std::vector<int> flattenIndexVector(const std::vector<VecT> &items, int width)
{
  std::vector<int> flat(items.size() * static_cast<size_t>(width));
  for (size_t i = 0; i < items.size(); ++i) {
    for (int j = 0; j < width; ++j) {
      flat[i * static_cast<size_t>(width) + static_cast<size_t>(j)] = items[i][j];
    }
  }
  return flat;
}

std::vector<double> flattenVertices(const std::vector<Vec3d> &positions)
{
  std::vector<double> res(positions.size() * 3);
  for (size_t i = 0; i < positions.size(); ++i) {
    res[i * 3 + 0] = positions[i][0];
    res[i * 3 + 1] = positions[i][1];
    res[i * 3 + 2] = positions[i][2];
  }
  return res;
}

MaterialSpecCore materialFromVolumeMesh(const VolumetricMeshes::VolumetricMesh &mesh)
{
  if (mesh.getNumMaterials() == 0) {
    return MaterialSpecCore();
  }

  auto *enumMat = VolumetricMeshes::downcastENuMaterial(mesh.getMaterial(0));
  if (enumMat) {
    return MaterialSpecCore(enumMat->getE(), enumMat->getNu(), enumMat->getDensity());
  }
  return MaterialSpecCore(1e9, 0.45, mesh.getMaterial(0)->getDensity());
}
}  // namespace

Mesh::MeshData<3> create_tri_mesh_data(const std::vector<double> &vertices, const std::vector<int> &elements)
{
  return Mesh::MeshData<3>::fromFlatElements(unpackVertices(vertices), elements);
}

Mesh::MeshData<4> create_tet_mesh_data(const std::vector<double> &vertices, const std::vector<int> &elements)
{
  return Mesh::MeshData<4>::fromFlatElements(unpackVertices(vertices), elements);
}

Mesh::MeshData<8> create_cubic_mesh_data(const std::vector<double> &vertices, const std::vector<int> &elements)
{
  return Mesh::MeshData<8>::fromFlatElements(unpackVertices(vertices), elements);
}

Mesh::TriMeshGeo create_tri_mesh_geo(const std::vector<double> &vertices, const std::vector<int> &triangles)
{
  return Mesh::TriMeshGeo(static_cast<int>(vertices.size() / 3), vertices.data(),
    static_cast<int>(triangles.size() / 3), triangles.data());
}

Mesh::TetMeshGeo create_tet_mesh_geo(const std::vector<double> &vertices, const std::vector<int> &tets)
{
  return Mesh::TetMeshGeo(static_cast<int>(vertices.size() / 3), vertices.data(),
    static_cast<int>(tets.size() / 4), tets.data());
}

Mesh::CubicMeshGeo create_cubic_mesh_geo(const std::vector<double> &vertices, const std::vector<int> &cubes)
{
  return Mesh::CubicMeshGeo(static_cast<int>(vertices.size() / 3), vertices.data(),
    static_cast<int>(cubes.size() / 8), cubes.data());
}

std::pair<nb::object, MaterialSpecCore> read_veg_geo(const std::string &path)
{
  auto type = VolumetricMeshes::VolumetricMesh::getElementType(path.c_str());
  if (type == VolumetricMeshes::VolumetricMesh::TET) {
    VolumetricMeshes::TetMesh tetMesh(path.c_str());
    std::vector<Vec3d> vertices;
    std::vector<Vec4i> tets;
    tetMesh.exportMeshGeometry(vertices, tets);
    auto data = Mesh::MeshData<4>::fromFlatElements(std::move(vertices), flattenIndexVector(tets, 4));
    return { nb::cast(std::move(data)), materialFromVolumeMesh(tetMesh) };
  }

  if (type == VolumetricMeshes::VolumetricMesh::CUBIC) {
    VolumetricMeshes::CubicMesh cubicMesh(path.c_str());
    std::vector<Vec3d> vertices;
    std::vector<int> elements;
    cubicMesh.exportMeshGeometry(vertices, elements);
    auto data = Mesh::MeshData<8>::fromFlatElements(std::move(vertices), std::move(elements));
    return { nb::cast(std::move(data)), materialFromVolumeMesh(cubicMesh) };
  }

  throw std::runtime_error("Unsupported or invalid volumetric mesh type in veg file: " + path);
}

void write_veg_geo(const std::string &path, const nb::object &meshDataObj, const MaterialSpecCore &mat)
{
  if (nb::isinstance<Mesh::MeshData<4>>(meshDataObj)) {
    const auto &data = nb::cast<const Mesh::MeshData<4> &>(meshDataObj);
    VolumetricMeshes::TetMesh tetMesh(data, mat.E(), mat.nu(), mat.density());
    if (tetMesh.saveToAscii(path.c_str()) != 0) {
      throw std::runtime_error("Failed to write TetMesh to " + path);
    }
    return;
  }

  if (nb::isinstance<Mesh::MeshData<8>>(meshDataObj)) {
    const auto &data = nb::cast<const Mesh::MeshData<8> &>(meshDataObj);
    VolumetricMeshes::CubicMesh cubicMesh(data, mat.E(), mat.nu(), mat.density());
    if (cubicMesh.saveToAscii(path.c_str()) != 0) {
      throw std::runtime_error("Failed to write CubicMesh to " + path);
    }
    return;
  }

  throw std::runtime_error("write_veg_geo expects TetMeshData or CubicMeshData");
}

Mesh::MeshData<3> read_obj_geo(const std::string &path)
{
  Mesh::TriMeshGeo mesh;
  if (!mesh.load(path)) {
    throw std::runtime_error("Failed to load TriMeshGeo from " + path);
  }
  return mesh.toMeshData();
}

void write_obj_geo(const std::string &path, const Mesh::MeshData<3> &data)
{
  Mesh::TriMeshGeo triMesh(data);
  if (!triMesh.save(path)) {
    throw std::runtime_error("Failed to save TriMeshGeo to " + path);
  }
}

void init_mesh_geo_bindings(nb::module_ &m)
{
  nb::enum_<Mesh::MeshDataType>(m, "MeshDataType")
    .value("Triangle", Mesh::MeshDataType::Triangle)
    .value("Tet", Mesh::MeshDataType::Tet)
    .value("Cubic", Mesh::MeshDataType::Cubic);

  nb::class_<Mesh::MeshData<3>>(m, "TriMeshDataCore")
    .def("num_vertices", &Mesh::MeshData<3>::numVertices)
    .def("num_elements", &Mesh::MeshData<3>::numElements)
    .def("mesh_type", &Mesh::MeshData<3>::meshType)
    .def("vertices", [](const Mesh::MeshData<3> &self) { return flattenVertices(self.positions()); })
    .def("elements", [](const Mesh::MeshData<3> &self) { return self.elementsFlat(); })
    .def("element_vtx_id", &Mesh::MeshData<3>::elementVtxID);

  nb::class_<Mesh::MeshData<4>>(m, "TetMeshDataCore")
    .def("num_vertices", &Mesh::MeshData<4>::numVertices)
    .def("num_elements", &Mesh::MeshData<4>::numElements)
    .def("mesh_type", &Mesh::MeshData<4>::meshType)
    .def("vertices", [](const Mesh::MeshData<4> &self) { return flattenVertices(self.positions()); })
    .def("elements", [](const Mesh::MeshData<4> &self) { return self.elementsFlat(); })
    .def("element_vtx_id", &Mesh::MeshData<4>::elementVtxID);

  nb::class_<Mesh::MeshData<8>>(m, "CubicMeshDataCore")
    .def("num_vertices", &Mesh::MeshData<8>::numVertices)
    .def("num_elements", &Mesh::MeshData<8>::numElements)
    .def("mesh_type", &Mesh::MeshData<8>::meshType)
    .def("vertices", [](const Mesh::MeshData<8> &self) { return flattenVertices(self.positions()); })
    .def("elements", [](const Mesh::MeshData<8> &self) { return self.elementsFlat(); })
    .def("element_vtx_id", &Mesh::MeshData<8>::elementVtxID);

  nb::class_<Mesh::TriMeshGeo>(m, "TriMeshGeoCore")
    .def(nb::init<const Mesh::MeshData<3> &>())
    .def("num_vertices", &Mesh::TriMeshGeo::numVertices)
    .def("num_triangles", &Mesh::TriMeshGeo::numTriangles)
    .def("vertices", [](const Mesh::TriMeshGeo &self) { return flattenVertices(self.positions()); })
    .def("triangles", [](const Mesh::TriMeshGeo &self) { return flattenIndexVector(self.triangles(), 3); })
    .def("tri_vtx_id", &Mesh::TriMeshGeo::triVtxID)
    .def("to_mesh_data", &Mesh::TriMeshGeo::toMeshData);

  nb::class_<Mesh::TetMeshGeo>(m, "TetMeshGeoCore")
    .def(nb::init<const Mesh::MeshData<4> &>())
    .def("num_vertices", &Mesh::TetMeshGeo::numVertices)
    .def("num_tets", &Mesh::TetMeshGeo::numTets)
    .def("vertices", [](const Mesh::TetMeshGeo &self) { return flattenVertices(self.positions()); })
    .def("tets", [](const Mesh::TetMeshGeo &self) { return flattenIndexVector(self.tets(), 4); })
    .def("tet_vtx_id", &Mesh::TetMeshGeo::tetVtxID)
    .def("to_mesh_data", &Mesh::TetMeshGeo::toMeshData);

  nb::class_<Mesh::CubicMeshGeo>(m, "CubicMeshGeoCore")
    .def(nb::init<const Mesh::MeshData<8> &>())
    .def("num_vertices", &Mesh::CubicMeshGeo::numVertices)
    .def("num_cubes", &Mesh::CubicMeshGeo::numCubes)
    .def("vertices", [](const Mesh::CubicMeshGeo &self) { return flattenVertices(self.positions()); })
    .def("cubes", [](const Mesh::CubicMeshGeo &self) { return flattenIndexVector(self.cubes(), 8); })
    .def("cube_vtx_id", &Mesh::CubicMeshGeo::cubeVtxID)
    .def("to_mesh_data", &Mesh::CubicMeshGeo::toMeshData);

  nb::class_<MaterialSpecCore>(m, "MaterialSpecCore")
    .def(nb::init<double, double, double>(), nb::arg("E") = 1e9, nb::arg("nu") = 0.45, nb::arg("density") = 1000.0)
    .def("E", &MaterialSpecCore::E)
    .def("nu", &MaterialSpecCore::nu)
    .def("density", &MaterialSpecCore::density);

  m.def("create_tri_mesh_data", &create_tri_mesh_data);
  m.def("create_tet_mesh_data", &create_tet_mesh_data);
  m.def("create_cubic_mesh_data", &create_cubic_mesh_data);
  m.def("create_tri_mesh_geo", &create_tri_mesh_geo);
  m.def("create_tet_mesh_geo", &create_tet_mesh_geo);
  m.def("create_cubic_mesh_geo", &create_cubic_mesh_geo);
  m.def("read_veg_geo", &read_veg_geo);
  m.def("write_veg_geo", &write_veg_geo);
  m.def("read_obj_geo", &read_obj_geo);
  m.def("write_obj_geo", &write_obj_geo);
}
