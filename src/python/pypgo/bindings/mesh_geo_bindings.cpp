#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "boundingVolumeTree.h"
#include "common.h"
#include "createTriMesh.h"
#include "cubicMesh.h"
#include "cubicMeshGeo.h"
#include "meshData.h"
#include "surface_remesh_backend.h"
#include "tetMesherBackend.h"
#include "tetMeshGeo.h"
#include "triangleMeshVoxelizer.h"
#include "triMeshGeo.h"
#include "triMeshNeighbor.h"

namespace nb = nanobind;
using namespace pgo;

namespace
{
class TemporaryObjFile
{
public:
  explicit TemporaryObjFile(const Mesh::MeshData<3> &surfaceData)
  {
    const auto base = std::filesystem::temp_directory_path();
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = base / ("libpgo-pypgo-mesher-" + std::to_string(stamp) + ".obj");

    Mesh::TriMeshGeo surface(surfaceData);
    bool ok = false;
    {
      nb::gil_scoped_release release;
      ok = surface.save(path_.string());
    }
    if (!ok) {
      throw std::runtime_error("Failed to write temporary mesher OBJ to " + path_.string());
    }
  }

  ~TemporaryObjFile()
  {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }

  const std::filesystem::path &path() const { return path_; }

private:
  std::filesystem::path path_;
};

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

template<int K, class VolumeMeshT>
Mesh::MeshData<K> exportVolumeMeshData(const VolumeMeshT &mesh)
{
  std::vector<Vec3d> vertices;
  std::vector<int> elements;
  mesh.exportMeshGeometry(vertices, elements);
  return Mesh::MeshData<K>::fromFlatElements(std::move(vertices), std::move(elements));
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

Mesh::MeshData<3> read_obj(const std::string &path)
{
  Mesh::TriMeshGeo mesh;
  bool ok = false;
  {
    nb::gil_scoped_release release;
    ok = mesh.load(path);
  }
  if (!ok) {
    throw std::runtime_error("Failed to load TriMeshGeo from " + path);
  }
  return mesh.toMeshData();
}

void write_obj(const std::string &path, const Mesh::MeshData<3> &data)
{
  Mesh::TriMeshGeo triMesh(data);
  bool ok = false;
  {
    nb::gil_scoped_release release;
    ok = triMesh.save(path);
  }
  if (!ok) {
    throw std::runtime_error("Failed to save TriMeshGeo to " + path);
  }
}

bool check_self_intersections(const Mesh::MeshData<3> &data)
{
  Mesh::TriMeshGeo mesh(data);
  std::vector<std::pair<int, int>> intersections;
  {
    nb::gil_scoped_release release;
    Mesh::TriMeshBVTree bvTree;
    bvTree.buildByInertiaPartition(mesh.ref());
    bvTree.selfIntersectionExact(mesh.ref(), intersections);
  }
  return intersections.empty() == false;
}

Vec3d vectorToVec3d(const std::vector<double> &values, const std::string &name)
{
  if (values.size() != 3) {
    throw std::runtime_error(name + " must contain exactly 3 values");
  }
  return Vec3d(values[0], values[1], values[2]);
}

Mesh::MeshData<3> create_box_mesh(const std::vector<double> &bmin, const std::vector<double> &bmax)
{
  return Mesh::createBoxMesh(vectorToVec3d(bmin, "bmin"), vectorToVec3d(bmax, "bmax")).toMeshData();
}

Mesh::MeshData<3> create_sphere_mesh(double radius, int axisSubdivisions, int heightSubdivisions)
{
  if (radius <= 0.0 || axisSubdivisions < 2 || heightSubdivisions < 2) {
    throw std::runtime_error("create_sphere_mesh requires radius > 0 and subdivisions >= 2");
  }
  return Mesh::createSphereMesh(radius, axisSubdivisions, heightSubdivisions).toMeshData();
}

Mesh::MeshData<3> create_cylinder_mesh(double radius, double height, int axisSubdivisions, int heightSubdivisions)
{
  if (radius <= 0.0 || height <= 0.0 || axisSubdivisions < 3 || heightSubdivisions < 1) {
    throw std::runtime_error("create_cylinder_mesh requires radius > 0, height > 0, axis subdivisions >= 3, and height subdivisions >= 1");
  }
  return Mesh::createCylinderMesh(radius, height, axisSubdivisions, heightSubdivisions).toMeshData();
}

Mesh::MeshData<3> create_torus_mesh(int radialResolution, int tubularResolution, double radius, double thickness)
{
  if (radialResolution < 3 || tubularResolution < 3 || radius <= 0.0 || thickness <= 0.0) {
    throw std::runtime_error("create_torus_mesh requires resolutions >= 3, radius > 0, and thickness > 0");
  }
  return Mesh::createTorus(radialResolution, tubularResolution, radius, thickness).toMeshData();
}

Mesh::MeshData<8> cubic_mesher_bind(
  const Mesh::MeshData<3> &surfaceData,
  int resolution,
  double E,
  double nu,
  double density)
{
  if (resolution <= 0) {
    throw std::runtime_error("cubic_mesher requires resolution > 0");
  }

  TemporaryObjFile input(surfaceData);
  cubic_mesher::TriangleMeshVoxelizerOptions options;
  options.inputMesh = input.path().string();
  options.resolution = resolution;
  options.E = E;
  options.nu = nu;
  options.density = density;

  std::unique_ptr<VolumetricMeshes::CubicMesh> cubicMesh;
  {
    nb::gil_scoped_release release;
    cubicMesh = cubic_mesher::createTriangleMeshCubicMesh(options);
  }
  return exportVolumeMeshData<8>(*cubicMesh);
}

Mesh::MeshData<4> tet_mesher_bind(
  const Mesh::MeshData<3> &surfaceData,
  const std::string &backend,
  const std::string &tetgenCommand,
  double tetwildLr,
  double tetwildLa,
  bool tetwildHasLa,
  double tetwildEpsr,
  double tetwildStopEnergy,
  int tetwildMaxThreads)
{
  TemporaryObjFile input(surfaceData);

  tet_mesher::CommonOptions common;
  common.inputMesh = input.path().string();
  common.outputMesh = (std::filesystem::temp_directory_path() / "libpgo-pypgo-tetwild-output.veg").string();
  common.quiet = true;

  std::unique_ptr<VolumetricMeshes::TetMesh> tetMesh;
  if (backend == "tetgen") {
    tet_mesher::TetgenOptions options;
    options.common = common;
    options.command = tetgenCommand.empty() ? "pq1.414" : tetgenCommand;
    {
      nb::gil_scoped_release release;
      tetMesh = tet_mesher::generateTetgenMesh(options);
    }
  }
  else if (backend == "tetwild") {
    tet_mesher::TetwildOptions options;
    options.common = common;
    options.lr = tetwildLr;
    options.la = tetwildLa;
    options.hasLa = tetwildHasLa;
    options.epsr = tetwildEpsr;
    options.stopEnergy = tetwildStopEnergy;
    options.maxThreads = tetwildMaxThreads;
    {
      nb::gil_scoped_release release;
      tetMesh = tet_mesher::generateTetwildMesh(options);
    }
  }
  else {
    throw std::runtime_error("unsupported tet mesher backend: " + backend);
  }

  return exportVolumeMeshData<4>(*tetMesh);
}

bool has_tetwild_bind()
{
#ifdef PGO_TET_MESHER_HAS_TET_WILD
  return true;
#else
  return false;
#endif
}

void init_mesh_geo_bindings(nb::module_ &m)
{
  nb::enum_<Mesh::MeshDataType>(m, "MeshDataType")
    .value("Triangle", Mesh::MeshDataType::Triangle)
    .value("Tet", Mesh::MeshDataType::Tet)
    .value("Cubic", Mesh::MeshDataType::Cubic);

  nb::class_<Mesh::MeshData<3>>(m, "PyTriMeshData")
    .def("num_vertices", &Mesh::MeshData<3>::numVertices)
    .def("num_elements", &Mesh::MeshData<3>::numElements)
    .def("mesh_type", &Mesh::MeshData<3>::meshType)
    .def("vertices", [](const Mesh::MeshData<3> &self) { return flattenVertices(self.positions()); })
    .def("elements", [](const Mesh::MeshData<3> &self) { return self.elementsFlat(); })
    .def("element_vtx_id", &Mesh::MeshData<3>::elementVtxID);

  nb::class_<Mesh::MeshData<4>>(m, "PyTetMeshData")
    .def("num_vertices", &Mesh::MeshData<4>::numVertices)
    .def("num_elements", &Mesh::MeshData<4>::numElements)
    .def("mesh_type", &Mesh::MeshData<4>::meshType)
    .def("vertices", [](const Mesh::MeshData<4> &self) { return flattenVertices(self.positions()); })
    .def("elements", [](const Mesh::MeshData<4> &self) { return self.elementsFlat(); })
    .def("element_vtx_id", &Mesh::MeshData<4>::elementVtxID);

  nb::class_<Mesh::MeshData<8>>(m, "PyCubicMeshData")
    .def("num_vertices", &Mesh::MeshData<8>::numVertices)
    .def("num_elements", &Mesh::MeshData<8>::numElements)
    .def("mesh_type", &Mesh::MeshData<8>::meshType)
    .def("vertices", [](const Mesh::MeshData<8> &self) { return flattenVertices(self.positions()); })
    .def("elements", [](const Mesh::MeshData<8> &self) { return self.elementsFlat(); })
    .def("element_vtx_id", &Mesh::MeshData<8>::elementVtxID);

  nb::class_<Mesh::TriMeshGeo>(m, "PyTriMeshGeo")
    .def(nb::init<const Mesh::MeshData<3> &>())
    .def("num_vertices", &Mesh::TriMeshGeo::numVertices)
    .def("num_triangles", &Mesh::TriMeshGeo::numTriangles)
    .def("vertices", [](const Mesh::TriMeshGeo &self) { return flattenVertices(self.positions()); })
    .def("triangles", [](const Mesh::TriMeshGeo &self) { return flattenIndexVector(self.triangles(), 3); })
    .def("tri_vtx_id", &Mesh::TriMeshGeo::triVtxID)
    .def("to_mesh_data", &Mesh::TriMeshGeo::toMeshData);

  nb::class_<Mesh::TetMeshGeo>(m, "PyTetMeshGeo")
    .def(nb::init<const Mesh::MeshData<4> &>())
    .def("num_vertices", &Mesh::TetMeshGeo::numVertices)
    .def("num_tets", &Mesh::TetMeshGeo::numTets)
    .def("vertices", [](const Mesh::TetMeshGeo &self) { return flattenVertices(self.positions()); })
    .def("tets", [](const Mesh::TetMeshGeo &self) { return flattenIndexVector(self.tets(), 4); })
    .def("tet_vtx_id", &Mesh::TetMeshGeo::tetVtxID)
    .def("to_mesh_data", &Mesh::TetMeshGeo::toMeshData);

  nb::class_<Mesh::CubicMeshGeo>(m, "PyCubicMeshGeo")
    .def(nb::init<const Mesh::MeshData<8> &>())
    .def("num_vertices", &Mesh::CubicMeshGeo::numVertices)
    .def("num_cubes", &Mesh::CubicMeshGeo::numCubes)
    .def("vertices", [](const Mesh::CubicMeshGeo &self) { return flattenVertices(self.positions()); })
    .def("cubes", [](const Mesh::CubicMeshGeo &self) { return flattenIndexVector(self.cubes(), 8); })
    .def("cube_vtx_id", &Mesh::CubicMeshGeo::cubeVtxID)
    .def("to_mesh_data", &Mesh::CubicMeshGeo::toMeshData);

  nb::class_<PyMaterialSpec>(m, "PyMaterialSpec")
    .def(nb::init<double, double, double>(), nb::arg("E") = 1e9, nb::arg("nu") = 0.45, nb::arg("density") = 1000.0)
    .def("E", &PyMaterialSpec::E)
    .def("nu", &PyMaterialSpec::nu)
    .def("density", &PyMaterialSpec::density);

  m.def("create_tri_mesh_data", &create_tri_mesh_data);
  m.def("create_tet_mesh_data", &create_tet_mesh_data);
  m.def("create_cubic_mesh_data", &create_cubic_mesh_data);
  m.def("create_tri_mesh_geo", &create_tri_mesh_geo);
  m.def("create_tet_mesh_geo", &create_tet_mesh_geo);
  m.def("create_cubic_mesh_geo", &create_cubic_mesh_geo);
  m.def("read_obj", &read_obj);
  m.def("write_obj", &write_obj);
  m.def("check_self_intersections", &check_self_intersections);
  m.def("create_box_mesh", &create_box_mesh, nb::arg("bmin"), nb::arg("bmax"));
  m.def("create_sphere_mesh", &create_sphere_mesh,
    nb::arg("radius"), nb::arg("axis_subdiv"), nb::arg("height_subdiv"));
  m.def("create_cylinder_mesh", &create_cylinder_mesh,
    nb::arg("radius"), nb::arg("height"), nb::arg("axis_subdiv"), nb::arg("height_subdiv"));
  m.def("create_torus_mesh", &create_torus_mesh,
    nb::arg("radial_res"), nb::arg("tubular_res"), nb::arg("radius"), nb::arg("thickness"));
  m.def("cubic_mesher", &cubic_mesher_bind,
    nb::arg("surface_data"), nb::arg("resolution"), nb::arg("E") = 1e6,
    nb::arg("nu") = 0.45, nb::arg("density") = 1000.0);
  m.def("tet_mesher", &tet_mesher_bind,
    nb::arg("surface_data"), nb::arg("backend"), nb::arg("tetgen_command") = "pq1.414",
    nb::arg("tetwild_lr") = 0.05, nb::arg("tetwild_la") = 0.0,
    nb::arg("tetwild_has_la") = false, nb::arg("tetwild_epsr") = 0.001,
    nb::arg("tetwild_stop_energy") = 10.0, nb::arg("tetwild_max_threads") = 0);
  m.def("has_tetwild", &has_tetwild_bind);

  // Connected component queries
  m.def("triangle_component_ids",
    [](const Mesh::MeshData<3> &surface) {
      Mesh::TriMeshGeo mesh(surface);
      std::vector<int> counts;
      nb::gil_scoped_release release;
      std::vector<int> ids = Mesh::computeTriangleEdgeComponentIDs(
        BasicAlgorithms::makeArrayRef(mesh.triangles()), &counts);
      return std::make_pair(ids, counts);
    },
    nb::arg("surface"));

  m.def("connected_components_by_edge",
    [](const Mesh::MeshData<3> &surface) {
      Mesh::TriMeshGeo mesh(surface);
      nb::gil_scoped_release release;
      return Mesh::getConnectedComponentsByEdge(BasicAlgorithms::makeArrayRef(mesh.triangles()));
    },
    nb::arg("surface"));

  m.def("connected_components_by_vertex",
    [](const Mesh::MeshData<3> &surface) {
      Mesh::TriMeshGeo mesh(surface);
      nb::gil_scoped_release release;
      return Mesh::getConnectedComponentsByVertex(BasicAlgorithms::makeArrayRef(mesh.triangles()));
    },
    nb::arg("surface"));

  m.def("filter_small_components",
    [](const Mesh::MeshData<3> &surface, int min_triangles, int keep_largest) {
      Mesh::TriMeshGeo mesh(surface);
      nb::gil_scoped_release release;
      return Mesh::filterSmallTriangleComponentsByEdge(mesh.ref(), min_triangles, keep_largest).toMeshData();
    },
    nb::arg("surface"), nb::arg("min_triangles"), nb::arg("keep_largest") = -1);

  m.def("get_outer_component",
    [](const Mesh::MeshData<3> &surface) {
      Mesh::TriMeshGeo mesh(surface);
      nb::gil_scoped_release release;
      Mesh::TriMeshRef ref = mesh.ref();
      std::vector<int> outer_ids = Mesh::getOneOuterTriMeshConnectedComponentByVertex(ref);
      // build complement: triangles NOT in outer_ids
      std::set<int> outer_set(outer_ids.begin(), outer_ids.end());
      std::vector<int> to_remove;
      to_remove.reserve(mesh.numTriangles() - static_cast<int>(outer_ids.size()));
      for (int i = 0; i < mesh.numTriangles(); ++i) {
        if (outer_set.find(i) == outer_set.end())
          to_remove.push_back(i);
      }
      return Mesh::removeIsolatedVertices(Mesh::removeTriangles(mesh.ref(), to_remove).ref()).toMeshData();
    },
    nb::arg("surface"));

  // Surface remeshing
  m.def("surface_remove_isolated_vertices",
    [](const Mesh::MeshData<3> &surface) {
      Mesh::TriMeshGeo mesh(surface);
      nb::gil_scoped_release release;
      return Mesh::removeIsolatedVertices(mesh.ref()).toMeshData();
    },
    nb::arg("surface"));

  m.def("cgal_smooth_surface",
    [](const Mesh::MeshData<3> &surface, int num_iter, double sharp_angle) {
      Mesh::TriMeshGeo mesh(surface);
      nb::gil_scoped_release release;
      return surface_remesh::cgal_smooth(mesh, num_iter, sharp_angle).toMeshData();
    },
    nb::arg("surface"), nb::arg("num_iter") = 10, nb::arg("sharp_angle") = 180.0);

  m.def("cgal_isotropic_remesh",
    [](const Mesh::MeshData<3> &surface, double target_edge_length, int num_iter, double sharp_angle) {
      Mesh::TriMeshGeo mesh(surface);
      nb::gil_scoped_release release;
      return surface_remesh::cgal_isotropic_remesh(mesh, target_edge_length, num_iter, sharp_angle).toMeshData();
    },
    nb::arg("surface"), nb::arg("target_edge_length"), nb::arg("num_iter") = 10, nb::arg("sharp_angle") = 180.0);

  m.def("cgal_repair_self_intersections",
    [](const Mesh::MeshData<3> &surface, const std::string &method) {
      Mesh::TriMeshGeo mesh(surface);
      nb::gil_scoped_release release;
      auto [result, all_fixed] = surface_remesh::cgal_repair_self_intersections(mesh, method);
      return std::make_pair(result.toMeshData(), all_fixed);
    },
    nb::arg("surface"), nb::arg("method") = std::string("autorefine"));

  m.def("cgal_simplify_surface",
    [](const Mesh::MeshData<3> &surface, double target_ratio) {
      Mesh::TriMeshGeo mesh(surface);
      nb::gil_scoped_release release;
      return surface_remesh::cgal_simplify(mesh, target_ratio).toMeshData();
    },
    nb::arg("surface"), nb::arg("target_ratio"));

  m.def("has_cgal_remesher", &surface_remesh::has_cgal);

  m.def("geogram_remesh_surface",
    [](const Mesh::MeshData<3> &surface, int target_num_vertices, double size_factor,
       double anisotropy, int num_threads) {
      Mesh::TriMeshGeo mesh(surface);
      nb::gil_scoped_release release;
      return surface_remesh::geogram_remesh(mesh, target_num_vertices, size_factor,
                                             anisotropy, num_threads).toMeshData();
    },
    nb::arg("surface"), nb::arg("target_num_vertices"),
    nb::arg("size_factor") = 1.0, nb::arg("anisotropy") = 1.0, nb::arg("num_threads") = 1);

  m.def("has_geogram_remesher", &surface_remesh::has_geogram);
}
