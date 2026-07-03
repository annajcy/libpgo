#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "core.h"
#include "common.h"
#include "meshData.h"
#include "tetMeshGeo.h"
#include "triMeshGeo.h"
#include "cubicMeshGeo.h"

namespace nb = nanobind;
using namespace pgo;

void init_mesh_geo_bindings(nb::module_ &m)
{
  nb::enum_<Mesh::MeshDataType>(m, "MeshDataType")
    .value("Triangle", Mesh::MeshDataType::Triangle)
    .value("Tet", Mesh::MeshDataType::Tet)
    .value("Cubic", Mesh::MeshDataType::Cubic);

  nb::class_<PyTriMeshData>(m, "PyTriMeshData")
    .def("num_vertices", &PyTriMeshData::numVertices)
    .def("num_elements", &PyTriMeshData::numElements)
    .def("mesh_type", &PyTriMeshData::meshType)
    .def("vertices", &triMeshDataVertices)
    .def("elements", &triMeshDataElements)
    .def("element_vtx_id", &PyTriMeshData::elementVtxID);

  nb::class_<PyTetMeshData>(m, "PyTetMeshData")
    .def("num_vertices", &PyTetMeshData::numVertices)
    .def("num_elements", &PyTetMeshData::numElements)
    .def("mesh_type", &PyTetMeshData::meshType)
    .def("vertices", &tetMeshDataVertices)
    .def("elements", &tetMeshDataElements)
    .def("element_vtx_id", &PyTetMeshData::elementVtxID);

  nb::class_<PyCubicMeshData>(m, "PyCubicMeshData")
    .def("num_vertices", &PyCubicMeshData::numVertices)
    .def("num_elements", &PyCubicMeshData::numElements)
    .def("mesh_type", &PyCubicMeshData::meshType)
    .def("vertices", &cubicMeshDataVertices)
    .def("elements", &cubicMeshDataElements)
    .def("element_vtx_id", &PyCubicMeshData::elementVtxID);

  nb::class_<PyTriMeshGeo>(m, "PyTriMeshGeo")
    .def(nb::init<const PyTriMeshData &>())
    .def("num_vertices", &PyTriMeshGeo::numVertices)
    .def("num_triangles", &PyTriMeshGeo::numTriangles)
    .def("vertices", &triMeshGeoVertices)
    .def("triangles", &triMeshGeoTriangles)
    .def("tri_vtx_id", &PyTriMeshGeo::triVtxID)
    .def("to_mesh_data", &PyTriMeshGeo::toMeshData);

  nb::class_<PyTetMeshGeo>(m, "PyTetMeshGeo")
    .def(nb::init<const PyTetMeshData &>())
    .def("num_vertices", &PyTetMeshGeo::numVertices)
    .def("num_tets", &PyTetMeshGeo::numTets)
    .def("vertices", &tetMeshGeoVertices)
    .def("tets", &tetMeshGeoTets)
    .def("tet_vtx_id", &PyTetMeshGeo::tetVtxID)
    .def("to_mesh_data", &PyTetMeshGeo::toMeshData);

  nb::class_<PyCubicMeshGeo>(m, "PyCubicMeshGeo")
    .def(nb::init<const PyCubicMeshData &>())
    .def("num_vertices", &PyCubicMeshGeo::numVertices)
    .def("num_cubes", &PyCubicMeshGeo::numCubes)
    .def("vertices", &cubicMeshGeoVertices)
    .def("cubes", &cubicMeshGeoCubes)
    .def("cube_vtx_id", &PyCubicMeshGeo::cubeVtxID)
    .def("to_mesh_data", &PyCubicMeshGeo::toMeshData);

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
    nb::arg("surface_data"), nb::arg("resolution"), nb::arg("occupancy") = "conservative", nb::arg("E") = 1e6,
    nb::arg("nu") = 0.45, nb::arg("density") = 1000.0);
  m.def("tet_mesher", &tet_mesher_bind,
    nb::arg("surface_data"), nb::arg("backend"), nb::arg("tetgen_command") = "pq1.414",
    nb::arg("tetwild_lr") = 0.05, nb::arg("tetwild_la") = 0.0,
    nb::arg("tetwild_has_la") = false, nb::arg("tetwild_epsr") = 0.001,
    nb::arg("tetwild_stop_energy") = 10.0, nb::arg("tetwild_max_threads") = 0);
  m.def("has_tetwild", &has_tetwild_bind);

  // Connected component queries
  m.def("triangle_component_ids", &triangleComponentIds, nb::arg("surface"));
  m.def("connected_components_by_edge", &connectedComponentsByEdge, nb::arg("surface"));
  m.def("connected_components_by_vertex", &connectedComponentsByVertex, nb::arg("surface"));
  m.def("filter_mesh_components", &filterTriMeshComponents,
    nb::arg("mesh"), nb::arg("min_elements") = 0, nb::arg("keep_largest") = -1);
  m.def("filter_mesh_components", &filterTetMeshComponents,
    nb::arg("mesh"), nb::arg("min_elements") = 0, nb::arg("keep_largest") = -1);
  m.def("filter_mesh_components", &filterCubicMeshComponents,
    nb::arg("mesh"), nb::arg("min_elements") = 0, nb::arg("keep_largest") = -1);
  m.def("get_outer_component", &getOuterComponent, nb::arg("surface"));

  // Surface remeshing
  m.def("surface_remove_isolated_vertices", &surfaceRemoveIsolatedVertices, nb::arg("surface"));

#ifdef PYPGO_HAS_CGAL
  m.def("surface_merge_close_vertices", &surfaceMergeCloseVertices,
    nb::arg("surface"), nb::arg("eps") = -1.0);
  m.def("raw_surface_cleanup", &rawSurfaceCleanup,
    nb::arg("surface"), nb::arg("expected_components") = -1,
    nb::arg("short_edge_threshold") = 1e-5, nb::arg("max_passes") = 3,
    nb::arg("max_collapses") = 10000);
#endif

  m.def("cgal_smooth_surface", &cgalSmoothSurface,
    nb::arg("surface"), nb::arg("num_iter") = 10, nb::arg("sharp_angle") = 180.0);
  m.def("cgal_isotropic_remesh", &cgalIsotropicRemesh,
    nb::arg("surface"), nb::arg("target_edge_length"), nb::arg("num_iter") = 10, nb::arg("sharp_angle") = 180.0);
  m.def("cgal_repair_self_intersections", &cgalRepairSelfIntersections,
    nb::arg("surface"), nb::arg("method") = std::string("autorefine"));
  m.def("cgal_simplify_surface", &cgalSimplifySurface,
    nb::arg("surface"), nb::arg("target_ratio"));
  m.def("has_cgal_remesher", &hasCgalRemesher);
}
