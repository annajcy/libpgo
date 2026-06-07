#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "core.h"

namespace nb = nanobind;
using namespace pgo;

void init_implicit_bindings(nb::module_ &m)
{
  nb::class_<PyGridSpec>(m, "PyGridSpec")
    .def(nb::init<ImplicitDoubleArray, ImplicitDoubleArray, int>(),
      nb::arg("bmin"), nb::arg("bmax"), nb::arg("resolution"))
    .def_prop_rw("resolution", &PyGridSpec::resolution, &PyGridSpec::setResolution)
    .def("bmin", &PyGridSpec::bmin)
    .def("bmax", &PyGridSpec::bmax);

  nb::class_<PyImplicitField>(m, "PyImplicitField")
    .def("eval", &PyImplicitField::eval)
    .def("sample_to_grid", &PyImplicitField::sampleToGrid, nb::arg("grid_spec"), nb::arg("num_threads") = 0)
    .def("bounds", &PyImplicitField::bounds);

  nb::class_<PyGridField, PyImplicitField>(m, "PyGridField")
    .def("__array__", &PyGridField::array, nb::rv_policy::reference_internal)
    .def("grid_spec", &PyGridField::gridSpec)
    .def("resolution", &PyGridField::resolution)
    .def("eval", &PyGridField::evalGrid)
    .def("alloc_like", &PyGridField::allocLike);

  nb::class_<PySphereField, PyImplicitField>(m, "PySphereField")
    .def(nb::init<ImplicitDoubleArray, double>(), nb::arg("center"), nb::arg("radius"))
    .def_static("from_mesh_bbox", &PySphereField::fromMeshBBox, nb::arg("surface_data"))
    .def("center", &PySphereField::center)
    .def("radius", &PySphereField::radius);

  nb::class_<PyMeshUnsignedDistanceField, PyImplicitField>(m, "PyMeshUnsignedDistanceField")
    .def(nb::init<const PyTriMeshData &>(), nb::arg("surface_data"));

  nb::class_<PyBoxField, PyImplicitField>(m, "PyBoxField")
    .def(nb::init<ImplicitDoubleArray, ImplicitDoubleArray>(), nb::arg("center"), nb::arg("half_extent"))
    .def_static("from_bbox", &PyBoxField::fromBBox, nb::arg("bmin"), nb::arg("bmax"));

  m.def("implicit_union", &implicit_union, nb::arg("a"), nb::arg("b"));
  m.def("implicit_intersection", &implicit_intersection, nb::arg("a"), nb::arg("b"));
  m.def("implicit_difference", &implicit_difference, nb::arg("a"), nb::arg("b"));
  m.def("implicit_offset", &implicit_offset, nb::arg("inner"), nb::arg("offset"));

  m.def("extract_marching_cubes", &extract_marching_cubes, nb::arg("field"), nb::arg("iso_offset") = 0.0);

  m.def("has_openvdb", &has_openvdb);

  nb::class_<PyOpenVDBOptions>(m, "PyOpenVDBOptions")
    .def(nb::init<double>(), nb::arg("voxel_size"))
    .def_prop_rw("voxel_size", &PyOpenVDBOptions::voxelSize, &PyOpenVDBOptions::setVoxelSize)
    .def_prop_rw("half_width", &PyOpenVDBOptions::halfWidth, &PyOpenVDBOptions::setHalfWidth)
    .def_prop_rw("adaptivity", &PyOpenVDBOptions::adaptivity, &PyOpenVDBOptions::setAdaptivity)
    .def_prop_rw("smooth_steps", &PyOpenVDBOptions::smoothSteps, &PyOpenVDBOptions::setSmoothSteps);

  nb::class_<PyOpenVDBLevelSet>(m, "PyOpenVDBLevelSet");

  m.def("build_openvdb_shell_from_mesh", &build_openvdb_shell_from_mesh,
    nb::arg("surface_data"), nb::arg("thickness"), nb::arg("options"));
  m.def("build_openvdb_from_grid_field", &build_openvdb_from_grid_field, nb::arg("field"), nb::arg("options"));
  m.def("extract_openvdb", &extract_openvdb, nb::arg("levelset"), nb::arg("options"));
}
