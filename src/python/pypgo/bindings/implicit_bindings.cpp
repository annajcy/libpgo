#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include <memory>
#include <stdexcept>
#include <vector>

#include "core/ImplicitField.h"
#include "extraction/marchingCubesExtractor.h"
#include "extraction/openVDBExtractor.h"
#include "fields/BoxField.h"
#include "fields/GridField.h"
#include "fields/MeshUnsignedDistanceField.h"
#include "fields/SphereField.h"
#include "meshData.h"
#include "operations/booleanOps.h"
#include "operations/OffsetField.h"
#include "triMeshGeo.h"

namespace nb = nanobind;
using namespace pgo;
using namespace pgo::ImplicitSurface;

namespace
{
using DoubleArray = nb::ndarray<nb::numpy, const double>;

V3d arrayToV3d(DoubleArray values, const char *name)
{
  if (values.dtype() != nb::dtype<double>() || values.ndim() != 1 || values.shape(0) != 3)
    throw nb::value_error((std::string(name) + " must be a float64 array with shape (3,)").c_str());
  return V3d(values.data()[0 * values.stride(0)],
    values.data()[1 * values.stride(0)],
    values.data()[2 * values.stride(0)]);
}

std::vector<double> vec3ToVector(const V3d &v)
{
  return { v[0], v[1], v[2] };
}

std::shared_ptr<OpenVDBLevelSet> toShared(std::unique_ptr<OpenVDBLevelSet> levelSet)
{
  return std::shared_ptr<OpenVDBLevelSet>(std::move(levelSet));
}
}  // namespace

void init_implicit_bindings(nb::module_ &m)
{
  nb::class_<GridSpec>(m, "PyGridSpec")
    .def("__init__", [](GridSpec *self, DoubleArray bmin, DoubleArray bmax, int resolution) {
      GridSpec spec;
      spec.bmin = arrayToV3d(bmin, "bmin");
      spec.bmax = arrayToV3d(bmax, "bmax");
      spec.resolution = resolution;
      validateGridSpec(spec);
      new (self) GridSpec(spec);
    }, nb::arg("bmin"), nb::arg("bmax"), nb::arg("resolution"))
    .def_prop_rw("resolution",
      [](const GridSpec &spec) { return spec.resolution; },
      [](GridSpec &spec, int resolution) {
        spec.resolution = resolution;
        validateGridSpec(spec);
      })
    .def("bmin", [](const GridSpec &spec) { return vec3ToVector(spec.bmin); })
    .def("bmax", [](const GridSpec &spec) { return vec3ToVector(spec.bmax); });

  nb::class_<ImplicitField>(m, "PyImplicitField")
    .def("eval", [](const ImplicitField &field, DoubleArray p) {
      const V3d point = arrayToV3d(p, "p");
      nb::gil_scoped_release release;
      return field.eval(point);
    })
    .def("sample_to_grid", [](const ImplicitField &field, const GridSpec &spec, int numThreads) {
      nb::gil_scoped_release release;
      return std::make_shared<GridField>(field.sampleToGrid(spec, numThreads));
    }, nb::arg("grid_spec"), nb::arg("num_threads") = 0)
    .def("bounds", [](const ImplicitField &field) -> nb::object {
      const auto bb = field.bounds();
      if (isUnbounded(bb))
        return nb::none();
      return nb::make_tuple(vec3ToVector(bb.bmin()), vec3ToVector(bb.bmax()));
    });

  nb::class_<GridField, ImplicitField>(m, "PyGridField")
    .def("__array__", [](GridField &grid) {
      const size_t r = static_cast<size_t>(grid.resolution());
      return nb::ndarray<nb::numpy, double, nb::ndim<3>>(
        grid.data(), { r, r, r });
    }, nb::rv_policy::reference_internal)
    .def("grid_spec", [](const GridField &grid) { return grid.gridSpec(); })
    .def("resolution", &GridField::resolution)
    .def("eval", [](const GridField &grid, DoubleArray p) {
      return grid.eval(arrayToV3d(p, "p"));
    })
    .def("alloc_like", [](const GridField &grid) {
      return std::make_shared<GridField>(grid.gridSpec());
    });

  nb::class_<SphereField, ImplicitField>(m, "PySphereField")
    .def("__init__", [](SphereField *self, DoubleArray center, double radius) {
      new (self) SphereField(arrayToV3d(center, "center"), radius);
    }, nb::arg("center"), nb::arg("radius"))
    .def_static("from_mesh_bbox", [](const Mesh::MeshData<3> &data) {
      return std::make_shared<SphereField>(SphereField::fromMeshBBox(Mesh::TriMeshGeo(data)));
    }, nb::arg("surface_data"))
    .def("center", [](const SphereField &sphere) { return vec3ToVector(sphere.center); })
    .def("radius", [](const SphereField &sphere) { return sphere.radius; });

  nb::class_<MeshUnsignedDistanceField, ImplicitField>(m, "PyMeshUnsignedDistanceField")
    .def("__init__", [](MeshUnsignedDistanceField *self, const Mesh::MeshData<3> &data) {
      new (self) MeshUnsignedDistanceField(Mesh::TriMeshGeo(data));
    }, nb::arg("surface_data"));

  nb::class_<BoxField, ImplicitField>(m, "PyBoxField")
    .def("__init__", [](BoxField *self, DoubleArray center, DoubleArray halfExtent) {
      new (self) BoxField(arrayToV3d(center, "center"), arrayToV3d(halfExtent, "half_extent"));
    }, nb::arg("center"), nb::arg("half_extent"))
    .def_static("from_bbox", [](DoubleArray bmin, DoubleArray bmax) {
      return std::make_shared<BoxField>(
        Mesh::LightBoundingBox(arrayToV3d(bmin, "bmin"), arrayToV3d(bmax, "bmax")));
    }, nb::arg("bmin"), nb::arg("bmax"));

  m.def("implicit_union", &makeUnion, nb::arg("a"), nb::arg("b"));
  m.def("implicit_intersection", &makeIntersection, nb::arg("a"), nb::arg("b"));
  m.def("implicit_difference", &makeDifference, nb::arg("a"), nb::arg("b"));
  m.def("implicit_offset", [](std::shared_ptr<ImplicitField> inner, double offset) -> std::shared_ptr<ImplicitField> {
    return std::make_shared<OffsetField>(std::move(inner), offset);
  }, nb::arg("inner"), nb::arg("offset"));

  m.def("extract_marching_cubes", [](const GridField &field, double isoOffset) {
    MarchingCubesOptions options;
    options.isoOffset = isoOffset;
    Mesh::TriMeshGeo out;
    {
      nb::gil_scoped_release release;
      extractMarchingCubes(field, options, out);
    }
    return out.toMeshData();
  }, nb::arg("field"), nb::arg("iso_offset") = 0.0);

  m.def("has_openvdb", []() {
#ifdef PGO_HAS_OPENVDB
    return true;
#else
    return false;
#endif
  });

  nb::class_<OpenVDBOptions>(m, "PyOpenVDBOptions")
    .def("__init__", [](OpenVDBOptions *self, double voxelSize) {
      OpenVDBOptions options;
      options.voxelSize = voxelSize;
      validateOpenVDBOptions(options);
      new (self) OpenVDBOptions(options);
    }, nb::arg("voxel_size"))
    .def_prop_rw("voxel_size", [](const OpenVDBOptions &o) { return o.voxelSize; }, [](OpenVDBOptions &o, double v) { o.voxelSize = v; validateOpenVDBOptions(o); })
    .def_prop_rw("half_width", [](const OpenVDBOptions &o) { return o.halfWidth; }, [](OpenVDBOptions &o, double v) { o.halfWidth = v; validateOpenVDBOptions(o); })
    .def_prop_rw("adaptivity", [](const OpenVDBOptions &o) { return o.adaptivity; }, [](OpenVDBOptions &o, double v) { o.adaptivity = v; validateOpenVDBOptions(o); })
    .def_prop_rw("smooth_steps", [](const OpenVDBOptions &o) { return o.smoothSteps; }, [](OpenVDBOptions &o, int v) { o.smoothSteps = v; validateOpenVDBOptions(o); });

  nb::class_<OpenVDBLevelSet>(m, "PyOpenVDBLevelSet");

  m.def("build_openvdb_shell_from_mesh", [](const Mesh::MeshData<3> &data, double thickness, const OpenVDBOptions &options) {
    nb::gil_scoped_release release;
    return toShared(buildOpenVDBShellFromMesh(Mesh::TriMeshGeo(data), thickness, options));
  }, nb::arg("surface_data"), nb::arg("thickness"), nb::arg("options"));

  m.def("build_openvdb_from_grid_field", [](const GridField &field, const OpenVDBOptions &options) {
    nb::gil_scoped_release release;
    return toShared(buildOpenVDBFromGridField(field, options));
  }, nb::arg("field"), nb::arg("options"));

  m.def("extract_openvdb", [](const OpenVDBLevelSet &levelSet, const OpenVDBOptions &options) {
    Mesh::TriMeshGeo out;
    {
      nb::gil_scoped_release release;
      extractOpenVDBLevelSet(levelSet, options, out);
    }
    return out.toMeshData();
  }, nb::arg("levelset"), nb::arg("options"));
}
