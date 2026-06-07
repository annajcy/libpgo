#include "core.h"

#include "extraction/marchingCubesExtractor.h"
#include "operations/OffsetField.h"
#include "operations/booleanOps.h"
#include "triMeshGeo.h"

#include <nanobind/stl/vector.h>

#include <stdexcept>
#include <string>
#include <utility>

namespace pgo
{
namespace
{
using ImplicitSurface::V3d;

V3d arrayToV3d(ImplicitDoubleArray values, const char *name)
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

std::shared_ptr<ImplicitSurface::OpenVDBLevelSet> toShared(
  std::unique_ptr<ImplicitSurface::OpenVDBLevelSet> levelSet)
{
  return std::shared_ptr<ImplicitSurface::OpenVDBLevelSet>(std::move(levelSet));
}
}  // namespace

PyGridSpec::PyGridSpec(ImplicitDoubleArray bmin, ImplicitDoubleArray bmax, int resolution)
{
  spec_.bmin = arrayToV3d(bmin, "bmin");
  spec_.bmax = arrayToV3d(bmax, "bmax");
  spec_.resolution = resolution;
  validateGridSpec(spec_);
}

PyGridSpec::PyGridSpec(ImplicitSurface::GridSpec spec)
  : spec_(std::move(spec))
{
  validateGridSpec(spec_);
}

int PyGridSpec::resolution() const
{
  return spec_.resolution;
}

void PyGridSpec::setResolution(int resolution)
{
  spec_.resolution = resolution;
  validateGridSpec(spec_);
}

std::vector<double> PyGridSpec::bmin() const
{
  return vec3ToVector(spec_.bmin);
}

std::vector<double> PyGridSpec::bmax() const
{
  return vec3ToVector(spec_.bmax);
}

PyImplicitField::PyImplicitField(std::shared_ptr<ImplicitSurface::ImplicitField> field)
  : field_(std::move(field))
{
  if (!field_) {
    throw std::runtime_error("PyImplicitField requires a non-null field");
  }
}

double PyImplicitField::eval(ImplicitDoubleArray p) const
{
  const V3d point = arrayToV3d(p, "p");
  nb::gil_scoped_release release;
  return field_->eval(point);
}

std::shared_ptr<PyGridField> PyImplicitField::sampleToGrid(const PyGridSpec &spec, int numThreads) const
{
  nb::gil_scoped_release release;
  return std::make_shared<PyGridField>(
    std::make_shared<ImplicitSurface::GridField>(field_->sampleToGrid(spec.core(), numThreads)));
}

nb::object PyImplicitField::bounds() const
{
  const auto bb = field_->bounds();
  if (ImplicitSurface::isUnbounded(bb))
    return nb::none();
  return nb::make_tuple(vec3ToVector(bb.bmin()), vec3ToVector(bb.bmax()));
}

PyGridField::PyGridField(std::shared_ptr<ImplicitSurface::GridField> grid)
  : PyImplicitField(grid), grid_(std::move(grid))
{
}

nb::ndarray<nb::numpy, double, nb::ndim<3>> PyGridField::array()
{
  const size_t r = static_cast<size_t>(grid_->resolution());
  return nb::ndarray<nb::numpy, double, nb::ndim<3>>(grid_->data(), { r, r, r });
}

PyGridSpec PyGridField::gridSpec() const
{
  return PyGridSpec(grid_->gridSpec());
}

int PyGridField::resolution() const
{
  return grid_->resolution();
}

double PyGridField::evalGrid(ImplicitDoubleArray p) const
{
  return grid_->eval(arrayToV3d(p, "p"));
}

std::shared_ptr<PyGridField> PyGridField::allocLike() const
{
  return std::make_shared<PyGridField>(
    std::make_shared<ImplicitSurface::GridField>(grid_->gridSpec()));
}

PySphereField::PySphereField(ImplicitDoubleArray center, double radius)
  : PySphereField(std::make_shared<ImplicitSurface::SphereField>(arrayToV3d(center, "center"), radius))
{
}

PySphereField::PySphereField(std::shared_ptr<ImplicitSurface::SphereField> sphere)
  : PyImplicitField(sphere), sphere_(std::move(sphere))
{
}

std::shared_ptr<PySphereField> PySphereField::fromMeshBBox(const PyTriMeshData &data)
{
  return std::make_shared<PySphereField>(
    std::make_shared<ImplicitSurface::SphereField>(
      ImplicitSurface::SphereField::fromMeshBBox(Mesh::TriMeshGeo(data.core()))));
}

std::vector<double> PySphereField::center() const
{
  return vec3ToVector(sphere_->center);
}

double PySphereField::radius() const
{
  return sphere_->radius;
}

PyMeshUnsignedDistanceField::PyMeshUnsignedDistanceField(const PyTriMeshData &data)
  : PyImplicitField(std::make_shared<ImplicitSurface::MeshUnsignedDistanceField>(Mesh::TriMeshGeo(data.core())))
{
}

PyBoxField::PyBoxField(ImplicitDoubleArray center, ImplicitDoubleArray halfExtent)
  : PyBoxField(std::make_shared<ImplicitSurface::BoxField>(
      arrayToV3d(center, "center"), arrayToV3d(halfExtent, "half_extent")))
{
}

PyBoxField::PyBoxField(std::shared_ptr<ImplicitSurface::BoxField> box)
  : PyImplicitField(box), box_(std::move(box))
{
}

std::shared_ptr<PyBoxField> PyBoxField::fromBBox(ImplicitDoubleArray bmin, ImplicitDoubleArray bmax)
{
  return std::make_shared<PyBoxField>(
    std::make_shared<ImplicitSurface::BoxField>(
      Mesh::LightBoundingBox(arrayToV3d(bmin, "bmin"), arrayToV3d(bmax, "bmax"))));
}

PyOpenVDBOptions::PyOpenVDBOptions(double voxelSize)
{
  options_.voxelSize = voxelSize;
  validateOpenVDBOptions(options_);
}

double PyOpenVDBOptions::voxelSize() const
{
  return options_.voxelSize;
}

void PyOpenVDBOptions::setVoxelSize(double value)
{
  options_.voxelSize = value;
  validateOpenVDBOptions(options_);
}

double PyOpenVDBOptions::halfWidth() const
{
  return options_.halfWidth;
}

void PyOpenVDBOptions::setHalfWidth(double value)
{
  options_.halfWidth = value;
  validateOpenVDBOptions(options_);
}

double PyOpenVDBOptions::adaptivity() const
{
  return options_.adaptivity;
}

void PyOpenVDBOptions::setAdaptivity(double value)
{
  options_.adaptivity = value;
  validateOpenVDBOptions(options_);
}

int PyOpenVDBOptions::smoothSteps() const
{
  return options_.smoothSteps;
}

void PyOpenVDBOptions::setSmoothSteps(int value)
{
  options_.smoothSteps = value;
  validateOpenVDBOptions(options_);
}

PyOpenVDBLevelSet::PyOpenVDBLevelSet(std::shared_ptr<ImplicitSurface::OpenVDBLevelSet> levelSet)
  : levelSet_(std::move(levelSet))
{
  if (!levelSet_) {
    throw std::runtime_error("PyOpenVDBLevelSet requires a non-null level set");
  }
}

std::shared_ptr<PyImplicitField> implicit_union(
  std::shared_ptr<PyImplicitField> a, std::shared_ptr<PyImplicitField> b)
{
  return std::make_shared<PyImplicitField>(
    ImplicitSurface::makeUnion(a->fieldHandle(), b->fieldHandle()));
}

std::shared_ptr<PyImplicitField> implicit_intersection(
  std::shared_ptr<PyImplicitField> a, std::shared_ptr<PyImplicitField> b)
{
  return std::make_shared<PyImplicitField>(
    ImplicitSurface::makeIntersection(a->fieldHandle(), b->fieldHandle()));
}

std::shared_ptr<PyImplicitField> implicit_difference(
  std::shared_ptr<PyImplicitField> a, std::shared_ptr<PyImplicitField> b)
{
  return std::make_shared<PyImplicitField>(
    ImplicitSurface::makeDifference(a->fieldHandle(), b->fieldHandle()));
}

std::shared_ptr<PyImplicitField> implicit_offset(std::shared_ptr<PyImplicitField> inner, double offset)
{
  return std::make_shared<PyImplicitField>(
    std::make_shared<ImplicitSurface::OffsetField>(inner->fieldHandle(), offset));
}

PyTriMeshData extract_marching_cubes(const PyGridField &field, double isoOffset)
{
  ImplicitSurface::MarchingCubesOptions options;
  options.isoOffset = isoOffset;
  Mesh::TriMeshGeo out;
  {
    nb::gil_scoped_release release;
    extractMarchingCubes(field.grid(), options, out);
  }
  return PyTriMeshData(out.toMeshData());
}

bool has_openvdb()
{
#ifdef PGO_HAS_OPENVDB
  return true;
#else
  return false;
#endif
}

std::shared_ptr<PyOpenVDBLevelSet> build_openvdb_shell_from_mesh(
  const PyTriMeshData &data, double thickness, const PyOpenVDBOptions &options)
{
  nb::gil_scoped_release release;
  return std::make_shared<PyOpenVDBLevelSet>(
    toShared(buildOpenVDBShellFromMesh(Mesh::TriMeshGeo(data.core()), thickness, options.core())));
}

std::shared_ptr<PyOpenVDBLevelSet> build_openvdb_from_grid_field(
  const PyGridField &field, const PyOpenVDBOptions &options)
{
  nb::gil_scoped_release release;
  return std::make_shared<PyOpenVDBLevelSet>(
    toShared(buildOpenVDBFromGridField(field.grid(), options.core())));
}

PyTriMeshData extract_openvdb(
  const PyOpenVDBLevelSet &levelSet, const PyOpenVDBOptions &options)
{
  Mesh::TriMeshGeo out;
  {
    nb::gil_scoped_release release;
    extractOpenVDBLevelSet(levelSet.core(), options.core(), out);
  }
  return PyTriMeshData(out.toMeshData());
}

}  // namespace pgo
