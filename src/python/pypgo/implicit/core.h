#pragma once

#include "core/ImplicitField.h"
#include "extraction/openVDBExtractor.h"
#include "fields/BoxField.h"
#include "fields/GridField.h"
#include "fields/MeshUnsignedDistanceField.h"
#include "fields/SphereField.h"
#include "../mesh/geo/core.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <memory>
#include <vector>

namespace pgo
{
namespace nb = nanobind;

using ImplicitDoubleArray = nb::ndarray<nb::numpy, const double>;

class PyGridSpec
{
public:
  PyGridSpec(ImplicitDoubleArray bmin, ImplicitDoubleArray bmax, int resolution);
  explicit PyGridSpec(ImplicitSurface::GridSpec spec);

  const ImplicitSurface::GridSpec &core() const { return spec_; }
  int resolution() const;
  void setResolution(int resolution);
  std::vector<double> bmin() const;
  std::vector<double> bmax() const;

private:
  ImplicitSurface::GridSpec spec_;
};

class PyImplicitField
{
public:
  explicit PyImplicitField(std::shared_ptr<ImplicitSurface::ImplicitField> field);
  virtual ~PyImplicitField() = default;

  const std::shared_ptr<ImplicitSurface::ImplicitField> &fieldHandle() const { return field_; }
  double eval(ImplicitDoubleArray p) const;
  std::shared_ptr<class PyGridField> sampleToGrid(const PyGridSpec &spec) const;
  nb::object bounds() const;

private:
  std::shared_ptr<ImplicitSurface::ImplicitField> field_;
};

class PyGridField final : public PyImplicitField
{
public:
  explicit PyGridField(std::shared_ptr<ImplicitSurface::GridField> grid);

  nb::ndarray<nb::numpy, double, nb::ndim<3>> array();
  PyGridSpec gridSpec() const;
  int resolution() const;
  double evalGrid(ImplicitDoubleArray p) const;
  std::shared_ptr<PyGridField> allocLike() const;
  const ImplicitSurface::GridField &grid() const { return *grid_; }

private:
  std::shared_ptr<ImplicitSurface::GridField> grid_;
};

class PySphereField final : public PyImplicitField
{
public:
  PySphereField(ImplicitDoubleArray center, double radius);
  explicit PySphereField(std::shared_ptr<ImplicitSurface::SphereField> sphere);

  static std::shared_ptr<PySphereField> fromMeshBBox(const PyTriMeshData &data);
  std::vector<double> center() const;
  double radius() const;

private:
  std::shared_ptr<ImplicitSurface::SphereField> sphere_;
};

class PyMeshUnsignedDistanceField final : public PyImplicitField
{
public:
  explicit PyMeshUnsignedDistanceField(const PyTriMeshData &data);
};

class PyBoxField final : public PyImplicitField
{
public:
  PyBoxField(ImplicitDoubleArray center, ImplicitDoubleArray halfExtent);
  explicit PyBoxField(std::shared_ptr<ImplicitSurface::BoxField> box);

  static std::shared_ptr<PyBoxField> fromBBox(ImplicitDoubleArray bmin, ImplicitDoubleArray bmax);

private:
  std::shared_ptr<ImplicitSurface::BoxField> box_;
};

class PyOpenVDBOptions
{
public:
  explicit PyOpenVDBOptions(double voxelSize);

  const ImplicitSurface::OpenVDBOptions &core() const { return options_; }
  double voxelSize() const;
  void setVoxelSize(double value);
  double halfWidth() const;
  void setHalfWidth(double value);
  double adaptivity() const;
  void setAdaptivity(double value);
  int smoothSteps() const;
  void setSmoothSteps(int value);

private:
  ImplicitSurface::OpenVDBOptions options_;
};

class PyOpenVDBLevelSet
{
public:
  explicit PyOpenVDBLevelSet(std::shared_ptr<ImplicitSurface::OpenVDBLevelSet> levelSet);

  const ImplicitSurface::OpenVDBLevelSet &core() const { return *levelSet_; }

private:
  std::shared_ptr<ImplicitSurface::OpenVDBLevelSet> levelSet_;
};

std::shared_ptr<PyImplicitField> implicit_union(
  std::shared_ptr<PyImplicitField> a, std::shared_ptr<PyImplicitField> b);
std::shared_ptr<PyImplicitField> implicit_intersection(
  std::shared_ptr<PyImplicitField> a, std::shared_ptr<PyImplicitField> b);
std::shared_ptr<PyImplicitField> implicit_difference(
  std::shared_ptr<PyImplicitField> a, std::shared_ptr<PyImplicitField> b);
std::shared_ptr<PyImplicitField> implicit_offset(std::shared_ptr<PyImplicitField> inner, double offset);

PyTriMeshData extract_marching_cubes(const PyGridField &field, double isoOffset);

bool has_openvdb();

std::shared_ptr<PyOpenVDBLevelSet> build_openvdb_shell_from_mesh(
  const PyTriMeshData &data, double thickness, const PyOpenVDBOptions &options);
std::shared_ptr<PyOpenVDBLevelSet> build_openvdb_from_grid_field(
  const PyGridField &field, const PyOpenVDBOptions &options);
PyTriMeshData extract_openvdb(
  const PyOpenVDBLevelSet &levelSet, const PyOpenVDBOptions &options);

}  // namespace pgo
