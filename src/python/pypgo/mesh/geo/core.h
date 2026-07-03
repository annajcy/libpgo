#pragma once

#include "meshData.h"
#include "tetMeshGeo.h"
#include "triMeshGeo.h"
#include "cubicMeshGeo.h"

#include <nanobind/nanobind.h>

#include <string>
#include <utility>
#include <vector>

namespace pgo
{

namespace nb = nanobind;

template<int K>
class PyMeshData
{
public:
  explicit PyMeshData(pgo::Mesh::MeshData<K> data)
    : data_(std::move(data))
  {
  }

  const pgo::Mesh::MeshData<K> &core() const { return data_; }
  pgo::Mesh::MeshData<K> &core() { return data_; }

  int numVertices() const { return data_.numVertices(); }
  int numElements() const { return data_.numElements(); }
  pgo::Mesh::MeshDataType meshType() const { return data_.meshType(); }
  int elementVtxID(int elementID, int localVertexID) const { return data_.elementVtxID(elementID, localVertexID); }

private:
  pgo::Mesh::MeshData<K> data_;
};

using PyTriMeshData = PyMeshData<3>;
using PyTetMeshData = PyMeshData<4>;
using PyCubicMeshData = PyMeshData<8>;

class PyTriMeshGeo
{
public:
  explicit PyTriMeshGeo(const PyTriMeshData &data);
  explicit PyTriMeshGeo(pgo::Mesh::TriMeshGeo mesh);

  const pgo::Mesh::TriMeshGeo &core() const { return mesh_; }
  int numVertices() const { return mesh_.numVertices(); }
  int numTriangles() const { return mesh_.numTriangles(); }
  int triVtxID(int triID, int localVertexID) const { return mesh_.triVtxID(triID, localVertexID); }
  PyTriMeshData toMeshData() const;

private:
  pgo::Mesh::TriMeshGeo mesh_;
};

class PyTetMeshGeo
{
public:
  explicit PyTetMeshGeo(const PyTetMeshData &data);
  explicit PyTetMeshGeo(pgo::Mesh::TetMeshGeo mesh);

  const pgo::Mesh::TetMeshGeo &core() const { return mesh_; }
  int numVertices() const { return mesh_.numVertices(); }
  int numTets() const { return mesh_.numTets(); }
  int tetVtxID(int tetID, int localVertexID) const { return mesh_.tetVtxID(tetID, localVertexID); }
  PyTetMeshData toMeshData() const;

private:
  pgo::Mesh::TetMeshGeo mesh_;
};

class PyCubicMeshGeo
{
public:
  explicit PyCubicMeshGeo(const PyCubicMeshData &data);
  explicit PyCubicMeshGeo(pgo::Mesh::CubicMeshGeo mesh);

  const pgo::Mesh::CubicMeshGeo &core() const { return mesh_; }
  int numVertices() const { return mesh_.numVertices(); }
  int numCubes() const { return mesh_.numCubes(); }
  int cubeVtxID(int cubeID, int localVertexID) const { return mesh_.cubeVtxID(cubeID, localVertexID); }
  PyCubicMeshData toMeshData() const;

private:
  pgo::Mesh::CubicMeshGeo mesh_;
};

// All mesh-geometry binding logic lives here so bindings.cpp is pure
// registration.  These are free functions (the first argument acts as `self`
// when bound as a method).

// ── Mesh construction factories ──────────────────────────────────────────

PyTriMeshData create_tri_mesh_data(const std::vector<double> &vertices, const std::vector<int> &elements);
PyTetMeshData create_tet_mesh_data(const std::vector<double> &vertices, const std::vector<int> &elements);
PyCubicMeshData create_cubic_mesh_data(const std::vector<double> &vertices, const std::vector<int> &elements);
PyTriMeshGeo create_tri_mesh_geo(const std::vector<double> &vertices, const std::vector<int> &triangles);
PyTetMeshGeo create_tet_mesh_geo(const std::vector<double> &vertices, const std::vector<int> &tets);
PyCubicMeshGeo create_cubic_mesh_geo(const std::vector<double> &vertices, const std::vector<int> &cubes);

// ── Geometry accessors (bound as methods) ────────────────────────────────

std::vector<double> triMeshDataVertices(const PyTriMeshData &self);
std::vector<double> tetMeshDataVertices(const PyTetMeshData &self);
std::vector<double> cubicMeshDataVertices(const PyCubicMeshData &self);
std::vector<double> triMeshGeoVertices(const PyTriMeshGeo &self);
std::vector<double> tetMeshGeoVertices(const PyTetMeshGeo &self);
std::vector<double> cubicMeshGeoVertices(const PyCubicMeshGeo &self);
std::vector<int> triMeshGeoTriangles(const PyTriMeshGeo &self);
std::vector<int> tetMeshGeoTets(const PyTetMeshGeo &self);
std::vector<int> cubicMeshGeoCubes(const PyCubicMeshGeo &self);
std::vector<int> triMeshDataElements(const PyTriMeshData &self);
std::vector<int> tetMeshDataElements(const PyTetMeshData &self);
std::vector<int> cubicMeshDataElements(const PyCubicMeshData &self);

// ── IO + queries ─────────────────────────────────────────────────────────

PyTriMeshData read_obj(const std::string &path);
void write_obj(const std::string &path, const PyTriMeshData &data);
bool check_self_intersections(const PyTriMeshData &data);

PyTriMeshData create_box_mesh(const std::vector<double> &bmin, const std::vector<double> &bmax);
PyTriMeshData create_sphere_mesh(double radius, int axisSubdivisions, int heightSubdivisions);
PyTriMeshData create_cylinder_mesh(double radius, double height, int axisSubdivisions, int heightSubdivisions);
PyTriMeshData create_torus_mesh(int radialResolution, int tubularResolution, double radius, double thickness);

PyCubicMeshData cubic_mesher_bind(
  const PyTriMeshData &surfaceData, int resolution, const std::string &occupancy, double E, double nu, double density);
PyTetMeshData tet_mesher_bind(
  const PyTriMeshData &surfaceData, const std::string &backend, const std::string &tetgenCommand,
  double tetwildLr, double tetwildLa, bool tetwildHasLa, double tetwildEpsr,
  double tetwildStopEnergy, int tetwildMaxThreads);
bool has_tetwild_bind();

// ── Connected components / cleanup ───────────────────────────────────────

std::pair<std::vector<int>, std::vector<int>> triangleComponentIds(const PyTriMeshData &surface);
std::vector<std::vector<int>> connectedComponentsByEdge(const PyTriMeshData &surface);
std::vector<std::vector<int>> connectedComponentsByVertex(const PyTriMeshData &surface);
PyTriMeshData filterTriMeshComponents(const PyTriMeshData &mesh, int minElements, int keepLargest);
PyTetMeshData filterTetMeshComponents(const PyTetMeshData &mesh, int minElements, int keepLargest);
PyCubicMeshData filterCubicMeshComponents(const PyCubicMeshData &mesh, int minElements, int keepLargest);
PyTriMeshData getOuterComponent(const PyTriMeshData &surface);
PyTriMeshData surfaceRemoveIsolatedVertices(const PyTriMeshData &surface);

#ifdef PYPGO_HAS_CGAL
nb::tuple surfaceMergeCloseVertices(const PyTriMeshData &surface, double eps);
nb::tuple rawSurfaceCleanup(const PyTriMeshData &surface, int expectedComponents,
  double shortEdgeThreshold, int maxPasses, int maxCollapses);
#endif

PyTriMeshData cgalSmoothSurface(const PyTriMeshData &surface, int numIter, double sharpAngle);
PyTriMeshData cgalIsotropicRemesh(const PyTriMeshData &surface, double targetEdgeLength, int numIter, double sharpAngle);
std::pair<PyTriMeshData, bool> cgalRepairSelfIntersections(const PyTriMeshData &surface, const std::string &method);
PyTriMeshData cgalSimplifySurface(const PyTriMeshData &surface, double targetRatio);
bool hasCgalRemesher();

}  // namespace pgo
