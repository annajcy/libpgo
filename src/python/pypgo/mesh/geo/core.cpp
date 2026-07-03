#include "core.h"

#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>

#include "boundingVolumeTree.h"
#include "common.h"
#include "createTriMesh.h"
#include "cubicMesh.h"
#include "../remesh_backend.h"
#include "tetMesherBackend.h"
#include "triangleMeshVoxelizer.h"
#include "triMeshNeighbor.h"
#ifdef PYPGO_HAS_CGAL
#include "cgalInterface.h"
#endif

namespace pgo
{

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

Vec3d vectorToVec3d(const std::vector<double> &values, const std::string &name)
{
  if (values.size() != 3) {
    throw std::runtime_error(name + " must contain exactly 3 values");
  }
  return Vec3d(values[0], values[1], values[2]);
}

#ifdef PYPGO_HAS_CGAL
nb::dict rawCleanupStatsToDict(const CGALInterface::RawSurfaceCleanupStats &stats)
{
  nb::dict d;
  d[nb::str("vertices")] = stats.vertices;
  d[nb::str("triangles")] = stats.triangles;
  d[nb::str("invalid_triangles")] = stats.invalidTriangles;
  d[nb::str("components")] = stats.components;
  d[nb::str("boundary_or_nonmanifold_edges")] = stats.boundaryOrNonmanifoldEdges;
  d[nb::str("is_manifold")] = stats.isManifold;
  return d;
}

nb::dict rawCleanupReportToDict(const CGALInterface::RawSurfaceCleanupReport &report)
{
  nb::dict d;
  d[nb::str("expected_components")] = report.expectedComponents;
  d[nb::str("short_edge_threshold")] = report.shortEdgeThreshold;
  d[nb::str("max_passes")] = report.maxPasses;
  d[nb::str("max_collapses")] = report.maxCollapses;
  d[nb::str("before")] = rawCleanupStatsToDict(report.before);
  d[nb::str("after")] = rawCleanupStatsToDict(report.after);
  d[nb::str("vertices_before")] = report.before.vertices;
  d[nb::str("vertices_after")] = report.after.vertices;
  d[nb::str("triangles_before")] = report.before.triangles;
  d[nb::str("triangles_after")] = report.after.triangles;
  d[nb::str("invalid_triangles_before")] = report.before.invalidTriangles;
  d[nb::str("invalid_triangles_after")] = report.after.invalidTriangles;
  d[nb::str("components_before")] = report.before.components;
  d[nb::str("components_after")] = report.after.components;
  d[nb::str("boundary_or_nonmanifold_edges_before")] = report.before.boundaryOrNonmanifoldEdges;
  d[nb::str("boundary_or_nonmanifold_edges_after")] = report.after.boundaryOrNonmanifoldEdges;
  d[nb::str("is_manifold_before")] = report.before.isManifold;
  d[nb::str("is_manifold_after")] = report.after.isManifold;
  d[nb::str("topology_preserved")] = report.topologyPreserved;
  d[nb::str("cleanup_complete")] = report.cleanupComplete;
  d[nb::str("attempted_deletions")] = report.attemptedDeletions;
  d[nb::str("accepted_deletions")] = report.acceptedDeletions;
  d[nb::str("attempted_collapses")] = report.attemptedCollapses;
  d[nb::str("accepted_collapses")] = report.acceptedCollapses;
  d[nb::str("rejected_by_topology")] = report.rejectedByTopology;
  d[nb::str("rejected_by_invalid_count")] = report.rejectedByInvalidCount;
  return d;
}
#endif

}  // namespace

// ── Mesh peer objects ─────────────────────────────────────────────────────

PyTriMeshGeo::PyTriMeshGeo(const PyTriMeshData &data)
  : mesh_(data.core())
{
}

PyTriMeshGeo::PyTriMeshGeo(Mesh::TriMeshGeo mesh)
  : mesh_(std::move(mesh))
{
}

PyTriMeshData PyTriMeshGeo::toMeshData() const
{
  return PyTriMeshData(mesh_.toMeshData());
}

PyTetMeshGeo::PyTetMeshGeo(const PyTetMeshData &data)
  : mesh_(data.core())
{
}

PyTetMeshGeo::PyTetMeshGeo(Mesh::TetMeshGeo mesh)
  : mesh_(std::move(mesh))
{
}

PyTetMeshData PyTetMeshGeo::toMeshData() const
{
  return PyTetMeshData(mesh_.toMeshData());
}

PyCubicMeshGeo::PyCubicMeshGeo(const PyCubicMeshData &data)
  : mesh_(data.core())
{
}

PyCubicMeshGeo::PyCubicMeshGeo(Mesh::CubicMeshGeo mesh)
  : mesh_(std::move(mesh))
{
}

PyCubicMeshData PyCubicMeshGeo::toMeshData() const
{
  return PyCubicMeshData(mesh_.toMeshData());
}

// ── Mesh construction factories ──────────────────────────────────────────

PyTriMeshData create_tri_mesh_data(const std::vector<double> &vertices, const std::vector<int> &elements)
{
  return PyTriMeshData(Mesh::MeshData<3>::fromFlatElements(unpackVertices(vertices), elements));
}

PyTetMeshData create_tet_mesh_data(const std::vector<double> &vertices, const std::vector<int> &elements)
{
  return PyTetMeshData(Mesh::MeshData<4>::fromFlatElements(unpackVertices(vertices), elements));
}

PyCubicMeshData create_cubic_mesh_data(const std::vector<double> &vertices, const std::vector<int> &elements)
{
  return PyCubicMeshData(Mesh::MeshData<8>::fromFlatElements(unpackVertices(vertices), elements));
}

PyTriMeshGeo create_tri_mesh_geo(const std::vector<double> &vertices, const std::vector<int> &triangles)
{
  return PyTriMeshGeo(Mesh::TriMeshGeo(static_cast<int>(vertices.size() / 3), vertices.data(),
    static_cast<int>(triangles.size() / 3), triangles.data()));
}

PyTetMeshGeo create_tet_mesh_geo(const std::vector<double> &vertices, const std::vector<int> &tets)
{
  return PyTetMeshGeo(Mesh::TetMeshGeo(static_cast<int>(vertices.size() / 3), vertices.data(),
    static_cast<int>(tets.size() / 4), tets.data()));
}

PyCubicMeshGeo create_cubic_mesh_geo(const std::vector<double> &vertices, const std::vector<int> &cubes)
{
  return PyCubicMeshGeo(Mesh::CubicMeshGeo(static_cast<int>(vertices.size() / 3), vertices.data(),
    static_cast<int>(cubes.size() / 8), cubes.data()));
}

// ── Geometry accessors ───────────────────────────────────────────────────

std::vector<double> triMeshDataVertices(const PyTriMeshData &self) { return flattenVertices(self.core().positions()); }
std::vector<double> tetMeshDataVertices(const PyTetMeshData &self) { return flattenVertices(self.core().positions()); }
std::vector<double> cubicMeshDataVertices(const PyCubicMeshData &self) { return flattenVertices(self.core().positions()); }
std::vector<double> triMeshGeoVertices(const PyTriMeshGeo &self) { return flattenVertices(self.core().positions()); }
std::vector<double> tetMeshGeoVertices(const PyTetMeshGeo &self) { return flattenVertices(self.core().positions()); }
std::vector<double> cubicMeshGeoVertices(const PyCubicMeshGeo &self) { return flattenVertices(self.core().positions()); }
std::vector<int> triMeshGeoTriangles(const PyTriMeshGeo &self) { return flattenIndexVector(self.core().triangles(), 3); }
std::vector<int> tetMeshGeoTets(const PyTetMeshGeo &self) { return flattenIndexVector(self.core().tets(), 4); }
std::vector<int> cubicMeshGeoCubes(const PyCubicMeshGeo &self) { return flattenIndexVector(self.core().cubes(), 8); }
std::vector<int> triMeshDataElements(const PyTriMeshData &self) { return self.core().elementsFlat(); }
std::vector<int> tetMeshDataElements(const PyTetMeshData &self) { return self.core().elementsFlat(); }
std::vector<int> cubicMeshDataElements(const PyCubicMeshData &self) { return self.core().elementsFlat(); }

// ── IO + queries ─────────────────────────────────────────────────────────

PyTriMeshData read_obj(const std::string &path)
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
  return PyTriMeshData(mesh.toMeshData());
}

void write_obj(const std::string &path, const PyTriMeshData &data)
{
  Mesh::TriMeshGeo triMesh(data.core());
  bool ok = false;
  {
    nb::gil_scoped_release release;
    ok = triMesh.save(path);
  }
  if (!ok) {
    throw std::runtime_error("Failed to save TriMeshGeo to " + path);
  }
}

bool check_self_intersections(const PyTriMeshData &data)
{
  Mesh::TriMeshGeo mesh(data.core());
  std::vector<std::pair<int, int>> intersections;
  {
    nb::gil_scoped_release release;
    Mesh::TriMeshBVTree bvTree;
    bvTree.buildByInertiaPartition(mesh.ref());
    bvTree.selfIntersectionExact(mesh.ref(), intersections);
  }
  return intersections.empty() == false;
}

PyTriMeshData create_box_mesh(const std::vector<double> &bmin, const std::vector<double> &bmax)
{
  return PyTriMeshData(Mesh::createBoxMesh(vectorToVec3d(bmin, "bmin"), vectorToVec3d(bmax, "bmax")).toMeshData());
}

PyTriMeshData create_sphere_mesh(double radius, int axisSubdivisions, int heightSubdivisions)
{
  if (radius <= 0.0 || axisSubdivisions < 2 || heightSubdivisions < 2) {
    throw std::runtime_error("create_sphere_mesh requires radius > 0 and subdivisions >= 2");
  }
  return PyTriMeshData(Mesh::createSphereMesh(radius, axisSubdivisions, heightSubdivisions).toMeshData());
}

PyTriMeshData create_cylinder_mesh(double radius, double height, int axisSubdivisions, int heightSubdivisions)
{
  if (radius <= 0.0 || height <= 0.0 || axisSubdivisions < 3 || heightSubdivisions < 1) {
    throw std::runtime_error("create_cylinder_mesh requires radius > 0, height > 0, axis subdivisions >= 3, and height subdivisions >= 1");
  }
  return PyTriMeshData(Mesh::createCylinderMesh(radius, height, axisSubdivisions, heightSubdivisions).toMeshData());
}

PyTriMeshData create_torus_mesh(int radialResolution, int tubularResolution, double radius, double thickness)
{
  if (radialResolution < 3 || tubularResolution < 3 || radius <= 0.0 || thickness <= 0.0) {
    throw std::runtime_error("create_torus_mesh requires resolutions >= 3, radius > 0, and thickness > 0");
  }
  return PyTriMeshData(Mesh::createTorus(radialResolution, tubularResolution, radius, thickness).toMeshData());
}

PyCubicMeshData cubic_mesher_bind(
  const PyTriMeshData &surfaceData,
  int resolution,
  const std::string &occupancy,
  double E,
  double nu,
  double density)
{
  if (resolution <= 0) {
    throw std::runtime_error("cubic_mesher requires resolution > 0");
  }

  TemporaryObjFile input(surfaceData.core());
  cubic_mesher::TriangleMeshVoxelizerOptions options;
  options.inputMesh = input.path().string();
  options.resolution = resolution;
  if (occupancy == "center") {
    options.occupancy = cubic_mesher::OccupancyMode::Center;
  }
  else if (occupancy != "conservative") {
    throw std::runtime_error("cubic_mesher occupancy must be 'conservative' or 'center'");
  }
  options.E = E;
  options.nu = nu;
  options.density = density;

  std::unique_ptr<VolumetricMeshes::CubicMesh> cubicMesh;
  {
    nb::gil_scoped_release release;
    cubicMesh = cubic_mesher::createTriangleMeshCubicMesh(options);
  }
  return PyCubicMeshData(exportVolumeMeshData<8>(*cubicMesh));
}

PyTetMeshData tet_mesher_bind(
  const PyTriMeshData &surfaceData,
  const std::string &backend,
  const std::string &tetgenCommand,
  double tetwildLr,
  double tetwildLa,
  bool tetwildHasLa,
  double tetwildEpsr,
  double tetwildStopEnergy,
  int tetwildMaxThreads)
{
  TemporaryObjFile input(surfaceData.core());

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

  return PyTetMeshData(exportVolumeMeshData<4>(*tetMesh));
}

bool has_tetwild_bind()
{
#ifdef PGO_TET_MESHER_HAS_TET_WILD
  return true;
#else
  return false;
#endif
}

// ── Connected components / cleanup ───────────────────────────────────────

std::pair<std::vector<int>, std::vector<int>> triangleComponentIds(const PyTriMeshData &surface)
{
  Mesh::TriMeshGeo mesh(surface.core());
  std::vector<int> counts;
  nb::gil_scoped_release release;
  std::vector<int> ids = Mesh::computeTriangleEdgeComponentIDs(
    BasicAlgorithms::makeArrayRef(mesh.triangles()), &counts);
  return std::make_pair(ids, counts);
}

std::vector<std::vector<int>> connectedComponentsByEdge(const PyTriMeshData &surface)
{
  Mesh::TriMeshGeo mesh(surface.core());
  nb::gil_scoped_release release;
  return Mesh::getConnectedComponentsByEdge(BasicAlgorithms::makeArrayRef(mesh.triangles()));
}

std::vector<std::vector<int>> connectedComponentsByVertex(const PyTriMeshData &surface)
{
  Mesh::TriMeshGeo mesh(surface.core());
  nb::gil_scoped_release release;
  return Mesh::getConnectedComponentsByVertex(BasicAlgorithms::makeArrayRef(mesh.triangles()));
}

PyTriMeshData filterTriMeshComponents(const PyTriMeshData &mesh, int minElements, int keepLargest)
{
  nb::gil_scoped_release release;
  return PyTriMeshData(Mesh::filterMeshComponentsByFace(mesh.core(), minElements, keepLargest));
}

PyTetMeshData filterTetMeshComponents(const PyTetMeshData &mesh, int minElements, int keepLargest)
{
  nb::gil_scoped_release release;
  return PyTetMeshData(Mesh::filterMeshComponentsByFace(mesh.core(), minElements, keepLargest));
}

PyCubicMeshData filterCubicMeshComponents(const PyCubicMeshData &mesh, int minElements, int keepLargest)
{
  nb::gil_scoped_release release;
  return PyCubicMeshData(Mesh::filterMeshComponentsByFace(mesh.core(), minElements, keepLargest));
}

PyTriMeshData getOuterComponent(const PyTriMeshData &surface)
{
  Mesh::TriMeshGeo mesh(surface.core());
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
  return PyTriMeshData(Mesh::removeIsolatedVertices(Mesh::removeTriangles(mesh.ref(), to_remove).ref()).toMeshData());
}

PyTriMeshData surfaceRemoveIsolatedVertices(const PyTriMeshData &surface)
{
  Mesh::TriMeshGeo mesh(surface.core());
  nb::gil_scoped_release release;
  return PyTriMeshData(Mesh::removeIsolatedVertices(mesh.ref()).toMeshData());
}

#ifdef PYPGO_HAS_CGAL
nb::tuple surfaceMergeCloseVertices(const PyTriMeshData &surface, double eps)
{
  Mesh::TriMeshGeo mesh(surface.core());
  CGALInterface::MergeCloseVerticesResult result;
  {
    nb::gil_scoped_release release;
    result = CGALInterface::mergeCloseVertices(mesh, eps);
  }
  return nb::make_tuple(PyTriMeshData(result.mesh.toMeshData()), result.mergedVertices, result.eps);
}

nb::tuple rawSurfaceCleanup(const PyTriMeshData &surface, int expectedComponents,
  double shortEdgeThreshold, int maxPasses, int maxCollapses)
{
  Mesh::TriMeshGeo mesh(surface.core());
  CGALInterface::RawSurfaceCleanupOptions options;
  options.expectedComponents = expectedComponents;
  options.shortEdgeThreshold = shortEdgeThreshold;
  options.maxPasses = maxPasses;
  options.maxCollapses = maxCollapses;

  CGALInterface::RawSurfaceCleanupResult result;
  {
    nb::gil_scoped_release release;
    result = CGALInterface::rawSurfaceCleanup(mesh, options);
  }
  return nb::make_tuple(PyTriMeshData(result.mesh.toMeshData()), rawCleanupReportToDict(result.report));
}
#endif

PyTriMeshData cgalSmoothSurface(const PyTriMeshData &surface, int numIter, double sharpAngle)
{
  Mesh::TriMeshGeo mesh(surface.core());
  nb::gil_scoped_release release;
  return PyTriMeshData(surface_remesh::cgal_smooth(mesh, numIter, sharpAngle).toMeshData());
}

PyTriMeshData cgalIsotropicRemesh(const PyTriMeshData &surface, double targetEdgeLength, int numIter, double sharpAngle)
{
  Mesh::TriMeshGeo mesh(surface.core());
  nb::gil_scoped_release release;
  return PyTriMeshData(surface_remesh::cgal_isotropic_remesh(mesh, targetEdgeLength, numIter, sharpAngle).toMeshData());
}

std::pair<PyTriMeshData, bool> cgalRepairSelfIntersections(const PyTriMeshData &surface, const std::string &method)
{
  Mesh::TriMeshGeo mesh(surface.core());
  nb::gil_scoped_release release;
  auto [result, all_fixed] = surface_remesh::cgal_repair_self_intersections(mesh, method);
  return std::make_pair(PyTriMeshData(result.toMeshData()), all_fixed);
}

PyTriMeshData cgalSimplifySurface(const PyTriMeshData &surface, double targetRatio)
{
  Mesh::TriMeshGeo mesh(surface.core());
  nb::gil_scoped_release release;
  return PyTriMeshData(surface_remesh::cgal_simplify(mesh, targetRatio).toMeshData());
}

bool hasCgalRemesher() { return surface_remesh::has_cgal(); }

}  // namespace pgo
