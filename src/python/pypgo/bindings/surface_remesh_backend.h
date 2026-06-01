#pragma once

#include "triMeshGeo.h"

#include <string>
#include <utility>

namespace surface_remesh
{

// CGAL-backed operations
pgo::Mesh::TriMeshGeo cgal_smooth(const pgo::Mesh::TriMeshGeo &mesh, int num_iter, double sharp_angle);
pgo::Mesh::TriMeshGeo cgal_isotropic_remesh(const pgo::Mesh::TriMeshGeo &mesh, double target_edge_length, int num_iter, double sharp_angle);
std::pair<pgo::Mesh::TriMeshGeo, bool> cgal_repair_self_intersections(const pgo::Mesh::TriMeshGeo &mesh, const std::string &method);
pgo::Mesh::TriMeshGeo cgal_simplify(const pgo::Mesh::TriMeshGeo &mesh, double target_ratio);
bool has_cgal();

// Geogram-backed operations
pgo::Mesh::TriMeshGeo geogram_remesh(const pgo::Mesh::TriMeshGeo &mesh, int target_num_vertices, double size_factor, double anisotropy);
bool has_geogram();

}  // namespace surface_remesh
