#include "remesh_backend.h"

#include <stdexcept>

namespace surface_remesh
{

static void throw_cgal_unavailable()
{
    throw std::runtime_error("CGAL surface remesher is not available in this build.");
}

pgo::Mesh::TriMeshGeo cgal_smooth(const pgo::Mesh::TriMeshGeo &, int, double)
{
    throw_cgal_unavailable();
}

pgo::Mesh::TriMeshGeo cgal_isotropic_remesh(const pgo::Mesh::TriMeshGeo &, double, int, double)
{
    throw_cgal_unavailable();
}

std::pair<pgo::Mesh::TriMeshGeo, bool> cgal_repair_self_intersections(const pgo::Mesh::TriMeshGeo &, const std::string &)
{
    throw_cgal_unavailable();
}

pgo::Mesh::TriMeshGeo cgal_simplify(const pgo::Mesh::TriMeshGeo &, double)
{
    throw_cgal_unavailable();
}

bool has_cgal()
{
    return false;
}

}  // namespace surface_remesh
