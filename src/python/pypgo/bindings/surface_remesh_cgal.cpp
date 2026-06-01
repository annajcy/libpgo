#include "surface_remesh_backend.h"
#include "cgalInterface.h"

namespace surface_remesh
{

pgo::Mesh::TriMeshGeo cgal_smooth(const pgo::Mesh::TriMeshGeo &mesh, int num_iter, double sharp_angle)
{
    return pgo::CGALInterface::smoothMesh(mesh, num_iter, sharp_angle);
}

pgo::Mesh::TriMeshGeo cgal_isotropic_remesh(const pgo::Mesh::TriMeshGeo &mesh, double target_edge_length, int num_iter, double sharp_angle)
{
    return pgo::CGALInterface::isotropicRemeshing(mesh, target_edge_length, num_iter, sharp_angle);
}

std::pair<pgo::Mesh::TriMeshGeo, bool> cgal_repair_self_intersections(const pgo::Mesh::TriMeshGeo &mesh, const std::string &method)
{
    bool all_fixed = false;
    pgo::Mesh::TriMeshGeo result = pgo::CGALInterface::repairSelfIntersections(mesh, method, &all_fixed);
    return { std::move(result), all_fixed };
}

pgo::Mesh::TriMeshGeo cgal_simplify(const pgo::Mesh::TriMeshGeo &mesh, double target_ratio)
{
    return pgo::CGALInterface::simplifyMeshGH(mesh, "ptri", target_ratio);
}

bool has_cgal()
{
    return true;
}

}  // namespace surface_remesh
