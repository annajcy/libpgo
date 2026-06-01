#include "surface_remesh_backend.h"
#include "geogramInterface.h"

namespace surface_remesh
{

pgo::Mesh::TriMeshGeo geogram_remesh(const pgo::Mesh::TriMeshGeo &mesh, int target_num_vertices, double size_factor, double anisotropy)
{
    pgo::GeogramInterface::initGEO();
    return pgo::GeogramInterface::remesh(mesh, target_num_vertices, size_factor, anisotropy);
}

bool has_geogram()
{
    return true;
}

}  // namespace surface_remesh
