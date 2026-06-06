#include "remesh_backend.h"

#include <stdexcept>

namespace surface_remesh
{

pgo::Mesh::TriMeshGeo geogram_remesh(const pgo::Mesh::TriMeshGeo &, int, double, double, int)
{
    throw std::runtime_error("Geogram surface remesher is not available in this build.");
}

bool has_geogram()
{
    return false;
}

}  // namespace surface_remesh
