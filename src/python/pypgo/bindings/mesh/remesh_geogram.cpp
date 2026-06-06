#include "remesh_backend.h"
#include "geogramInterface.h"

#include <geogram/basic/process.h>

#include <mutex>

namespace surface_remesh
{

static void init_geogram_once()
{
    static std::once_flag flag;
    std::call_once(flag, []() {
        pgo::GeogramInterface::initGEO();
    });
}

pgo::Mesh::TriMeshGeo geogram_remesh(const pgo::Mesh::TriMeshGeo &mesh, int target_num_vertices,
    double size_factor, double anisotropy, int num_threads)
{
    init_geogram_once();
    if (num_threads > 0)
        GEO::Process::set_max_threads(static_cast<GEO::index_t>(num_threads));
    return pgo::GeogramInterface::remesh(mesh, target_num_vertices, size_factor, anisotropy);
}

bool has_geogram()
{
    return true;
}

}  // namespace surface_remesh
