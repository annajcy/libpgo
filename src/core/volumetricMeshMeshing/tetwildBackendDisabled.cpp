#include "tetMesherBackend.h"

#include <stdexcept>

namespace pgo::tet_mesher
{

std::unique_ptr<pgo::VolumetricMeshes::TetMesh> generateTetwildMesh(const TetwildOptions &)
{
  throw std::runtime_error("tetwild backend is not enabled. Reconfigure with -DPGO_TET_MESHER_USE_TET_WILD=ON.");
}

}  // namespace pgo::tet_mesher
