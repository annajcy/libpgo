#include "core.h"

#include <stdexcept>

namespace pgo
{

bool has_animation_io()
{
  return false;
}

bool has_stress_vdb_export()
{
  return false;
}

void dump_abc_unavailable(const nanobind::object &,
  const nanobind::object &,
  const nanobind::object &,
  const nanobind::object &,
  const nanobind::object &)
{
  throw std::runtime_error("Animation IO (Alembic) is not available in this build.");
}

}  // namespace pgo
