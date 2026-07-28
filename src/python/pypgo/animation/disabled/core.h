#pragma once

#include <nanobind/nanobind.h>

namespace pgo
{

bool has_animation_io();
bool has_stress_vdb_export();
void dump_abc_unavailable(const nanobind::object &filename,
  const nanobind::object &name,
  const nanobind::object &restPositions,
  const nanobind::object &displacements,
  const nanobind::object &triangles,
  const nanobind::object &fps);

}  // namespace pgo
