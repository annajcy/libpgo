#include "core.h"

namespace pgo
{

nanobind::dict buildInfo()
{
  nanobind::dict info;
  info["module"] = "pypgo._core";
  info["binding"] = "nanobind";
  info["mesh_geo"] = true;
  return info;
}

}  // namespace pgo
