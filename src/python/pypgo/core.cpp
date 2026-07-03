#include "core.h"

namespace pgo
{

nanobind::dict buildInfo()
{
  nanobind::dict info;
  info["module"] = "pypgo._core";
  info["binding"] = "nanobind";
  info["mesh_geo"] = true;

  // Solver-backend capability flags. These let callers (and CI) verify a binary
  // is actually the build they expect — e.g. that the MKL package genuinely has
  // the MKL Pardiso Newton backend wired up rather than silently falling back.
  // The conditions mirror the gating in newtonSparseSolverBackend.cpp.
#if defined(PGO_HAS_MKL)
  info["mkl"] = true;
#else
  info["mkl"] = false;
#endif

#if defined(PGO_HAS_ORIG_PARDISO)
  info["orig_pardiso"] = true;
#else
  info["orig_pardiso"] = false;
#endif

  nanobind::list backends;
  backends.append("eigen_ldlt");  // always available
#if defined(PGO_HAS_MKL) && !defined(PGO_HAS_ORIG_PARDISO)
  backends.append("mkl_pardiso");
#endif
#if defined(PGO_HAS_ORIG_PARDISO)
  backends.append("orig_pardiso");
#endif
  info["solver_backends"] = backends;

  return info;
}

}  // namespace pgo
