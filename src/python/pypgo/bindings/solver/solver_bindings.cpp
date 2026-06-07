#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "core.h"

namespace nb = nanobind;

void init_solver_bindings(nb::module_ &m)
{
  // ── PyNewtonOptimizerOptions ──────────────────────────────────────────

  nb::class_<PyNewtonOptimizerOptions>(m, "PyNewtonOptimizerOptions")
    .def(nb::init<>())
    .def_rw("max_iterations", &PyNewtonOptimizerOptions::maxIterations)
    .def_rw("gradient_tolerance", &PyNewtonOptimizerOptions::gradientTolerance)
    .def_rw("damping", &PyNewtonOptimizerOptions::damping)
    .def_rw("line_search", &PyNewtonOptimizerOptions::lineSearch)
    .def_rw("verbose", &PyNewtonOptimizerOptions::verbose)
    .def_rw("sparse_solver_kind", &PyNewtonOptimizerOptions::sparseSolverKind);

  // ── PyOptimizationProblem ─────────────────────────────────────────────

  nb::class_<PyOptimizationProblem>(m, "PyOptimizationProblem")
    .def("set_variable_bounds", &PyOptimizationProblem::setVariableBounds,
      nb::arg("lower"), nb::arg("has_lower"), nb::arg("upper"), nb::arg("has_upper"))
    .def("fix_variables", &PyOptimizationProblem::fixVariables,
      nb::arg("dofs"), nb::arg("values"), nb::arg("num_dofs"));

  // Free-function factory; accepts PyPotentialEnergy& so nanobind
  // can pass any derived peer without shared_ptr upcast issues.
  m.def("_create_optimization_problem", &createOptimizationProblemFromPeer,
    nb::arg("objective"));

  // ── PyOptimizer (abstract base) ──────────────────────────────────────

  nb::class_<PyOptimizer>(m, "PyOptimizer")
    .def("solve", &PyOptimizer::solve, nb::arg("problem"), nb::arg("x0"));

  // ── PyNewtonOptimizer ────────────────────────────────────────────────

  nb::class_<PyNewtonOptimizer, PyOptimizer>(m, "PyNewtonOptimizer")
    .def(nb::init<PyNewtonOptimizerOptions>(), nb::arg("options"));
}
