#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "core.h"

namespace nb = nanobind;

void init_solver_bindings(nb::module_ &m)
{
  // ── Line-search policies ──────────────────────────────────────────────

  nb::class_<PyLineSearchPolicy>(m, "PyLineSearchPolicy");

  nb::class_<PyGoldenLineSearch, PyLineSearchPolicy>(m, "PyGoldenLineSearch")
    .def(nb::init<>());

  nb::class_<PyBrentsLineSearch, PyLineSearchPolicy>(m, "PyBrentsLineSearch")
    .def(nb::init<>());

  nb::class_<PyBacktrackLineSearch, PyLineSearchPolicy>(m, "PyBacktrackLineSearch")
    .def(nb::init<double, double, double>(),
      nb::arg("armijo_c"), nb::arg("shrink"), nb::arg("initial_alpha"));

  nb::class_<PySimpleLineSearch, PyLineSearchPolicy>(m, "PySimpleLineSearch")
    .def(nb::init<int, double>(), nb::arg("max_iterations"), nb::arg("shrink"));

  // ── Sparse linear solvers ─────────────────────────────────────────────

  nb::class_<PySparseSolver>(m, "PySparseSolver");

  nb::class_<PyAutoSparseSolver, PySparseSolver>(m, "PyAutoSparseSolver")
    .def(nb::init<>());

  nb::class_<PyEigenLDLTSparseSolver, PySparseSolver>(m, "PyEigenLDLTSparseSolver")
    .def(nb::init<>());

  nb::class_<PyMKLPardisoSparseSolver, PySparseSolver>(m, "PyMKLPardisoSparseSolver")
    .def(nb::init<>());

  nb::class_<PyOrigPardisoSparseSolver, PySparseSolver>(m, "PyOrigPardisoSparseSolver")
    .def(nb::init<>());

  // ── Damping policies ──────────────────────────────────────────────────

  nb::class_<PyDampingPolicy>(m, "PyDampingPolicy");

  nb::class_<PyNoDamping, PyDampingPolicy>(m, "PyNoDamping")
    .def(nb::init<>());

  nb::class_<PyFixedDamping, PyDampingPolicy>(m, "PyFixedDamping")
    .def(nb::init<double>(), nb::arg("damping_scale"));

  // ── Termination policies ──────────────────────────────────────────────

  nb::class_<PyTerminationPolicy>(m, "PyTerminationPolicy");

  nb::class_<PyFixedTermination, PyTerminationPolicy>(m, "PyFixedTermination")
    .def(nb::init<>());

  // ── PyNewtonOptimizerOptions ──────────────────────────────────────────

  nb::class_<PyNewtonOptimizerOptions>(m, "PyNewtonOptimizerOptions")
    .def(nb::init<>())
    .def_rw("max_iterations", &PyNewtonOptimizerOptions::maxIterations)
    .def_rw("gradient_tolerance", &PyNewtonOptimizerOptions::gradientTolerance)
    .def_rw("line_search", &PyNewtonOptimizerOptions::lineSearch)
    .def_rw("damping", &PyNewtonOptimizerOptions::damping)
    .def_rw("termination", &PyNewtonOptimizerOptions::termination)
    .def_rw("sparse_solver", &PyNewtonOptimizerOptions::sparseSolver)
    .def("set_threading", &PyNewtonOptimizerOptions::setThreading,
      nb::arg("evaluation"), nb::arg("linear_solver"))
    .def_rw("verbose", &PyNewtonOptimizerOptions::verbose);

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
