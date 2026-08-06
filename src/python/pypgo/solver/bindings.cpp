#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "eigen_numpy.h"
#include "sparse/core.h"

#include "core.h"

#include <stdexcept>

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

  // One-shot sparse linear solve: builds the backend for the matrix pattern,
  // factorizes, and solves A x = rhs.  A fresh backend is created per call, so
  // the handle stays immutable and thread-safe (no shared mutable state).
  nb::class_<PySparseSolver>(m, "PySparseSolver")
    .def("solve", [](const PySparseSolver &self, const PySparseMatrix &A,
                     nb::ndarray<nb::numpy, const double> rhs) {
        const auto &M = A.eigenMatrix();
        if (M.rows() != M.cols()) {
          throw nb::value_error("solve: sparse matrix must be square");
        }
        pgo::EigenSupport::VXd rhsVec = pgo::python::ndarrayToVectorXd(rhs);
        if (static_cast<size_t>(rhsVec.size()) != static_cast<size_t>(M.rows())) {
          throw nb::value_error("solve: rhs size must match matrix rows");
        }

        auto backend = self.handle()->build(M);
        if (!backend->factorize(M)) {
          throw std::runtime_error(
            "sparse solve failed: backend '" + std::string(backend->name()) +
            "' factorization did not succeed (matrix may be singular)");
        }
        pgo::EigenSupport::VXd x = rhsVec;
        if (!backend->solve(M, x.data(), rhsVec.data())) {
          throw std::runtime_error(
            "sparse solve failed: backend '" + std::string(backend->name()) +
            "' triangular solve did not succeed");
        }
        return pgo::python::vectorXdToNdarray(std::move(x));
      },
      nb::arg("matrix"), nb::arg("rhs"));

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

  nb::class_<PyHybridTermination, PyTerminationPolicy>(m, "PyHybridTermination")
    .def(nb::init<double, double>(),
      nb::arg("abs_tolerance"),
      nb::arg("relative_tolerance"));

  nb::class_<PyAbsoluteTermination, PyTerminationPolicy>(m, "PyAbsoluteTermination")
    .def(nb::init<double>(), nb::arg("abs_tolerance"));

  nb::class_<PyRelativeTermination, PyTerminationPolicy>(m, "PyRelativeTermination")
    .def(nb::init<double>(), nb::arg("relative_tolerance"));

  // ── PyNewtonOptimizerOptions ──────────────────────────────────────────

  nb::class_<PyNewtonOptimizerOptions>(m, "PyNewtonOptimizerOptions")
    .def(nb::init<>())
    .def_rw("max_iterations", &PyNewtonOptimizerOptions::maxIterations)
    .def_rw("line_search", &PyNewtonOptimizerOptions::lineSearch)
    .def_rw("damping", &PyNewtonOptimizerOptions::damping)
    .def_rw("termination", &PyNewtonOptimizerOptions::termination)
    .def_rw("sparse_solver", &PyNewtonOptimizerOptions::sparseSolver)
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
