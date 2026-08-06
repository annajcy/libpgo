#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "../energy/peer.h"
#include "eigen_numpy.h"
#include "solver/newton/NewtonOptimizer.h"
#include "solver/service/optimizationProblem.h"
#include "solver/service/optimizerUtils.h"
#include "solver/service/optimizationResult.h"

#include <memory>
#include <optional>
#include <vector>

namespace nb = nanobind;
namespace NOO = pgo::NonlinearOptimization::Optimization;

class PyOptimizationProblem
{
public:
  PyOptimizationProblem() = default;

  void setObjective(std::shared_ptr<const NO::PotentialEnergy> coreEnergy);

  const NOO::OptimizationProblem &handle() const { return problem_; }
  NOO::OptimizationProblem &handle() { return problem_; }

  void setVariableBounds(
    nb::ndarray<nb::numpy, const double> lower,
    bool hasLower,
    nb::ndarray<nb::numpy, const double> upper,
    bool hasUpper);

  void fixVariables(std::vector<int> dofs, nb::ndarray<nb::numpy, const double> values, int numDofs);

private:
  std::shared_ptr<const NO::PotentialEnergy> objectiveOwner_;
  NOO::OptimizationProblem problem_;
};

// Factory: creates a PyOptimizationProblem with the given core energy.
// The Python side keeps the PyPotentialEnergy peer alive via self.objective.
std::shared_ptr<PyOptimizationProblem> createOptimizationProblem(
  std::shared_ptr<const NO::PotentialEnergy> objective);

// Binding entry point: accepts any PyPotentialEnergy-derived peer by reference
// so nanobind can pass derived types without shared_ptr upcast issues.
std::shared_ptr<PyOptimizationProblem> createOptimizationProblemFromPeer(
  PyPotentialEnergy &objective);

// ── Line-search policy handles ──────────────────────────────────────────────
// Python-facing wrapper around a concrete core NewtonLineSearchPolicy.  Each
// subclass constructs its policy (with that policy's own parameters) and exposes
// the shared handle; PyNewtonOptimizerOptions stores a reference to one of them.

class PyLineSearchPolicy
{
public:
  virtual ~PyLineSearchPolicy() = default;

  const std::shared_ptr<const pgo::NonlinearOptimization::NewtonLineSearchPolicy> &handle() const { return handle_; }

protected:
  std::shared_ptr<const pgo::NonlinearOptimization::NewtonLineSearchPolicy> handle_;
};

class PyGoldenLineSearch final : public PyLineSearchPolicy
{
public:
  PyGoldenLineSearch()
  {
    handle_ = std::make_shared<pgo::NonlinearOptimization::GoldenLineSearchPolicy>();
  }
};

class PyBrentsLineSearch final : public PyLineSearchPolicy
{
public:
  PyBrentsLineSearch()
  {
    handle_ = std::make_shared<pgo::NonlinearOptimization::BrentsLineSearchPolicy>();
  }
};

class PyBacktrackLineSearch final : public PyLineSearchPolicy
{
public:
  PyBacktrackLineSearch(double armijoC, double shrink, double initialAlpha)
  {
    handle_ = std::make_shared<pgo::NonlinearOptimization::BacktrackingLineSearchPolicy>(
      pgo::NonlinearOptimization::BacktrackingLineSearchPolicy::Params{ armijoC, shrink, initialAlpha });
  }
};

class PySimpleLineSearch final : public PyLineSearchPolicy
{
public:
  PySimpleLineSearch(int maxIterations, double shrink)
  {
    handle_ = std::make_shared<pgo::NonlinearOptimization::SimpleLineSearchPolicy>(
      pgo::NonlinearOptimization::SimpleLineSearchPolicy::Params{ shrink, maxIterations });
  }
};

// ── Sparse linear-solver backends ───────────────────────────────────────────
// Python-facing selector for the Newton step's sparse linear solver.  Each
// subclass constructs the concrete C++ selector and exposes the shared handle,
// identical in shape to PyLineSearchPolicy.  PyNewtonOptimizerOptions stores a
// reference to one of them.

class PySparseSolver
{
public:
  virtual ~PySparseSolver() = default;

  const std::shared_ptr<const pgo::NonlinearOptimization::NewtonSparseSolverSelector> &handle() const { return handle_; }

protected:
  std::shared_ptr<const pgo::NonlinearOptimization::NewtonSparseSolverSelector> handle_;
};

class PyAutoSparseSolver final : public PySparseSolver
{
public:
  PyAutoSparseSolver()
  {
    handle_ = std::make_shared<pgo::NonlinearOptimization::AutoSparseSolverSelector>();
  }
};

class PyEigenLDLTSparseSolver final : public PySparseSolver
{
public:
  PyEigenLDLTSparseSolver()
  {
    handle_ = std::make_shared<pgo::NonlinearOptimization::EigenLDLTSparseSolverSelector>();
  }
};

class PyMKLPardisoSparseSolver final : public PySparseSolver
{
public:
  PyMKLPardisoSparseSolver()
  {
    handle_ = std::make_shared<pgo::NonlinearOptimization::MKLPardisoSparseSolverSelector>();
  }
};

class PyOrigPardisoSparseSolver final : public PySparseSolver
{
public:
  PyOrigPardisoSparseSolver()
  {
    handle_ = std::make_shared<pgo::NonlinearOptimization::OrigPardisoSparseSolverSelector>();
  }
};

// ── Damping policy handles ───────────────────────────────────────────────────
// Python-facing wrapper around a concrete core NewtonDampingPolicy.  Each
// subclass constructs its policy with its own parameters and exposes the
// shared handle; PyNewtonOptimizerOptions stores a reference to one of them.

class PyDampingPolicy
{
public:
  virtual ~PyDampingPolicy() = default;

  const std::shared_ptr<const pgo::NonlinearOptimization::NewtonDampingPolicy> &handle() const { return handle_; }

protected:
  std::shared_ptr<const pgo::NonlinearOptimization::NewtonDampingPolicy> handle_;
};

class PyNoDamping final : public PyDampingPolicy
{
public:
  PyNoDamping()
  {
    handle_ = std::make_shared<pgo::NonlinearOptimization::NoDampingPolicy>();
  }
};

class PyFixedDamping final : public PyDampingPolicy
{
public:
  explicit PyFixedDamping(double dampingScale)
  {
    handle_ = std::make_shared<pgo::NonlinearOptimization::FixedDampingPolicy>(
      pgo::NonlinearOptimization::FixedDampingPolicy::Params{ dampingScale });
  }
};

// ── Termination policy handles ────────────────────────────────────────────────
// Python-facing wrapper around a concrete core NewtonTerminationPolicy.

class PyTerminationPolicy
{
public:
  virtual ~PyTerminationPolicy() = default;

  const std::shared_ptr<const pgo::NonlinearOptimization::NewtonTerminationPolicy> &handle() const { return handle_; }

protected:
  std::shared_ptr<const pgo::NonlinearOptimization::NewtonTerminationPolicy> handle_;
};

class PyHybridTermination final : public PyTerminationPolicy
{
public:
  PyHybridTermination(double absTolerance, double relativeTolerance)
  {
    handle_ = std::make_shared<pgo::NonlinearOptimization::HybridNewtonTerminationPolicy>(
      absTolerance, relativeTolerance);
  }
};

class PyAbsoluteTermination final : public PyTerminationPolicy
{
public:
  explicit PyAbsoluteTermination(double absTolerance)
  {
    handle_ = std::make_shared<pgo::NonlinearOptimization::AbsoluteNewtonTerminationPolicy>(
      absTolerance);
  }
};

class PyRelativeTermination final : public PyTerminationPolicy
{
public:
  explicit PyRelativeTermination(double relativeTolerance)
  {
    handle_ = std::make_shared<pgo::NonlinearOptimization::RelativeNewtonTerminationPolicy>(
      relativeTolerance);
  }
};

// ── Newton optimizer options ──────────────────────────────────────────────────

struct PyNewtonOptimizerOptions
{
  int maxIterations = 50;
  // References to concrete subclasses (null => defaults).
  std::shared_ptr<PyLineSearchPolicy> lineSearch;
  std::shared_ptr<PyDampingPolicy> damping;
  std::shared_ptr<PyTerminationPolicy> termination;
  std::shared_ptr<PySparseSolver> sparseSolver;
  int verbose = 0;
};

class PyOptimizer
{
public:
  virtual ~PyOptimizer() = default;
  virtual NOO::Optimizer &asOptimizer() = 0;
  nb::dict solve(const PyOptimizationProblem &problem, nb::ndarray<nb::numpy, const double> x0);
};

class PyNewtonOptimizer final : public PyOptimizer
{
public:
  explicit PyNewtonOptimizer(PyNewtonOptimizerOptions options);
  NOO::Optimizer &asOptimizer() override { return optimizer_; }

private:
  NOO::NewtonOptimizer optimizer_;
};

nb::dict optimizationResultToDict(NOO::OptimizationResult result);
NOO::NewtonOptimizer::Options makeNewtonOptions(const PyNewtonOptimizerOptions &options);
