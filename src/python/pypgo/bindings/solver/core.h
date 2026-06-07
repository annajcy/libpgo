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
#include <string>
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

struct PyNewtonOptimizerOptions
{
  int maxIterations = 50;
  double gradientTolerance = 1e-6;
  bool damping = true;
  std::string lineSearch = "backtrack";
  int verbose = 0;
  int sparseSolverKind = 0;
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
