#pragma once

#include "constraints/constraintFunctions.h"
#include "solver/newton/newtonLineSearchPolicy.h"
#include "solver/newton/newtonSparseSolverBackend.h"
#include "potentialEnergy.h"
#include "solver/common/solverResult.h"

#include <optional>
#include <variant>
#include <vector>

namespace pgo::NonlinearOptimization
{

using LineSearchMethod = NewtonLineSearchKind;

struct SolverControl
{
  int maxIterations = 50;
  double tolerance = 1e-6;
  int verbose = 0;
};

struct NewtonOptions
{
  SolverControl control;
  bool damping = true;
  LineSearchMethod lineSearch = LineSearchMethod::Backtrack;
  NewtonSparseSolverOptions sparseSolver;
};

// Multi-backend solver options.   Each alternative MUST expose a `control`
// member of type `SolverControl`.   Use `getSolverControl(opts)` to read it
// generically; use `std::get<NewtonOptions>(opts)` when you know the backend.
//
// Add IpoptOptions / KnitroOptions here when the solver service grows them.
using SolverOptions = std::variant<NewtonOptions>;

inline const SolverControl &getSolverControl(const SolverOptions &opts)
{
  return std::visit([](const auto &o) -> const SolverControl & { return o.control; }, opts);
}

struct FixedVariables
{
  std::vector<int> dofs;
  std::optional<EigenSupport::VXd> values;
};

struct BoxBounds
{
  EigenSupport::VXd lower;
  EigenSupport::VXd upper;
};

struct NonlinearConstraints
{
  ConstraintFunctions_const_p functions;
  EigenSupport::VXd lower;
  EigenSupport::VXd upper;
};

struct OptimizationProblem
{
  PotentialEnergy_const_p energy;
  std::optional<FixedVariables> fixedVariables;
  std::optional<BoxBounds> bounds;
  std::optional<NonlinearConstraints> constraints;
};

struct OptimizationResult
{
  SolverResult solver;
  EigenSupport::VXd x;
  bool hasFinalObjective = false;
  double finalObjective = 0.0;
  std::optional<EigenSupport::VXd> lambda;
  std::optional<EigenSupport::VXd> constraintValues;
};

OptimizationResult minimize(
  const OptimizationProblem &problem,
  EigenSupport::ConstRefVecXd x0,
  const NewtonOptions &options);

}  // namespace pgo::NonlinearOptimization
