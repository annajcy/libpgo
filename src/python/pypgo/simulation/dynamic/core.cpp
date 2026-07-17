#include "core.h"

#include "eigen_numpy.h"

#include "dynamicStepOptions.h"
#include "dynamicStepService.h"

#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <cmath>
#include <stdexcept>
#include <utility>

using namespace pgo;
namespace ES = pgo::EigenSupport;
namespace NO = pgo::NonlinearOptimization;
namespace SIM = pgo::Simulation;

namespace
{

ES::SpMatD buildSparse(int n, const std::vector<int> &rows, const std::vector<int> &cols, const std::vector<double> &vals)
{
  if (rows.size() != cols.size() || rows.size() != vals.size())
    throw nb::value_error("mass COO arrays must have matching lengths");
  std::vector<ES::TripletD> t;
  t.reserve(vals.size());
  for (size_t k = 0; k < vals.size(); k++)
    t.emplace_back(rows[k], cols[k], vals[k]);
  ES::SpMatD m(n, n);
  m.setFromTriplets(t.begin(), t.end());
  return m;
}

nb::object finiteOrNone(double value)
{
  return std::isfinite(value) ? nb::object(nb::float_(value)) : nb::none();
}

nb::dict newtonIterationTraceToDict(const NO::NewtonIterationTrace &trace)
{
  nb::dict out;
  out["iteration"] = trace.iteration;
  out["energy_before"] = finiteOrNone(trace.energyBefore);
  out["energy_after"] = finiteOrNone(trace.energyAfter);
  out["energy_delta"] = finiteOrNone(trace.energyDelta);
  out["grad_norm_before"] = finiteOrNone(trace.gradNormBefore);
  out["grad_norm_after"] = finiteOrNone(trace.gradNormAfter);
  out["grad_max_before"] = finiteOrNone(trace.gradMaxBefore);
  out["grad_max_after"] = finiteOrNone(trace.gradMaxAfter);
  out["grad_reduction_ratio"] = finiteOrNone(trace.gradReductionRatio);
  out["grad_dot_dx"] = finiteOrNone(trace.gradDotDx);
  out["raw_step_norm"] = finiteOrNone(trace.rawStepNorm);
  out["raw_step_max_norm"] = finiteOrNone(trace.rawStepMaxNorm);
  out["accepted_alpha"] = finiteOrNone(trace.acceptedAlpha);
  out["accepted_step_norm"] = finiteOrNone(trace.acceptedStepNorm);
  out["accepted_step_max_norm"] = finiteOrNone(trace.acceptedStepMaxNorm);
  out["line_search_iterations"] = trace.lineSearchIterations;
  out["line_search_status"] = trace.lineSearchStatus;
  out["min_feasible_alpha"] = finiteOrNone(trace.minFeasibleAlpha);
  out["feasible_alpha"] = finiteOrNone(trace.feasibleAlpha);
  out["line_search_alpha"] = finiteOrNone(trace.lineSearchAlpha);
  out["contact_clamp_count"] = trace.contactClampCount;
  out["material_clamp_count"] = trace.materialClampCount;
  out["damping_value"] = finiteOrNone(trace.dampingValue);
  out["hessian_rows"] = trace.hessianRows;
  out["hessian_cols"] = trace.hessianCols;
  out["hessian_nnz"] = trace.hessianNnz;
  out["symbolic_rebuilt"] = trace.symbolicRebuilt;
  out["evaluate_current_state_seconds"] = finiteOrNone(trace.evaluateCurrentStateSeconds);
  out["func_grad_hessian_seconds"] = finiteOrNone(trace.funcGradHessianSeconds);
  out["prepare_reduced_system_seconds"] = finiteOrNone(trace.prepareReducedSystemSeconds);
  out["ensure_linear_solver_seconds"] = finiteOrNone(trace.ensureLinearSolverSeconds);
  out["symbolic_analyze_seconds"] = finiteOrNone(trace.symbolicAnalyzeSeconds);
  out["factorize_seconds"] = finiteOrNone(trace.factorizeSeconds);
  out["solve_seconds"] = finiteOrNone(trace.solveSeconds);
  out["expand_reduced_step_seconds"] = finiteOrNone(trace.expandReducedStepSeconds);
  out["line_search_seconds"] = finiteOrNone(trace.lineSearchSeconds);
  out["iteration_wall_seconds"] = finiteOrNone(trace.iterationWallSeconds);
  out["low_value"] = trace.lowValue;
  return out;
}

nb::dict solverResultToDict(const NO::SolverResult &r)
{
  nb::dict out;
  out["status"] = static_cast<int>(r.status);
  out["converged"] = r.converged();
  out["iterations"] = r.iterations;
  out["raw_status_code"] = r.rawStatusCode;
  out["final_gradient_norm"] = r.hasFinalGradientStats ? nb::object(nb::float_(r.finalGradientNorm)) : nb::none();
  out["final_gradient_max_norm"] = r.hasFinalGradientStats ? nb::object(nb::float_(r.finalGradientMaxNorm)) : nb::none();
  nb::dict diag;
  diag["min_feasible_alpha"] = r.diagnostics.minFeasibleAlpha;
  diag["min_line_search_alpha"] = r.diagnostics.minLineSearchAlpha;
  diag["min_effective_alpha"] = r.diagnostics.minEffectiveAlpha;
  diag["material_clamp_count"] = r.diagnostics.clampCounts[static_cast<int>(NO::StepSource::Material)];
  diag["contact_clamp_count"] = r.diagnostics.clampCounts[static_cast<int>(NO::StepSource::Contact)];
  diag["final_gradient_norm"] = r.diagnostics.hasFinalGradientStats ? nb::object(nb::float_(r.diagnostics.finalGradientNorm)) : nb::none();
  diag["final_gradient_max_norm"] = r.diagnostics.hasFinalGradientStats ? nb::object(nb::float_(r.diagnostics.finalGradientMaxNorm)) : nb::none();
  diag["last_grad_dot_dx"] = r.diagnostics.lastGradDotDx;
  diag["last_raw_step_max_norm"] = r.diagnostics.lastRawStepMaxNorm;
  diag["last_raw_step_norm"] = r.diagnostics.lastRawStepNorm;
  diag["last_accepted_step_max_norm"] = r.diagnostics.lastAcceptedStepMaxNorm;
  diag["last_line_search_iterations"] = r.diagnostics.lastLineSearchIterations;
  diag["max_line_search_iterations"] = r.diagnostics.maxLineSearchIterations;
  diag["total_line_search_iterations"] = r.diagnostics.totalLineSearchIterations;
  diag["last_current_energy"] = r.diagnostics.lastCurrentEnergy;
  diag["last_accepted_energy"] = r.diagnostics.lastAcceptedEnergy;
  diag["last_energy_delta"] = r.diagnostics.lastEnergyDelta;
  diag["last_damping_value"] = r.diagnostics.lastDampingValue;
  diag["last_active_system_nnz"] = r.diagnostics.lastActiveSystemNnz;
  diag["last_active_system_rows"] = r.diagnostics.lastActiveSystemRows;
  diag["last_active_system_cols"] = r.diagnostics.lastActiveSystemCols;
  diag["linear_solver_symbolic_rebuild_count"] = r.diagnostics.linearSolverSymbolicRebuildCount;
  diag["linear_solver_symbolic_reuse_count"] = r.diagnostics.linearSolverSymbolicReuseCount;
  diag["newton_convergence_reason"] = static_cast<int>(r.diagnostics.newtonConvergenceReason);
  diag["newton_convergence_reason_name"] = NO::newtonConvergenceReasonName(r.diagnostics.newtonConvergenceReason);
  diag["newton_convergence_threshold"] = finiteOrNone(r.diagnostics.newtonConvergenceThreshold);
  diag["newton_low_value_iteration_count"] = r.diagnostics.newtonLowValueIterationCount;
  diag["newton_tiny_step_count"] = r.diagnostics.newtonTinyStepCount;
  diag["newton_small_alpha_count"] = r.diagnostics.newtonSmallAlphaCount;
  diag["newton_symbolic_rebuild_count"] = r.diagnostics.newtonSymbolicRebuildCount;
  diag["newton_worst_progress_iteration"] = r.diagnostics.newtonWorstProgressIteration;
  diag["newton_worst_progress_ratio"] = r.diagnostics.newtonWorstProgressRatio;
  diag["optimizer_preparation_seconds"] = r.diagnostics.optimizerPreparationSeconds;
  diag["newton_solver_setup_seconds"] = r.diagnostics.newtonSolverSetupSeconds;
  diag["initial_hessian_seconds"] = r.diagnostics.initialHessianSeconds;
  diag["initial_reduced_system_seconds"] = r.diagnostics.initialReducedSystemSeconds;
  diag["initial_symbolic_analyze_seconds"] = r.diagnostics.initialSymbolicAnalyzeSeconds;
  diag["newton_solve_seconds"] = r.diagnostics.newtonSolveSeconds;
  diag["newton_total_iteration_seconds"] = r.diagnostics.newtonTotalIterationSeconds;
  diag["newton_total_evaluate_current_state_seconds"] = r.diagnostics.newtonTotalEvaluateCurrentStateSeconds;
  diag["newton_total_func_grad_hessian_seconds"] = r.diagnostics.newtonTotalFuncGradHessianSeconds;
  diag["newton_total_prepare_reduced_system_seconds"] = r.diagnostics.newtonTotalPrepareReducedSystemSeconds;
  diag["newton_total_ensure_linear_solver_seconds"] = r.diagnostics.newtonTotalEnsureLinearSolverSeconds;
  diag["newton_total_symbolic_analyze_seconds"] = r.diagnostics.newtonTotalSymbolicAnalyzeSeconds;
  diag["newton_total_factorize_seconds"] = r.diagnostics.newtonTotalFactorizeSeconds;
  diag["newton_total_solve_seconds"] = r.diagnostics.newtonTotalSolveSeconds;
  diag["newton_total_expand_reduced_step_seconds"] = r.diagnostics.newtonTotalExpandReducedStepSeconds;
  diag["newton_total_line_search_seconds"] = r.diagnostics.newtonTotalLineSearchSeconds;
  diag["final_objective_seconds"] = r.diagnostics.finalObjectiveSeconds;
  diag["linear_solver_cleanup_seconds"] = r.diagnostics.linearSolverCleanupSeconds;
  diag["optimizer_total_seconds"] = r.diagnostics.optimizerTotalSeconds;
  nb::list iterations;
  for (const NO::NewtonIterationTrace &trace : r.diagnostics.newtonIterations)
    iterations.append(newtonIterationTraceToDict(trace));
  diag["newton_iterations"] = iterations;
  out["diagnostics"] = diag;
  return out;
}

}  // namespace

PyTRBDF2DynamicStepper::PyTRBDF2DynamicStepper(double gamma)
  : gamma_(gamma)
{
  if (!(gamma > 0.0 && gamma <= 1.0))
    throw nb::value_error("gamma must be in (0, 1]");
}

PyDynamicSimulation::PyDynamicSimulation(
  int numDofs,
  std::vector<int> massRows, std::vector<int> massCols, std::vector<double> massVals,
  std::shared_ptr<PyPotentialEnergy> energy,
  double massDamping, double stiffnessDamping,
  nb::ndarray<nb::numpy, const double> displacement,
  nb::ndarray<nb::numpy, const double> velocity,
  nb::ndarray<nb::numpy, const double> acceleration,
  std::uint64_t timestepId,
  double time,
  double timestep,
  std::shared_ptr<PyDynamicStepper> integrator,
  std::vector<int> fixedDofs)
  : n_(numDofs)
{
  if (!integrator)
    throw nb::value_error("integrator must be a DynamicStepper");

  SIM::DynamicProblem problem;
  problem.mass = buildSparse(numDofs, massRows, massCols, massVals);
  problem.timestep = timestep;
  problem.fixedDofs = std::move(fixedDofs);

  if (energy) {
    SIM::ImplicitModelTerm term;
    // The stepper needs non-const access (StepAwareEnergy::beginStep); the
    // underlying energy objects are constructed mutable, so this cast is safe.
    term.energy = std::const_pointer_cast<NO::PotentialEnergy>(energy->potentialEnergyHandle());
    term.stiffnessDamping = stiffnessDamping;
    term.massDamping = massDamping;
    problem.persistentTerms.push_back(std::move(term));
  }

  state_.displacement = python::ndarrayToVectorXd(displacement);
  state_.velocity = python::ndarrayToVectorXd(velocity);
  state_.acceleration = python::ndarrayToVectorXd(acceleration);
  state_.timestepId = timestepId;
  state_.time = time;

  stepper_ = SIM::makeDynamicStepper(integrator->kind(), std::move(problem), integrator->trbdf2Gamma());
}

nb::dict PyDynamicSimulation::step(nb::ndarray<nb::numpy, const double> externalForce,
  nb::ndarray<nb::numpy, const double> fixedValues, bool hasFixedValues,
  PyOptimizer &optimizer)
{
  SIM::DynamicStepRequest request;
  request.externalForce = python::ndarrayToVectorXd(externalForce);
  if (hasFixedValues)
    request.fixedValues = python::ndarrayToVectorXd(fixedValues);

  SIM::DynamicStepResult result;
  {
    nb::gil_scoped_release release;
    result = stepper_->step(state_, request, optimizer.asOptimizer());
  }
  state_ = result.state;

  nb::dict out;
  out["displacement"] = python::vectorXdToNdarray(ES::VXd(state_.displacement));
  out["velocity"] = python::vectorXdToNdarray(ES::VXd(state_.velocity));
  out["acceleration"] = python::vectorXdToNdarray(ES::VXd(state_.acceleration));
  out["timestep_id"] = static_cast<std::uint64_t>(state_.timestepId);
  out["time"] = state_.time;
  out["accepted"] = result.accepted;
  out["solver"] = solverResultToDict(result.solver);

  nb::list stages;
  for (const NO::SolverResult &s : result.stageResults)
    stages.append(solverResultToDict(s));
  out["stage_results"] = stages;
  return out;
}

nb::ndarray<nb::numpy, double> PyDynamicSimulation::displacement() const { return python::vectorXdToNdarray(ES::VXd(state_.displacement)); }
nb::ndarray<nb::numpy, double> PyDynamicSimulation::velocity() const { return python::vectorXdToNdarray(ES::VXd(state_.velocity)); }
nb::ndarray<nb::numpy, double> PyDynamicSimulation::acceleration() const { return python::vectorXdToNdarray(ES::VXd(state_.acceleration)); }
std::uint64_t PyDynamicSimulation::timestepId() const { return state_.timestepId; }
double PyDynamicSimulation::time() const { return state_.time; }
int PyDynamicSimulation::numDofs() const { return n_; }
