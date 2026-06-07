#include "core.h"

#include "eigen_numpy.h"

#include "dynamicStepOptions.h"
#include "dynamicStepService.h"

#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

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
