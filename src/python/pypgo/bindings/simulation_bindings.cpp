#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/optional.h>

#include "eigen_numpy.h"
#include "energy_core.h"

#include "dynamicState.h"
#include "dynamicStepOptions.h"
#include "dynamicStepper.h"
#include "dynamicStepService.h"

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace nb = nanobind;
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

SIM::TimeIntegratorKind parseIntegrator(const std::string &name)
{
  if (name == "implicit_euler")
    return SIM::TimeIntegratorKind::ImplicitEuler;
  if (name == "trbdf2")
    return SIM::TimeIntegratorKind::TRBDF2;
  throw nb::value_error("unknown integrator; expected 'implicit_euler' or 'trbdf2'");
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

// Low-level dynamic simulation: owns a DynamicStepper plus the current state.
class PyDynamicSimulation
{
public:
  PyDynamicSimulation(
    int numDofs,
    std::vector<int> massRows, std::vector<int> massCols, std::vector<double> massVals,
    std::shared_ptr<PyPotentialEnergy> energy,
    double massDamping, double stiffnessDamping,
    nb::ndarray<nb::numpy, const double> displacement,
    nb::ndarray<nb::numpy, const double> velocity,
    nb::ndarray<nb::numpy, const double> acceleration,
    double timestep,
    const std::string &integrator,
    std::vector<int> fixedDofs,
    int maxIter, double tol, int verbose,
    int sparseSolverKind,
    double gamma)
    : n_(numDofs)
  {
    SIM::DynamicProblem problem;
    problem.mass = buildSparse(numDofs, massRows, massCols, massVals);
    problem.timestep = timestep;
    problem.fixedDofs = std::move(fixedDofs);
    auto &no = std::get<NO::NewtonOptions>(problem.solver);
    no.control.maxIterations = maxIter;
    no.control.tolerance = tol;
    no.control.verbose = verbose;
    no.sparseSolver.kind = static_cast<NO::NewtonSparseSolverKind>(sparseSolverKind);

    if (energy) {
      SIM::ImplicitModelTerm term;
      // The stepper needs non-const access (StepAwareEnergy::beginStep); the
      // underlying energy objects are constructed mutable, so this cast is safe.
      term.energy = std::const_pointer_cast<NO::PotentialEnergy>(energy->handle_);
      term.stiffnessDamping = stiffnessDamping;
      term.massDamping = massDamping;
      problem.persistentTerms.push_back(std::move(term));
    }

    state_.displacement = python::ndarrayToVectorXd(displacement);
    state_.velocity = python::ndarrayToVectorXd(velocity);
    state_.acceleration = python::ndarrayToVectorXd(acceleration);

    stepper_ = SIM::makeDynamicStepper(parseIntegrator(integrator), std::move(problem), gamma);
  }

  nb::dict step(nb::ndarray<nb::numpy, const double> externalForce,
    nb::ndarray<nb::numpy, const double> fixedValues, bool hasFixedValues)
  {
    SIM::DynamicStepRequest request;
    request.externalForce = python::ndarrayToVectorXd(externalForce);
    if (hasFixedValues)
      request.fixedValues = python::ndarrayToVectorXd(fixedValues);

    SIM::DynamicStepResult result;
    {
      nb::gil_scoped_release release;
      result = stepper_->step(state_, request);
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

  nb::ndarray<nb::numpy, double> displacement() const { return python::vectorXdToNdarray(ES::VXd(state_.displacement)); }
  nb::ndarray<nb::numpy, double> velocity() const { return python::vectorXdToNdarray(ES::VXd(state_.velocity)); }
  nb::ndarray<nb::numpy, double> acceleration() const { return python::vectorXdToNdarray(ES::VXd(state_.acceleration)); }
  std::uint64_t timestepId() const { return state_.timestepId; }
  double time() const { return state_.time; }
  int numDofs() const { return n_; }

private:
  int n_;
  SIM::DynamicState state_;
  std::unique_ptr<SIM::DynamicStepper> stepper_;
};

}  // namespace

void init_simulation_bindings(nb::module_ &m)
{
  nb::class_<PyDynamicSimulation>(m, "PyDynamicSimulation")
    .def(nb::init<int, std::vector<int>, std::vector<int>, std::vector<double>,
           std::shared_ptr<PyPotentialEnergy>, double, double,
           nb::ndarray<nb::numpy, const double>, nb::ndarray<nb::numpy, const double>, nb::ndarray<nb::numpy, const double>,
           double, const std::string &, std::vector<int>, int, double, int, int, double>(),
      nb::arg("num_dofs"),
      nb::arg("mass_rows"), nb::arg("mass_cols"), nb::arg("mass_vals"),
      nb::arg("energy").none(),
      nb::arg("mass_damping"), nb::arg("stiffness_damping"),
      nb::arg("displacement"), nb::arg("velocity"), nb::arg("acceleration"),
      nb::arg("timestep"), nb::arg("integrator"), nb::arg("fixed_dofs"),
      nb::arg("max_iter"), nb::arg("tol"), nb::arg("verbose"), nb::arg("sparse_solver_kind"), nb::arg("gamma"))
    .def("step", &PyDynamicSimulation::step,
      nb::arg("external_force"), nb::arg("fixed_values"), nb::arg("has_fixed_values"))
    .def_prop_ro("displacement", &PyDynamicSimulation::displacement, nb::rv_policy::move)
    .def_prop_ro("velocity", &PyDynamicSimulation::velocity, nb::rv_policy::move)
    .def_prop_ro("acceleration", &PyDynamicSimulation::acceleration, nb::rv_policy::move)
    .def_prop_ro("timestep_id", &PyDynamicSimulation::timestepId)
    .def_prop_ro("time", &PyDynamicSimulation::time)
    .def_prop_ro("num_dofs", &PyDynamicSimulation::numDofs);
}
