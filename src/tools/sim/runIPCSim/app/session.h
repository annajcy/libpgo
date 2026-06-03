#pragma once

#include "EigenSupport.h"
#include "app/config.h"
#include "app/output.h"
#include "dynamicStepOptions.h"
#include "setup/setup.h"

#include <memory>
#include <vector>

namespace pgo::Simulation
{
class ImplicitEulerStepper;
}

namespace pgo::RunIPCSim
{
struct RunIPCSimSession
{
  // Per-frame contact energy accumulator (filled by contact backend,
  // consumed by loop to build the stepper).
  std::vector<pgo::Simulation::ImplicitModelTerm> transientContactModels;

  // Immutable problem data.
  EigenSupport::SpMatD mass;
  double timestep = 0.01;
  int solverMaxIter = 20;
  double solverEps = 1e-5;

  // Current kinematic state.
  EigenSupport::VXd u;
  EigenSupport::VXd uvel;
  EigenSupport::VXd uacc;
  EigenSupport::VXd usurf;
  EigenSupport::VXd gravityForce;
  EigenSupport::VXd fext;
  int frameStart = -1;

  // Last-step diagnostics for logging.
  pgo::NonlinearOptimization::SolveDiagnostics lastDiagnostics;
};

RunIPCSimSession createRunIPCSimSession(const RunIPCSimRuntimeConfig &runtimeConfig,
  const IpcSimulationContext &context);
void restoreRestartStateIfRequested(const RunIPCSimRuntimeConfig &runtimeConfig,
  const RunIPCSimOutput &output, RunIPCSimSession &session);
}  // namespace pgo::RunIPCSim
