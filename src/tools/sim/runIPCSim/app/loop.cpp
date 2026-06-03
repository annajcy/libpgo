#include "app/loop.h"

#include "deformationModelEnergy.h"
#include "dynamicStepper.h"
#include "multiVertexPullingSoftConstraints.h"
#include "solver/newton/NewtonOptimizer.h"

#include <algorithm>
#include <iostream>

namespace pgo::RunIPCSim
{
namespace ES = pgo::EigenSupport;

void runIPCSimLoop(const RunIPCSimRuntimeConfig &runtimeConfig,
  IpcSimulationContext &context,
  RunIPCSimSession &session,
  const RunIPCSimOutput &output)
{
  const double ratioDenom = runtimeConfig.numSimSteps > 1 ? static_cast<double>(runtimeConfig.numSimSteps - 1) : 1.0;

  bool executedStep = false;
  for (int framei = session.frameStart + 1; framei < runtimeConfig.numSimSteps; ++framei) {
    // ── Update pulling targets ──────────────────────────────────────
    const double ratio = runtimeConfig.numSimSteps > 1 ? static_cast<double>(framei) / ratioDenom : 1.0;
    for (std::size_t pi = 0; pi < context.pullingEnergies.size(); ++pi) {
      const ES::VXd curTgt = context.pullingTargetRests[pi] * (1.0 - ratio) + context.pullingTargets[pi] * ratio;
      context.pullingEnergies[pi]->setTargetPositions(curTgt);
      std::cout << "Frame " << framei << ", attachment " << pi << " target: " << curTgt.transpose().head(3) << std::endl;
    }

    // ── Contact backend: accumulate per-frame contact models ────────
    session.transientContactModels.clear();
    context.contactBackend->beginFrame(framei, runtimeConfig, context, session);
    context.contactBackend->addForces(framei, runtimeConfig, context, session);

    // ── External force (gravity + optional surface pressure) ────────
    if (context.surfacePressureForceEnabled) {
      const double ramp = std::min(1.0, static_cast<double>(framei + 1) / static_cast<double>(context.surfacePressureRampSteps));
      session.fext.noalias() = session.gravityForce + ramp * context.surfacePressureSimulationForce;
    }

    // ── Build stepper (persistent + transient terms) ────────────────
    pgo::Simulation::DynamicProblem problem;
    problem.mass = session.mass;
    problem.timestep = session.timestep;
    pgo::NonlinearOptimization::Optimization::NewtonOptimizer::Options optimizerOptions;
    optimizerOptions.maxIterations = session.solverMaxIter;
    optimizerOptions.gradientTolerance = session.solverEps;
    optimizerOptions.verbose = 0;
    pgo::NonlinearOptimization::Optimization::NewtonOptimizer optimizer(optimizerOptions);

    // Persistent terms: elastic energy + pulling (attachment) energies.
    {
      pgo::Simulation::ImplicitModelTerm t;
      t.energy = std::const_pointer_cast<pgo::SolidDeformationModel::DeformationModelEnergy>(context.elasticEnergy);
      problem.persistentTerms.push_back(t);
    }
    for (auto &pe : context.pullingEnergies) {
      pgo::Simulation::ImplicitModelTerm t;
      t.energy = std::const_pointer_cast<pgo::ConstraintPotentialEnergies::MultipleVertexPulling>(pe);
      problem.persistentTerms.push_back(t);
    }

    // Per-frame contact terms.
    for (auto &ct : session.transientContactModels)
      problem.persistentTerms.push_back(ct);

    // ── Step ────────────────────────────────────────────────────────
    pgo::Simulation::ImplicitEulerStepper stepper(std::move(problem));

    pgo::Simulation::DynamicState state;
    state.displacement = session.u;
    state.velocity = session.uvel;
    state.acceleration = session.uacc;

    pgo::Simulation::DynamicStepRequest request;
    request.externalForce = session.fext;

    pgo::Simulation::DynamicStepResult result = stepper.step(state, request, optimizer);
    executedStep = true;
    session.lastDiagnostics = result.solver.diagnostics;

    if (!result.accepted) {
      std::cerr << "Frame " << framei << ": step not accepted, status="
                << pgo::NonlinearOptimization::solveStatusToString(result.solver.status) << std::endl;
    }

    session.u = result.state.displacement;
    session.uvel = result.state.velocity;
    session.uacc = result.state.acceleration;

    // ── Contact afterStep / log ─────────────────────────────────────
    context.contactBackend->afterStep(framei, runtimeConfig, context, session);
    context.contactBackend->logSummary(context, session);

    // ── Output ──────────────────────────────────────────────────────
    const bool dumpDeformThisFrame = runtimeConfig.dumpDeformEveryFrame || (framei % runtimeConfig.frameGap == 0);
    const bool dumpSurfaceThisFrame = (framei % runtimeConfig.frameGap == 0);
    output.writeStateAndSurfaceFrame(framei, framei / runtimeConfig.frameGap, context,
      session.u, session.uvel, session.uacc, runtimeConfig.scale,
      dumpDeformThisFrame, dumpSurfaceThisFrame);

    if (runtimeConfig.outputVonMises)
      output.writeVonMisesStressJson(framei, runtimeConfig.timestep, context, session.u);
  }

  if (!executedStep)
    context.contactBackend->logSummary(context, session);
}
}  // namespace pgo::RunIPCSim
