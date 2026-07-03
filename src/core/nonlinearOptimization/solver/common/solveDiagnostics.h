#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace pgo::NonlinearOptimization
{

// Identifies the source of a step-size constraint for diagnostic attribution.
// Add new values before kCount; all per-source loops automatically cover them.
enum class StepSource : int
{
  Material = 0,
  Contact  = 1,
  kCount   = 2,
};

enum class NewtonConvergenceReason : int
{
  None = 0,
  AbsoluteGradient = 1,
  RelativeGradient = 2,
  TerminationPolicy = 3,
  AbsoluteGradientFpLimit = 4,
  LooseRelativeGradientFpLimit = 5,
};

inline const char *newtonConvergenceReasonName(NewtonConvergenceReason reason)
{
  switch (reason) {
  case NewtonConvergenceReason::None:
    return "None";
  case NewtonConvergenceReason::AbsoluteGradient:
    return "AbsoluteGradient";
  case NewtonConvergenceReason::RelativeGradient:
    return "RelativeGradient";
  case NewtonConvergenceReason::TerminationPolicy:
    return "TerminationPolicy";
  case NewtonConvergenceReason::AbsoluteGradientFpLimit:
    return "AbsoluteGradientFpLimit";
  case NewtonConvergenceReason::LooseRelativeGradientFpLimit:
    return "LooseRelativeGradientFpLimit";
  }
  return "Unknown";
}

// Largest feasible step along a search direction, attributed to one source.
// Energies report a single source; aggregators keep the binding (min-alpha) one.
struct StepConstraint
{
  StepSource source = StepSource::Material;
  double alpha = 1.0;

  bool clamped() const { return alpha < 1.0; }
};

// Callback interface for collecting per-source step constraints during
// computeMaxStepLimit traversals. Each energy reports its own constraint;
// the sink accumulates them independently by source.
struct StepConstraintSink
{
  virtual ~StepConstraintSink() = default;
  virtual void report(StepConstraint c) = 0;
};

struct NewtonIterationTrace
{
  int iteration = 0;

  double energyBefore = std::numeric_limits<double>::quiet_NaN();
  double energyAfter = std::numeric_limits<double>::quiet_NaN();
  double energyDelta = std::numeric_limits<double>::quiet_NaN();

  double gradNormBefore = std::numeric_limits<double>::quiet_NaN();
  double gradNormAfter = std::numeric_limits<double>::quiet_NaN();
  double gradMaxBefore = std::numeric_limits<double>::quiet_NaN();
  double gradMaxAfter = std::numeric_limits<double>::quiet_NaN();
  double gradReductionRatio = std::numeric_limits<double>::quiet_NaN();

  double gradDotDx = std::numeric_limits<double>::quiet_NaN();
  double rawStepNorm = std::numeric_limits<double>::quiet_NaN();
  double rawStepMaxNorm = std::numeric_limits<double>::quiet_NaN();
  double acceptedAlpha = std::numeric_limits<double>::quiet_NaN();
  double acceptedStepNorm = std::numeric_limits<double>::quiet_NaN();
  double acceptedStepMaxNorm = std::numeric_limits<double>::quiet_NaN();

  int lineSearchIterations = 0;
  int lineSearchStatus = -1;
  double feasibleAlpha = 1.0;
  double lineSearchAlpha = 1.0;
  double minFeasibleAlpha = 1.0;
  std::int64_t contactClampCount = 0;
  std::int64_t materialClampCount = 0;

  double dampingValue = 0.0;
  std::int64_t hessianRows = 0;
  std::int64_t hessianCols = 0;
  std::int64_t hessianNnz = 0;
  bool symbolicRebuilt = false;

  double funcGradHessianSeconds = 0.0;
  double symbolicAnalyzeSeconds = 0.0;
  double factorizeSeconds = 0.0;
  double solveSeconds = 0.0;
  double lineSearchSeconds = 0.0;
  double iterationWallSeconds = 0.0;

  bool lowValue = false;
};

struct SolveDiagnostics : StepConstraintSink
{
  StepConstraint lastMaxStep;
  std::array<std::int64_t, static_cast<int>(StepSource::kCount)> clampCounts;
  double minFeasibleAlpha = 1.0;
  std::array<double, static_cast<int>(StepSource::kCount)> minSourceFeasibleAlpha;
  double minLineSearchAlpha = 1.0;
  double minEffectiveAlpha = 1.0;
  bool hasFinalGradientStats = false;
  double finalGradientNorm = 0.0;
  double finalGradientMaxNorm = 0.0;
  double lastGradDotDx = 0.0;
  double lastRawStepMaxNorm = 0.0;
  double lastRawStepNorm = 0.0;
  double lastAcceptedStepMaxNorm = 0.0;
  int lastLineSearchIterations = 0;
  int maxLineSearchIterations = 0;
  std::int64_t totalLineSearchIterations = 0;
  double lastCurrentEnergy = 0.0;
  double lastAcceptedEnergy = 0.0;
  double lastEnergyDelta = 0.0;
  double lastDampingValue = 0.0;
  std::int64_t lastActiveSystemNnz = 0;
  std::int64_t lastActiveSystemRows = 0;
  std::int64_t lastActiveSystemCols = 0;
  std::int64_t linearSolverSymbolicRebuildCount = 0;
  std::int64_t linearSolverSymbolicReuseCount = 0;
  std::vector<NewtonIterationTrace> newtonIterations;
  NewtonConvergenceReason newtonConvergenceReason = NewtonConvergenceReason::None;
  double newtonConvergenceThreshold = std::numeric_limits<double>::quiet_NaN();
  std::int64_t newtonLowValueIterationCount = 0;
  std::int64_t newtonTinyStepCount = 0;
  std::int64_t newtonSmallAlphaCount = 0;
  std::int64_t newtonSymbolicRebuildCount = 0;
  int newtonWorstProgressIteration = -1;
  double newtonWorstProgressRatio = 0.0;
  double newtonTotalFactorizeSeconds = 0.0;
  double newtonTotalSolveSeconds = 0.0;

  SolveDiagnostics()
  {
    clampCounts.fill(0);
    minSourceFeasibleAlpha.fill(1.0);
  }

  void reset()
  {
    *this = SolveDiagnostics{};
  }

  // Merge a step constraint into the per-source diagnostics, attributed by source.
  // This is the only place step constraints are combined across calls.
  void report(StepConstraint result) override
  {
    lastMaxStep = result;
    minFeasibleAlpha = std::min(minFeasibleAlpha, result.alpha);
    const int i = static_cast<int>(result.source);
    minSourceFeasibleAlpha[i] = std::min(minSourceFeasibleAlpha[i], result.alpha);
    if (result.clamped())
      clampCounts[i] += 1;
  }

  void recordNewtonDirection(double gradDotDx, double rawStepMaxNorm, double rawStepNorm)
  {
    lastGradDotDx = gradDotDx;
    lastRawStepMaxNorm = rawStepMaxNorm;
    lastRawStepNorm = rawStepNorm;
  }

  void recordLineSearch(double feasibleAlpha, double lineSearchAlpha, double effectiveAlpha,
    int iterations, double currentEnergy, double acceptedEnergy, double acceptedStepMaxNorm)
  {
    minFeasibleAlpha   = std::min(minFeasibleAlpha,   feasibleAlpha);
    minLineSearchAlpha = std::min(minLineSearchAlpha, lineSearchAlpha);
    minEffectiveAlpha  = std::min(minEffectiveAlpha,  effectiveAlpha);
    lastLineSearchIterations = iterations;
    maxLineSearchIterations = std::max(maxLineSearchIterations, iterations);
    totalLineSearchIterations += iterations;
    lastCurrentEnergy = currentEnergy;
    lastAcceptedEnergy = acceptedEnergy;
    lastEnergyDelta = acceptedEnergy - currentEnergy;
    lastAcceptedStepMaxNorm = acceptedStepMaxNorm;
  }

  void recordLineSearch(double feasibleAlpha, double lineSearchAlpha, double effectiveAlpha)
  {
    recordLineSearch(feasibleAlpha, lineSearchAlpha, effectiveAlpha, 0, 0.0, 0.0, 0.0);
  }

  void recordDampingValue(double damping)
  {
    lastDampingValue = damping;
  }

  void recordActiveSystemPattern(std::int64_t rows, std::int64_t cols, std::int64_t nnz)
  {
    lastActiveSystemRows = rows;
    lastActiveSystemCols = cols;
    lastActiveSystemNnz = nnz;
  }

  void recordLinearSolverSymbolicReuse(bool reused)
  {
    if (reused)
      linearSolverSymbolicReuseCount += 1;
    else
      linearSolverSymbolicRebuildCount += 1;
  }

  void recordFinalGradientStats(double gradientNorm, double gradientMaxNorm)
  {
    hasFinalGradientStats = true;
    finalGradientNorm     = gradientNorm;
    finalGradientMaxNorm  = gradientMaxNorm;
  }

  void recordNewtonConvergence(NewtonConvergenceReason reason, double threshold)
  {
    newtonConvergenceReason = reason;
    newtonConvergenceThreshold = threshold;
  }

  void recordNewtonIteration(NewtonIterationTrace trace)
  {
    if (std::isfinite(trace.energyBefore) && std::isfinite(trace.energyAfter))
      trace.energyDelta = trace.energyAfter - trace.energyBefore;
    if (std::isfinite(trace.gradMaxBefore) && std::isfinite(trace.gradMaxAfter) && trace.gradMaxBefore > 0.0)
      trace.gradReductionRatio = trace.gradMaxAfter / trace.gradMaxBefore;

    if (std::isfinite(trace.acceptedAlpha) && trace.acceptedAlpha < 1e-2)
      newtonSmallAlphaCount += 1;
    if (std::isfinite(trace.acceptedStepMaxNorm) && trace.acceptedStepMaxNorm < 1e-15)
      newtonTinyStepCount += 1;
    if (trace.symbolicRebuilt)
      newtonSymbolicRebuildCount += 1;

    newtonTotalFactorizeSeconds += trace.factorizeSeconds;
    newtonTotalSolveSeconds += trace.solveSeconds;

    if (std::isfinite(trace.gradReductionRatio) &&
      (newtonWorstProgressIteration < 0 || trace.gradReductionRatio > newtonWorstProgressRatio)) {
      newtonWorstProgressIteration = trace.iteration;
      newtonWorstProgressRatio = trace.gradReductionRatio;
    }

    markLowValue(trace);

    newtonIterations.push_back(trace);
  }

  void completeLastNewtonIterationAfterState(int iteration, double energy, double gradNorm, double gradMaxNorm)
  {
    if (newtonIterations.empty())
      return;
    NewtonIterationTrace &trace = newtonIterations.back();
    if (trace.iteration + 1 != iteration)
      return;

    trace.energyAfter = energy;
    trace.energyDelta = energy - trace.energyBefore;
    trace.gradNormAfter = gradNorm;
    trace.gradMaxAfter = gradMaxNorm;
    if (std::isfinite(trace.gradMaxBefore) && trace.gradMaxBefore > 0.0)
      trace.gradReductionRatio = gradMaxNorm / trace.gradMaxBefore;

    if (std::isfinite(trace.gradReductionRatio) &&
      (newtonWorstProgressIteration < 0 || trace.gradReductionRatio > newtonWorstProgressRatio)) {
      newtonWorstProgressIteration = trace.iteration;
      newtonWorstProgressRatio = trace.gradReductionRatio;
    }
    markLowValue(trace);
  }

private:
  void markLowValue(NewtonIterationTrace &trace)
  {
    if (trace.lowValue)
      return;
    const double energyTol = std::numeric_limits<double>::epsilon() * 64.0 *
      std::max(1.0, std::abs(trace.energyBefore));
    const bool poorSmallAlpha = std::isfinite(trace.acceptedAlpha) && trace.acceptedAlpha < 1e-2 &&
      std::isfinite(trace.gradReductionRatio) && trace.gradReductionRatio > 0.9;
    const bool expensiveLineSearch = trace.lineSearchIterations > 8 &&
      std::isfinite(trace.energyDelta) && std::abs(trace.energyDelta) <= energyTol;
    const bool tinyUnresolvedStep = std::isfinite(trace.acceptedStepMaxNorm) && trace.acceptedStepMaxNorm < 1e-15 &&
      std::isfinite(trace.gradMaxAfter) && trace.gradMaxAfter > 0.0;
    const bool expensivePoorProgress = trace.iterationWallSeconds > 1.0 &&
      std::isfinite(trace.gradReductionRatio) && trace.gradReductionRatio > 0.95;
    if (poorSmallAlpha || expensiveLineSearch || tinyUnresolvedStep || expensivePoorProgress) {
      trace.lowValue = true;
      newtonLowValueIterationCount += 1;
    }
  }
};

}  // namespace pgo::NonlinearOptimization

namespace pgo
{
using NonlinearOptimization::StepConstraintSink;
using NonlinearOptimization::StepSource;
using NonlinearOptimization::StepConstraint;
using NonlinearOptimization::NewtonConvergenceReason;
using NonlinearOptimization::newtonConvergenceReasonName;
using NonlinearOptimization::SolveDiagnostics;
}  // namespace pgo
