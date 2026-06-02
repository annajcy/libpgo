#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

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

  void recordLineSearch(double feasibleAlpha, double lineSearchAlpha, double effectiveAlpha)
  {
    minFeasibleAlpha   = std::min(minFeasibleAlpha,   feasibleAlpha);
    minLineSearchAlpha = std::min(minLineSearchAlpha, lineSearchAlpha);
    minEffectiveAlpha  = std::min(minEffectiveAlpha,  effectiveAlpha);
  }

  void recordFinalGradientStats(double gradientNorm, double gradientMaxNorm)
  {
    hasFinalGradientStats = true;
    finalGradientNorm     = gradientNorm;
    finalGradientMaxNorm  = gradientMaxNorm;
  }
};

}  // namespace pgo::NonlinearOptimization

namespace pgo
{
using NonlinearOptimization::StepConstraintSink;
using NonlinearOptimization::StepSource;
using NonlinearOptimization::StepConstraint;
using NonlinearOptimization::SolveDiagnostics;
}  // namespace pgo
