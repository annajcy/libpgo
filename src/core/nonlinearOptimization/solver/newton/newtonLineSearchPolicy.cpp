#include "solver/newton/newtonLineSearchPolicy.h"

#include <cmath>
#include <limits>

namespace pgo::NonlinearOptimization
{

// ── Golden ────────────────────────────────────────────────────────────────

double GoldenLineSearchPolicy::maxProbeAlpha() const
{
  return std::numeric_limits<double>::infinity();
}

NewtonLineSearchResult GoldenLineSearchPolicy::search(const NewtonLineSearchContext &ctx) const
{
  ctx.helper.setMaxIterations(ctx.maxIterations);
  const LineSearch::Result ret = ctx.helper.golden(
    ctx.x.data(), ctx.direction.data(), ctx.currentEnergy);
  return {ret.alpha, ret.f, ret.numIter};
}

// ── Brents ────────────────────────────────────────────────────────────────

double BrentsLineSearchPolicy::maxProbeAlpha() const
{
  return std::numeric_limits<double>::infinity();
}

NewtonLineSearchResult BrentsLineSearchPolicy::search(const NewtonLineSearchContext &ctx) const
{
  ctx.helper.setMaxIterations(ctx.maxIterations);
  const LineSearch::Result ret = ctx.helper.BrentsMethod(
    ctx.x.data(), ctx.direction.data(), ctx.currentEnergy);
  return {ret.alpha, ret.f, ret.numIter};
}

// ── Backtracking ────────────────────────────────────────────────────────────

double BacktrackingLineSearchPolicy::maxProbeAlpha() const
{
  return params_.initialAlpha;
}

NewtonLineSearchResult BacktrackingLineSearchPolicy::search(const NewtonLineSearchContext &ctx) const
{
  ctx.helper.setMaxIterations(ctx.maxIterations);
  const LineSearch::Result ret = ctx.helper.backtrackingWithInitialValue(
    ctx.x.data(), ctx.direction.data(), ctx.currentEnergy, ctx.gradient.data(),
    params_.armijoC, params_.shrink, params_.initialAlpha, ctx.trialEnergyAtAlphaOne);
  return {ret.alpha, ret.f, ret.numIter};
}

// ── Simple ──────────────────────────────────────────────────────────────────

double SimpleLineSearchPolicy::maxProbeAlpha() const
{
  return 1.0;
}

NewtonLineSearchResult SimpleLineSearchPolicy::search(const NewtonLineSearchContext &ctx) const
{
  EigenSupport::VXd trial(ctx.x.size());
  NewtonLineSearchResult result;
  result.energy = ctx.currentEnergy;

  for (int i = 0; i < params_.maxIterations; i++) {
    trial.noalias() = ctx.x + ctx.direction * result.alpha;
    double f = 0.0;
    ctx.evaluate(trial.data(), &f, nullptr);
    result.energy = f;
    result.iterations = i + 1;
    if (!std::isfinite(result.energy)) {
      break;
    }

    if (result.energy < ctx.currentEnergy) {
      break;
    }

    result.alpha *= params_.shrink;
  }

  return result;
}

}  // namespace pgo::NonlinearOptimization
