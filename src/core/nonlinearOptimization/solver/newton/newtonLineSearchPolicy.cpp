#include "solver/newton/newtonLineSearchPolicy.h"

#include <cmath>
#include <limits>

namespace pgo::NonlinearOptimization
{

namespace
{
NewtonLineSearchResult classifyLineSearchResult(const LineSearch::Result &ret,
  double currentEnergy, double gradientDotDirection)
{
  NewtonLineSearchResult result;
  result.alpha = ret.alpha;
  result.energy = ret.f;
  result.iterations = ret.numIter;

  if (!std::isfinite(result.alpha)) {
    result.status = NewtonLineSearchStatus::NonFiniteAlpha;
  }
  else if (!std::isfinite(result.energy)) {
    result.status = NewtonLineSearchStatus::NonFiniteEnergy;
  }
  else if (lineSearchAcceptsEnergy(currentEnergy, result.energy, gradientDotDirection)) {
    result.status = NewtonLineSearchStatus::Accepted;
  }
  else {
    result.status = NewtonLineSearchStatus::NoAcceptableStep;
  }

  return result;
}
}  // namespace

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
  return classifyLineSearchResult(ret, ctx.currentEnergy, ctx.gradient.dot(ctx.direction));
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
  return classifyLineSearchResult(ret, ctx.currentEnergy, ctx.gradient.dot(ctx.direction));
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
  return classifyLineSearchResult(ret, ctx.currentEnergy, ctx.gradient.dot(ctx.direction));
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
  const int maxIterations = ctx.maxIterations > 0 ? ctx.maxIterations : params_.maxIterations;
  const double gradientDotDirection = ctx.gradient.dot(ctx.direction);

  double alpha = result.alpha;
  for (int i = 0; i < maxIterations; i++) {
    trial.noalias() = ctx.x + ctx.direction * alpha;
    double f = 0.0;
    ctx.evaluate(trial.data(), &f, nullptr);
    result.alpha = alpha;
    result.energy = f;
    result.iterations = i + 1;
    if (!std::isfinite(result.energy)) {
      result.status = NewtonLineSearchStatus::NonFiniteEnergy;
      return result;
    }

    if (lineSearchAcceptsEnergy(ctx.currentEnergy, result.energy, gradientDotDirection)) {
      result.status = NewtonLineSearchStatus::Accepted;
      return result;
    }

    alpha *= params_.shrink;
  }

  result.status = NewtonLineSearchStatus::NoAcceptableStep;
  return result;
}

}  // namespace pgo::NonlinearOptimization
