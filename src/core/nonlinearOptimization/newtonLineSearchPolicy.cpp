#include "newtonLineSearchPolicy.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace pgo::NonlinearOptimization
{
namespace
{
constexpr double kBacktrackArmijo = 0.0001;
constexpr double kBacktrackShrink = 0.5;
constexpr double kBacktrackInitAlpha = 1.0;
constexpr int kSimpleLineSearchMaxIter = 100;

class NativeLineSearchPolicy : public NewtonLineSearchPolicy
{
public:
  NativeLineSearchPolicy(int numDofs, LineSearch::EvaluateFunction evaluate):
    lineSearch_(numDofs, std::move(evaluate))
  {
  }

protected:
  LineSearch lineSearch_;
};

class GoldenLineSearchPolicy final : public NativeLineSearchPolicy
{
public:
  using NativeLineSearchPolicy::NativeLineSearchPolicy;

  double maxProbeAlpha() const override { return std::numeric_limits<double>::infinity(); }

  NewtonLineSearchResult search(const NewtonLineSearchInput &input) override
  {
    lineSearch_.setMaxIterations(input.maxIterations);
    const LineSearch::Result ret = lineSearch_.golden(
      input.x.data(), input.direction.data(), input.currentEnergy);
    return {ret.alpha, ret.f, ret.numIter};
  }
};

class BrentLineSearchPolicy final : public NativeLineSearchPolicy
{
public:
  using NativeLineSearchPolicy::NativeLineSearchPolicy;

  double maxProbeAlpha() const override { return std::numeric_limits<double>::infinity(); }

  NewtonLineSearchResult search(const NewtonLineSearchInput &input) override
  {
    lineSearch_.setMaxIterations(input.maxIterations);
    const LineSearch::Result ret = lineSearch_.BrentsMethod(
      input.x.data(), input.direction.data(), input.currentEnergy);
    return {ret.alpha, ret.f, ret.numIter};
  }
};

class BacktrackingLineSearchPolicy final : public NativeLineSearchPolicy
{
public:
  using NativeLineSearchPolicy::NativeLineSearchPolicy;

  double maxProbeAlpha() const override { return 1.0; }

  NewtonLineSearchResult search(const NewtonLineSearchInput &input) override
  {
    lineSearch_.setMaxIterations(input.maxIterations);
    const LineSearch::Result ret = lineSearch_.backtrackingWithInitialValue(
      input.x.data(), input.direction.data(), input.currentEnergy, input.gradient.data(),
      kBacktrackArmijo, kBacktrackShrink, kBacktrackInitAlpha, input.trialEnergyAtAlphaOne);
    return {ret.alpha, ret.f, ret.numIter};
  }
};

class SimpleLineSearchPolicy final : public NewtonLineSearchPolicy
{
public:
  explicit SimpleLineSearchPolicy(LineSearch::EvaluateFunction evaluate): evaluate_(std::move(evaluate)) {}

  double maxProbeAlpha() const override { return 1.0; }

  NewtonLineSearchResult search(const NewtonLineSearchInput &input) override
  {
    EigenSupport::VXd trial(input.x.size());
    NewtonLineSearchResult result;
    result.energy = input.currentEnergy;

    for (int i = 0; i < kSimpleLineSearchMaxIter; i++) {
      trial.noalias() = input.x + input.direction * result.alpha;
      double f = 0.0;
      evaluate_(trial.data(), &f, nullptr);
      result.energy = f;
      result.iterations = i + 1;
      if (!std::isfinite(result.energy)) {
        break;
      }

      if (result.energy < input.currentEnergy) {
        break;
      }

      result.alpha *= 0.5;
    }

    return result;
  }

private:
  LineSearch::EvaluateFunction evaluate_;
};

}  // namespace

std::unique_ptr<NewtonLineSearchPolicy> createNewtonLineSearchPolicy(
  NewtonLineSearchKind kind,
  int numDofs,
  LineSearch::EvaluateFunction evaluate)
{
  switch (kind) {
    case NewtonLineSearchKind::Golden:
      return std::make_unique<GoldenLineSearchPolicy>(numDofs, std::move(evaluate));
    case NewtonLineSearchKind::Brents:
      return std::make_unique<BrentLineSearchPolicy>(numDofs, std::move(evaluate));
    case NewtonLineSearchKind::Backtrack:
      return std::make_unique<BacktrackingLineSearchPolicy>(numDofs, std::move(evaluate));
    case NewtonLineSearchKind::Simple:
      return std::make_unique<SimpleLineSearchPolicy>(std::move(evaluate));
  }

  throw std::invalid_argument("Unknown Newton line search policy");
}

}  // namespace pgo::NonlinearOptimization
