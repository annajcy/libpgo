#pragma once

#include "EigenDef.h"
#include "solver/newton/lineSearch.h"

namespace pgo::NonlinearOptimization
{

// Working context handed to a policy for one line-search step.  The LineSearch
// helper and the energy evaluator are per-solve scratch owned by the solver; the
// policy borrows them and keeps no state of its own, so a policy object is
// immutable and freely shareable across solves.
struct NewtonLineSearchContext
{
  EigenSupport::ConstRefVecXd x;
  EigenSupport::ConstRefVecXd direction;
  EigenSupport::ConstRefVecXd gradient;
  double currentEnergy = 0.0;
  double trialEnergyAtAlphaOne = 0.0;
  int maxIterations = 0;
  LineSearch &helper;
  const LineSearch::EvaluateFunction &evaluate;
};

struct NewtonLineSearchResult
{
  double alpha = 1.0;
  double energy = 0.0;
  int iterations = 0;
};

// A line-search policy is a stateless, immutable strategy: it carries only its
// own parameters, and `search` reads everything else from the context.  Build it
// directly with its parameters and hand it to the optimizer.
class NewtonLineSearchPolicy
{
public:
  virtual ~NewtonLineSearchPolicy() = default;

  virtual double maxProbeAlpha() const = 0;
  virtual NewtonLineSearchResult search(const NewtonLineSearchContext &ctx) const = 0;
};

// Golden-section search. No tunable parameters.
class GoldenLineSearchPolicy final : public NewtonLineSearchPolicy
{
public:
  GoldenLineSearchPolicy() = default;

  double maxProbeAlpha() const override;
  NewtonLineSearchResult search(const NewtonLineSearchContext &ctx) const override;
};

// Brent's method (parabolic interpolation + golden fallback). No tunables.
class BrentsLineSearchPolicy final : public NewtonLineSearchPolicy
{
public:
  BrentsLineSearchPolicy() = default;

  double maxProbeAlpha() const override;
  NewtonLineSearchResult search(const NewtonLineSearchContext &ctx) const override;
};

// Backtracking with the Armijo sufficient-decrease condition.
class BacktrackingLineSearchPolicy final : public NewtonLineSearchPolicy
{
public:
  struct Params
  {
    double armijoC = 1e-4;     // sufficient-decrease constant c in (0, 1)
    double shrink = 0.5;       // step shrink factor rho in (0, 1)
    double initialAlpha = 1.0; // initial trial step
  };

  explicit BacktrackingLineSearchPolicy(Params params): params_(params) {}

  double maxProbeAlpha() const override;
  NewtonLineSearchResult search(const NewtonLineSearchContext &ctx) const override;

private:
  Params params_;
};

// Simple search: halve the step until the energy strictly decreases.
class SimpleLineSearchPolicy final : public NewtonLineSearchPolicy
{
public:
  struct Params
  {
    double shrink = 0.5;     // step shrink factor in (0, 1)
    int maxIterations = 100; // max halving steps
  };

  explicit SimpleLineSearchPolicy(Params params): params_(params) {}

  double maxProbeAlpha() const override;
  NewtonLineSearchResult search(const NewtonLineSearchContext &ctx) const override;

private:
  Params params_;
};

}  // namespace pgo::NonlinearOptimization
