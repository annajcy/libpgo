#pragma once

#include "EigenDef.h"
#include "lineSearch.h"

#include <memory>

namespace pgo::NonlinearOptimization
{

enum class NewtonLineSearchKind
{
  Golden,
  Brents,
  Backtrack,
  Simple
};

struct NewtonLineSearchInput
{
  EigenSupport::ConstRefVecXd x;
  EigenSupport::ConstRefVecXd direction;
  EigenSupport::ConstRefVecXd gradient;
  double currentEnergy = 0.0;
  double trialEnergyAtAlphaOne = 0.0;
  int maxIterations = 0;
};

struct NewtonLineSearchResult
{
  double alpha = 1.0;
  double energy = 0.0;
  int iterations = 0;
};

class NewtonLineSearchPolicy
{
public:
  virtual ~NewtonLineSearchPolicy() = default;

  virtual double maxProbeAlpha() const = 0;
  virtual NewtonLineSearchResult search(const NewtonLineSearchInput &input) = 0;
};

std::unique_ptr<NewtonLineSearchPolicy> createNewtonLineSearchPolicy(
  NewtonLineSearchKind kind,
  int numDofs,
  LineSearch::EvaluateFunction evaluate);

}  // namespace pgo::NonlinearOptimization
