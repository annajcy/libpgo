#include "solver/newton/NewtonOptimizer.h"
#include "solver/service/optimizerUtils.h"

#include "energy/evaluationStateAwareEnergy.h"
#include "pgoLogging.h"
#include "energy/potentialEnergy.h"

#include <Eigen/Sparse>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
namespace NO = pgo::NonlinearOptimization;
namespace OPT = pgo::NonlinearOptimization::Optimization;

void initializeLogging()
{
  static const bool initialized = []() {
    pgo::Logging::init();
    return true;
  }();
  (void)initialized;
}

class QuadraticEnergy : public NO::PotentialEnergy
{
public:
  QuadraticEnergy(ES::SpMatD A, ES::VXd b):
    A_(std::move(A)),
    b_(std::move(b))
  {
    allDofs_.resize(A_.rows());
    std::iota(allDofs_.begin(), allDofs_.end(), 0);
  }

  double func(ES::ConstRefVecXd x) const override
  {
    ES::VXd Ax(A_.rows());
    ES::mv(A_, x, Ax);
    return 0.5 * x.dot(Ax) + b_.dot(x);
  }

  void gradient(ES::ConstRefVecXd x, ES::RefVecXd grad) const override
  {
    ES::mv(A_, x, grad);
    grad += b_;
  }

  void hessianInPlace(ES::ConstRefVecXd, ES::SpMatD &hess) const override
  {
    hess = A_;
  }

  void hessianAlloc(ES::SpMatD &hess) const override
  {
    hess = A_;
  }

  void getDOFs(std::vector<int> &dofs) const override
  {
    dofs = allDofs_;
  }

  int getNumDOFs() const override
  {
    return static_cast<int>(allDofs_.size());
  }

private:
  ES::SpMatD A_;
  ES::VXd b_;
  std::vector<int> allDofs_;
};

class FinalObjectiveCacheEnergy : public NO::PotentialEnergy, public NO::EvaluationStateAwareEnergy
{
public:
  double func(ES::ConstRefVecXd x) const override
  {
    requirePreparedFor(x);
    return 0.5 * x.squaredNorm();
  }

  void gradient(ES::ConstRefVecXd x, ES::RefVecXd grad) const override
  {
    requirePreparedFor(x);
    grad = x;
  }

  void hessianInPlace(ES::ConstRefVecXd x, ES::SpMatD &hess) const override
  {
    requirePreparedFor(x);
    hess.setIdentity();
  }

  void hessianAlloc(ES::SpMatD &hess) const override
  {
    hess.resize(1, 1);
    hess.setIdentity();
  }

  double func_grad_hessian(ES::ConstRefVecXd x, ES::RefVecXd grad, ES::SpMatD &hess) const override
  {
    requirePreparedFor(x);
    gradient(x, grad);
    hessianInPlace(x, hess);
    return 0.5 * x.squaredNorm();
  }

  void getDOFs(std::vector<int> &dofs) const override
  {
    dofs = { 0 };
  }

  int getNumDOFs() const override
  {
    return 1;
  }

  void prepareEvaluationState(ES::ConstRefVecXd x) const override
  {
    prepareCalls++;
    lastPreparedState = x;
  }

  mutable int prepareCalls = 0;
  mutable ES::VXd lastPreparedState;

private:
  void requirePreparedFor(ES::ConstRefVecXd x) const
  {
    if (lastPreparedState.size() != x.size() || !(lastPreparedState.array() == x.array()).all())
      throw std::logic_error("evaluation state was not prepared for this point");
  }
};

std::shared_ptr<QuadraticEnergy> makeQuadraticEnergy()
{
  ES::SpMatD A(3, 3);
  A.setIdentity();
  ES::VXd b(3);
  b << -1.0, 2.0, -4.0;
  return std::make_shared<QuadraticEnergy>(std::move(A), std::move(b));
}

OPT::NewtonOptimizer makeOptimizer()
{
  OPT::NewtonOptimizer::Options options;
  options.maxIterations = 8;
  options.gradientTolerance = 1e-10;
  options.verbose = 0;
  options.damping = std::make_shared<NO::NoDampingPolicy>();
  options.lineSearch = std::make_shared<NO::BacktrackingLineSearchPolicy>(NO::BacktrackingLineSearchPolicy::Params{});
  return OPT::NewtonOptimizer(options);
}

OPT::OptimizationProblem makeProblem(std::shared_ptr<const NO::PotentialEnergy> energy)
{
  OPT::OptimizationProblem problem;
  problem.objective = std::move(energy);
  return problem;
}

struct SimpleLineSearchRun
{
  NO::NewtonLineSearchResult result;
  std::vector<double> evaluatedAlphas;
};

template<class EnergyForAlpha>
SimpleLineSearchRun runSimpleLineSearch(double gradientDotDirection, double currentEnergy,
  int ctxMaxIterations, NO::SimpleLineSearchPolicy::Params params, EnergyForAlpha energyForAlpha)
{
  ES::VXd x(1);
  x << 0.0;
  ES::VXd direction(1);
  direction << 1.0;
  ES::VXd gradient(1);
  gradient << gradientDotDirection;

  SimpleLineSearchRun run;
  LineSearch::EvaluateFunction evaluate = [&](const double *trialX, double *f, double *) {
    run.evaluatedAlphas.push_back(trialX[0]);
    if (f != nullptr) {
      *f = energyForAlpha(trialX[0]);
    }
    return 0;
  };
  LineSearch helper(1, evaluate);
  NO::NewtonLineSearchContext ctx{ x, direction, gradient, currentEnergy, energyForAlpha(1.0),
    ctxMaxIterations, helper, evaluate };
  run.result = NO::SimpleLineSearchPolicy(params).search(ctx);
  return run;
}

}  // namespace

TEST(NewtonOptimizer, SolveOwnsResultAndDoesNotMutateInitialState)
{
  initializeLogging();
  auto energy = makeQuadraticEnergy();
  OPT::OptimizationProblem problem = makeProblem(energy);
  ES::VXd x0(3);
  x0 << 10.0, -3.0, 5.0;
  const double *x0Data = x0.data();
  const ES::VXd original = x0;

  const OPT::OptimizationResult result = makeOptimizer().solve(problem, x0);

  EXPECT_TRUE(result.solver.converged());
  ASSERT_TRUE(result.finalObjective.has_value());
  EXPECT_NE(result.x.data(), x0Data);
  EXPECT_TRUE(x0.isApprox(original));
  ES::VXd expected(3);
  expected << 1.0, -2.0, 4.0;
  EXPECT_TRUE(result.x.isApprox(expected, 1e-8)) << result.x.transpose();
}

TEST(NewtonOptimizer, RecordsStructuredLifecycleTimings)
{
  initializeLogging();
  auto energy = makeQuadraticEnergy();
  OPT::OptimizationProblem problem = makeProblem(energy);
  ES::VXd x0(3);
  x0 << 10.0, -3.0, 5.0;

  const OPT::OptimizationResult result = makeOptimizer().solve(problem, x0);
  const NO::SolveDiagnostics &diagnostics = result.solver.diagnostics;

  EXPECT_GE(diagnostics.optimizerPreparationSeconds, 0.0);
  EXPECT_GE(diagnostics.newtonSolverSetupSeconds, 0.0);
  EXPECT_GE(diagnostics.initialHessianSeconds, 0.0);
  EXPECT_GE(diagnostics.initialReducedSystemSeconds, 0.0);
  EXPECT_GE(diagnostics.initialSymbolicAnalyzeSeconds, 0.0);
  EXPECT_GE(diagnostics.newtonSolveSeconds, 0.0);
  EXPECT_GE(diagnostics.finalObjectiveSeconds, 0.0);
  EXPECT_GE(diagnostics.linearSolverCleanupSeconds, 0.0);
  EXPECT_GE(diagnostics.optimizerTotalSeconds, diagnostics.newtonSolveSeconds);
}

TEST(NewtonOptimizer, SolvesWithFixedVariablesAsEqualityBounds)
{
  initializeLogging();
  auto energy = makeQuadraticEnergy();
  OPT::OptimizationProblem problem = makeProblem(energy);
  ES::VXd fixedValues(2);
  fixedValues << -3.0, 7.0;
  const std::vector<int> fixedDofs = { 2, 0 };
  OPT::fixVariables(problem, fixedDofs, fixedValues, 3);

  ES::VXd x0(3);
  x0 << 10.0, -3.0, 5.0;

  const OPT::OptimizationResult result = makeOptimizer().solve(problem, x0);

  EXPECT_TRUE(result.solver.converged());
  EXPECT_NEAR(result.x[0], 7.0, 1e-10);
  EXPECT_NEAR(result.x[2], -3.0, 1e-10);
  EXPECT_NEAR(result.x[1], -2.0, 1e-8);
}

TEST(NewtonOptimizer, PreparesEvaluationStateBeforeFinalObjective)
{
  initializeLogging();
  auto energy = std::make_shared<FinalObjectiveCacheEnergy>();
  OPT::OptimizationProblem problem = makeProblem(energy);
  ES::VXd x0(1);
  x0 << 1.0;

  const OPT::OptimizationResult result = makeOptimizer().solve(problem, x0);

  EXPECT_TRUE(result.solver.converged());
  ASSERT_TRUE(result.finalObjective.has_value());
  EXPECT_NEAR(*result.finalObjective, 0.0, 1e-14);
  EXPECT_GE(energy->prepareCalls, 2);
  EXPECT_TRUE(energy->lastPreparedState.isApprox(result.x));
}

TEST(NewtonOptimizer, RejectsInvalidProblemAndOptions)
{
  initializeLogging();
  auto energy = makeQuadraticEnergy();
  OPT::OptimizationProblem problem = makeProblem(energy);
  ES::VXd x0(3);
  x0 << 1.0, 2.0, 3.0;

  EXPECT_THROW(makeOptimizer().solve(OPT::OptimizationProblem{}, x0), std::invalid_argument);

  ES::VXd wrongSize(2);
  wrongSize << 1.0, 2.0;
  EXPECT_THROW(makeOptimizer().solve(problem, wrongSize), std::invalid_argument);

  OPT::NewtonOptimizer::Options bad;
  bad.maxIterations = -1;
  EXPECT_THROW(OPT::NewtonOptimizer(bad).solve(problem, x0), std::invalid_argument);

  bad = OPT::NewtonOptimizer::Options();
  bad.gradientTolerance = -1.0;
  EXPECT_THROW(OPT::NewtonOptimizer(bad).solve(problem, x0), std::invalid_argument);

  bad = OPT::NewtonOptimizer::Options();
  bad.verbose = -1;
  EXPECT_THROW(OPT::NewtonOptimizer(bad).solve(problem, x0), std::invalid_argument);
}

TEST(NewtonOptimizer, RejectsUnsupportedBoundsAndConstraints)
{
  initializeLogging();
  auto energy = makeQuadraticEnergy();
  ES::VXd x0(3);
  x0 << 1.0, 2.0, 3.0;

  OPT::OptimizationProblem problem = makeProblem(energy);
  problem.variableBounds.lower = ES::VXd::Constant(3, -1.0);
  problem.variableBounds.upper = ES::VXd::Constant(3, 1.0);
  EXPECT_THROW(makeOptimizer().solve(problem, x0), std::invalid_argument);

  problem = makeProblem(energy);
  problem.constraints.push_back(OPT::ConstraintBlock{});
  EXPECT_THROW(makeOptimizer().solve(problem, x0), std::invalid_argument);
}

TEST(NewtonOptimizer, AllLineSearchMethodsAreAccepted)
{
  initializeLogging();
  auto energy = makeQuadraticEnergy();
  OPT::OptimizationProblem problem = makeProblem(energy);
  ES::VXd x0(3);
  x0 << 10.0, -3.0, 5.0;

  const std::vector<std::shared_ptr<NO::NewtonLineSearchPolicy>> methods = {
    std::make_shared<NO::GoldenLineSearchPolicy>(),
    std::make_shared<NO::BrentsLineSearchPolicy>(),
    std::make_shared<NO::BacktrackingLineSearchPolicy>(NO::BacktrackingLineSearchPolicy::Params{}),
    std::make_shared<NO::SimpleLineSearchPolicy>(NO::SimpleLineSearchPolicy::Params{}),
  };
  for (const auto &method : methods) {
    OPT::NewtonOptimizer::Options options;
    options.maxIterations = 8;
    options.gradientTolerance = 1e-10;
    options.damping = std::make_shared<NO::NoDampingPolicy>();
    options.lineSearch = method;
    const OPT::OptimizationResult result = OPT::NewtonOptimizer(options).solve(problem, x0);
    EXPECT_TRUE(result.solver.converged());
    EXPECT_TRUE(result.finalObjective.has_value());
  }
}

TEST(SimpleLineSearchPolicy, DescentDirectionAcceptsEqualEnergyAtAlphaOne)
{
  const double currentEnergy = 10.0;
  auto run = runSimpleLineSearch(-1.0, currentEnergy, 3,
    NO::SimpleLineSearchPolicy::Params{ 0.5, 100 },
    [currentEnergy](double) { return currentEnergy; });

  EXPECT_EQ(run.result.iterations, 1);
  ASSERT_EQ(run.evaluatedAlphas.size(), 1u);
  EXPECT_DOUBLE_EQ(run.evaluatedAlphas[0], 1.0);
  EXPECT_DOUBLE_EQ(run.result.alpha, 1.0);
  EXPECT_DOUBLE_EQ(run.result.energy, currentEnergy);
  EXPECT_EQ(run.result.status, NO::NewtonLineSearchStatus::Accepted);
  EXPECT_TRUE(run.result.accepted());
  EXPECT_FALSE(run.result.nonFinite());
}

TEST(SimpleLineSearchPolicy, NonDescentDirectionDoesNotAcceptEqualEnergyByTolerance)
{
  const double currentEnergy = 10.0;
  auto run = runSimpleLineSearch(1.0, currentEnergy, 3,
    NO::SimpleLineSearchPolicy::Params{ 0.5, 100 },
    [currentEnergy](double) { return currentEnergy; });

  EXPECT_EQ(run.result.iterations, 3);
  ASSERT_EQ(run.evaluatedAlphas.size(), 3u);
  EXPECT_DOUBLE_EQ(run.evaluatedAlphas[0], 1.0);
  EXPECT_DOUBLE_EQ(run.evaluatedAlphas[1], 0.5);
  EXPECT_DOUBLE_EQ(run.evaluatedAlphas[2], 0.25);
  EXPECT_DOUBLE_EQ(run.result.alpha, 0.25);
  EXPECT_DOUBLE_EQ(run.result.energy, currentEnergy);
  EXPECT_EQ(run.result.status, NO::NewtonLineSearchStatus::NoAcceptableStep);
  EXPECT_FALSE(run.result.accepted());
  EXPECT_FALSE(run.result.nonFinite());
}

TEST(SimpleLineSearchPolicy, FailureReturnsLastEvaluatedAlphaAndEnergy)
{
  const double currentEnergy = 10.0;
  auto run = runSimpleLineSearch(1.0, currentEnergy, 3,
    NO::SimpleLineSearchPolicy::Params{ 0.5, 100 },
    [currentEnergy](double alpha) { return currentEnergy + alpha; });

  EXPECT_EQ(run.result.iterations, 3);
  ASSERT_EQ(run.evaluatedAlphas.size(), 3u);
  EXPECT_DOUBLE_EQ(run.evaluatedAlphas.back(), 0.25);
  EXPECT_DOUBLE_EQ(run.result.alpha, 0.25);
  EXPECT_DOUBLE_EQ(run.result.energy, 10.25);
  EXPECT_EQ(run.result.status, NO::NewtonLineSearchStatus::NoAcceptableStep);
  EXPECT_FALSE(run.result.accepted());
  EXPECT_FALSE(run.result.nonFinite());
}

TEST(SimpleLineSearchPolicy, NonFiniteTrialReturnsNonFiniteEnergyStatus)
{
  const double currentEnergy = 10.0;
  auto run = runSimpleLineSearch(-1.0, currentEnergy, 3,
    NO::SimpleLineSearchPolicy::Params{ 0.5, 100 },
    [](double) { return std::numeric_limits<double>::quiet_NaN(); });

  EXPECT_EQ(run.result.iterations, 1);
  ASSERT_EQ(run.evaluatedAlphas.size(), 1u);
  EXPECT_DOUBLE_EQ(run.evaluatedAlphas[0], 1.0);
  EXPECT_DOUBLE_EQ(run.result.alpha, 1.0);
  EXPECT_TRUE(std::isnan(run.result.energy));
  EXPECT_EQ(run.result.status, NO::NewtonLineSearchStatus::NonFiniteEnergy);
  EXPECT_FALSE(run.result.accepted());
  EXPECT_TRUE(run.result.nonFinite());
}

TEST(NewtonOptimizer, DefaultSparseSolverOptionIsAuto)
{
  OPT::NewtonOptimizer::Options options;
  // Default: no selector → solver picks the best available backend.
  EXPECT_EQ(options.sparseSolver, nullptr);
}
