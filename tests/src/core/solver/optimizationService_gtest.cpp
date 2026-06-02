#include <gtest/gtest.h>

#include "lineSearchAwareEnergy.h"
#include "solver/newton/newtonLineSearchPolicy.h"
#include "solver/service/optimizationService.h"
#include "pgoLogging.h"

#include <Eigen/Sparse>

#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
namespace NO = pgo::NonlinearOptimization;

void initializeLogging()
{
  static const bool initialized = []() {
    pgo::Logging::init();
    return true;
  }();
  (void)initialized;
}

class ServiceQuadraticEnergy : public NO::PotentialEnergy
{
public:
  ServiceQuadraticEnergy(ES::SpMatD A, ES::VXd b): A_(std::move(A)), b_(std::move(b))
  {
    allDOFs_.resize(A_.rows());
    std::iota(allDOFs_.begin(), allDOFs_.end(), 0);
  }

  double func(ES::ConstRefVecXd x) const override
  {
    ES::VXd Ax;
    Ax.resize(A_.rows());
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
    dofs = allDOFs_;
  }

  int getNumDOFs() const override
  {
    return static_cast<int>(allDOFs_.size());
  }

private:
  ES::SpMatD A_;
  ES::VXd b_;
  std::vector<int> allDOFs_;
};

std::shared_ptr<ServiceQuadraticEnergy> makeQuadraticEnergy()
{
  ES::SpMatD A(3, 3);
  A.setIdentity();
  ES::VXd b(3);
  b << -1.0, 2.0, -4.0;
  return std::make_shared<ServiceQuadraticEnergy>(std::move(A), std::move(b));
}

NO::NewtonOptions oneStepOptions()
{
  NO::NewtonOptions options;
  options.control.maxIterations = 8;
  options.control.tolerance = 1e-10;
  options.control.verbose = 0;
  options.damping = false;
  options.lineSearch = NO::LineSearchMethod::Backtrack;
  return options;
}

NO::OptimizationProblem makeProblem(std::shared_ptr<const NO::PotentialEnergy> energy)
{
  NO::OptimizationProblem problem;
  problem.energy = std::move(energy);
  return problem;
}
}  // namespace

TEST(OptimizationServiceGTest, SolveOwnsResultAndDoesNotMutateInitialState)
{
  initializeLogging();
  auto energy = makeQuadraticEnergy();
  NO::OptimizationProblem problem = makeProblem(energy);
  ES::VXd x0(3);
  x0 << 10.0, -3.0, 5.0;
  const double *x0Data = x0.data();
  const ES::VXd original = x0;

  const NO::OptimizationResult result = NO::minimize(problem, x0, oneStepOptions());

  EXPECT_TRUE(result.solver.converged());
  EXPECT_TRUE(result.hasFinalObjective);
  EXPECT_NE(result.x.data(), x0Data);
  EXPECT_TRUE(x0.isApprox(original));
  ES::VXd expected(3);
  expected << 1.0, -2.0, 4.0;
  EXPECT_TRUE(result.x.isApprox(expected, 1e-8)) << result.x.transpose();
}

TEST(OptimizationServiceGTest, ImplicitFixedValuesComeFromInitialStateAndAreSorted)
{
  initializeLogging();
  auto energy = makeQuadraticEnergy();
  NO::OptimizationProblem problem = makeProblem(energy);
  NO::FixedVariables fixed;
  fixed.dofs = {2, 0};
  problem.fixedVariables = std::move(fixed);

  ES::VXd x0(3);
  x0 << 7.0, 10.0, -3.0;

  const NO::OptimizationResult result = NO::minimize(problem, x0, oneStepOptions());

  EXPECT_TRUE(result.solver.converged());
  EXPECT_NEAR(result.x[0], 7.0, 1e-10);
  EXPECT_NEAR(result.x[2], -3.0, 1e-10);
  EXPECT_NEAR(result.x[1], -2.0, 1e-8);
}

TEST(OptimizationServiceGTest, ExplicitFixedValuesOverrideInitialState)
{
  initializeLogging();
  auto energy = makeQuadraticEnergy();
  NO::OptimizationProblem problem = makeProblem(energy);
  NO::FixedVariables fixed;
  fixed.dofs = {2};
  fixed.values = ES::VXd::Constant(1, 9.0);
  problem.fixedVariables = std::move(fixed);

  ES::VXd x0(3);
  x0 << 10.0, -3.0, 5.0;

  const NO::OptimizationResult result = NO::minimize(problem, x0, oneStepOptions());

  EXPECT_TRUE(result.solver.converged());
  EXPECT_NEAR(result.x[2], 9.0, 1e-10);
  EXPECT_NEAR(result.x[0], 1.0, 1e-8);
  EXPECT_NEAR(result.x[1], -2.0, 1e-8);
}

TEST(OptimizationServiceGTest, RejectsInvalidProblemAndOptions)
{
  initializeLogging();
  auto energy = makeQuadraticEnergy();
  NO::OptimizationProblem problem = makeProblem(energy);
  ES::VXd x0(3);
  x0 << 1.0, 2.0, 3.0;

  EXPECT_THROW(NO::minimize(NO::OptimizationProblem{}, x0, oneStepOptions()), std::invalid_argument);

  ES::VXd wrongSize(2);
  wrongSize << 1.0, 2.0;
  EXPECT_THROW(NO::minimize(problem, wrongSize, oneStepOptions()), std::invalid_argument);

  NO::NewtonOptions bad = oneStepOptions();
  bad.control.maxIterations = -1;
  EXPECT_THROW(NO::minimize(problem, x0, bad), std::invalid_argument);

  bad = oneStepOptions();
  bad.control.tolerance = -1.0;
  EXPECT_THROW(NO::minimize(problem, x0, bad), std::invalid_argument);

  bad = oneStepOptions();
  bad.control.verbose = -1;
  EXPECT_THROW(NO::minimize(problem, x0, bad), std::invalid_argument);
}

TEST(OptimizationServiceGTest, RejectsInvalidFixedVariables)
{
  initializeLogging();
  auto energy = makeQuadraticEnergy();
  ES::VXd x0(3);
  x0 << 1.0, 2.0, 3.0;

  NO::OptimizationProblem problem = makeProblem(energy);
  problem.fixedVariables = NO::FixedVariables{{1, 1}, std::nullopt};
  EXPECT_THROW(NO::minimize(problem, x0, oneStepOptions()), std::invalid_argument);

  problem = makeProblem(energy);
  problem.fixedVariables = NO::FixedVariables{{3}, std::nullopt};
  EXPECT_THROW(NO::minimize(problem, x0, oneStepOptions()), std::invalid_argument);

  problem = makeProblem(energy);
  NO::FixedVariables fixed;
  fixed.dofs = {0, 1};
  fixed.values = ES::VXd::Constant(1, 5.0);
  problem.fixedVariables = std::move(fixed);
  EXPECT_THROW(NO::minimize(problem, x0, oneStepOptions()), std::invalid_argument);
}

TEST(OptimizationServiceGTest, NewtonRejectsUnsupportedBoundsAndConstraints)
{
  initializeLogging();
  auto energy = makeQuadraticEnergy();
  ES::VXd x0(3);
  x0 << 1.0, 2.0, 3.0;

  NO::OptimizationProblem problem = makeProblem(energy);
  problem.bounds = NO::BoxBounds{ES::VXd::Constant(3, -1.0), ES::VXd::Constant(3, 1.0)};
  EXPECT_THROW(NO::minimize(problem, x0, oneStepOptions()), std::invalid_argument);

  problem = makeProblem(energy);
  problem.constraints = NO::NonlinearConstraints{};
  EXPECT_THROW(NO::minimize(problem, x0, oneStepOptions()), std::invalid_argument);
}

TEST(OptimizationServiceGTest, AllServiceLineSearchMethodsAreAccepted)
{
  initializeLogging();
  auto energy = makeQuadraticEnergy();
  NO::OptimizationProblem problem = makeProblem(energy);
  ES::VXd x0(3);
  x0 << 10.0, -3.0, 5.0;

  for (NO::LineSearchMethod method : {
         NO::LineSearchMethod::Golden,
         NO::LineSearchMethod::Brents,
         NO::LineSearchMethod::Backtrack,
         NO::LineSearchMethod::Simple,
       }) {
    NO::NewtonOptions options = oneStepOptions();
    options.lineSearch = method;
    const NO::OptimizationResult result = NO::minimize(problem, x0, options);
    EXPECT_TRUE(result.solver.converged());
    EXPECT_TRUE(result.hasFinalObjective);
  }
}

TEST(OptimizationServiceGTest, LineSearchPolicyMaxProbeAlphaControlsActiveSetFreezing)
{
  LineSearch::EvaluateFunction evaluate = [](const double *, double *f, double *) {
    if (f)
      *f = 0.0;
    return 0;
  };

  EXPECT_DOUBLE_EQ(
    NO::createNewtonLineSearchPolicy(NO::NewtonLineSearchKind::Backtrack, 1, evaluate)->maxProbeAlpha(),
    1.0);
  EXPECT_DOUBLE_EQ(
    NO::createNewtonLineSearchPolicy(NO::NewtonLineSearchKind::Simple, 1, evaluate)->maxProbeAlpha(),
    1.0);
  EXPECT_FALSE(std::isfinite(
    NO::createNewtonLineSearchPolicy(NO::NewtonLineSearchKind::Golden, 1, evaluate)->maxProbeAlpha()));
  EXPECT_FALSE(std::isfinite(
    NO::createNewtonLineSearchPolicy(NO::NewtonLineSearchKind::Brents, 1, evaluate)->maxProbeAlpha()));
}

TEST(OptimizationServiceGTest, DefaultSparseSolverOptionIsAuto)
{
  NO::NewtonOptions options;
  EXPECT_EQ(options.sparseSolver.kind, NO::NewtonSparseSolverKind::Auto);
}
