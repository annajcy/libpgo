#include "solver/newton/NewtonOptimizer.h"
#include "solver/service/optimizerUtils.h"

#include "pgoLogging.h"
#include "potentialEnergy.h"

#include <Eigen/Sparse>
#include <gtest/gtest.h>

#include <memory>
#include <numeric>
#include <utility>

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
  options.damping = false;
  options.lineSearch = NO::NewtonLineSearchKind::Backtrack;
  return OPT::NewtonOptimizer(options);
}

OPT::OptimizationProblem makeProblem(std::shared_ptr<const NO::PotentialEnergy> energy)
{
  OPT::OptimizationProblem problem;
  problem.objective = std::move(energy);
  return problem;
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

  for (NO::NewtonLineSearchKind method : {
         NO::NewtonLineSearchKind::Golden,
         NO::NewtonLineSearchKind::Brents,
         NO::NewtonLineSearchKind::Backtrack,
         NO::NewtonLineSearchKind::Simple,
       }) {
    OPT::NewtonOptimizer::Options options;
    options.maxIterations = 8;
    options.gradientTolerance = 1e-10;
    options.damping = false;
    options.lineSearch = method;
    const OPT::OptimizationResult result = OPT::NewtonOptimizer(options).solve(problem, x0);
    EXPECT_TRUE(result.solver.converged());
    EXPECT_TRUE(result.finalObjective.has_value());
  }
}

TEST(NewtonOptimizer, DefaultSparseSolverOptionIsAuto)
{
  OPT::NewtonOptimizer::Options options;
  EXPECT_EQ(options.sparseSolver.kind, NO::NewtonSparseSolverKind::Auto);
}
