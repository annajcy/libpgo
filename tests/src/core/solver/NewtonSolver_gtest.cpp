#include <gtest/gtest.h>

#include "solver/newton/NewtonSolver.h"
#include "energySet.h"
#include "evaluationStateAwareEnergy.h"
#include "lineSearchAwareEnergy.h"
#include "pgoLogging.h"
#include "solver/common/solveDiagnostics.h"

#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace
{
namespace ES = pgo::EigenSupport;
using pgo::NonlinearOptimization::NewtonSolver;
using pgo::NonlinearOptimization::EnergySet;
using pgo::NonlinearOptimization::EvaluationStateAwareEnergy;
using pgo::NonlinearOptimization::PotentialEnergy;
using pgo::NonlinearOptimization::LineSearchAwareEnergy;
using pgo::NonlinearOptimization::NewtonLineSearchKind;
using pgo::NonlinearOptimization::SolveDiagnostics;
using pgo::NonlinearOptimization::SolverResult;
using pgo::NonlinearOptimization::StepSource;
using pgo::NonlinearOptimization::StepConstraint;
constexpr int src(StepSource s) { return static_cast<int>(s); }
using pgo::NonlinearOptimization::SolveStatus;
using pgo::NonlinearOptimization::acceptsDynamicSolveStatus;
using pgo::NonlinearOptimization::acceptsStrictSolveStatus;
using pgo::NonlinearOptimization::makeIpoptSolverResult;
using pgo::NonlinearOptimization::makeKnitroSolverResult;
using pgo::NonlinearOptimization::solveStatusToString;

void initializeLogging()
{
  static const bool initialized = []() {
    pgo::Logging::init();
    return true;
  }();
  (void)initialized;
}

class TestQuadraticEnergy : public PotentialEnergy, public LineSearchAwareEnergy
{
public:
  explicit TestQuadraticEnergy(int n, StepConstraint maxStep = {}): n(n), maxStep(maxStep) {}

  double func(ES::ConstRefVecXd x) const override
  {
    funcCalls++;
    return 0.5 * x.squaredNorm();
  }

  void gradient(ES::ConstRefVecXd x, ES::RefVecXd grad) const override
  {
    gradientCalls++;
    grad = x;
  }

  void hessianInPlace(ES::ConstRefVecXd, ES::SpMatD &hess) const override
  {
    hessianCalls++;
    hess.setIdentity();
  }

  void hessianAlloc(ES::SpMatD &hess) const override
  {
    hess.resize(n, n);
    hess.setIdentity();
  }

  void getDOFs(std::vector<int> &dofs) const override
  {
    dofs.resize(n);
    std::iota(dofs.begin(), dofs.end(), 0);
  }

  int getNumDOFs() const override { return n; }
  StepConstraint computeMaxStepLimit(ES::ConstRefVecXd, ES::ConstRefVecXd, pgo::NonlinearOptimization::StepConstraintSink *sink = nullptr) const override { if (sink) sink->report(maxStep); return maxStep; }
  void beginLineSearch(ES::ConstRefVecXd, ES::ConstRefVecXd) const override { beginLineSearchCalls++; }
  void endLineSearch() const override { endLineSearchCalls++; }

  mutable int funcCalls = 0;
  mutable int gradientCalls = 0;
  mutable int hessianCalls = 0;
  mutable int beginLineSearchCalls = 0;
  mutable int endLineSearchCalls = 0;

private:
  int n;
  StepConstraint maxStep;
};

class TestEvaluationStateEnergy : public PotentialEnergy, public EvaluationStateAwareEnergy
{
public:
  explicit TestEvaluationStateEnergy(std::vector<int> dofs):
    dofs_(std::move(dofs)) {}

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
    hess.resize(getNumDOFs(), getNumDOFs());
    hess.setIdentity();
  }

  void getDOFs(std::vector<int> &dofs) const override { dofs = dofs_; }
  int getNumDOFs() const override { return static_cast<int>(dofs_.size()); }

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

  std::vector<int> dofs_;
};

class TestNonFixedQuadraticEnergy : public PotentialEnergy
{
public:
  explicit TestNonFixedQuadraticEnergy(int n): n(n) {}

  double func(ES::ConstRefVecXd x) const override
  {
    return 0.5 * x.squaredNorm();
  }

  void gradient(ES::ConstRefVecXd x, ES::RefVecXd grad) const override
  {
    gradientCalls++;
    grad = x;
  }

  void hessianInPlace(ES::ConstRefVecXd, ES::SpMatD &hess) const override
  {
    hessianCalls++;
    hess.setIdentity();
  }

  void gradient_hessian(ES::ConstRefVecXd x, ES::RefVecXd grad, ES::SpMatD &hess) const override
  {
    gradientHessianCalls++;
    grad = x;
    hess.resize(n, n);
    hess.setIdentity();
  }

  double func_grad_hessian(ES::ConstRefVecXd x, ES::RefVecXd grad, ES::SpMatD &hess) const override
  {
    gradient_hessian(x, grad, hess);
    return func(x);
  }

  void hessianAlloc(ES::SpMatD &hess) const override
  {
    hess.resize(n, n);
    hess.setIdentity();
  }

  void getDOFs(std::vector<int> &dofs) const override
  {
    dofs.resize(n);
    std::iota(dofs.begin(), dofs.end(), 0);
  }

  int getNumDOFs() const override { return n; }
  int isHessianTopologyFixed() const override { return 0; }
  StepConstraint computeMaxStepLimit(ES::ConstRefVecXd, ES::ConstRefVecXd, pgo::NonlinearOptimization::StepConstraintSink *sink = nullptr) const override { return {}; }

  mutable int gradientCalls = 0;
  mutable int hessianCalls = 0;
  mutable int gradientHessianCalls = 0;

private:
  int n;
};

class TestSlowQuadraticEnergy : public PotentialEnergy
{
public:
  double func(ES::ConstRefVecXd x) const override
  {
    return 0.5 * x.squaredNorm();
  }

  void gradient(ES::ConstRefVecXd x, ES::RefVecXd grad) const override
  {
    grad = x;
  }

  void hessianInPlace(ES::ConstRefVecXd, ES::SpMatD &hess) const override
  {
    hess.setIdentity();
    hess *= 10.0;
  }

  void hessianAlloc(ES::SpMatD &hess) const override
  {
    hess.resize(1, 1);
    hess.setIdentity();
  }

  void getDOFs(std::vector<int> &dofs) const override
  {
    dofs = { 0 };
  }

  int getNumDOFs() const override { return 1; }

  StepConstraint computeMaxStepLimit(ES::ConstRefVecXd x, ES::ConstRefVecXd, pgo::NonlinearOptimization::StepConstraintSink *sink = nullptr) const override
  {
    if (std::abs(x[0]) < 2e-4) {
      StepConstraint c{StepSource::Contact, 0.0};
      if (sink) sink->report(c);
      return c;
    }
    return {};
  }
};

class TestNonFiniteTrialEnergy : public PotentialEnergy, public LineSearchAwareEnergy
{
public:
  double func(ES::ConstRefVecXd x) const override
  {
    funcCalls++;
    if (std::abs(x[0]) < 1e-14)
      return std::numeric_limits<double>::quiet_NaN();
    return 0.5 * x.squaredNorm();
  }

  void gradient(ES::ConstRefVecXd x, ES::RefVecXd grad) const override
  {
    grad = x;
  }

  void hessianInPlace(ES::ConstRefVecXd, ES::SpMatD &hess) const override
  {
    hess.setIdentity();
  }

  void hessianAlloc(ES::SpMatD &hess) const override
  {
    hess.resize(1, 1);
    hess.setIdentity();
  }

  void getDOFs(std::vector<int> &dofs) const override
  {
    dofs = { 0 };
  }

  int getNumDOFs() const override { return 1; }
  StepConstraint computeMaxStepLimit(ES::ConstRefVecXd, ES::ConstRefVecXd, pgo::NonlinearOptimization::StepConstraintSink *sink = nullptr) const override { return {}; }
  void beginLineSearch(ES::ConstRefVecXd, ES::ConstRefVecXd) const override { beginLineSearchCalls++; }
  void endLineSearch() const override { endLineSearchCalls++; }

  mutable int funcCalls = 0;
  mutable int beginLineSearchCalls = 0;
  mutable int endLineSearchCalls = 0;
};
}  // namespace

TEST(SolveDiagnosticsGTest, RecordsAndResetsMaxStepAndLineSearch)
{
  SolveDiagnostics diagnostics;

  diagnostics.report(StepConstraint{StepSource::Material, 0.4});
  diagnostics.report(StepConstraint{StepSource::Contact, 0.25});
  diagnostics.recordLineSearch(0.25, 0.5, 0.125);
  diagnostics.recordFinalGradientStats(2.0, 1.5);

  EXPECT_EQ(diagnostics.clampCounts[src(StepSource::Material)], 1);
  EXPECT_EQ(diagnostics.clampCounts[src(StepSource::Contact)], 1);
  EXPECT_DOUBLE_EQ(diagnostics.minFeasibleAlpha, 0.25);
  EXPECT_DOUBLE_EQ(diagnostics.minSourceFeasibleAlpha[src(StepSource::Material)], 0.4);
  EXPECT_DOUBLE_EQ(diagnostics.minSourceFeasibleAlpha[src(StepSource::Contact)], 0.25);
  EXPECT_DOUBLE_EQ(diagnostics.minLineSearchAlpha, 0.5);
  EXPECT_DOUBLE_EQ(diagnostics.minEffectiveAlpha, 0.125);
  EXPECT_EQ(diagnostics.lastMaxStep.source, StepSource::Contact);
  EXPECT_DOUBLE_EQ(diagnostics.lastMaxStep.alpha, 0.25);
  EXPECT_TRUE(diagnostics.hasFinalGradientStats);
  EXPECT_DOUBLE_EQ(diagnostics.finalGradientNorm, 2.0);
  EXPECT_DOUBLE_EQ(diagnostics.finalGradientMaxNorm, 1.5);

  diagnostics.reset();

  EXPECT_EQ(diagnostics.clampCounts[src(StepSource::Material)], 0);
  EXPECT_EQ(diagnostics.clampCounts[src(StepSource::Contact)], 0);
  EXPECT_DOUBLE_EQ(diagnostics.minFeasibleAlpha, 1.0);
  EXPECT_DOUBLE_EQ(diagnostics.minSourceFeasibleAlpha[src(StepSource::Material)], 1.0);
  EXPECT_DOUBLE_EQ(diagnostics.minSourceFeasibleAlpha[src(StepSource::Contact)], 1.0);
  EXPECT_DOUBLE_EQ(diagnostics.minLineSearchAlpha, 1.0);
  EXPECT_DOUBLE_EQ(diagnostics.minEffectiveAlpha, 1.0);
  EXPECT_DOUBLE_EQ(diagnostics.lastMaxStep.alpha, 1.0);
  EXPECT_FALSE(diagnostics.hasFinalGradientStats);
  EXPECT_DOUBLE_EQ(diagnostics.finalGradientNorm, 0.0);
  EXPECT_DOUBLE_EQ(diagnostics.finalGradientMaxNorm, 0.0);
}

// Verify that the sink accumulates the same source independently (clamp-counter
// increments per clamped call; min-alpha tracks the tightest).
TEST(SolveDiagnosticsGTest, SameSourceTracksTightestAlpha)
{
  SolveDiagnostics diagnostics;
  diagnostics.report(StepConstraint{StepSource::Material, 0.4});
  diagnostics.report(StepConstraint{StepSource::Material, 0.3});
  diagnostics.report(StepConstraint{StepSource::Material, 0.5});

  EXPECT_EQ(diagnostics.clampCounts[src(StepSource::Material)], 3);
  EXPECT_DOUBLE_EQ(diagnostics.minSourceFeasibleAlpha[src(StepSource::Material)], 0.3);
  // Contact untouched.
  EXPECT_EQ(diagnostics.clampCounts[src(StepSource::Contact)], 0);
  EXPECT_DOUBLE_EQ(diagnostics.minSourceFeasibleAlpha[src(StepSource::Contact)], 1.0);
  // Binding alpha.
  EXPECT_DOUBLE_EQ(diagnostics.minFeasibleAlpha, 0.3);
  // lastMaxStep is the last report.
  EXPECT_EQ(diagnostics.lastMaxStep.source, StepSource::Material);
  EXPECT_DOUBLE_EQ(diagnostics.lastMaxStep.alpha, 0.5);
}

// An energy that unconditionally reports a fixed constraint through the sink
// during computeMaxStepLimit.
class SinkTestEnergy : public PotentialEnergy
{
public:
  explicit SinkTestEnergy(int n, StepConstraint c): n_(n), c_(c) {}
  double func(ES::ConstRefVecXd) const override { return 0.0; }
  void gradient(ES::ConstRefVecXd, ES::RefVecXd) const override {}
  void hessianInPlace(ES::ConstRefVecXd, ES::SpMatD &) const override {}
  void hessianAlloc(ES::SpMatD &h) const override { h.resize(n_, n_); h.setIdentity(); }
  void getDOFs(std::vector<int> &dofs) const override { dofs.resize(n_); std::iota(dofs.begin(), dofs.end(), 0); }
  int getNumDOFs() const override { return n_; }
  StepConstraint computeMaxStepLimit(ES::ConstRefVecXd, ES::ConstRefVecXd, pgo::NonlinearOptimization::StepConstraintSink *sink = nullptr) const override
  {
    if (sink) sink->report(c_); return c_;
  }

private:
  int n_;
  StepConstraint c_;
};

// End-to-end: a sink-aware energy reports through the traversal path.
TEST(SolveDiagnosticsGTest, SinkReportsThroughTraversal)
{
  SolveDiagnostics diagnostics;
  SinkTestEnergy energy(4, StepConstraint{StepSource::Material, 0.3});
  ES::VXd x = ES::VXd::Zero(4);
  ES::VXd dx = ES::VXd::Ones(4);

  // computeMaxStepLimit without sink → diagnostics untouched.
  (void)energy.computeMaxStepLimit(x, dx);
  EXPECT_EQ(diagnostics.clampCounts[src(StepSource::Material)], 0);

  // computeMaxStepLimit WITH sink → recorded.
  (void)energy.computeMaxStepLimit(x, dx, &diagnostics);
  EXPECT_EQ(diagnostics.clampCounts[src(StepSource::Material)], 1);
  EXPECT_DOUBLE_EQ(diagnostics.minSourceFeasibleAlpha[src(StepSource::Material)], 0.3);
  EXPECT_DOUBLE_EQ(diagnostics.minFeasibleAlpha, 0.3);
}

// Two different sources in the same step → both independently recorded.
TEST(SolveDiagnosticsGTest, CrossSourceRecognizesBoth)
{
  SolveDiagnostics diagnostics;

  // Simulate a Newton step with both material (0.3) and contact (0.5) constraints.
  diagnostics.report(StepConstraint{StepSource::Material, 0.3});
  diagnostics.report(StepConstraint{StepSource::Contact, 0.5});

  EXPECT_EQ(diagnostics.clampCounts[src(StepSource::Material)], 1);
  EXPECT_EQ(diagnostics.clampCounts[src(StepSource::Contact)], 1);
  EXPECT_DOUBLE_EQ(diagnostics.minSourceFeasibleAlpha[src(StepSource::Material)], 0.3);
  EXPECT_DOUBLE_EQ(diagnostics.minSourceFeasibleAlpha[src(StepSource::Contact)], 0.5);
  // Binding across sources is the material one.
  EXPECT_DOUBLE_EQ(diagnostics.minFeasibleAlpha, 0.3);
  // lastMaxStep is the last report (contact).
  EXPECT_EQ(diagnostics.lastMaxStep.source, StepSource::Contact);
}

TEST(NewtonSolverGTest, ConvergedSolveReturnsConvergedStatus)
{
  initializeLogging();

  auto energy = std::make_shared<TestQuadraticEnergy>(2);
  ES::VXd x(2);
  x[0] = 2.0;
  x[1] = 0.0;

  NewtonSolver::SolverParam solverParam;
  const std::vector<int> fixedDOFs = { 1 };
  const double fixedValues[1] = { 0.0 };
  NewtonSolver solver(x.data(), solverParam, energy, fixedDOFs, fixedValues);

  const SolverResult result = solver.solve(x.data(), 8, 1e-10, 0);

  EXPECT_EQ(result.status, SolveStatus::Converged);
  EXPECT_TRUE(result.converged());
  EXPECT_NEAR(x[0], 0.0, 1e-10);
  EXPECT_NEAR(x[1], 0.0, 1e-10);
}

TEST(NewtonSolverGTest, ZeroFeasibleStepWithLargeResidualReturnsStepTooSmall)
{
  initializeLogging();

  auto energy = std::make_shared<TestQuadraticEnergy>(2, StepConstraint{StepSource::Contact, 0.0});
  ES::VXd x(2);
  x[0] = 2.0;
  x[1] = 0.0;

  NewtonSolver::SolverParam solverParam;
  const std::vector<int> fixedDOFs = { 1 };
  const double fixedValues[1] = { 0.0 };
  NewtonSolver solver(x.data(), solverParam, energy, fixedDOFs, fixedValues);

  testing::internal::CaptureStdout();
  const SolverResult result = solver.solve(x.data(), 8, 1e-10, 1);
  const std::string output = testing::internal::GetCapturedStdout();

  EXPECT_EQ(result.status, SolveStatus::StepTooSmall);
  EXPECT_NE(output.find("status=StepTooSmall"), std::string::npos);
  EXPECT_EQ(output.find("T2330"), std::string::npos);
}

TEST(NewtonSolverGTest, SolveDiagnosticsRecordsMaxStepBreakdown)
{
  initializeLogging();

  auto energy = std::make_shared<TestQuadraticEnergy>(2, StepConstraint{StepSource::Material, 0.25});
  ES::VXd x(2);
  x[0] = 2.0;
  x[1] = 0.0;

  NewtonSolver::SolverParam solverParam;
  const std::vector<int> fixedDOFs = { 1 };
  const double fixedValues[1] = { 0.0 };
  NewtonSolver solver(x.data(), solverParam, energy, fixedDOFs, fixedValues);

  testing::internal::CaptureStdout();
  const SolverResult result = solver.solve(x.data(), 1, 1e-10, 2);
  const std::string output = testing::internal::GetCapturedStdout();

  const SolveDiagnostics &diagnostics = solver.getSolveDiagnostics();
  EXPECT_EQ(result.status, SolveStatus::MaxIterations);
  EXPECT_EQ(result.diagnostics.clampCounts[src(StepSource::Material)], 1);
  EXPECT_EQ(diagnostics.clampCounts[src(StepSource::Material)], 1);
  EXPECT_EQ(diagnostics.clampCounts[src(StepSource::Contact)], 0);
  EXPECT_DOUBLE_EQ(diagnostics.minFeasibleAlpha, 0.25);
  EXPECT_DOUBLE_EQ(diagnostics.minSourceFeasibleAlpha[src(StepSource::Material)], 0.25);
  EXPECT_DOUBLE_EQ(diagnostics.minSourceFeasibleAlpha[src(StepSource::Contact)], 1.0);
  EXPECT_DOUBLE_EQ(diagnostics.minLineSearchAlpha, 1.0);
  EXPECT_DOUBLE_EQ(diagnostics.minEffectiveAlpha, 0.25);
  EXPECT_NE(output.find("feasible alpha clamped by material: 0.25"), std::string::npos);
}

TEST(NewtonSolverGTest, NonFiniteTrialEnergyEndsLineSearchScope)
{
  initializeLogging();

  auto energy = std::make_shared<TestNonFiniteTrialEnergy>();
  ES::VXd x(1);
  x[0] = 2.0;

  NewtonSolver::SolverParam solverParam;
  solverParam.lineSearch = NewtonLineSearchKind::Backtrack;
  const std::vector<int> fixedDOFs;
  NewtonSolver solver(x.data(), solverParam, energy, fixedDOFs);

  const SolverResult result = solver.solve(x.data(), 1, 1e-12, 0);

  EXPECT_EQ(result.status, SolveStatus::NonFinite);
  EXPECT_EQ(energy->beginLineSearchCalls, 1);
  EXPECT_EQ(energy->endLineSearchCalls, 1);
}

TEST(NewtonSolverGTest, BacktrackingReusesInitialTrialEnergy)
{
  initializeLogging();

  auto energy = std::make_shared<TestQuadraticEnergy>(2);
  ES::VXd x(2);
  x[0] = 2.0;
  x[1] = 0.0;

  NewtonSolver::SolverParam solverParam;
  solverParam.lineSearch = NewtonLineSearchKind::Backtrack;
  const std::vector<int> fixedDOFs = { 1 };
  const double fixedValues[1] = { 0.0 };
  NewtonSolver solver(x.data(), solverParam, energy, fixedDOFs, fixedValues);

  const SolverResult result = solver.solve(x.data(), 1, 1e-10, 0);

  EXPECT_EQ(result.status, SolveStatus::MaxIterations);
  EXPECT_EQ(energy->funcCalls, 2);
  EXPECT_EQ(energy->gradientCalls, 1);
  EXPECT_EQ(energy->hessianCalls, 2);
  EXPECT_EQ(energy->beginLineSearchCalls, 1);
  EXPECT_EQ(energy->endLineSearchCalls, 1);
}

TEST(NewtonSolverGTest, EnergySetPrepareEvaluationStateMapsGlobalStateToChildDofs)
{
  auto energy = std::make_shared<TestEvaluationStateEnergy>(std::vector<int>{ 2, 0 });
  EnergySet set(3, { EnergySet::Term{ energy, 1.0 } });

  ES::VXd x(3);
  x << 10.0, 20.0, 30.0;
  set.prepareEvaluationState(x);

  ASSERT_EQ(energy->prepareCalls, 1);
  ASSERT_EQ(energy->lastPreparedState.size(), 2);
  EXPECT_DOUBLE_EQ(energy->lastPreparedState[0], 30.0);
  EXPECT_DOUBLE_EQ(energy->lastPreparedState[1], 10.0);
}

TEST(NewtonSolverGTest, SubiterationOnePreparesEvaluationStateBeforeCurrentAndAcceptedEvaluations)
{
  initializeLogging();

  auto energy = std::make_shared<TestEvaluationStateEnergy>(std::vector<int>{ 0 });
  ES::VXd x(1);
  x[0] = 2.0;

  NewtonSolver::SolverParam solverParam;
  solverParam.sst = NewtonSolver::SST_SUBITERATION_ONE;
  const std::vector<int> fixedDOFs;
  NewtonSolver solver(x.data(), solverParam, energy, fixedDOFs);

  const SolverResult result = solver.solve(x.data(), 1, 1e-10, 0);

  EXPECT_EQ(result.status, SolveStatus::Converged);
  EXPECT_GE(energy->prepareCalls, 2);
  ASSERT_EQ(energy->lastPreparedState.size(), 1);
  EXPECT_NEAR(energy->lastPreparedState[0], x[0], 1e-12);
}

TEST(NewtonSolverGTest, GoldenLineSearchDoesNotUseBoundedActiveSetScope)
{
  initializeLogging();

  auto energy = std::make_shared<TestQuadraticEnergy>(2);
  ES::VXd x(2);
  x[0] = 2.0;
  x[1] = 0.0;

  NewtonSolver::SolverParam solverParam;
  solverParam.lineSearch = NewtonLineSearchKind::Golden;
  const std::vector<int> fixedDOFs = { 1 };
  const double fixedValues[1] = { 0.0 };
  NewtonSolver solver(x.data(), solverParam, energy, fixedDOFs, fixedValues);

  const SolverResult result = solver.solve(x.data(), 1, 1e-10, 0);

  EXPECT_EQ(result.status, SolveStatus::MaxIterations);
  EXPECT_EQ(energy->beginLineSearchCalls, 0);
  EXPECT_EQ(energy->endLineSearchCalls, 0);
}

TEST(NewtonSolverGTest, StepTooSmallConvergenceRecordsFinalGradientStats)
{
  initializeLogging();

  auto energy = std::make_shared<TestSlowQuadraticEnergy>();
  ES::VXd x(1);
  x[0] = 2.0;

  NewtonSolver::SolverParam solverParam;
  const std::vector<int> fixedDOFs;
  NewtonSolver solver(x.data(), solverParam, energy, fixedDOFs);

  testing::internal::CaptureStdout();
  const SolverResult result = solver.solve(x.data(), 120, 1e-12, 1);
  const std::string output = testing::internal::GetCapturedStdout();

  const SolveDiagnostics &diagnostics = solver.getSolveDiagnostics();
  EXPECT_EQ(result.status, SolveStatus::Converged);
  EXPECT_TRUE(result.hasFinalGradientStats);
  EXPECT_DOUBLE_EQ(result.finalGradientMaxNorm, diagnostics.finalGradientMaxNorm);
  EXPECT_NE(output.find("dx too small"), std::string::npos);
  EXPECT_TRUE(diagnostics.hasFinalGradientStats);
  EXPECT_GT(diagnostics.finalGradientMaxNorm, 0.0);
  EXPECT_LT(diagnostics.finalGradientMaxNorm, 2.0e-4);
}

TEST(NewtonSolverGTest, VerboseIterationLogLabelsMaxGradientNorm)
{
  initializeLogging();

  auto energy = std::make_shared<TestQuadraticEnergy>(2);
  ES::VXd x(2);
  x[0] = 3.0;
  x[1] = 4.0;

  NewtonSolver::SolverParam solverParam;
  const std::vector<int> fixedDOFs;
  NewtonSolver solver(x.data(), solverParam, energy, fixedDOFs);

  testing::internal::CaptureStdout();
  solver.solve(x.data(), 1, 0.0, 2);
  const std::string output = testing::internal::GetCapturedStdout();

  EXPECT_NE(output.find("||grad||_max=4"), std::string::npos);
  EXPECT_EQ(output.find("; ||grad||="), std::string::npos);
}

TEST(NewtonSolverGTest, NonFixedTopologyIterationsUseGradientHessian)
{
  initializeLogging();

  auto energy = std::make_shared<TestNonFixedQuadraticEnergy>(2);
  ES::VXd x(2);
  x[0] = 2.0;
  x[1] = 0.0;

  NewtonSolver::SolverParam solverParam;
  solverParam.sst = NewtonSolver::SST_SUBITERATION_ONE;
  const std::vector<int> fixedDOFs = { 1 };
  const double fixedValues[1] = { 0.0 };
  NewtonSolver solver(x.data(), solverParam, energy, fixedDOFs, fixedValues);

  const SolverResult result = solver.solve(x.data(), 1, 1e-10, 0);

  EXPECT_EQ(result.status, SolveStatus::Converged);
  EXPECT_EQ(energy->gradientCalls, 1);
  EXPECT_EQ(energy->gradientHessianCalls, 1);
  EXPECT_EQ(energy->hessianCalls, 0);
}

TEST(NewtonSolverGTest, StaticDampingConvergesOnQuadratic)
{
  auto energy = std::make_shared<TestQuadraticEnergy>(2);
  ES::VXd x(2);
  x[0] = 3.0;
  x[1] = 4.0;

  NewtonSolver::SolverParam solverParam;
  solverParam.sst = NewtonSolver::SST_SUBITERATION_STATIC_DAMPING;
  solverParam.alpha = 0.5;
  const std::vector<int> fixedDOFs;
  NewtonSolver solver(x.data(), solverParam, energy, fixedDOFs);

  const SolverResult result = solver.solve(x.data(), 200, 1e-6, 0);

  EXPECT_EQ(result.status, SolveStatus::Converged);
  EXPECT_LT(x.cwiseAbs().maxCoeff(), 1e-4);
}

TEST(NewtonSolverGTest, AddDampingConvergesOnQuadratic)
{
  auto energy = std::make_shared<TestQuadraticEnergy>(2);
  ES::VXd x(2);
  x[0] = 3.0;
  x[1] = 4.0;

  NewtonSolver::SolverParam solverParam;
  solverParam.lineSearch = NewtonLineSearchKind::Backtrack;
  solverParam.addDamping = 1;
  const std::vector<int> fixedDOFs;
  NewtonSolver solver(x.data(), solverParam, energy, fixedDOFs);

  const SolverResult result = solver.solve(x.data(), 200, 1e-6, 0);

  EXPECT_EQ(result.status, SolveStatus::Converged);
  EXPECT_LT(x.cwiseAbs().maxCoeff(), 1e-4);
}

TEST(NewtonSolverGTest, SolveStatusToStringReturnsStableNames)
{
  EXPECT_STREQ(solveStatusToString(SolveStatus::Converged), "Converged");
  EXPECT_STREQ(solveStatusToString(static_cast<int>(SolveStatus::Converged)), "Converged");
  EXPECT_STREQ(solveStatusToString(SolveStatus::MaxIterations), "MaxIterations");
  EXPECT_STREQ(solveStatusToString(SolveStatus::LineSearchFailed), "LineSearchFailed");
  EXPECT_STREQ(solveStatusToString(SolveStatus::StepTooSmall), "StepTooSmall");
  EXPECT_STREQ(solveStatusToString(SolveStatus::NonFinite), "NonFinite");
  EXPECT_STREQ(solveStatusToString(SolveStatus::LinearSolveFailed), "LinearSolveFailed");
  EXPECT_STREQ(solveStatusToString(SolveStatus::ExternalSolverFailure), "ExternalSolverFailure");
  EXPECT_STREQ(solveStatusToString(SolveStatus::UnsupportedBackend), "UnsupportedBackend");
  EXPECT_STREQ(solveStatusToString(999), "Unknown");
}

TEST(SolverResultGTest, MapsExternalRawStatusCodes)
{
  EXPECT_EQ(makeIpoptSolverResult(0).status, SolveStatus::Converged);
  EXPECT_EQ(makeIpoptSolverResult(1).status, SolveStatus::Converged);
  EXPECT_EQ(makeIpoptSolverResult(-1).status, SolveStatus::MaxIterations);
  EXPECT_EQ(makeIpoptSolverResult(3).status, SolveStatus::StepTooSmall);
  EXPECT_EQ(makeIpoptSolverResult(-13).status, SolveStatus::NonFinite);
  EXPECT_EQ(makeIpoptSolverResult(-3).status, SolveStatus::LinearSolveFailed);
  EXPECT_EQ(makeIpoptSolverResult(-199).status, SolveStatus::ExternalSolverFailure);
  EXPECT_EQ(makeIpoptSolverResult(-199).rawStatusCode, -199);

  EXPECT_EQ(makeKnitroSolverResult(0).status, SolveStatus::Converged);
  EXPECT_EQ(makeKnitroSolverResult(-100).status, SolveStatus::Converged);
  EXPECT_EQ(makeKnitroSolverResult(-400).status, SolveStatus::MaxIterations);
  EXPECT_EQ(makeKnitroSolverResult(-500).status, SolveStatus::LinearSolveFailed);
  EXPECT_EQ(makeKnitroSolverResult(-200).status, SolveStatus::ExternalSolverFailure);
  EXPECT_EQ(makeKnitroSolverResult(-200).rawStatusCode, -200);
}

TEST(SolverResultGTest, EncodesDynamicAndStrictAcceptancePolicies)
{
  EXPECT_TRUE(acceptsStrictSolveStatus(SolveStatus::Converged));
  EXPECT_FALSE(acceptsStrictSolveStatus(SolveStatus::MaxIterations));

  EXPECT_TRUE(acceptsDynamicSolveStatus(SolveStatus::Converged));
  EXPECT_TRUE(acceptsDynamicSolveStatus(SolveStatus::MaxIterations));
  EXPECT_TRUE(acceptsDynamicSolveStatus(SolveStatus::StepTooSmall));
  EXPECT_FALSE(acceptsDynamicSolveStatus(SolveStatus::LineSearchFailed));
  EXPECT_FALSE(acceptsDynamicSolveStatus(SolveStatus::ExternalSolverFailure));
}

TEST(NewtonSolverGTest, SolveRecordsIterationsForImmediateConvergence)
{
  initializeLogging();

  auto energy = std::make_shared<TestQuadraticEnergy>(1);
  ES::VXd x(1);
  x[0] = 0.0;

  NewtonSolver::SolverParam solverParam;
  const std::vector<int> fixedDOFs;
  NewtonSolver solver(x.data(), solverParam, energy, fixedDOFs);

  const SolverResult result = solver.solve(x.data(), 8, 1e-10, 0);

  EXPECT_EQ(result.status, SolveStatus::Converged);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_EQ(result.rawStatusCode, static_cast<int>(SolveStatus::Converged));
}

TEST(NewtonSolverGTest, SolveRecordsZeroIterationsForZeroMaxIter)
{
  initializeLogging();

  auto energy = std::make_shared<TestQuadraticEnergy>(1);
  ES::VXd x(1);
  x[0] = 2.0;

  NewtonSolver::SolverParam solverParam;
  const std::vector<int> fixedDOFs;
  NewtonSolver solver(x.data(), solverParam, energy, fixedDOFs);

  const SolverResult result = solver.solve(x.data(), 0, 1e-10, 0);

  EXPECT_EQ(result.status, SolveStatus::MaxIterations);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_EQ(result.rawStatusCode, static_cast<int>(SolveStatus::MaxIterations));
}
