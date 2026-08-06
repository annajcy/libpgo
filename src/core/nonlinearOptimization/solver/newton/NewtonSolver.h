#pragma once

#include "solver/newton/newtonLineSearchPolicy.h"
#include "solver/newton/newtonTerminationPolicy.h"
#include "solver/newton/newtonDampingPolicy.h"
#include "solver/newton/newtonSparseSolverBackend.h"
#include "energy/potentialEnergy.h"
#include "solver/common/solverResult.h"

#include <cfloat>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace pgo
{
namespace NonlinearOptimization
{
class NewtonSolver
{
public:
  enum SolverSubiterationType
  {
    SST_SUBITERATION_LINE_SEARCH,
    SST_SUBITERATION_STATIC_DAMPING,
    SST_SUBITERATION_ONE
  };

  struct SolverParam
  {
    double alpha = 0.5;
    SolverSubiterationType sst = SST_SUBITERATION_LINE_SEARCH;
    // Concrete (immutable) line-search policy handle; null selects a default
    // Backtracking policy.
    std::shared_ptr<const NewtonLineSearchPolicy> lineSearch;
    // Sparse linear-solver selector; null picks Auto (best available backend).
    std::shared_ptr<const NewtonSparseSolverSelector> sparseSolver;
    int stopAfterIncrease = 1;
    std::shared_ptr<const NewtonTerminationPolicy> termination;
    std::shared_ptr<const NewtonDampingPolicy> damping;
  };

  NewtonSolver(const double *x, SolverParam sp, PotentialEnergy_const_p energy_,
    const std::vector<int> &fixedDOFs, const double *fixedValues_ = nullptr);
  ~NewtonSolver() noexcept;

  struct CleanupMetrics
  {
    double wallSeconds = 0.0;
  };

  void setFixedDOFs(const std::vector<int> &fixedDOFs, const double *fixedValues);
  SolverResult solve(double *x, int numIter, int verbose);
  CleanupMetrics closeLinearSolver();

  using StepFunc = std::function<void(const EigenSupport::VXd &, int)>;
  void setStepFunc(StepFunc func) { stepFunc = func; }

  const EigenSupport::VXd &getx() const { return x; }
  const SolveDiagnostics &getSolveDiagnostics() const { return solveDiagnostics; }

protected:
  struct IterationState
  {
    int iter = 0;
    double energy = 0.0;
    double gradMaxNorm = 0.0;
    double gradNorm = 0.0;
    double lambda0 = 1.0;
    bool nonFiniteEnergy = false;
    bool nonFiniteGradient = false;
    double evaluateCurrentStateSeconds = 0.0;
    double funcGradHessianSeconds = 0.0;
  };

  struct StepAcceptance
  {
    double feasibleAlpha = 1.0;
    double lineSearchAlpha = 1.0;
    double effectiveAlpha = 1.0;
    double acceptedEnergy = 0.0;
    double acceptedStepMaxNorm = 0.0;
    double acceptedStepNorm = 0.0;
    double lineSearchSeconds = 0.0;
    int lineSearchIterations = 0;
    std::int64_t materialClampCount = 0;
    std::int64_t contactClampCount = 0;
    NewtonLineSearchStatus lineSearchStatus = NewtonLineSearchStatus::NoAcceptableStep;

    enum class NonFiniteReason
    {
      None,
      FeasibleAlpha,
      TrialEnergy,
      LineSearchResult
    };

    NonFiniteReason nonFiniteReason = NonFiniteReason::None;

    bool nonFinite() const
    {
      return nonFiniteReason != NonFiniteReason::None ||
        lineSearchStatus == NewtonLineSearchStatus::NonFiniteEnergy ||
        lineSearchStatus == NewtonLineSearchStatus::NonFiniteAlpha;
    }
  };

  struct LinearSolverPatternCache
  {
    using StorageIndex = EigenSupport::SpMatD::StorageIndex;

    Eigen::Index rows = 0;
    Eigen::Index cols = 0;
    Eigen::Index nnz = 0;
    std::vector<StorageIndex> outerIndices;
    std::vector<StorageIndex> innerIndices;
    bool valid = false;
  };

  struct EnsureLinearSolverResult
  {
    bool symbolicRebuilt = false;
    double symbolicAnalyzeSeconds = 0.0;
  };

  struct LinearSolveResult
  {
    bool success = false;
    double factorizeSeconds = 0.0;
    double solveSeconds = 0.0;
  };

  struct PendingSetupMetrics
  {
    double initialHessianSeconds = 0.0;
    double initialReducedSystemSeconds = 0.0;
    double initialSymbolicAnalyzeSeconds = 0.0;

    void reset() { *this = PendingSetupMetrics{}; }
  };

  // Mutable per-solve state shared between solve() and the step strategy.
  struct SolveContext
  {
    IterationState state;
    double *xOut = nullptr;
    double lambda0 = 1.0;
    double lambdaScale = 1.0;
    double gradMaxNormLast = 0.0;
    bool hasInitialGradNorm = false;
    int verbose = 0;
    int printGap = 10;
    int iter = 0;
    SolveStatus status = SolveStatus::MaxIterations;
    int completedIterations = 0;
    NewtonIterationTrace *currentTrace = nullptr;
  };

  // Per-subiteration step strategy. Each concrete strategy owns one mode's full
  // lifecycle (begin -> beforeStep -> takeStep -> finalize). begin/beforeStep
  // default to no-ops; the default finalize writes the working x to xOut.
  // beforeStep/takeStep return true when the iteration loop should stop.
  class StepStrategy
  {
  public:
    explicit StepStrategy(NewtonSolver &solver): s(solver) {}
    virtual ~StepStrategy() = default;
    virtual void begin(SolveContext &ctx) {}
    virtual bool beforeStep(SolveContext &ctx) { return false; }
    virtual bool takeStep(SolveContext &ctx) = 0;
    virtual void finalize(SolveContext &ctx);

  protected:
    NewtonSolver &s;
  };
  class LineSearchStrategy;
  class HistoryTrackingStrategy;
  class SubiterationOneStrategy;
  class StaticDampingStrategy;

  void filterVector(EigenSupport::VXd &v);
  void applyFixedValues();
  IterationState evaluateCurrentState(int iter, double lambda0, bool hasInitialGradNorm);
  bool prepareReducedSystem(double lambdaScale, double lambda0);
  EnsureLinearSolverResult ensureLinearSolver(bool fixedHessianTopology);
  LinearSolveResult solveReducedNewtonDirection(bool fixedHessianTopology);
  bool expandReducedStep();
  EigenSupport::SpMatD &activeSystemMatrix();
  const EigenSupport::SpMatD &activeSystemMatrix() const;
  EigenSupport::VXd &activeNewtonDirection();
  const EigenSupport::VXd &activeNewtonDirection() const;
  void restoreSysFullDamping();
  StepAcceptance runLineSearchStep(double currentEnergy, int verbose, int printGap, int iter);
  void dispatchPrepareEvaluationState(EigenSupport::ConstRefVecXd xEval) const;
  bool looseRelativeConverged(double gradMaxNorm, double lambda0) const;
  SolveStatus resolveFpLimitFallback(SolveStatus failStatus, double gradMaxNorm, double lambda0);
  double makeLinearSolver(const EigenSupport::SpMatD &A);
  void invalidateLinearSolverPatternCache();
  bool activeSystemPatternMatches(const EigenSupport::SpMatD &A) const;
  void updateLinearSolverPatternCache(const EigenSupport::SpMatD &A);
  void resetLinearSolver();

  PotentialEnergy_const_p energy;
  SolverParam solverParam;
  std::shared_ptr<const NewtonSparseSolverSelector> sparseSolverSelector;
  std::shared_ptr<const NewtonLineSearchPolicy> lineSearchPolicy;
  std::shared_ptr<const NewtonTerminationPolicy> terminationPolicy;
  std::shared_ptr<const NewtonDampingPolicy> dampingPolicy;
  // Per-solve line-search scratch owned here; policies borrow these via the context.
  LineSearch::EvaluateFunction lineSearchEval;
  std::optional<LineSearch> lineSearchHelper;

  EigenSupport::VXd x, grad, deltax, deltaxSmall, lineSearchx;
  EigenSupport::SpMatD sysFull, A11, A12;
  EigenSupport::SpMatI A11Mapping, A12Mapping;

  std::unique_ptr<NewtonSparseSolverBackend> solver;
  LinearSolverPatternCache linearSolverPatternCache;

  std::vector<int> allDOFs, fixedDOFs;
  std::vector<int> rhss2b, rhsb2s;
  EigenSupport::VXd rhs;
  EigenSupport::VXd fixedValues;
  double dampingAddedToSysFull = 0.0;
  int n3;

  EigenSupport::VXd historyx;
  double historyGradNormMin;
  SolveDiagnostics solveDiagnostics;
  PendingSetupMetrics pendingSetupMetrics;
  bool lineSearchEvaluationStateFrozen = false;

  std::unique_ptr<StepStrategy> stepStrategy;
  StepFunc stepFunc;
};
}  // namespace NonlinearOptimization
}  // namespace pgo
