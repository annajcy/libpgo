#include "solver/newton/NewtonSolver.h"

#include "EigenSupport.h"
#include "solver/newton/lineSearch.h"
#include "energy/evaluationStateAwareEnergy.h"
#include "energy/lineSearchAwareEnergy.h"
#include "pgoLogging.h"
#include "processMemory.h"
#include "scopedProfileSection.h"
#include "threadRuntime.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <chrono>

using namespace pgo;
using namespace pgo::NonlinearOptimization;
namespace ES = pgo::EigenSupport;
using hclock = std::chrono::steady_clock;

namespace
{
void logMemoryCheckpoint(const char *stage)
{
  constexpr double bytesPerMiB = 1024.0 * 1024.0;
  const Profiling::ProcessMemoryUsage usage =
    Profiling::recordProcessMemoryProfileCounters(stage);
  SPDLOG_LOGGER_INFO(Logging::lgr(),
    "Process memory checkpoint stage={} currentMiB={:.2f} peakMiB={:.2f}",
    stage, usage.residentBytes / bytesPerMiB, usage.peakResidentBytes / bytesPerMiB);
}

void ensureDiagonalEntries(ES::SpMatD &A)
{
  const int n = std::min<int>(A.rows(), A.cols());
  for (int i = 0; i < n; i++)
    A.coeffRef(i, i) += 0.0;
  A.makeCompressed();
}

constexpr double kRelTolFactor = 1e-5;        // absolute->relative gradient tolerance scale
constexpr double kLooseRelFactor = 1e-4;      // FP-limit "good enough" relative reduction
constexpr double kGradSmallThreshold = 1e-4;  // below this gradient, drop damping entirely
constexpr double kLambdaScaleFloor = 1e-8;    // below this damping scale, snap to zero
constexpr double kDampingDecay = 0.9;         // per-iteration damping decay when gradient is not increasing
constexpr double kStepTooSmallEps = 1e-15;    // accepted step max-norm below this == stalled
constexpr double kHistoryGradNormInit = 1e100;
constexpr int kLineSearchMaxIter = 50;
constexpr int kLineSearchMaxIterDescent = 3;  // fewer iters when the full step already decreases energy

class LineSearchScope
{
public:
  LineSearchScope(const LineSearchAwareEnergy *energy, EigenSupport::ConstRefVecXd x,
    EigenSupport::ConstRefVecXd dx):
    energy_(energy)
  {
    if (energy_)
      energy_->beginLineSearch(x, dx);
  }

  ~LineSearchScope()
  {
    if (energy_)
      energy_->endLineSearch();
  }

  LineSearchScope(const LineSearchScope &) = delete;
  LineSearchScope &operator=(const LineSearchScope &) = delete;

private:
  const LineSearchAwareEnergy *energy_ = nullptr;
};

class ScopedBoolAssignment
{
public:
  ScopedBoolAssignment(bool &target, bool value):
    target_(target), oldValue_(target)
  {
    target_ = value;
  }

  ~ScopedBoolAssignment()
  {
    target_ = oldValue_;
  }

  ScopedBoolAssignment(const ScopedBoolAssignment &) = delete;
  ScopedBoolAssignment &operator=(const ScopedBoolAssignment &) = delete;

private:
  bool &target_;
  bool oldValue_;
};

}  // namespace

inline double dura(const hclock::time_point &t1, const hclock::time_point &t2)
{
  return std::chrono::duration<double>(t2 - t1).count();
}

void NewtonSolver::resetLinearSolver()
{
  if (!solver)
    return;
  solver.reset();
  invalidateLinearSolverPatternCache();
}

void NewtonSolver::StepStrategy::finalize(SolveContext &ctx)
{
  Eigen::Map<ES::VXd>(ctx.xOut, s.energy->getNumDOFs()) = s.x;
}

// Standard damped-Newton with line search; accepts a fraction of the Newton step.
class NewtonSolver::LineSearchStrategy : public NewtonSolver::StepStrategy
{
public:
  explicit LineSearchStrategy(NewtonSolver &solver): StepStrategy(solver) {}

  bool takeStep(SolveContext &ctx) override
  {
    const double eng = ctx.state.energy;
    const double gradMaxNorm = ctx.state.gradMaxNorm;

    StepAcceptance accepted = s.runLineSearchStep(eng, ctx.verbose, ctx.printGap, ctx.iter);
    if (ctx.currentTrace) {
      ctx.currentTrace->acceptedAlpha = accepted.effectiveAlpha;
      ctx.currentTrace->acceptedStepNorm = accepted.acceptedStepNorm;
      ctx.currentTrace->acceptedStepMaxNorm = accepted.acceptedStepMaxNorm;
      ctx.currentTrace->lineSearchIterations = accepted.lineSearchIterations;
      ctx.currentTrace->lineSearchStatus = static_cast<int>(accepted.lineSearchStatus);
      ctx.currentTrace->minFeasibleAlpha = accepted.feasibleAlpha;
      ctx.currentTrace->feasibleAlpha = accepted.feasibleAlpha;
      ctx.currentTrace->lineSearchAlpha = accepted.lineSearchAlpha;
      ctx.currentTrace->energyAfter = accepted.acceptedEnergy;
      ctx.currentTrace->energyDelta = accepted.acceptedEnergy - ctx.currentTrace->energyBefore;
      ctx.currentTrace->contactClampCount = accepted.contactClampCount;
      ctx.currentTrace->materialClampCount = accepted.materialClampCount;
      ctx.currentTrace->lineSearchSeconds = accepted.lineSearchSeconds;
    }
    if (accepted.nonFinite()) {
      ctx.status = SolveStatus::NonFinite;
      ctx.completedIterations = ctx.iter + 1;
      return true;
    }

    const double stepSize = accepted.acceptedStepMaxNorm;
    const bool stepTooSmall = stepSize < kStepTooSmallEps;
    auto finishStepTooSmall = [&]() {
      // Same loose relative fallback as the line-search-failed branch below.
      ctx.status = s.resolveFpLimitFallback(SolveStatus::StepTooSmall, gradMaxNorm, ctx.lambda0, ctx.epsilon);
      ctx.completedIterations = ctx.iter + 1;
      if (ctx.verbose >= 1) {
        std::cout << "    Iter=" << ctx.iter << "; dx = " << stepSize
                  << "; dx too small; ||grad||_max=" << gradMaxNorm
                  << " (lambda0=" << ctx.lambda0 << ", looseRel=" << ctx.lambda0 * kLooseRelFactor << ")"
                  << "; status=" << solveStatusToString(ctx.status) << "." << std::endl;
      }
      return true;
    };

    if (accepted.lineSearchStatus != NewtonLineSearchStatus::Accepted) {
      if (stepTooSmall && lineSearchEnergyWithinFpTolerance(eng, accepted.acceptedEnergy)) {
        return finishStepTooSmall();
      }

      // Loose relative fallback: if line search can't find descent but Newton already
      // reduced the gradient by 4+ orders of magnitude from the initial state, treat
      // this as converged-at-FP-limit rather than failure.
      ctx.status = s.resolveFpLimitFallback(SolveStatus::LineSearchFailed, gradMaxNorm, ctx.lambda0, ctx.epsilon);
      ctx.completedIterations = ctx.iter + 1;
      if (ctx.verbose >= 1) {
        std::cout << "    Iter=" << ctx.iter << "; line search failed; ||grad||_max=" << gradMaxNorm
                  << " (lambda0=" << ctx.lambda0 << ", looseRel=" << ctx.lambda0 * kLooseRelFactor << ")"
                  << "; status=" << solveStatusToString(ctx.status) << "." << std::endl;
      }
      return true;
    }

    s.x += s.deltax * accepted.lineSearchAlpha;

    if (stepSize < kStepTooSmallEps) {
      return finishStepTooSmall();
    }

    return false;
  }
};

// Shared lifecycle for the fixed-step modes: track the lowest-gradient iterate and
// return that iterate at the end (optionally stopping once the gradient increases).
class NewtonSolver::HistoryTrackingStrategy : public NewtonSolver::StepStrategy
{
public:
  explicit HistoryTrackingStrategy(NewtonSolver &solver): StepStrategy(solver) {}

  void begin(SolveContext &) override { s.historyGradNormMin = kHistoryGradNormInit; }

  bool beforeStep(SolveContext &ctx) override
  {
    const double gradMaxNorm = ctx.state.gradMaxNorm;
    if (gradMaxNorm < s.historyGradNormMin) {
      s.historyx.noalias() = s.x;
      s.historyGradNormMin = gradMaxNorm;
      return false;
    }
    return s.solverParam.stopAfterIncrease != 0;
  }

  void finalize(SolveContext &ctx) override
  {
    if (s.historyGradNormMin < ctx.epsilon)
      ctx.status = SolveStatus::Converged;
    if (ctx.verbose >= 1)
      std::cout << "        Final ||grad||=" << s.historyGradNormMin << std::endl;
    Eigen::Map<ES::VXd>(ctx.xOut, s.energy->getNumDOFs()) = s.historyx;
  }
};

// Full Newton step each iteration, recomputing the gradient afterwards.
class NewtonSolver::SubiterationOneStrategy : public NewtonSolver::HistoryTrackingStrategy
{
public:
  explicit SubiterationOneStrategy(NewtonSolver &solver): HistoryTrackingStrategy(solver) {}

  bool takeStep(SolveContext &ctx) override
  {
    s.x += s.deltax;
    if (ctx.currentTrace) {
      ctx.currentTrace->acceptedAlpha = 1.0;
      ctx.currentTrace->acceptedStepNorm = s.deltax.norm();
      ctx.currentTrace->acceptedStepMaxNorm = s.deltax.cwiseAbs().maxCoeff();
      ctx.currentTrace->lineSearchStatus = static_cast<int>(NewtonLineSearchStatus::Accepted);
    }
    s.historyx.noalias() = s.x;
    s.dispatchPrepareEvaluationState(s.x);
    memset(s.grad.data(), 0, sizeof(double) * s.grad.size());
    s.energy->gradient(s.x, s.grad);
    s.filterVector(s.grad);

    s.historyGradNormMin = s.grad.norm();

    if (ctx.verbose >= 2 && ctx.iter % ctx.printGap == 0) {
      const double value = s.energy->func(s.x);
      std::cout << "    f=" << value << std::endl;
    }
    return false;
  }
};

// Fixed fraction of the Newton step each iteration.
class NewtonSolver::StaticDampingStrategy : public NewtonSolver::HistoryTrackingStrategy
{
public:
  explicit StaticDampingStrategy(NewtonSolver &solver): HistoryTrackingStrategy(solver) {}

  bool takeStep(SolveContext &ctx) override
  {
    if (ctx.verbose >= 2 && ctx.iter % ctx.printGap == 0)
      std::cout << "        E= " << ctx.state.energy << "; ||grad||_max=" << s.grad.cwiseAbs().maxCoeff() << "; ||grad||=" << s.grad.norm() << std::endl;
    s.x += s.deltax * s.solverParam.alpha;
    if (ctx.currentTrace) {
      ctx.currentTrace->acceptedAlpha = s.solverParam.alpha;
      ctx.currentTrace->acceptedStepNorm = std::abs(s.solverParam.alpha) * s.deltax.norm();
      ctx.currentTrace->acceptedStepMaxNorm = std::abs(s.solverParam.alpha) * s.deltax.cwiseAbs().maxCoeff();
      ctx.currentTrace->lineSearchStatus = static_cast<int>(NewtonLineSearchStatus::Accepted);
    }
    return false;
  }
};

NewtonSolver::NewtonSolver(const double *x_, SolverParam sp, PotentialEnergy_const_p energy_, const std::vector<int> &fixedDOFs_, const double *fixedValues_):
  energy(energy_), solverParam(sp),
  sparseSolverSelector(sp.sparseSolver ? sp.sparseSolver : std::make_shared<AutoSparseSolverSelector>())
{
  n3 = (int)energy->getNumDOFs();
  allDOFs.resize(energy->getNumDOFs());
  std::iota(allDOFs.begin(), allDOFs.end(), 0);

  grad.resize(energy->getNumDOFs());
  x = Eigen::Map<const ES::VXd>(x_, energy->getNumDOFs());
  deltax.resize(energy->getNumDOFs());
  lineSearchx.resize(energy->getNumDOFs());
  historyx.resize(energy->getNumDOFs());
  historyGradNormMin = kHistoryGradNormInit;

  setFixedDOFs(fixedDOFs_, fixedValues_);

  if (solverParam.sst == SST_SUBITERATION_LINE_SEARCH) {
    lineSearchEval = [this](const double *x, double *f, double *grad) -> int {
      const Eigen::Map<const ES::VXd> xEval(x, n3);
      dispatchPrepareEvaluationState(xEval);

      if (f && grad) {
        memset(grad, 0, sizeof(double) * n3);
        *f = energy->funcGradient(xEval, Eigen::Map<ES::VXd>(grad, n3));
      }
      else {
        if (f)
          *f = energy->func(xEval);

        if (grad) {
          memset(grad, 0, sizeof(double) * n3);
          energy->gradient(xEval, Eigen::Map<ES::VXd>(grad, n3));
        }
      }

      return 0;
    };

    // The helper is per-solve scratch; immutable policies borrow it via the context.
    lineSearchHelper.emplace(n3, lineSearchEval);
    if (solverParam.lineSearch)
      lineSearchPolicy = solverParam.lineSearch;
    else
      lineSearchPolicy = std::make_shared<BacktrackingLineSearchPolicy>(BacktrackingLineSearchPolicy::Params{});
  }

  // Termination policy
  if (solverParam.termination)
    terminationPolicy = solverParam.termination;
  else
    terminationPolicy = std::make_shared<FixedNewtonTerminationPolicy>();

  // Damping policy
  if (solverParam.damping)
    dampingPolicy = solverParam.damping;
  else
    dampingPolicy = std::make_shared<NoDampingPolicy>();

  switch (solverParam.sst) {
  case SST_SUBITERATION_ONE:
    stepStrategy = std::make_unique<SubiterationOneStrategy>(*this);
    break;
  case SST_SUBITERATION_STATIC_DAMPING:
    stepStrategy = std::make_unique<StaticDampingStrategy>(*this);
    break;
  case SST_SUBITERATION_LINE_SEARCH:
  default:
    stepStrategy = std::make_unique<LineSearchStrategy>(*this);
    break;
  }
}

NewtonSolver::~NewtonSolver() noexcept
{
  closeLinearSolver();
}

NewtonSolver::CleanupMetrics NewtonSolver::closeLinearSolver()
{
  const hclock::time_point cleanupStart = hclock::now();
  resetLinearSolver();
  return CleanupMetrics{ dura(cleanupStart, hclock::now()) };
}

void NewtonSolver::setFixedDOFs(const std::vector<int> &fixedDOFs_, const double *fixedValues_)
{
  const bool sameAsBefore = fixedDOFs_.size() != 0 && fixedDOFs.size() == fixedDOFs_.size() &&
    std::memcmp(fixedDOFs.data(), fixedDOFs_.data(), sizeof(int) * fixedDOFs.size()) == 0;

  if (!sameAsBefore) {
    resetLinearSolver();
    invalidateLinearSolverPatternCache();

    fixedDOFs = fixedDOFs_;

    // dofs
    rhsb2s.clear();
    rhss2b.clear();
    if (fixedDOFs.empty()) {
      rhs.resize(n3);
      deltaxSmall.resize(0);
      A11 = ES::SpMatD();
      A11Mapping = ES::SpMatI();
    }
    else {
      ES::removeRows(n3, fixedDOFs, rhsb2s, rhss2b);
      rhs.resize(n3 - (int)fixedDOFs.size());
      deltaxSmall.resize(n3 - (int)fixedDOFs.size());
    }

    if (energy->isHessianTopologyFixed()) {
      // sparse matrix
      const hclock::time_point hessianStart = hclock::now();
      dispatchPrepareEvaluationState(x);
      energy->hessianAlloc(sysFull);
      energy->hessianInPlace(x, sysFull);
      pendingSetupMetrics.initialHessianSeconds += dura(hessianStart, hclock::now());
      logMemoryCheckpoint("newton.after_full_hessian");

      const hclock::time_point reducedSystemStart = hclock::now();
      if (fixedDOFs.empty()) {
        ensureDiagonalEntries(sysFull);
      }
      else {
        ES::removeRowsCols(sysFull, fixedDOFs, A11);
        ES::removeRowsCols(sysFull, A11, fixedDOFs, A11Mapping);
      }

      ES::SpMatD &A = activeSystemMatrix();
      A.makeCompressed();
      pendingSetupMetrics.initialReducedSystemSeconds += dura(reducedSystemStart, hclock::now());
      if (fixedDOFs.empty())
        logMemoryCheckpoint("newton.after_no_fixed_fast_path");
      else
        logMemoryCheckpoint("newton.after_reduced_system_setup");
      pendingSetupMetrics.initialSymbolicAnalyzeSeconds += makeLinearSolver(A);
    }
  }

  fixedValues = ES::Mp<const ES::VXd>(fixedValues_, fixedDOFs_.size());
}

double NewtonSolver::makeLinearSolver(const ES::SpMatD &A)
{
  Profiling::ScopedProfileSection scopedProfile("newton.solver.symbolic_analyze");
  Profiling::ScopedThreadRuntimePhase threadProfile("newton.solver.symbolic_analyze");
  const hclock::time_point symbolicAnalyzeStart = hclock::now();
  solver.reset();
  auto newSolver = sparseSolverSelector->build(A);
  if (newSolver == nullptr)
    throw std::runtime_error("Newton sparse solver selector returned a null backend");
  solver = std::move(newSolver);
  const double symbolicAnalyzeSeconds = dura(symbolicAnalyzeStart, hclock::now());
  invalidateLinearSolverPatternCache();
  updateLinearSolverPatternCache(A);
  logMemoryCheckpoint("newton.after_sparse_symbolic_analysis");
  return symbolicAnalyzeSeconds;
}

void NewtonSolver::invalidateLinearSolverPatternCache()
{
  linearSolverPatternCache.valid = false;
  linearSolverPatternCache.rows = 0;
  linearSolverPatternCache.cols = 0;
  linearSolverPatternCache.nnz = 0;
  linearSolverPatternCache.outerIndices.clear();
  linearSolverPatternCache.innerIndices.clear();
}

bool NewtonSolver::activeSystemPatternMatches(const ES::SpMatD &A) const
{
  const LinearSolverPatternCache &cache = linearSolverPatternCache;
  if (!cache.valid)
    return false;

  if (cache.rows != A.rows() || cache.cols != A.cols() || cache.nnz != A.nonZeros())
    return false;

  const Eigen::Index outerCount = A.outerSize() + 1;
  if (cache.outerIndices.size() != static_cast<size_t>(outerCount) ||
    cache.innerIndices.size() != static_cast<size_t>(A.nonZeros()))
    return false;

  using StorageIndex = ES::SpMatD::StorageIndex;
  const std::size_t outerBytes = static_cast<std::size_t>(outerCount) * sizeof(StorageIndex);
  if (std::memcmp(A.outerIndexPtr(), cache.outerIndices.data(), outerBytes) != 0)
    return false;

  const std::size_t innerBytes = static_cast<std::size_t>(A.nonZeros()) * sizeof(StorageIndex);
  return innerBytes == 0 ||
    std::memcmp(A.innerIndexPtr(), cache.innerIndices.data(), innerBytes) == 0;
}

void NewtonSolver::updateLinearSolverPatternCache(const ES::SpMatD &A)
{
  LinearSolverPatternCache &cache = linearSolverPatternCache;
  cache.rows = A.rows();
  cache.cols = A.cols();
  cache.nnz = A.nonZeros();

  const Eigen::Index outerCount = A.outerSize() + 1;
  cache.outerIndices.resize(static_cast<size_t>(outerCount));
  for (Eigen::Index i = 0; i < outerCount; i++)
    cache.outerIndices[static_cast<size_t>(i)] = A.outerIndexPtr()[i];

  cache.innerIndices.resize(static_cast<size_t>(A.nonZeros()));
  for (Eigen::Index i = 0; i < A.nonZeros(); i++)
    cache.innerIndices[static_cast<size_t>(i)] = A.innerIndexPtr()[i];

  cache.valid = true;
}

SolverResult NewtonSolver::solve(double *x_, int numIter, double epsilon, int verbose)
{
  hclock::time_point t1 = hclock::now();
  solveDiagnostics.reset();
  solveDiagnostics.initialHessianSeconds = pendingSetupMetrics.initialHessianSeconds;
  solveDiagnostics.initialReducedSystemSeconds = pendingSetupMetrics.initialReducedSystemSeconds;
  solveDiagnostics.initialSymbolicAnalyzeSeconds = pendingSetupMetrics.initialSymbolicAnalyzeSeconds;
  solveDiagnostics.newtonTotalSymbolicAnalyzeSeconds = pendingSetupMetrics.initialSymbolicAnalyzeSeconds;
  pendingSetupMetrics.reset();

  x.noalias() = Eigen::Map<ES::VXd>(x_, energy->getNumDOFs());

  SolveContext ctx;
  ctx.xOut = x_;
  ctx.epsilon = epsilon;
  ctx.verbose = verbose;
  ctx.printGap = (verbose == 3) ? 1 : 10;

  stepStrategy->begin(ctx);
  applyFixedValues();

  for (ctx.iter = 0; ctx.iter < numIter; ctx.iter++) {
    const hclock::time_point iterationStart = hclock::now();
    if (verbose >= 2 && ctx.iter % ctx.printGap == 0)
      std::cout << "    Iter=" << ctx.iter << std::endl;

    // we solve f(x_i) + K(x_i) deltax = 0
    ctx.state = evaluateCurrentState(ctx.iter, epsilon, ctx.lambda0, ctx.hasInitialGradNorm);
    solveDiagnostics.newtonTotalEvaluateCurrentStateSeconds += ctx.state.evaluateCurrentStateSeconds;
    solveDiagnostics.newtonTotalFuncGradHessianSeconds += ctx.state.funcGradHessianSeconds;
    if (ctx.state.nonFiniteEnergy) {
      ctx.status = SolveStatus::NonFinite;
      if (verbose >= 1)
        std::cout << "    Iter=" << ctx.iter << "; energy is non-finite; status=" << solveStatusToString(ctx.status) << std::endl;
      break;
    }
    if (ctx.state.nonFiniteGradient) {
      ctx.status = SolveStatus::NonFinite;
      if (verbose >= 1)
        std::cout << "    Iter=" << ctx.iter << "; gradient is non-finite; status=" << solveStatusToString(ctx.status) << std::endl;
      break;
    }
    solveDiagnostics.completeLastNewtonIterationAfterState(ctx.iter, ctx.state.energy, ctx.state.gradNorm, ctx.state.gradMaxNorm);

    const double eng = ctx.state.energy;
    const double gradMaxNorm = ctx.state.gradMaxNorm;
    if (!ctx.hasInitialGradNorm) {
      ctx.lambda0 = ctx.state.lambda0;
      ctx.gradMaxNormLast = gradMaxNorm;
      ctx.hasInitialGradNorm = true;
    }

    if (verbose >= 2 && ctx.iter % ctx.printGap == 0)
      std::cout << "        E= " << eng << "; ||grad||_max=" << gradMaxNorm << "; ||x||=" << x.norm() << std::endl;

    // Convergence test via pluggable termination policy. Default policy accepts
    // either absolute (||grad||_max < eps) or relative reduction from initial
    // gradient (||grad||_max < lambda0 * relTolFactor).
    {
      NewtonIterationContext policyCtx;
      policyCtx.iteration = ctx.iter;
      policyCtx.energy = eng;
      policyCtx.gradMaxNorm = gradMaxNorm;
      policyCtx.gradNorm = ctx.state.gradNorm;
      policyCtx.lambda0 = ctx.lambda0;
      policyCtx.lambdaScale = ctx.lambdaScale;
      policyCtx.epsilon = ctx.epsilon;

      const NewtonTerminationDecision termDecision = terminationPolicy->beforeLinearSolve(policyCtx);
      if (termDecision == NewtonTerminationDecision::Converged) {
        ctx.status = SolveStatus::Converged;
        ctx.completedIterations = ctx.iter;
        solveDiagnostics.recordFinalGradientStats(ctx.state.gradNorm, gradMaxNorm);
        const bool fixedTerminationPolicy = dynamic_cast<const FixedNewtonTerminationPolicy *>(terminationPolicy.get()) != nullptr;
        if (!fixedTerminationPolicy) {
          solveDiagnostics.recordNewtonConvergence(
            NewtonConvergenceReason::TerminationPolicy,
            std::numeric_limits<double>::quiet_NaN());
        }
        else if (ctx.state.absConverged) {
          solveDiagnostics.recordNewtonConvergence(
            NewtonConvergenceReason::AbsoluteGradient,
            ctx.epsilon);
        }
        else if (ctx.state.relConverged) {
          solveDiagnostics.recordNewtonConvergence(
            NewtonConvergenceReason::RelativeGradient,
            ctx.state.relThreshold);
        }
        else {
          solveDiagnostics.recordNewtonConvergence(
            NewtonConvergenceReason::TerminationPolicy,
            std::numeric_limits<double>::quiet_NaN());
        }
        if (verbose >= 1) {
          const double relThreshold = ctx.lambda0 * kRelTolFactor;
          std::cout << "    Iter=" << ctx.iter << "; ||grad||_max=" << gradMaxNorm
                    << (gradMaxNorm < ctx.epsilon ? " < eps" : " < lambda0*relTol")
                    << " (eps=" << epsilon << ", relThreshold=" << relThreshold
                    << "). Done.; status=" << solveStatusToString(ctx.status) << std::endl;
        }
        break;
      }
      else if (termDecision == NewtonTerminationDecision::Failed) {
        ctx.status = SolveStatus::NonFinite;
        ctx.completedIterations = ctx.iter;
        if (verbose >= 1)
          std::cout << "    Iter=" << ctx.iter << "; termination policy failed; status="
                    << solveStatusToString(ctx.status) << std::endl;
        break;
      }
    }

    if (stepStrategy->beforeStep(ctx))
      break;

    NewtonIterationTrace trace;
    trace.iteration = ctx.iter;
    trace.energyBefore = ctx.state.energy;
    trace.gradNormBefore = ctx.state.gradNorm;
    trace.gradMaxBefore = ctx.state.gradMaxNorm;
    trace.evaluateCurrentStateSeconds = ctx.state.evaluateCurrentStateSeconds;
    trace.funcGradHessianSeconds = ctx.state.funcGradHessianSeconds;
    ctx.currentTrace = &trace;

    // Update damping scale inline (was updateDampingScale method)
    {
      constexpr double kGradSmallThreshold = 1e-4;
      constexpr double kLambdaScaleFloor = 1e-8;
      constexpr double kDampingDecay = 0.9;
      if (gradMaxNorm < kGradSmallThreshold)
        ctx.lambdaScale = 0.0;
      else if (ctx.lambdaScale < kLambdaScaleFloor)
        ctx.lambdaScale = 0.0;
      else if (gradMaxNorm <= ctx.gradMaxNormLast)
        ctx.lambdaScale *= kDampingDecay;
      ctx.lambdaScale = std::min(ctx.lambdaScale, 1.0);
    }
    ctx.gradMaxNormLast = gradMaxNorm;

    const hclock::time_point prepareReducedSystemStart = hclock::now();
    const bool fixedHessianTopology = prepareReducedSystem(ctx.lambdaScale, ctx.lambda0);
    trace.prepareReducedSystemSeconds = dura(prepareReducedSystemStart, hclock::now());
    trace.dampingValue = solveDiagnostics.lastDampingValue;

    const hclock::time_point ensureLinearSolverStart = hclock::now();
    const EnsureLinearSolverResult symbolicResult = ensureLinearSolver(fixedHessianTopology);
    trace.ensureLinearSolverSeconds = dura(ensureLinearSolverStart, hclock::now());
    trace.symbolicRebuilt = symbolicResult.symbolicRebuilt;
    trace.symbolicAnalyzeSeconds = symbolicResult.symbolicAnalyzeSeconds;
    trace.hessianRows = solveDiagnostics.lastActiveSystemRows;
    trace.hessianCols = solveDiagnostics.lastActiveSystemCols;
    trace.hessianNnz = solveDiagnostics.lastActiveSystemNnz;

    const LinearSolveResult linearSolveResult = solveReducedNewtonDirection(fixedHessianTopology);
    trace.factorizeSeconds = linearSolveResult.factorizeSeconds;
    trace.solveSeconds = linearSolveResult.solveSeconds;
    if (!linearSolveResult.success) {
      ctx.status = SolveStatus::NonFinite;
      ctx.completedIterations = ctx.iter + 1;
      trace.iterationWallSeconds = dura(iterationStart, hclock::now());
      solveDiagnostics.recordNewtonIteration(trace);
      ctx.currentTrace = nullptr;
      if (verbose >= 1)
        std::cout << "    Iter=" << ctx.iter << "; dx is non-finite; status=" << solveStatusToString(ctx.status) << std::endl;
      break;
    }

    if (verbose >= 3 && ctx.iter % ctx.printGap == 0)
      std::cout << (activeSystemMatrix() * activeNewtonDirection() - rhs).norm() << ' ' << rhs.norm() << std::endl;

    restoreSysFullDamping();

    const hclock::time_point expandReducedStepStart = hclock::now();
    const bool expandedStep = expandReducedStep();
    trace.expandReducedStepSeconds = dura(expandReducedStepStart, hclock::now());
    if (!expandedStep) {
      ctx.status = SolveStatus::NonFinite;
      ctx.completedIterations = ctx.iter + 1;
      trace.iterationWallSeconds = dura(iterationStart, hclock::now());
      solveDiagnostics.recordNewtonIteration(trace);
      ctx.currentTrace = nullptr;
      if (verbose >= 1)
        std::cout << "    Iter=" << ctx.iter << "; dx is non-finite; status=" << solveStatusToString(ctx.status) << std::endl;
      break;
    }

    const double rawStepMaxNorm = deltax.cwiseAbs().maxCoeff();
    solveDiagnostics.recordNewtonDirection(grad.dot(deltax), rawStepMaxNorm, deltax.norm());
    trace.gradDotDx = solveDiagnostics.lastGradDotDx;
    trace.rawStepMaxNorm = solveDiagnostics.lastRawStepMaxNorm;
    trace.rawStepNorm = solveDiagnostics.lastRawStepNorm;
    if (verbose >= 2 && ctx.iter % ctx.printGap == 0)
      std::cout << "        rawStepMaxNorm=" << rawStepMaxNorm << std::endl;

    const bool stopAfterStep = stepStrategy->takeStep(ctx);
    trace.iterationWallSeconds = dura(iterationStart, hclock::now());
    solveDiagnostics.recordNewtonIteration(trace);
    ctx.currentTrace = nullptr;
    if (stopAfterStep)
      break;

    // afterStep termination check
    {
      NewtonIterationContext postCtx;
      postCtx.iteration = ctx.iter;
      postCtx.energy = ctx.state.energy;
      // Post-step gradient values are set by completeLastNewtonIterationAfterState
      // on the next iteration — leave as NaN for now.
      postCtx.lambda0 = ctx.lambda0;
      postCtx.lambdaScale = ctx.lambdaScale;
      postCtx.epsilon = ctx.epsilon;
      postCtx.acceptedAlpha = trace.acceptedAlpha;
      postCtx.acceptedEnergy = trace.energyAfter;
      postCtx.acceptedStepMaxNorm = trace.acceptedStepMaxNorm;

      const NewtonTerminationDecision postDecision = terminationPolicy->afterStep(postCtx);
      if (postDecision == NewtonTerminationDecision::Converged) {
        ctx.status = SolveStatus::Converged;
        ctx.completedIterations = ctx.iter + 1;
        solveDiagnostics.recordNewtonConvergence(
          NewtonConvergenceReason::TerminationPolicy,
          std::numeric_limits<double>::quiet_NaN());
        if (verbose >= 1)
          std::cout << "    Iter=" << ctx.iter << "; after-step convergence; status="
                    << solveStatusToString(ctx.status) << std::endl;
        break;
      }
      else if (postDecision == NewtonTerminationDecision::Failed) {
        ctx.status = SolveStatus::NonFinite;
        ctx.completedIterations = ctx.iter + 1;
        if (verbose >= 1)
          std::cout << "    Iter=" << ctx.iter << "; after-step policy failed; status="
                    << solveStatusToString(ctx.status) << std::endl;
        break;
      }
    }

    if (stepFunc) {
      stepFunc(x, ctx.iter);
    }

    ctx.completedIterations = ctx.iter + 1;
  }

  stepStrategy->finalize(ctx);

  hclock::time_point t2 = hclock::now();

  double timeCost = dura(t1, t2);
  solveDiagnostics.newtonSolveSeconds = timeCost;

  SPDLOG_LOGGER_INFO(Logging::lgr(), "Newton solve time: {}", timeCost);
  SolverResult result;
  result.status = ctx.status;
  result.iterations = ctx.completedIterations;
  result.rawStatusCode = static_cast<int>(ctx.status);
  result.diagnostics = solveDiagnostics;
  if (solveDiagnostics.hasFinalGradientStats) {
    result.hasFinalGradientStats = true;
    result.finalGradientNorm = solveDiagnostics.finalGradientNorm;
    result.finalGradientMaxNorm = solveDiagnostics.finalGradientMaxNorm;
  }
  return result;
}

void NewtonSolver::filterVector(ES::VXd &v)
{
  for (int dof : fixedDOFs)
    v[dof] = 0;
}

void NewtonSolver::applyFixedValues()
{
  for (size_t i = 0; i < fixedDOFs.size(); i++) {
    x[fixedDOFs[i]] = fixedValues[i];
  }
}

NewtonSolver::IterationState NewtonSolver::evaluateCurrentState(int iter, double epsilon, double lambda0, bool hasInitialGradNorm)
{
  const hclock::time_point evaluationStart = hclock::now();
  Profiling::ScopedProfileSection scopedProfile("newton.evaluate_current_state");
  Profiling::ScopedThreadRuntimePhase threadProfile("newton.evaluate_current_state");
  IterationState state;
  state.iter = iter;

  memset(grad.data(), 0, sizeof(double) * grad.size());
  {
    Profiling::ScopedProfileSection prepareProfile("newton.prepare_evaluation_state");
    dispatchPrepareEvaluationState(x);
  }
  {
    const hclock::time_point start = hclock::now();
    Profiling::ScopedProfileSection energyProfile("newton.funcGradientHessian");
    Profiling::ScopedThreadRuntimePhase energyThreadProfile("newton.funcGradientHessian");
    state.energy = energy->funcGradientHessian(x, grad, sysFull);
    state.funcGradHessianSeconds = dura(start, hclock::now());
  }
  if (!energy->isHessianTopologyFixed())
    logMemoryCheckpoint("newton.after_full_hessian");
  if (!std::isfinite(state.energy)) {
    state.nonFiniteEnergy = true;
    state.evaluateCurrentStateSeconds = dura(evaluationStart, hclock::now());
    return state;
  }

  {
    Profiling::ScopedProfileSection compressProfile("newton.compress_hessian_and_gradient_stats");
    sysFull.makeCompressed();
    filterVector(grad);

    state.gradMaxNorm = grad.cwiseAbs().maxCoeff();
    state.gradNorm = grad.norm();
  }
  if (!grad.allFinite() || !std::isfinite(state.gradMaxNorm)) {
    state.nonFiniteGradient = true;
    state.evaluateCurrentStateSeconds = dura(evaluationStart, hclock::now());
    return state;
  }

  state.lambda0 = hasInitialGradNorm ? lambda0 : state.gradMaxNorm;
  state.relThreshold = state.lambda0 * kRelTolFactor;
  state.absConverged = state.gradMaxNorm < epsilon;
  state.relConverged = state.gradMaxNorm < state.relThreshold;
  state.evaluateCurrentStateSeconds = dura(evaluationStart, hclock::now());
  return state;
}

bool NewtonSolver::looseRelativeConverged(double gradMaxNorm, double lambda0) const
{
  return gradMaxNorm < lambda0 * kLooseRelFactor;
}

SolveStatus NewtonSolver::resolveFpLimitFallback(SolveStatus failStatus, double gradMaxNorm, double lambda0, double epsilon)
{
  if (gradMaxNorm < epsilon) {
    solveDiagnostics.recordFinalGradientStats(grad.norm(), gradMaxNorm);
    solveDiagnostics.recordNewtonConvergence(
      NewtonConvergenceReason::AbsoluteGradientFpLimit,
      epsilon);
    return SolveStatus::Converged;
  }
  if (looseRelativeConverged(gradMaxNorm, lambda0)) {
    solveDiagnostics.recordFinalGradientStats(grad.norm(), gradMaxNorm);
    solveDiagnostics.recordNewtonConvergence(
      NewtonConvergenceReason::LooseRelativeGradientFpLimit,
      lambda0 * kLooseRelFactor);
    return SolveStatus::Converged;
  }
  return failStatus;
}

bool NewtonSolver::prepareReducedSystem(double lambdaScale, double lambda0)
{
  Profiling::ScopedProfileSection scopedProfile("newton.prepare_reduced_system");
  Profiling::ScopedThreadRuntimePhase threadProfile("newton.prepare_reduced_system");
  const bool fixedHessianTopology = energy->isHessianTopologyFixed();
  dampingAddedToSysFull = 0.0;
  if (fixedDOFs.empty()) {
    ensureDiagonalEntries(sysFull);
    rhs.noalias() = -grad;
    logMemoryCheckpoint("newton.after_no_fixed_fast_path");
  }
  else {
    if (fixedHessianTopology) {
      ES::transferBigToSmall(sysFull, A11, A11Mapping, 1);
    }
    else {
      ES::removeRowsCols(sysFull, fixedDOFs, A11);
      logMemoryCheckpoint("newton.after_reduced_system_setup");
    }

    ES::transferBigToSmall(grad, rhs, rhsb2s, 1);
    rhs *= -1.0;
  }

  if (dampingPolicy) {
    NewtonIterationContext policyCtx;
    policyCtx.lambdaScale = lambdaScale;
    policyCtx.lambda0 = lambda0;
    const double damping = dampingPolicy->dampingForIteration(policyCtx);
    solveDiagnostics.recordDampingValue(damping);
    if (damping != 0.0) {
      ES::SpMatD &A = activeSystemMatrix();
      for (int i = 0; i < A.rows(); i++) {
        A.coeffRef(i, i) += damping;
      }
      if (fixedDOFs.empty())
        dampingAddedToSysFull = damping;
    }
  }
  else {
    solveDiagnostics.recordDampingValue(0.0);
  }

  return fixedHessianTopology;
}

NewtonSolver::EnsureLinearSolverResult NewtonSolver::ensureLinearSolver(bool fixedHessianTopology)
{
  Profiling::ScopedProfileSection scopedProfile("newton.ensure_linear_solver");
  (void)fixedHessianTopology;
  EnsureLinearSolverResult result;

  ES::SpMatD &A = activeSystemMatrix();
  {
    Profiling::ScopedProfileSection compressProfile("make_compressed");
    A.makeCompressed();
  }

  if (solver != nullptr) {
    bool patternMatches = false;
    {
      Profiling::ScopedProfileSection matchProfile("pattern_match");
      patternMatches = activeSystemPatternMatches(A);
    }
    if (patternMatches) {
      solveDiagnostics.recordActiveSystemPattern(A.rows(), A.cols(), A.nonZeros());
      solveDiagnostics.recordLinearSolverSymbolicReuse(true);
      result.symbolicRebuilt = false;
      return result;
    }
  }

  solveDiagnostics.recordActiveSystemPattern(A.rows(), A.cols(), A.nonZeros());
  solveDiagnostics.recordLinearSolverSymbolicReuse(false);
  result.symbolicRebuilt = true;
  result.symbolicAnalyzeSeconds = makeLinearSolver(A);
  return result;
}

NewtonSolver::LinearSolveResult NewtonSolver::solveReducedNewtonDirection(bool fixedHessianTopology)
{
  LinearSolveResult result;
  {
    Profiling::ScopedProfileSection scopedProfile("solver.linear_solve");
    Profiling::ScopedThreadRuntimePhase threadProfile("solver.linear_solve");
    ES::SpMatD &A = activeSystemMatrix();
    ES::VXd &dx = activeNewtonDirection();
    {
      const hclock::time_point start = hclock::now();
      Profiling::ScopedProfileSection factorProfile("solver.factorize");
      Profiling::ScopedThreadRuntimePhase factorThreadProfile("solver.factorize");
      result.success = solver->factorize(A);
      result.factorizeSeconds = dura(start, hclock::now());
    }
    if (result.success) {
      const hclock::time_point start = hclock::now();
      Profiling::ScopedProfileSection solveProfile("solver.solve");
      Profiling::ScopedThreadRuntimePhase solveThreadProfile("solver.solve");
      result.success = solver->solve(A, dx.data(), rhs.data());
      result.solveSeconds = dura(start, hclock::now());
    }
    if (!result.success)
      solver.reset();
  }
  if (!result.success) {
    restoreSysFullDamping();
    invalidateLinearSolverPatternCache();
    return result;
  }
  (void)fixedHessianTopology;

  result.success = activeNewtonDirection().allFinite();
  return result;
}

bool NewtonSolver::expandReducedStep()
{
  Profiling::ScopedProfileSection scopedProfile("newton.expand_reduced_step");
  if (fixedDOFs.empty())
    return deltax.allFinite();

  memset(deltax.data(), 0, sizeof(double) * n3);
  ES::transferSmallToBig(deltaxSmall, deltax, rhss2b);
  return deltax.allFinite();
}

ES::SpMatD &NewtonSolver::activeSystemMatrix()
{
  return fixedDOFs.empty() ? sysFull : A11;
}

const ES::SpMatD &NewtonSolver::activeSystemMatrix() const
{
  return fixedDOFs.empty() ? sysFull : A11;
}

ES::VXd &NewtonSolver::activeNewtonDirection()
{
  return fixedDOFs.empty() ? deltax : deltaxSmall;
}

const ES::VXd &NewtonSolver::activeNewtonDirection() const
{
  return fixedDOFs.empty() ? deltax : deltaxSmall;
}

void NewtonSolver::restoreSysFullDamping()
{
  if (dampingAddedToSysFull == 0.0)
    return;

  for (int i = 0; i < sysFull.rows(); i++)
    sysFull.coeffRef(i, i) -= dampingAddedToSysFull;
  dampingAddedToSysFull = 0.0;
}

void NewtonSolver::dispatchPrepareEvaluationState(EigenSupport::ConstRefVecXd xEval) const
{
  if (lineSearchEvaluationStateFrozen)
    return;
  if (const auto *aware = dynamic_cast<const EvaluationStateAwareEnergy *>(energy.get()))
    aware->prepareEvaluationState(xEval);
}

NewtonSolver::StepAcceptance NewtonSolver::runLineSearchStep(double currentEnergy, int verbose, int printGap, int iter)
{
  const hclock::time_point start = hclock::now();
  Profiling::ScopedProfileSection scopedProfile("newton.line_search_step");
  Profiling::ScopedThreadRuntimePhase threadProfile("newton.line_search_step");
  StepAcceptance accepted;
  int lineSearchIterations = 0;
  const std::int64_t materialClampCountBefore = solveDiagnostics.clampCounts[static_cast<int>(StepSource::Material)];
  const std::int64_t contactClampCountBefore = solveDiagnostics.clampCounts[static_cast<int>(StepSource::Contact)];

  StepConstraint maxStep;
  {
    Profiling::ScopedProfileSection maxStepProfile("newton.line_search.max_step_limit");
    Profiling::ScopedThreadRuntimePhase maxStepThreadProfile("newton.line_search.max_step_limit");
    maxStep = energy->computeMaxStepLimit(x, deltax, &solveDiagnostics);
  }
  accepted.feasibleAlpha = maxStep.alpha;
  accepted.materialClampCount =
    solveDiagnostics.clampCounts[static_cast<int>(StepSource::Material)] - materialClampCountBefore;
  accepted.contactClampCount =
    solveDiagnostics.clampCounts[static_cast<int>(StepSource::Contact)] - contactClampCountBefore;
  if (!std::isfinite(accepted.feasibleAlpha)) {
    accepted.nonFiniteReason = StepAcceptance::NonFiniteReason::FeasibleAlpha;
    accepted.lineSearchSeconds = dura(start, hclock::now());
    if (verbose >= 1)
      std::cout << "    Iter=" << iter << "; feasible alpha is non-finite; status=" << solveStatusToString(SolveStatus::NonFinite) << std::endl;
    return accepted;
  }
  deltax *= accepted.feasibleAlpha;

  {
    // Freeze the energy's line-search state (e.g. IPC's swept active-set superset)
    // only when the energy advertises the capability AND the chosen method stays
    // within the energy's valid alpha window. Golden/Brent may expand the bracket
    // past that window, so they run without the frozen state.
    const auto *lineSearchAware = dynamic_cast<const LineSearchAwareEnergy *>(energy.get());
    const bool useFrozenActiveSet = lineSearchAware != nullptr &&
      lineSearchPolicy->maxProbeAlpha() <= lineSearchAware->maxValidLineSearchAlpha();
    ScopedBoolAssignment frozenGuard(lineSearchEvaluationStateFrozen, useFrozenActiveSet);
    LineSearchScope lineSearchScope(useFrozenActiveSet ? lineSearchAware : nullptr, x, deltax);

    lineSearchx.noalias() = x + deltax;
    {
      Profiling::ScopedProfileSection prepareProfile("newton.line_search.prepare_trial_state");
      dispatchPrepareEvaluationState(lineSearchx);
    }
    {
      Profiling::ScopedProfileSection trialProfile("newton.line_search.trial_energy");
      accepted.acceptedEnergy = energy->func(lineSearchx);
    }
    if (!std::isfinite(accepted.acceptedEnergy)) {
      accepted.nonFiniteReason = StepAcceptance::NonFiniteReason::TrialEnergy;
      accepted.lineSearchStatus = NewtonLineSearchStatus::NonFiniteEnergy;
      if (verbose >= 1)
        std::cout << "    Iter=" << iter << "; trial energy is non-finite; status=" << solveStatusToString(SolveStatus::NonFinite) << std::endl;
    }
    else {
      int maxIter = kLineSearchMaxIter;
      if (accepted.acceptedEnergy < currentEnergy) {
        maxIter = kLineSearchMaxIterDescent;
      }

      NewtonLineSearchContext ctx{ x, deltax, grad, currentEnergy, accepted.acceptedEnergy, maxIter,
        *lineSearchHelper, lineSearchEval };
      NewtonLineSearchResult ret;
      {
        Profiling::ScopedProfileSection searchProfile("newton.line_search.policy_search");
        Profiling::ScopedThreadRuntimePhase searchThreadProfile("newton.line_search.policy_search");
        ret = lineSearchPolicy->search(ctx);
      }
      accepted.lineSearchAlpha = ret.alpha;
      accepted.acceptedEnergy = ret.energy;
      accepted.lineSearchStatus = ret.status;
      lineSearchIterations = ret.iterations;

      if (!std::isfinite(accepted.lineSearchAlpha) || !std::isfinite(accepted.acceptedEnergy))
        accepted.nonFiniteReason = StepAcceptance::NonFiniteReason::LineSearchResult;
    }
  }

  if (accepted.nonFinite()) {
    accepted.lineSearchIterations = lineSearchIterations;
    accepted.lineSearchSeconds = dura(start, hclock::now());
    if (verbose >= 1 && accepted.nonFiniteReason == StepAcceptance::NonFiniteReason::LineSearchResult)
      std::cout << "    Iter=" << iter << "; line search energy is non-finite; status=" << solveStatusToString(SolveStatus::NonFinite) << std::endl;
    return accepted;
  }

  accepted.effectiveAlpha = accepted.feasibleAlpha * accepted.lineSearchAlpha;
  accepted.acceptedStepMaxNorm = std::abs(accepted.lineSearchAlpha) * deltax.cwiseAbs().maxCoeff();
  accepted.acceptedStepNorm = std::abs(accepted.lineSearchAlpha) * deltax.norm();
  accepted.lineSearchIterations = lineSearchIterations;
  accepted.lineSearchSeconds = dura(start, hclock::now());
  solveDiagnostics.recordLineSearch(accepted.feasibleAlpha, accepted.lineSearchAlpha,
    accepted.effectiveAlpha, lineSearchIterations, currentEnergy,
    accepted.acceptedEnergy, accepted.acceptedStepMaxNorm);

  if (verbose >= 2 && iter % printGap == 0) {
    std::cout << "        feasibleAlpha=" << accepted.feasibleAlpha << std::endl;
    if (accepted.feasibleAlpha < 1.0) {
      const char *clampSource = solveDiagnostics.lastMaxStep.source == StepSource::Contact ? "contact" : "material";
      std::cout << "        feasible alpha clamped by " << clampSource << ": "
                << solveDiagnostics.lastMaxStep.alpha << std::endl;
    }
    std::cout << "        lineSearchAlpha=" << accepted.lineSearchAlpha << std::endl;
    std::cout << "        effectiveAlpha=" << accepted.effectiveAlpha << std::endl;
    std::cout << "        acceptedStepMaxNorm=" << accepted.acceptedStepMaxNorm << std::endl;
    std::cout << "        acceptedEnergy=" << accepted.acceptedEnergy << std::endl;
  }

  return accepted;
}
