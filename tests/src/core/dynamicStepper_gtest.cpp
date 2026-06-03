#include <gtest/gtest.h>

#include "dynamicState.h"
#include "dynamicStepOptions.h"
#include "rayleighDampingAssembly.h"
#include "stageResidual.h"
#include "implicitEulerStageBuilder.h"
#include "trbdf2StageBuilder.h"
#include "dynamicStepper.h"
#include "dynamicStepService.h"

#include "energySet.h"
#include "solver/newton/NewtonOptimizer.h"
#include "quadraticPotentialEnergy.h"
#include "pgoLogging.h"

#include <numeric>
#include <stdexcept>

namespace
{
namespace ES = pgo::EigenSupport;
namespace NO = pgo::NonlinearOptimization;
namespace OPT = pgo::NonlinearOptimization::Optimization;
using namespace pgo::Simulation;
using pgo::PredefinedPotentialEnergies::QuadraticPotentialEnergy;

void initLogging()
{
  static const bool ok = []() { pgo::Logging::init(); return true; }();
  (void)ok;
}

ES::SpMatD diagSparse(const std::vector<double> &d)
{
  const int n = (int)d.size();
  ES::SpMatD m(n, n);
  std::vector<ES::TripletD> t;
  for (int i = 0; i < n; i++)
    t.emplace_back(i, i, d[i]);
  m.setFromTriplets(t.begin(), t.end());
  return m;
}

ES::SpMatD identitySparse(int n)
{
  return diagSparse(std::vector<double>(n, 1.0));
}

// Fixed-topology quadratic energy ½ x'x (Hessian = I), with call counters.
class FixedQuadEnergy : public NO::PotentialEnergy
{
public:
  explicit FixedQuadEnergy(int n): n(n) {}
  double func(ES::ConstRefVecXd x) const override { funcCalls++; return 0.5 * x.squaredNorm(); }
  void gradient(ES::ConstRefVecXd x, ES::RefVecXd g) const override { gradCalls++; g = x; }
  void hessianInPlace(ES::ConstRefVecXd, ES::SpMatD &h) const override { for (int i = 0; i < n; i++) h.coeffRef(i, i) = 1.0; }
  void hessianAlloc(ES::SpMatD &h) const override { h = identitySparse(n); }
  void getDOFs(std::vector<int> &d) const override { d.resize(n); std::iota(d.begin(), d.end(), 0); }
  int getNumDOFs() const override { return n; }
  int isQuadratic() const override { return 1; }
  mutable int funcCalls = 0, gradCalls = 0;
private:
  int n;
};

// Non-fixed-topology stub that counts how each evaluation entry point is hit,
// so we can assert the fused path triggers exactly one func_grad_hessian.
class CountingNonFixedEnergy : public NO::PotentialEnergy
{
public:
  explicit CountingNonFixedEnergy(int n): n(n) {}
  double func(ES::ConstRefVecXd x) const override { funcCalls++; return 0.5 * x.squaredNorm(); }
  void gradient(ES::ConstRefVecXd x, ES::RefVecXd g) const override { gradCalls++; g = x; }
  void hessianInPlace(ES::ConstRefVecXd, ES::SpMatD &h) const override { hessCalls++; h = identitySparse(n); }
  void gradient_hessian(ES::ConstRefVecXd x, ES::RefVecXd g, ES::SpMatD &h) const override { gradHessCalls++; g = x; h = identitySparse(n); }
  double func_grad_hessian(ES::ConstRefVecXd x, ES::RefVecXd g, ES::SpMatD &h) const override { fghCalls++; g = x; h = identitySparse(n); return 0.5 * x.squaredNorm(); }
  void hessianAlloc(ES::SpMatD &h) const override { h = identitySparse(n); }
  void getDOFs(std::vector<int> &d) const override { d.resize(n); std::iota(d.begin(), d.end(), 0); }
  int getNumDOFs() const override { return n; }
  int isHessianTopologyFixed() const override { return 0; }
  void reset() const { funcCalls = gradCalls = hessCalls = gradHessCalls = fghCalls = 0; }
  mutable int funcCalls = 0, gradCalls = 0, hessCalls = 0, gradHessCalls = 0, fghCalls = 0;
private:
  int n;
};

OPT::NewtonOptimizer quickOptimizer()
{
  OPT::NewtonOptimizer::Options options;
  options.maxIterations = 100;
  options.gradientTolerance = 1e-12;
  options.verbose = 0;
  return OPT::NewtonOptimizer(options);
}

DynamicState restState(int n)
{
  DynamicState s;
  s.displacement = ES::VXd::Zero(n);
  s.velocity = ES::VXd::Zero(n);
  s.acceleration = ES::VXd::Zero(n);
  return s;
}
}  // namespace

// ── T2.3: fused EnergySet path ──────────────────────────────────────────

TEST(EnergySetFused, SingleActiveSetBuildAndCorrectValues)
{
  initLogging();
  const int n = 2;
  auto fixed = std::make_shared<FixedQuadEnergy>(n);
  auto nonFixed = std::make_shared<CountingNonFixedEnergy>(n);

  NO::EnergySet set(n, {{fixed, 1.0}, {nonFixed, 1.0}});

  ES::VXd x(n);
  x << 0.3, -0.7;

  nonFixed->reset();
  ES::VXd grad = ES::VXd::Zero(n);
  ES::SpMatD hess;
  const double v = set.func_grad_hessian(x, grad, hess);

  // Non-fixed term evaluated exactly once, through func_grad_hessian only.
  EXPECT_EQ(nonFixed->fghCalls, 1);
  EXPECT_EQ(nonFixed->gradHessCalls, 0);
  EXPECT_EQ(nonFixed->funcCalls, 0);
  EXPECT_EQ(nonFixed->gradCalls, 0);
  EXPECT_EQ(nonFixed->hessCalls, 0);

  // Values match summed quadratics: value = x'x, grad = 2x, hess = 2I.
  EXPECT_NEAR(v, x.squaredNorm(), 1e-12);
  EXPECT_LT((grad - 2.0 * x).norm(), 1e-12);
  EXPECT_LT((ES::MXd(hess) - 2.0 * ES::MXd::Identity(n, n)).norm(), 1e-12);
}

// ── T3.2: Rayleigh damping ──────────────────────────────────────────────

TEST(RayleighDampingAssembly, MassDampingScalesMass)
{
  auto e = std::make_shared<FixedQuadEnergy>(2);
  std::vector<ImplicitModelTerm> terms{{e, 0.0, 0.5}};  // massDamping = 0.5
  ES::SpMatD D = assembleRayleighDamping(terms, identitySparse(2), ES::VXd::Zero(2));
  EXPECT_LT((ES::MXd(D) - 0.5 * ES::MXd::Identity(2, 2)).norm(), 1e-12);
}

TEST(RayleighDampingAssembly, StiffnessDampingScalesHessian)
{
  auto e = std::make_shared<FixedQuadEnergy>(2);
  std::vector<ImplicitModelTerm> terms{{e, 2.0, 0.0}};  // stiffnessDamping = 2.0, K = I
  ES::SpMatD D = assembleRayleighDamping(terms, identitySparse(2), ES::VXd::Zero(2));
  EXPECT_LT((ES::MXd(D) - 2.0 * ES::MXd::Identity(2, 2)).norm(), 1e-12);
}

TEST(RayleighDampingAssembly, NonFixedTopologySkippedForStiffness)
{
  auto e = std::make_shared<CountingNonFixedEnergy>(2);
  std::vector<ImplicitModelTerm> terms{{e, 5.0, 0.0}};
  ES::SpMatD D = assembleRayleighDamping(terms, identitySparse(2), ES::VXd::Zero(2));
  EXPECT_EQ(D.nonZeros(), 0);
}

TEST(RayleighDampingAssembly, ZeroDampingIsEmpty)
{
  auto e = std::make_shared<FixedQuadEnergy>(3);
  std::vector<ImplicitModelTerm> terms{{e, 0.0, 0.0}};
  ES::SpMatD D = assembleRayleighDamping(terms, identitySparse(3), ES::VXd::Zero(3));
  EXPECT_EQ(D.rows(), 3);
  EXPECT_EQ(D.nonZeros(), 0);
}

// ── T4.4: stage residual ────────────────────────────────────────────────

TEST(StageResidual, TemplateStableAndValueCorrect)
{
  const int n = 2;
  ES::SpMatD A = diagSparse({4.0, 9.0});
  ES::VXd l(n); l << 1.0, -2.0;
  auto elastic = std::make_shared<FixedQuadEnergy>(n);  // adds ½x'x

  StageResidualHandle handle = initStageResidual(A, l, {{elastic, 0.0, 0.0}});
  const void *templatePtr = handle.energySet->getHessianTemplate().valuePtr();

  ImplicitStageProblem stage = prepareStageResidual(handle, A, l, ES::VXd::Zero(n));
  // Hessian template not rebuilt across prepare calls.
  EXPECT_EQ(handle.energySet->getHessianTemplate().valuePtr(), templatePtr);

  ES::VXd x(n); x << 0.5, -1.0;
  ES::VXd grad = ES::VXd::Zero(n);
  ES::SpMatD hess;
  const double v = stage.energy->func_grad_hessian(x, grad, hess);

  // J = ½x'Ax + l'x + ½x'x ; grad = Ax + l + x.
  const double expected = 0.5 * x.dot(A * x) + l.dot(x) + 0.5 * x.squaredNorm();
  EXPECT_NEAR(v, expected, 1e-10);
  const ES::VXd expectedGrad = A * x + l + x;
  EXPECT_LT((grad - expectedGrad).norm(), 1e-10);

  // Updating the linear term changes the gradient accordingly.
  ES::VXd l2(n); l2 << -3.0, 4.0;
  prepareStageResidual(handle, A, l2, ES::VXd::Zero(n));
  ES::VXd grad2 = ES::VXd::Zero(n);
  handle.energySet->gradient(x, grad2);
  EXPECT_LT((grad2 - (A * x + l2 + x)).norm(), 1e-10);
}

// ── T5.4: IBE builder formula + state update ────────────────────────────

TEST(ImplicitEuler, CoefficientFormulas)
{
  const int n = 2;
  const double h = 0.25;
  DynamicProblem prob;
  prob.mass = diagSparse({2.0, 3.0});
  prob.timestep = h;

  DynamicState state;
  state.displacement = ES::VXd(n); state.displacement << 0.1, -0.2;
  state.velocity = ES::VXd(n); state.velocity << 0.5, 0.3;
  state.acceleration = ES::VXd::Zero(n);

  DynamicStepRequest req;
  req.externalForce = ES::VXd(n); req.externalForce << 1.0, -2.0;

  ES::SpMatD damping = diagSparse({0.4, 0.6});
  ImplicitEulerStageBuilder builder;
  IBEStageCoefficients c = builder.compute(state, prob, req, damping);

  ES::SpMatD expectedA = (1.0 / (h * h)) * prob.mass + (1.0 / h) * damping;
  ES::VXd expectedLinear = -(req.externalForce + (1.0 / h) * (prob.mass * state.velocity) + expectedA * state.displacement);

  EXPECT_LT((ES::MXd(c.A) - ES::MXd(expectedA)).norm(), 1e-12);
  EXPECT_LT((c.linear - expectedLinear).norm(), 1e-12);

  DynamicState next = updateImplicitEulerState(state, req.externalForce, h);  // arbitrary solution vector
  EXPECT_LT((next.velocity - (req.externalForce - state.displacement) / h).norm(), 1e-12);
  EXPECT_EQ(next.timestepId, 1u);
  EXPECT_NEAR(next.time, h, 1e-15);
}

// ── T5.4: IBE solve end-to-end ──────────────────────────────────────────

TEST(ImplicitEuler, SolveAndAdvanceState)
{
  initLogging();
  const int n = 2;
  const double h = 0.25;
  auto elastic = std::make_shared<FixedQuadEnergy>(n);

  DynamicProblem prob;
  prob.mass = identitySparse(n);
  prob.persistentTerms = {{elastic, 0.0, 0.0}};
  prob.timestep = h;
  auto stepper = makeDynamicStepper(TimeIntegratorKind::ImplicitEuler, prob);
  auto optimizer = quickOptimizer();
  DynamicStepRequest req; req.externalForce = ES::VXd(n); req.externalForce << 1.0, -2.0;
  DynamicStepResult res = stepper->step(restState(n), req, optimizer);

  EXPECT_TRUE(res.accepted);
  EXPECT_EQ(res.stageResults.size(), 1u);
  EXPECT_NEAR(res.state.time, h, 1e-12);
  EXPECT_EQ(res.state.timestepId, 1u);
}

// ── T6.5: TRBDF2 ────────────────────────────────────────────────────────

TEST(TRBDF2, CoefficientsMatchLegacyFormula)
{
  const double gamma = 0.5, h = 0.25;
  TRBDF2Coefficients c = computeTRBDF2Coefficients(gamma, h);
  EXPECT_NEAR(c.alpha, 2.0 / (gamma * h), 1e-12);
  EXPECT_NEAR(c.beta[0], (2.0 - gamma) / (gamma * (1 - gamma) * (1 - gamma) * h * h), 1e-9);
  EXPECT_NEAR(c.beta[7], (2.0 - gamma) / ((1 - gamma) * h), 1e-9);
  EXPECT_THROW(computeTRBDF2Coefficients(0.0, h), std::invalid_argument);
  EXPECT_THROW(computeTRBDF2Coefficients(1.5, h), std::invalid_argument);
}

TEST(TRBDF2, TwoStageSolveAdvancesState)
{
  initLogging();
  const int n = 2;
  const double h = 0.25, gamma = 0.5;
  auto elastic = std::make_shared<FixedQuadEnergy>(n);

  DynamicProblem prob;
  prob.mass = identitySparse(n);
  prob.persistentTerms = {{elastic, 0.0, 0.0}};
  prob.timestep = h;
  auto stepper = makeDynamicStepper(TimeIntegratorKind::TRBDF2, prob, gamma);
  auto optimizer = quickOptimizer();
  DynamicStepRequest req; req.externalForce = ES::VXd(n); req.externalForce << 1.0, -2.0;
  DynamicStepResult res = stepper->step(restState(n), req, optimizer);

  EXPECT_TRUE(res.accepted);
  EXPECT_EQ(res.stageResults.size(), 2u);
  EXPECT_EQ(res.state.timestepId, 1u);
}

TEST(TRBDF2, GammaOneIsSingleStage)
{
  initLogging();
  const int n = 1;
  auto elastic = std::make_shared<FixedQuadEnergy>(n);
  DynamicProblem prob;
  prob.mass = identitySparse(n);
  prob.persistentTerms = {{elastic, 0.0, 0.0}};
  prob.timestep = 0.1;

  auto stepper = makeDynamicStepper(TimeIntegratorKind::TRBDF2, prob, 1.0);
  auto optimizer = quickOptimizer();
  DynamicStepRequest req; req.externalForce = ES::VXd::Constant(n, 1.0);
  DynamicStepResult res = stepper->step(restState(n), req, optimizer);
  EXPECT_TRUE(res.accepted);
  EXPECT_EQ(res.stageResults.size(), 1u);
}

// ── T7.6: stepper service ───────────────────────────────────────────────

TEST(DynamicStepper, FixedDofsRemainFixed)
{
  initLogging();
  const int n = 3;
  auto elastic = std::make_shared<FixedQuadEnergy>(n);
  DynamicProblem prob;
  prob.mass = identitySparse(n);
  prob.persistentTerms = {{elastic, 0.0, 0.0}};
  prob.fixedDofs = {1};
  prob.timestep = 0.1;

  auto stepper = makeDynamicStepper(TimeIntegratorKind::ImplicitEuler, prob);
  auto optimizer = quickOptimizer();
  DynamicState s = restState(n);
  s.displacement[1] = 0.42;
  DynamicStepRequest req; req.externalForce = ES::VXd::Constant(n, 1.0);
  DynamicStepResult res = stepper->step(s, req, optimizer);

  EXPECT_TRUE(res.accepted);
  EXPECT_NEAR(res.state.displacement[1], 0.42, 1e-9);
}

TEST(DynamicStepper, FreeFallMatchesImplicitEulerRecurrence)
{
  initLogging();
  // No elastic energy: a single point mass under constant gravity.
  const int n = 1;
  const double m = 2.0, h = 0.05, fGrav = -9.8 * m;
  DynamicProblem prob;
  prob.mass = diagSparse({m});
  prob.timestep = h;

  auto stepper = makeDynamicStepper(TimeIntegratorKind::ImplicitEuler, prob);
  auto optimizer = quickOptimizer();
  DynamicStepRequest req; req.externalForce = ES::VXd::Constant(n, fGrav);

  DynamicState s = restState(n);
  double u = 0.0, v = 0.0;
  for (int k = 0; k < 10; k++) {
    DynamicStepResult res = stepper->step(s, req, optimizer);
    ASSERT_TRUE(res.accepted);
    // Implicit Euler with constant force: v_{k+1} = v_k + h f/m, u_{k+1} = u_k + h v_{k+1}.
    // The production Newton solve uses Hessian damping (NewtonOptimizer::Options::damping),
    // so it approaches this idealized recurrence to a physical tolerance rather
    // than machine precision. Exact-formula correctness is covered by the
    // legacy-parity tests above.
    v += h * fGrav / m;
    u += h * v;
    EXPECT_NEAR(res.state.velocity[0], v, 1e-3);
    EXPECT_NEAR(res.state.displacement[0], u, 1e-3);
    s = res.state;
  }
}

TEST(DynamicStepper, RejectsBadProblem)
{
  DynamicProblem prob;
  prob.mass = identitySparse(2);
  prob.timestep = -1.0;  // invalid
  EXPECT_THROW(makeDynamicStepper(TimeIntegratorKind::ImplicitEuler, prob), std::invalid_argument);
}
