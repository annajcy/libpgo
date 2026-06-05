#include <gtest/gtest.h>

#include "mappedContactEnergy.h"
#include "statefulContactEnergy.h"
#include "evaluationStateAwareEnergy.h"
#include "lineSearchAwareEnergy.h"
#include "stepAwareEnergy.h"
#include "stepDependentEnergy.h"

#include <numeric>

namespace
{
namespace Contact = pgo::Contact;
namespace ES = pgo::EigenSupport;
namespace NO = pgo::NonlinearOptimization;

class SurfaceQuadraticEnergy : public Contact::StatefulContactEnergy
{
public:
  explicit SurfaceQuadraticEnergy(int n): n_(n) {}

  Contact::ContactModelKind contactModelKind() const override { return Contact::ContactModelKind::SampledPenalty; }
  double func(ES::ConstRefVecXd x) const override { return 0.5 * x.squaredNorm(); }
  void gradient(ES::ConstRefVecXd x, ES::RefVecXd grad) const override { grad = x; }
  void hessianInPlace(ES::ConstRefVecXd, ES::SpMatD &hess) const override
  {
    hess.resize(n_, n_);
    hess.setIdentity();
  }
  void hessianAlloc(ES::SpMatD &hess) const override
  {
    hess.resize(n_, n_);
    hess.setIdentity();
  }
  void getDOFs(std::vector<int> &dofs) const override
  {
    dofs.resize(n_);
    std::iota(dofs.begin(), dofs.end(), 0);
  }
  int getNumDOFs() const override { return n_; }
  int isHessianTopologyFixed() const override { return 1; }

private:
  int n_;
};

class SurfaceLifecycleEnergy :
  public SurfaceQuadraticEnergy,
  public NO::EvaluationStateAwareEnergy,
  public NO::LineSearchAwareEnergy,
  public NO::StepAwareEnergy,
  public NO::StepDependentEnergy
{
public:
  explicit SurfaceLifecycleEnergy(int n): SurfaceQuadraticEnergy(n) {}

  void prepareEvaluationState(ES::ConstRefVecXd x) const override { prepared = x; }
  void beginLineSearch(ES::ConstRefVecXd x, ES::ConstRefVecXd dx) const override
  {
    lineX = x;
    lineDx = dx;
    lineSearchOpen = true;
  }
  void endLineSearch() const override { lineSearchOpen = false; }
  void beginStep(const NO::StepState &state) override
  {
    stepTime = state.time;
    stepTimestep = state.timestep;
    previous = state.previousX ? *state.previousX : ES::VXd();
  }

  mutable ES::VXd prepared;
  mutable ES::VXd lineX;
  mutable ES::VXd lineDx;
  mutable bool lineSearchOpen = false;
  double stepTime = -1.0;
  double stepTimestep = -1.0;
  ES::VXd previous;
};

ES::MXd makeTwoRestVertices()
{
  ES::MXd V(2, 3);
  V << 1.0, 2.0, 3.0,
    4.0, 5.0, 6.0;
  return V;
}

ES::SpMatD makeSurfaceFromSimulationMap()
{
  std::vector<ES::TripletD> triplets;
  triplets.emplace_back(0, 0, 2.0);
  triplets.emplace_back(1, 1, 3.0);
  triplets.emplace_back(2, 2, 4.0);
  triplets.emplace_back(3, 0, -1.0);
  triplets.emplace_back(4, 1, 0.5);
  triplets.emplace_back(5, 2, 1.5);
  ES::SpMatD W(6, 4);
  W.setFromTriplets(triplets.begin(), triplets.end());
  return W;
}
}  // namespace

TEST(MappedContactEnergyGTest, MapsSimulationDisplacementToSurfacePositionsAndPullsBackDerivatives)
{
  auto child = std::make_shared<SurfaceQuadraticEnergy>(6);
  const ES::MXd rest = makeTwoRestVertices();
  const ES::SpMatD W = makeSurfaceFromSimulationMap();
  auto mapped = Contact::makeMappedContactEnergy(rest, W, child);

  ES::VXd u(4);
  u << 0.25, -0.5, 0.75, 10.0;
  ES::VXd restFlat(6);
  restFlat << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  const ES::VXd xSurface = restFlat + W * u;

  EXPECT_NEAR(mapped->func(u), 0.5 * xSurface.squaredNorm(), 1e-12);

  ES::VXd grad(4);
  mapped->gradient(u, grad);
  EXPECT_TRUE(grad.isApprox(W.transpose() * xSurface, 1e-12));
  EXPECT_DOUBLE_EQ(grad[3], 0.0);

  ES::SpMatD H;
  mapped->hessian(u, H);
  EXPECT_TRUE(ES::MXd(H).isApprox(ES::MXd(W.transpose() * W), 1e-12));

  EXPECT_EQ(mapped->getNumDOFs(), 4);
  EXPECT_EQ(mapped->contactModelKind(), Contact::ContactModelKind::SampledPenalty);
  EXPECT_EQ(dynamic_cast<const NO::StepAwareEnergy *>(mapped.get()), nullptr);
  EXPECT_EQ(dynamic_cast<const NO::StepDependentEnergy *>(mapped.get()), nullptr);
}

TEST(MappedContactEnergyGTest, MapsActiveSetLineSearchAndStepLifecycle)
{
  auto child = std::make_shared<SurfaceLifecycleEnergy>(6);
  const ES::MXd rest = makeTwoRestVertices();
  const ES::SpMatD W = makeSurfaceFromSimulationMap();
  auto mapped = Contact::makeMappedContactEnergy(rest, W, child);

  auto *evalAware = dynamic_cast<const NO::EvaluationStateAwareEnergy *>(mapped.get());
  auto *lineAware = dynamic_cast<const NO::LineSearchAwareEnergy *>(mapped.get());
  auto *stepAware = dynamic_cast<NO::StepAwareEnergy *>(mapped.get());
  auto *stepDependent = dynamic_cast<const NO::StepDependentEnergy *>(mapped.get());
  ASSERT_NE(evalAware, nullptr);
  ASSERT_NE(lineAware, nullptr);
  ASSERT_NE(stepAware, nullptr);
  ASSERT_NE(stepDependent, nullptr);

  ES::VXd u(4);
  u << 0.25, -0.5, 0.75, 10.0;
  ES::VXd du(4);
  du << 0.1, 0.2, -0.3, 1.0;
  ES::VXd restFlat(6);
  restFlat << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  evalAware->prepareEvaluationState(u);
  EXPECT_TRUE(child->prepared.isApprox(restFlat + W * u, 1e-12));

  lineAware->beginLineSearch(u, du);
  EXPECT_TRUE(child->lineX.isApprox(restFlat + W * u, 1e-12));
  EXPECT_TRUE(child->lineDx.isApprox(W * du, 1e-12));
  EXPECT_TRUE(child->lineSearchOpen);
  lineAware->endLineSearch();
  EXPECT_FALSE(child->lineSearchOpen);

  NO::StepState state;
  state.time = 2.0;
  state.timestep = 0.25;
  state.previousX = &u;
  stepAware->beginStep(state);
  EXPECT_DOUBLE_EQ(child->stepTime, 2.0);
  EXPECT_DOUBLE_EQ(child->stepTimestep, 0.25);
  EXPECT_TRUE(child->previous.isApprox(restFlat + W * u, 1e-12));
}
