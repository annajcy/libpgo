#include <gtest/gtest.h>

#include "material/core/optimizableParameters.h"

#include <array>
#include <algorithm>
#include <memory>
#include <vector>

namespace
{
using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

class SquareEvaluator final : public DifferentiableMaterialEvaluator
{
public:
  explicit SquareEvaluator(std::array<double, 2> scales): scales_(scales) {}

  int numParameters() const override { return 2; }
  int numChannels() const override { return 2; }
  bool isAffine() const override { return false; }

  void evaluate(
    int, int, std::span<const double> z,
    std::span<double> p) const override
  {
    if (z.size() != 2 || p.size() != 2)
      throw std::invalid_argument("SquareEvaluator shape mismatch.");
    for (int c = 0; c < 2; c++)
      p[c] = scales_[c] * z[c] * z[c];
  }

  void evaluateJacobian(
    int, int, std::span<const double> z,
    ES::RefMatXd output) const override
  {
    output.setZero();
    for (int c = 0; c < 2; c++)
      output(c, c) = 2.0 * scales_[c] * z[c];
  }

  void evaluateHessians(
    int, int, std::span<const double>,
    std::span<ES::MXd> channelHessians) const override
  {
    for (ES::MXd &hessian : channelHessians)
      hessian.setZero();
    for (int c = 0; c < 2; c++)
      channelHessians[static_cast<std::size_t>(c)](c, c) = 2.0 * scales_[c];
  }

private:
  std::array<double, 2> scales_;
};

TEST(ElementwiseParameterLayout, GathersPerElementColumns)
{
  ElementwiseParameterLayout layout(3, 2);
  const std::array<double, 6> global{ 1, 2, 3, 4, 5, 6 };
  std::array<double, 2> local{};
  layout.gather(2, global, local);
  EXPECT_EQ(local[0], 5);
  EXPECT_EQ(local[1], 6);
  EXPECT_EQ(layout.globalParameter(2, 0), 4);
  EXPECT_EQ(layout.globalParameter(2, 1), 5);
  EXPECT_EQ(layout.numValueRows(), 3);
}

TEST(DifferentiableMaterialEvaluator, ValueJacobianHessiansAndFiniteDifference)
{
  SquareEvaluator evaluator({ 2.0, -3.0 });
  std::array<double, 2> z{ 1.5, -0.7 };
  std::array<double, 2> p{};
  ES::MXd jacobian(2, 2);
  std::vector<ES::MXd> hessians(2);
  for (ES::MXd &hessian : hessians)
    hessian.resize(2, 2);
  evaluator.evaluate(0, 0, z, p);
  evaluator.evaluateJacobian(0, 0, z, jacobian);
  evaluator.evaluateHessians(0, 0, z, hessians);

  EXPECT_DOUBLE_EQ(p[0], 4.5);
  EXPECT_DOUBLE_EQ(p[1], -1.47);
  EXPECT_DOUBLE_EQ(jacobian(0, 0), 6.0);
  EXPECT_DOUBLE_EQ(jacobian(1, 1), 4.2);
  EXPECT_DOUBLE_EQ(hessians[0](0, 0), 4.0);
  EXPECT_DOUBLE_EQ(hessians[1](1, 1), -6.0);

  constexpr double h = 1e-6;
  for (int k = 0; k < 2; k++) {
    auto zp = z;
    auto zm = z;
    zp[k] += h;
    zm[k] -= h;
    std::array<double, 2> pp{}, pm{};
    evaluator.evaluate(0, 0, zp, pp);
    evaluator.evaluate(0, 0, zm, pm);
    for (int c = 0; c < 2; c++)
      EXPECT_NEAR((pp[c] - pm[c]) / (2 * h), jacobian(c, k), 1e-8);
  }

  for (int derivativeParameter = 0; derivativeParameter < 2;
       derivativeParameter++) {
    auto zp = z;
    auto zm = z;
    zp[derivativeParameter] += h;
    zm[derivativeParameter] -= h;
    ES::MXd jp(2, 2), jm(2, 2);
    evaluator.evaluateJacobian(0, 0, zp, jp);
    evaluator.evaluateJacobian(0, 0, zm, jm);
    for (int channel = 0; channel < 2; channel++) {
      for (int jacobianParameter = 0; jacobianParameter < 2;
           jacobianParameter++) {
        const double fd =
          (jp(channel, jacobianParameter) -
            jm(channel, jacobianParameter)) /
          (2 * h);
        const double analytic =
          hessians[static_cast<std::size_t>(channel)](
            derivativeParameter, jacobianParameter);
        EXPECT_NEAR(fd, analytic, 1e-8);
      }
    }
  }
}

TEST(OptimizableParameterField, RejectsInvalidSchema)
{
  EXPECT_THROW(
    std::make_shared<const OptimizableParameterField>(
      ParameterSchema({ "same", "same" }),
      std::make_shared<ElementwiseParameterLayout>(2, 2),
      std::make_shared<IdentityMaterialEvaluator>(2)),
    std::invalid_argument);

  EXPECT_THROW(
    std::make_shared<const OptimizableParameterField>(
      ParameterSchema({ "valid", "" }),
      std::make_shared<ElementwiseParameterLayout>(2, 2),
      std::make_shared<IdentityMaterialEvaluator>(2)),
    std::invalid_argument);

  EXPECT_THROW(
    std::make_shared<const OptimizableParameterField>(
      ParameterSchema({ "first", "second" }),
      std::make_shared<ElementwiseParameterLayout>(2, 1),
      std::make_shared<IdentityMaterialEvaluator>(2)),
    std::invalid_argument);

  EXPECT_THROW(
    std::make_shared<const OptimizableParameterField>(
      ParameterSchema({ "only_one_name" }),
      std::make_shared<ElementwiseParameterLayout>(2, 2),
      std::make_shared<IdentityMaterialEvaluator>(2)),
    std::invalid_argument);
}

TEST(OptimizableParameters, RejectsMismatchedFieldElementCounts)
{
  auto elastic = std::make_shared<const OptimizableParameterField>(
    ParameterSchema{},
    std::make_shared<ElementwiseParameterLayout>(2, 0),
    std::make_shared<IdentityMaterialEvaluator>(0));
  auto plastic = std::make_shared<const OptimizableParameterField>(
    ParameterSchema{},
    std::make_shared<ElementwiseParameterLayout>(3, 0),
    std::make_shared<IdentityMaterialEvaluator>(0));
  EXPECT_THROW(
    OptimizableParameters(
      std::move(elastic), std::move(plastic),
      ES::VXd{}, ES::VXd{}),
    std::invalid_argument);
}

TEST(OptimizableParameters, FieldIdentitySnapshotAndSemanticReference)
{
  auto elastic = std::make_shared<const OptimizableParameterField>(
    ParameterSchema({ "first", "thickness" }),
    std::make_shared<ElementwiseParameterLayout>(2, 2),
    std::make_shared<SquareEvaluator>(std::array<double, 2>{ 2.0, 3.0 }));
  auto plastic = std::make_shared<const OptimizableParameterField>(
    ParameterSchema({ "stretch" }),
    std::make_shared<ConstantParameterLayout>(2, 1),
    std::make_shared<IdentityMaterialEvaluator>(1));
  ES::VXd elasticValues(4);
  elasticValues << 1.0, 2.0, 3.0, 4.0;
  ES::VXd plasticValues(1);
  plasticValues << 1.1;
  OptimizableParameters parameters(
    elastic, plastic, elasticValues, plasticValues);
  OptimizableParameterSnapshot snapshot = parameters.snapshot();

  std::array<double, 2> localParameters{};
  std::array<double, 4> materialValues{};
  snapshot.view().evaluateElement(
    *elastic, 1, 2, localParameters, materialValues);
  EXPECT_DOUBLE_EQ(localParameters[0], 3.0);
  EXPECT_DOUBLE_EQ(localParameters[1], 4.0);
  EXPECT_EQ(
    materialValues,
    (std::array<double, 4>{ 18.0, 48.0, 18.0, 48.0 }));
  EXPECT_THROW(
    snapshot.view().evaluateElement(
      *elastic, 1, 2, localParameters,
      std::span<double>(materialValues.data(), materialValues.size() - 1)),
    std::invalid_argument);
  OptimizableParameterEvaluationScratch evaluationScratch;
  const std::span<const double> evaluatedValues =
    snapshot.view().evaluateElement(
      *elastic, 1, 2, evaluationScratch);
  EXPECT_EQ(
    std::vector<double>(evaluatedValues.begin(), evaluatedValues.end()),
    (std::vector<double>{ 18.0, 48.0, 18.0, 48.0 }));

  OptimizableParameterRef thickness = elastic->parameter("thickness");
  EXPECT_THROW(elastic->parameter("missing"), std::invalid_argument);
  EXPECT_DOUBLE_EQ(thickness.value(1, 0, snapshot.view()), 4.0);
  ES::VXd derivative(2);
  thickness.localDerivative(1, 0, snapshot.view(), derivative);
  EXPECT_DOUBLE_EQ(derivative[0], 0.0);
  EXPECT_DOUBLE_EQ(derivative[1], 1.0);

  ES::VXd changed = elasticValues;
  changed.setZero();
  parameters.setElasticValues(changed);
  EXPECT_DOUBLE_EQ(thickness.value(1, 0, snapshot.view()), 4.0);

  auto otherElastic = std::make_shared<const OptimizableParameterField>(
      ParameterSchema({ "first", "thickness" }),
      std::make_shared<ElementwiseParameterLayout>(2, 2),
      std::make_shared<IdentityMaterialEvaluator>(2));
  auto otherPlastic = std::make_shared<const OptimizableParameterField>(
      ParameterSchema({ "stretch" }),
      std::make_shared<ConstantParameterLayout>(2, 1),
      std::make_shared<IdentityMaterialEvaluator>(1));
  EXPECT_THROW(
    thickness.value(
      0, 0,
      OptimizableParameters(
        otherElastic, otherPlastic,
        elasticValues, plasticValues).snapshot().view()),
    std::invalid_argument);
  EXPECT_THROW(
    snapshot.view().evaluateElement(
      *otherElastic, 0, 1, localParameters,
      std::span<double>(materialValues.data(), 2)),
    std::invalid_argument);

  EXPECT_THROW(
    snapshot.withValues(
      std::span<const double>(elasticValues.data(), elasticValues.size() - 1),
      std::span<const double>(plasticValues.data(), plasticValues.size())),
    std::invalid_argument);
}

}  // namespace
