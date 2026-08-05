#include <gtest/gtest.h>

#include "material/runtime/optimizableParameterRef.h"

#include <array>
#include <algorithm>
#include <memory>
#include <vector>

namespace
{
using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

class SquareMapping final : public DifferentiableMaterialChannelMapping
{
public:
  explicit SquareMapping(std::array<double, 2> scales): scales_(scales) {}

  int numInputs() const override { return 2; }
  int numChannels() const override { return 2; }
  bool isAffine() const override { return false; }

  void evaluate(
    int, int, std::span<const double> z,
    std::span<double> p) const override
  {
    if (z.size() != 2 || p.size() != 2)
      throw std::invalid_argument("SquareMapping shape mismatch.");
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
  EXPECT_EQ(layout.numGlobalParameters(), 6);
}

TEST(DifferentiableMaterialChannelMapping, ValueJacobianHessiansAndFiniteDifference)
{
  SquareMapping mapping({ 2.0, -3.0 });
  std::array<double, 2> z{ 1.5, -0.7 };
  std::array<double, 2> p{};
  ES::MXd jacobian(2, 2);
  std::vector<ES::MXd> hessians(2);
  for (ES::MXd &hessian : hessians)
    hessian.resize(2, 2);
  mapping.evaluate(0, 0, z, p);
  mapping.evaluateJacobian(0, 0, z, jacobian);
  mapping.evaluateHessians(0, 0, z, hessians);

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
    mapping.evaluate(0, 0, zp, pp);
    mapping.evaluate(0, 0, zm, pm);
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
    mapping.evaluateJacobian(0, 0, zp, jp);
    mapping.evaluateJacobian(0, 0, zm, jm);
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
      ParameterInputSchema({ "same", "same" }),
      std::make_shared<ElementwiseParameterLayout>(2, 2),
      std::make_shared<IdentityMaterialChannelMapping>(2)),
    std::invalid_argument);

  EXPECT_THROW(
    std::make_shared<const OptimizableParameterField>(
      ParameterInputSchema({ "valid", "" }),
      std::make_shared<ElementwiseParameterLayout>(2, 2),
      std::make_shared<IdentityMaterialChannelMapping>(2)),
    std::invalid_argument);

  EXPECT_THROW(
    std::make_shared<const OptimizableParameterField>(
      ParameterInputSchema({ "first", "second" }),
      std::make_shared<ElementwiseParameterLayout>(2, 1),
      std::make_shared<IdentityMaterialChannelMapping>(2)),
    std::invalid_argument);

  EXPECT_THROW(
    std::make_shared<const OptimizableParameterField>(
      ParameterInputSchema({ "only_one_name" }),
      std::make_shared<ElementwiseParameterLayout>(2, 2),
      std::make_shared<IdentityMaterialChannelMapping>(2)),
    std::invalid_argument);
}

TEST(MaterialState, RejectsMismatchedFieldElementCounts)
{
  auto elastic = std::make_shared<const OptimizableParameterField>(
    ParameterInputSchema{},
    std::make_shared<ElementwiseParameterLayout>(2, 0),
    std::make_shared<IdentityMaterialChannelMapping>(0));
  auto plastic = std::make_shared<const OptimizableParameterField>(
    ParameterInputSchema{},
    std::make_shared<ElementwiseParameterLayout>(3, 0),
    std::make_shared<IdentityMaterialChannelMapping>(0));
  EXPECT_THROW(
    MaterialState(
      std::move(elastic), std::move(plastic),
      ES::VXd{}, ES::VXd{}),
    std::invalid_argument);
}

TEST(MaterialState, FieldIdentitySnapshotAndSemanticReference)
{
  auto elastic = std::make_shared<const OptimizableParameterField>(
    ParameterInputSchema({ "first", "thickness" }),
    std::make_shared<ElementwiseParameterLayout>(2, 2),
    std::make_shared<SquareMapping>(std::array<double, 2>{ 2.0, 3.0 }));
  auto plastic = std::make_shared<const OptimizableParameterField>(
    ParameterInputSchema({ "stretch" }),
    std::make_shared<ConstantParameterLayout>(2, 1),
    std::make_shared<IdentityMaterialChannelMapping>(1));
  ES::VXd elasticValues(4);
  elasticValues << 1.0, 2.0, 3.0, 4.0;
  ES::VXd plasticValues(1);
  plasticValues << 1.1;
  MaterialState parameters(
    elastic, plastic, elasticValues, plasticValues);
  const MaterialState snapshot = parameters;

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
  MaterialStateEvaluationScratch evaluationScratch;
  const std::span<const double> evaluatedValues =
    snapshot.view().evaluateElement(
      *elastic, 1, 2, evaluationScratch);
  EXPECT_EQ(
    std::vector<double>(evaluatedValues.begin(), evaluatedValues.end()),
    (std::vector<double>{ 18.0, 48.0, 18.0, 48.0 }));

  OptimizableParameterRef thickness(elastic, "thickness");
  EXPECT_THROW(
    OptimizableParameterRef(elastic, "missing"), std::invalid_argument);
  EXPECT_DOUBLE_EQ(thickness.value(1, 0, snapshot.view()), 4.0);
  ES::VXd derivative(2);
  thickness.localDerivative(1, 0, snapshot.view(), derivative);
  EXPECT_DOUBLE_EQ(derivative[0], 0.0);
  EXPECT_DOUBLE_EQ(derivative[1], 1.0);

  ES::VXd changed = elasticValues;
  changed.setZero();
  const MaterialState changedState = parameters.withElasticValues(
    std::span<const double>(changed.data(), changed.size()));
  EXPECT_DOUBLE_EQ(thickness.value(1, 0, snapshot.view()), 4.0);
  EXPECT_DOUBLE_EQ(thickness.value(1, 0, changedState.view()), 0.0);

  auto otherElastic = std::make_shared<const OptimizableParameterField>(
      ParameterInputSchema({ "first", "thickness" }),
      std::make_shared<ElementwiseParameterLayout>(2, 2),
      std::make_shared<IdentityMaterialChannelMapping>(2));
  auto otherPlastic = std::make_shared<const OptimizableParameterField>(
      ParameterInputSchema({ "stretch" }),
      std::make_shared<ConstantParameterLayout>(2, 1),
      std::make_shared<IdentityMaterialChannelMapping>(1));
  EXPECT_THROW(
    thickness.value(
      0, 0,
      MaterialState(
        otherElastic, otherPlastic,
        elasticValues, plasticValues).view()),
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
