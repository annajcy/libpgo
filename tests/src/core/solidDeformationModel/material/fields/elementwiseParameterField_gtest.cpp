#include <gtest/gtest.h>

#include "material/core/materialParameters.h"

#include <array>
#include <algorithm>
#include <memory>
#include <vector>

namespace
{
using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

class SquareMapping final : public MaterialChannelMapping
{
public:
  explicit SquareMapping(std::array<double, 2> scales): scales_(scales) {}

  int numInputDofs() const override { return 2; }
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

TEST(ElementwiseParameterDofLayout, GathersPerElementColumns)
{
  ElementwiseParameterDofLayout layout(3, 2);
  const std::array<double, 6> global{ 1, 2, 3, 4, 5, 6 };
  std::array<double, 2> local{};
  layout.gather(2, global, local);
  EXPECT_EQ(local[0], 5);
  EXPECT_EQ(local[1], 6);
  EXPECT_EQ(layout.globalDof(2, 0), 4);
  EXPECT_EQ(layout.globalDof(2, 1), 5);
  EXPECT_EQ(layout.numValueRows(), 3);
}

TEST(NonlinearMaterialChannelMapping, ValueJacobianHessiansAndFiniteDifference)
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

  for (int derivativeDof = 0; derivativeDof < 2; derivativeDof++) {
    auto zp = z;
    auto zm = z;
    zp[derivativeDof] += h;
    zm[derivativeDof] -= h;
    ES::MXd jp(2, 2), jm(2, 2);
    mapping.evaluateJacobian(0, 0, zp, jp);
    mapping.evaluateJacobian(0, 0, zm, jm);
    for (int channel = 0; channel < 2; channel++) {
      for (int jacobianDof = 0; jacobianDof < 2; jacobianDof++) {
        const double fd =
            (jp(channel, jacobianDof) -
            jm(channel, jacobianDof)) /
          (2 * h);
        const double analytic =
          hessians[static_cast<std::size_t>(channel)](derivativeDof, jacobianDof);
        EXPECT_NEAR(fd, analytic, 1e-8);
      }
    }
  }
}

TEST(MaterialParameterField, RejectsInvalidSchema)
{
  EXPECT_THROW(
    MaterialParameterField::create(
      { "same", "same" },
      std::make_shared<ElementwiseParameterDofLayout>(2, 2),
      std::make_shared<IdentityMaterialChannelMapping>(2)),
    std::invalid_argument);

  EXPECT_THROW(
    MaterialParameterField::create(
      { "valid", "" },
      std::make_shared<ElementwiseParameterDofLayout>(2, 2),
      std::make_shared<IdentityMaterialChannelMapping>(2)),
    std::invalid_argument);

  EXPECT_THROW(
    MaterialParameterField::create(
      { "first", "second" },
      std::make_shared<ElementwiseParameterDofLayout>(2, 1),
      std::make_shared<IdentityMaterialChannelMapping>(2)),
    std::invalid_argument);

  EXPECT_THROW(
    MaterialParameterField::create(
      { "only_one_name" },
      std::make_shared<ElementwiseParameterDofLayout>(2, 2),
      std::make_shared<IdentityMaterialChannelMapping>(2)),
    std::invalid_argument);
}

TEST(MaterialParameterSpace, RejectsMismatchedElementCounts)
{
  EXPECT_THROW(
    MaterialParameterSpace(
      MaterialParameterField::create(
        std::vector<std::string>{},
        std::make_shared<ElementwiseParameterDofLayout>(2, 0),
        std::make_shared<IdentityMaterialChannelMapping>(0)),
      MaterialParameterField::create(
        std::vector<std::string>{},
        std::make_shared<ElementwiseParameterDofLayout>(3, 0),
        std::make_shared<IdentityMaterialChannelMapping>(0))),
    std::invalid_argument);
}

TEST(MaterialParameterSpace, StateIdentitySnapshotAndSemanticReference)
{
  auto elastic = MaterialParameterField::create(
    { "first", "thickness" },
    std::make_shared<ElementwiseParameterDofLayout>(2, 2),
    std::make_shared<SquareMapping>(std::array<double, 2>{ 2.0, 3.0 }));
  auto plastic = MaterialParameterField::create(
    { "stretch" },
    std::make_shared<ConstantParameterDofLayout>(2, 1),
    std::make_shared<IdentityMaterialChannelMapping>(1));
  auto space = std::make_shared<MaterialParameterSpace>(
    std::move(elastic), std::move(plastic));

  ES::VXd elasticValues(4);
  elasticValues << 1.0, 2.0, 3.0, 4.0;
  ES::VXd plasticValues(1);
  plasticValues << 1.1;
  MaterialParameters parameters(space, elasticValues, plasticValues);
  MaterialParameterSnapshot snapshot = parameters.snapshot();

  MaterialParameterRef thickness = space->elastic().parameter("thickness");
  EXPECT_THROW(space->elastic().parameter("missing"), std::invalid_argument);
  EXPECT_DOUBLE_EQ(thickness.value(1, 0, snapshot.view()), 48.0);
  ES::VXd derivative(2);
  thickness.localDerivative(1, 0, snapshot.view(), derivative);
  EXPECT_DOUBLE_EQ(derivative[0], 0.0);
  EXPECT_DOUBLE_EQ(derivative[1], 24.0);

  ES::VXd changed = elasticValues;
  changed.setZero();
  parameters.setElasticValues(changed);
  EXPECT_DOUBLE_EQ(thickness.value(1, 0, snapshot.view()), 48.0);

  auto otherSpace = std::make_shared<MaterialParameterSpace>(
    MaterialParameterField::create(
      std::vector<std::string>{ "first", "thickness" },
      std::make_shared<ElementwiseParameterDofLayout>(2, 2),
      std::make_shared<IdentityMaterialChannelMapping>(2)),
    MaterialParameterField::create(
      std::vector<std::string>{ "stretch" },
      std::make_shared<ConstantParameterDofLayout>(2, 1),
      std::make_shared<IdentityMaterialChannelMapping>(1)));
  EXPECT_THROW(
    thickness.value(
      0, 0,
      MaterialParameters(otherSpace, elasticValues, plasticValues).snapshot().view()),
    std::invalid_argument);

  EXPECT_THROW(
    snapshot.withValues(
      std::span<const double>(elasticValues.data(), elasticValues.size() - 1),
      std::span<const double>(plasticValues.data(), plasticValues.size())),
    std::invalid_argument);
}

}  // namespace
