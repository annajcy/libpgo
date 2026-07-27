#include <gtest/gtest.h>

#include "material/core/parameterLayout.h"
#include "material/core/materialEvaluator.h"
#include "material/core/optimizableParameters.h"

#include <array>
#include <memory>
#include <span>
#include <type_traits>
#include <vector>

namespace
{
using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

class SumEvaluator final : public MaterialEvaluator
{
public:
  int numParameters() const override { return 2; }
  int numChannels() const override { return 1; }

  void evaluate(
    int, int,
    std::span<const double> parameters,
    std::span<double> channels) const override
  {
    if (parameters.size() != 2 || channels.size() != 1)
      throw std::invalid_argument("SumEvaluator shape mismatch.");
    channels[0] = parameters[0] + parameters[1];
  }
};

static_assert(
  std::is_base_of_v<MaterialEvaluator,
    DifferentiableMaterialEvaluator>);
static_assert(
  !std::is_base_of_v<DifferentiableMaterialEvaluator, SumEvaluator>);

TEST(ConstantParameterLayout, GathersSharedColumns)
{
  ConstantParameterLayout layout(4, 3);
  EXPECT_EQ(layout.numElements(), 4);
  EXPECT_EQ(layout.numLocalParameters(), 3);
  EXPECT_EQ(layout.numGlobalParameters(), 3);
  EXPECT_EQ(layout.numValueRows(), 1);

  const std::array<double, 3> global{ 2.0, 3.0, 5.0 };
  std::array<double, 3> local{};
  for (int ele = 0; ele < 4; ele++) {
    layout.gather(ele, global, local);
    EXPECT_EQ(local, global);
    for (int k = 0; k < 3; k++)
      EXPECT_EQ(layout.globalParameter(ele, k), k);
  }
}

TEST(ConstantParameterLayout, RequiresExplicitGlobalValues)
{
  ConstantParameterLayout layout(3, 2);
  const std::array<double, 6> defaults{ 1, 2, 3, 4, 5, 6 };
  (void)layout;
  (void)defaults;
}

TEST(ConstantParameterLayout, RejectsInvalidShapeAndIndices)
{
  EXPECT_THROW(ConstantParameterLayout(-1, 2), std::invalid_argument);
  EXPECT_THROW(ConstantParameterLayout(2, -1), std::invalid_argument);

  ConstantParameterLayout layout(2, 2);
  std::array<double, 2> global{};
  std::array<double, 2> local{};
  EXPECT_THROW(layout.globalParameter(-1, 0), std::out_of_range);
  EXPECT_THROW(layout.globalParameter(0, 2), std::out_of_range);
  EXPECT_THROW(layout.gather(2, global, local), std::out_of_range);
  EXPECT_THROW(
    layout.gather(0, std::span<const double>(global.data(), 1), local),
    std::invalid_argument);
}

TEST(IdentityMaterialEvaluator, ValueJacobianAndHessian)
{
  IdentityMaterialEvaluator evaluator(3);
  const std::array<double, 3> local{ 2.0, -1.0, 4.0 };
  std::array<double, 3> material{};
  ES::MXd jacobian(3, 3);
  std::vector<ES::MXd> hessians(3);
  for (ES::MXd &hessian : hessians)
    hessian.resize(3, 3);

  evaluator.evaluate(1, 2, local, material);
  evaluator.evaluateJacobian(1, 2, local, jacobian);
  evaluator.evaluateHessians(1, 2, local, hessians);

  EXPECT_EQ(material, local);
  for (int col = 0; col < 3; col++)
    for (int row = 0; row < 3; row++)
      EXPECT_DOUBLE_EQ(jacobian(row, col), row == col ? 1.0 : 0.0);
  for (const ES::MXd &hessian : hessians)
    EXPECT_DOUBLE_EQ(hessian.norm(), 0.0);
  EXPECT_TRUE(evaluator.isAffine());
}

TEST(MaterialEvaluator, FixedFieldAcceptsForwardOnlyEvaluator)
{
  auto layout = std::make_shared<ConstantParameterLayout>(3, 2);
  auto evaluator = std::make_shared<SumEvaluator>();
  FixedParameterField field(
    ParameterSchema({ "a", "b" }), layout, evaluator);

  const std::array<double, 2> parameters{ 2.5, -0.75 };
  std::array<double, 1> channels{};
  field.evaluate(2, 0, parameters, channels);

  EXPECT_DOUBLE_EQ(channels[0], 1.75);
  EXPECT_EQ(field.numLocalParameters(), 2);
  EXPECT_EQ(field.numMaterialChannels(), 1);
}

}  // namespace
