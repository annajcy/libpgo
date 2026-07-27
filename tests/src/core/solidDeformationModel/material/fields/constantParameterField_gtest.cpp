#include <gtest/gtest.h>

#include "material/parameterization/parameterLayout.h"
#include "material/parameterization/materialChannelMapping.h"
#include "material/runtime/optimizableParameters.h"

#include <array>
#include <memory>
#include <span>
#include <type_traits>
#include <vector>

namespace
{
using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

class SumMapping final : public MaterialChannelMapping
{
public:
  int numInputs() const override { return 2; }
  int numChannels() const override { return 1; }

  void evaluate(
    int, int,
    std::span<const double> parameters,
    std::span<double> channels) const override
  {
    if (parameters.size() != 2 || channels.size() != 1)
      throw std::invalid_argument("SumMapping shape mismatch.");
    channels[0] = parameters[0] + parameters[1];
  }
};

static_assert(
  std::is_base_of_v<MaterialChannelMapping,
    DifferentiableMaterialChannelMapping>);
static_assert(
  !std::is_base_of_v<DifferentiableMaterialChannelMapping, SumMapping>);

TEST(ConstantParameterLayout, GathersSharedColumns)
{
  ConstantParameterLayout layout(4, 3);
  EXPECT_EQ(layout.numElements(), 4);
  EXPECT_EQ(layout.numLocalParameters(), 3);
  EXPECT_EQ(layout.numGlobalParameters(), 3);

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

TEST(IdentityMaterialChannelMapping, ValueJacobianAndHessian)
{
  IdentityMaterialChannelMapping mapping(3);
  const std::array<double, 3> local{ 2.0, -1.0, 4.0 };
  std::array<double, 3> material{};
  ES::MXd jacobian(3, 3);
  std::vector<ES::MXd> hessians(3);
  for (ES::MXd &hessian : hessians)
    hessian.resize(3, 3);

  mapping.evaluate(1, 2, local, material);
  mapping.evaluateJacobian(1, 2, local, jacobian);
  mapping.evaluateHessians(1, 2, local, hessians);

  EXPECT_EQ(material, local);
  for (int col = 0; col < 3; col++)
    for (int row = 0; row < 3; row++)
      EXPECT_DOUBLE_EQ(jacobian(row, col), row == col ? 1.0 : 0.0);
  for (const ES::MXd &hessian : hessians)
    EXPECT_DOUBLE_EQ(hessian.norm(), 0.0);
  EXPECT_TRUE(mapping.isAffine());
}

TEST(MaterialChannelMapping, FixedFieldAcceptsForwardOnlyMapping)
{
  auto layout = std::make_shared<ConstantParameterLayout>(3, 2);
  auto mapping = std::make_shared<SumMapping>();
  FixedParameterField field(
    ParameterInputSchema({ "a", "b" }), layout, mapping);

  const std::array<double, 2> parameters{ 2.5, -0.75 };
  std::array<double, 1> channels{};
  field.evaluate(2, 0, parameters, channels);

  EXPECT_DOUBLE_EQ(channels[0], 1.75);
  EXPECT_EQ(field.numLocalParameters(), 2);
  EXPECT_EQ(field.numMaterialChannels(), 1);
}

}  // namespace
