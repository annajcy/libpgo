#include <gtest/gtest.h>

#include "material/fields/parameterDofLayout.h"
#include "material/fields/parameterFieldMapping.h"

#include <array>

namespace
{
using namespace pgo::SolidDeformationModel;

TEST(ConstantParameterDofLayout, GathersSharedColumns)
{
  ConstantParameterDofLayout layout(4, 3);
  EXPECT_EQ(layout.numElements(), 4);
  EXPECT_EQ(layout.numLocalDofs(), 3);
  EXPECT_EQ(layout.numGlobalDofs(), 3);
  EXPECT_EQ(layout.numValueRows(), 1);

  const std::array<double, 3> global{ 2.0, 3.0, 5.0 };
  std::array<double, 3> local{};
  for (int ele = 0; ele < 4; ele++) {
    layout.gather(ele, global, local);
    EXPECT_EQ(local, global);
    for (int k = 0; k < 3; k++)
      EXPECT_EQ(layout.globalDof(ele, k), k);
  }
}

TEST(ConstantParameterDofLayout, ConvertsElementDefaultsToGlobalValues)
{
  ConstantParameterDofLayout layout(3, 2);
  const std::array<double, 6> defaults{1, 2, 3, 4, 5, 6};
  const auto global = layout.globalValuesFromElementDefaults(defaults);
  ASSERT_EQ(global.size(), 2);
  EXPECT_DOUBLE_EQ(global[0], 1);
  EXPECT_DOUBLE_EQ(global[1], 2);
}

TEST(ConstantParameterDofLayout, RejectsInvalidShapeAndIndices)
{
  EXPECT_THROW(ConstantParameterDofLayout(-1, 2), std::invalid_argument);
  EXPECT_THROW(ConstantParameterDofLayout(2, -1), std::invalid_argument);

  ConstantParameterDofLayout layout(2, 2);
  std::array<double, 2> global{};
  std::array<double, 2> local{};
  EXPECT_THROW(layout.globalDof(-1, 0), std::out_of_range);
  EXPECT_THROW(layout.globalDof(0, 2), std::out_of_range);
  EXPECT_THROW(layout.gather(2, global, local), std::out_of_range);
  EXPECT_THROW(
    layout.gather(0, std::span<const double>(global.data(), 1), local),
    std::invalid_argument);
}

TEST(IdentityParameterFieldMapping, ValueJacobianAndHessian)
{
  IdentityParameterFieldMapping mapping(3);
  const std::array<double, 3> local{ 2.0, -1.0, 4.0 };
  std::array<double, 3> material{};
  std::array<double, 9> jacobian{};
  std::array<double, 27> hessians{};

  mapping.evaluate(1, 2, local, material);
  mapping.evaluateJacobian(1, 2, local, jacobian.data());
  mapping.evaluateHessians(1, 2, local, hessians.data());

  EXPECT_EQ(material, local);
  for (int col = 0; col < 3; col++)
    for (int row = 0; row < 3; row++)
      EXPECT_DOUBLE_EQ(jacobian[col * 3 + row], row == col ? 1.0 : 0.0);
  for (double value : hessians)
    EXPECT_DOUBLE_EQ(value, 0.0);
  EXPECT_TRUE(mapping.isAffine());
}

}  // namespace
