#include "gtest/gtest.h"

#include "formulations/parameters/constantParameterField.h"
#include "EigenSupport.h"

using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

TEST(ConstantParameterField, ComputeValueReturnsCorrectElementConstantValue)
{
  ES::VXd globalParams(12);
  globalParams << 1.0, 2.0, 3.0,
                  4.0, 5.0, 6.0,
                  7.0, 8.0, 9.0,
                  10.0, 11.0, 12.0;

  ConstantParameterField field(3, 4, globalParams.data());

  ES::VXd value(3);
  field.computeValue(0, 0, value.data());
  EXPECT_EQ(value.size(), 3);
  EXPECT_DOUBLE_EQ(value[0], 1.0);
  EXPECT_DOUBLE_EQ(value[1], 2.0);
  EXPECT_DOUBLE_EQ(value[2], 3.0);

  field.computeValue(2, 0, value.data());
  EXPECT_DOUBLE_EQ(value[0], 7.0);
  EXPECT_DOUBLE_EQ(value[1], 8.0);
  EXPECT_DOUBLE_EQ(value[2], 9.0);
}

TEST(ConstantParameterField, ComputeValueIgnoresQuadratureId)
{
  ES::VXd globalParams(6);
  globalParams << 10.0, 20.0, 30.0, 40.0, 50.0, 60.0;

  ConstantParameterField field(2, 3, globalParams.data());

  ES::VXd v0(2), v1(2), v2(2);
  field.computeValue(1, 0, v0.data());
  field.computeValue(1, 3, v1.data());
  field.computeValue(1, 7, v2.data());

  EXPECT_DOUBLE_EQ(v0[0], 30.0);
  EXPECT_DOUBLE_EQ(v0[1], 40.0);
  EXPECT_DOUBLE_EQ(v1[0], 30.0);
  EXPECT_DOUBLE_EQ(v1[1], 40.0);
  EXPECT_DOUBLE_EQ(v2[0], 30.0);
  EXPECT_DOUBLE_EQ(v2[1], 40.0);
}

TEST(ConstantParameterField, ComputeDerivativeIsIdentity)
{
  ES::VXd globalParams(6);
  globalParams << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  ConstantParameterField field(2, 3, globalParams.data());

  ES::MXd deriv(2, 2);
  field.computeDerivative(0, 0, deriv.data());

  EXPECT_EQ(deriv.rows(), 2);
  EXPECT_EQ(deriv.cols(), 2);
  EXPECT_TRUE(deriv.isIdentity());
}

TEST(ConstantParameterField, DofLayoutHasCorrectSizes)
{
  ES::VXd globalParams(20);
  ConstantParameterField field(5, 4, globalParams.data());

  const auto *dofLayout = field.dofLayout();
  ASSERT_NE(dofLayout, nullptr);
  EXPECT_EQ(dofLayout->numLocalDofs(), 5);
  EXPECT_EQ(dofLayout->numGlobalDofs(), 20);
}

TEST(ConstantParameterField, DofLayoutGather)
{
  ES::VXd globalParams(9);
  globalParams << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;

  ConstantParameterField field(3, 3, globalParams.data());

  const auto *dofLayout = field.dofLayout();
  ES::VXd local(3);
  dofLayout->gather(0, globalParams.data(), local.data());
  EXPECT_DOUBLE_EQ(local[0], 1.0);
  EXPECT_DOUBLE_EQ(local[1], 2.0);
  EXPECT_DOUBLE_EQ(local[2], 3.0);

  dofLayout->gather(2, globalParams.data(), local.data());
  EXPECT_DOUBLE_EQ(local[0], 7.0);
  EXPECT_DOUBLE_EQ(local[1], 8.0);
  EXPECT_DOUBLE_EQ(local[2], 9.0);
}

TEST(ConstantParameterField, ZeroChannelField)
{
  ConstantParameterField field(0, 10, nullptr);

  EXPECT_EQ(field.numChannels(), 0);
  EXPECT_EQ(field.numLocalDofs(), 0);
  EXPECT_EQ(field.kind(), ParameterFieldKind::CONSTANT);

  // computeValue should be a no-op for zero channels
  field.computeValue(0, 0, nullptr);

  // computeDerivative should be a no-op for zero channels
  field.computeDerivative(0, 0, nullptr);

  const auto *dofLayout = field.dofLayout();
  ASSERT_NE(dofLayout, nullptr);
  EXPECT_EQ(dofLayout->numLocalDofs(), 0);
  EXPECT_EQ(dofLayout->numGlobalDofs(), 0);
}

TEST(ConstantParameterField, SetGlobalData)
{
  ES::VXd data1(6);
  data1 << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  ES::VXd data2(6);
  data2 << 10.0, 20.0, 30.0, 40.0, 50.0, 60.0;

  ConstantParameterField field(2, 3, data1.data());

  ES::VXd value(2);
  field.computeValue(0, 0, value.data());
  EXPECT_DOUBLE_EQ(value[0], 1.0);

  field.setGlobalData(data2.data());
  field.computeValue(0, 0, value.data());
  EXPECT_DOUBLE_EQ(value[0], 10.0);
}

TEST(ConstantParameterField, MultipleElementsCorrectSlices)
{
  ES::VXd globalParams(8);
  globalParams << 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0;

  ConstantParameterField field(2, 4, globalParams.data());

  ES::VXd value(2);
  field.computeValue(0, 0, value.data());
  EXPECT_DOUBLE_EQ(value[0], 10.0);
  EXPECT_DOUBLE_EQ(value[1], 20.0);

  field.computeValue(1, 0, value.data());
  EXPECT_DOUBLE_EQ(value[0], 30.0);
  EXPECT_DOUBLE_EQ(value[1], 40.0);

  field.computeValue(3, 0, value.data());
  EXPECT_DOUBLE_EQ(value[0], 70.0);
  EXPECT_DOUBLE_EQ(value[1], 80.0);
}

TEST(ConstantParameterField, CastToOptimizableFieldSucceeds)
{
  ES::VXd globalParams(4);
  globalParams << 1.0, 2.0, 3.0, 4.0;

  ConstantParameterField field(2, 2, globalParams.data());

  auto *opt = dynamic_cast<OptimizableField *>(&field);
  ASSERT_NE(opt, nullptr);

  ES::MXd deriv(2, 2);
  opt->computeDerivative(0, 0, deriv.data());
  EXPECT_TRUE(deriv.isIdentity());
}
