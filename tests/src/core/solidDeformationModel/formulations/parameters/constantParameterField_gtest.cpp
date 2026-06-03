#include "gtest/gtest.h"

#include "formulations/parameters/constantParameterField.h"
#include "EigenSupport.h"

using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

TEST(ConstantParameterField, ComputeValueReturnsSharedValueForEveryElement)
{
  ES::VXd shared(3);
  shared << 1.0, 2.0, 3.0;

  ConstantParameterField field(3, 4, shared.data());

  ES::VXd value(3);
  field.computeValue(0, 0, value.data());
  EXPECT_DOUBLE_EQ(value[0], 1.0);
  EXPECT_DOUBLE_EQ(value[1], 2.0);
  EXPECT_DOUBLE_EQ(value[2], 3.0);

  // Every element sees the same shared set, regardless of element index.
  field.computeValue(3, 0, value.data());
  EXPECT_DOUBLE_EQ(value[0], 1.0);
  EXPECT_DOUBLE_EQ(value[1], 2.0);
  EXPECT_DOUBLE_EQ(value[2], 3.0);
}

TEST(ConstantParameterField, ComputeValueIgnoresElementAndQuadratureId)
{
  ES::VXd shared(2);
  shared << 30.0, 40.0;

  ConstantParameterField field(2, 3, shared.data());

  ES::VXd v0(2), v1(2), v2(2);
  field.computeValue(0, 0, v0.data());
  field.computeValue(1, 3, v1.data());
  field.computeValue(2, 7, v2.data());

  EXPECT_DOUBLE_EQ(v0[0], 30.0);
  EXPECT_DOUBLE_EQ(v0[1], 40.0);
  EXPECT_DOUBLE_EQ(v1[0], 30.0);
  EXPECT_DOUBLE_EQ(v1[1], 40.0);
  EXPECT_DOUBLE_EQ(v2[0], 30.0);
  EXPECT_DOUBLE_EQ(v2[1], 40.0);
}

TEST(ConstantParameterField, ComputeDerivativeIsIdentity)
{
  ES::VXd shared(2);
  shared << 1.0, 2.0;

  ConstantParameterField field(2, 3, shared.data());

  ES::MXd deriv(2, 2);
  field.computeDerivative(0, 0, deriv.data());

  EXPECT_EQ(deriv.rows(), 2);
  EXPECT_EQ(deriv.cols(), 2);
  EXPECT_TRUE(deriv.isIdentity());
}

TEST(ConstantParameterField, DofLayoutGlobalDofsEqualsNumChannels)
{
  ES::VXd shared(5);
  shared.setOnes();
  ConstantParameterField field(5, 4, shared.data());

  const auto *dofLayout = field.dofLayout();
  ASSERT_NE(dofLayout, nullptr);
  EXPECT_EQ(dofLayout->numLocalDofs(), 5);
  // Shared across the whole mesh: numGlobalDofs == numChannels, NOT * numElements.
  EXPECT_EQ(dofLayout->numGlobalDofs(), 5);
}

TEST(ConstantParameterField, DofLayoutGatherIgnoresElement)
{
  ES::VXd shared(3);
  shared << 1.0, 2.0, 3.0;

  ConstantParameterField field(3, 3, shared.data());

  const auto *dofLayout = field.dofLayout();
  ES::VXd local(3);
  dofLayout->gather(0, shared.data(), local.data());
  EXPECT_DOUBLE_EQ(local[0], 1.0);
  EXPECT_DOUBLE_EQ(local[1], 2.0);
  EXPECT_DOUBLE_EQ(local[2], 3.0);

  dofLayout->gather(2, shared.data(), local.data());
  EXPECT_DOUBLE_EQ(local[0], 1.0);
  EXPECT_DOUBLE_EQ(local[1], 2.0);
  EXPECT_DOUBLE_EQ(local[2], 3.0);
}

TEST(ConstantParameterField, ZeroChannelField)
{
  ConstantParameterField field(0, 10, nullptr);

  EXPECT_EQ(field.numChannels(), 0);
  EXPECT_EQ(field.numLocalDofs(), 0);
  EXPECT_EQ(field.kind(), ParameterFieldKind::CONSTANT);

  field.computeValue(0, 0, nullptr);
  field.computeDerivative(0, 0, nullptr);

  const auto *dofLayout = field.dofLayout();
  ASSERT_NE(dofLayout, nullptr);
  EXPECT_EQ(dofLayout->numLocalDofs(), 0);
  EXPECT_EQ(dofLayout->numGlobalDofs(), 0);
}

TEST(ConstantParameterField, SetGlobalDataReplacesSharedValues)
{
  ES::VXd data1(2);
  data1 << 1.0, 2.0;
  ES::VXd data2(2);
  data2 << 10.0, 20.0;

  ConstantParameterField field(2, 3, data1.data());

  ES::VXd value(2);
  field.computeValue(0, 0, value.data());
  EXPECT_DOUBLE_EQ(value[0], 1.0);

  field.setGlobalData(data2.data());
  field.computeValue(2, 0, value.data());
  EXPECT_DOUBLE_EQ(value[0], 10.0);
  EXPECT_DOUBLE_EQ(value[1], 20.0);
}

TEST(ConstantParameterField, SpecConstructorValidatesSize)
{
  ParameterFieldSpec spec;
  spec.numChannels = 3;

  ES::VXd good(3);
  good << 1.0, 2.0, 3.0;
  EXPECT_NO_THROW(ConstantParameterField(spec, 4, good));

  ES::VXd bad(5);
  bad.setZero();
  EXPECT_THROW(ConstantParameterField(spec, 4, bad), std::invalid_argument);
}

TEST(ConstantParameterField, ValuesAccessorReflectsSharedSet)
{
  ParameterFieldSpec spec;
  spec.numChannels = 2;

  ES::VXd vals(2);
  vals << 7.0, 8.0;
  ConstantParameterField field(spec, 5, vals);

  EXPECT_EQ(field.numElements(), 5);
  ASSERT_EQ(field.values().size(), 2);
  EXPECT_DOUBLE_EQ(field.values()[0], 7.0);
  EXPECT_DOUBLE_EQ(field.values()[1], 8.0);
  EXPECT_EQ(field.globalData(), field.values().data());
}

TEST(ConstantParameterField, CastToOptimizableFieldSucceeds)
{
  ES::VXd shared(2);
  shared << 1.0, 2.0;

  ConstantParameterField field(2, 2, shared.data());

  auto *opt = dynamic_cast<OptimizableField *>(&field);
  ASSERT_NE(opt, nullptr);

  ES::MXd deriv(2, 2);
  opt->computeDerivative(0, 0, deriv.data());
  EXPECT_TRUE(deriv.isIdentity());
}
