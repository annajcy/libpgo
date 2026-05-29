#include "gtest/gtest.h"

#include "formulations/parameters/constantParameterField.h"
#include "EigenSupport.h"

using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

TEST(ConstantParameterField, SampleReturnsCorrectElementConstantValue)
{
  ES::VXd globalParams(12);
  globalParams << 1.0, 2.0, 3.0,
                  4.0, 5.0, 6.0,
                  7.0, 8.0, 9.0,
                  10.0, 11.0, 12.0;

  ConstantParameterField field(3, 4, globalParams.data());

  ParameterSample sample;
  field.sample(0, 0, sample);
  EXPECT_EQ(sample.value.size(), 3);
  EXPECT_DOUBLE_EQ(sample.value[0], 1.0);
  EXPECT_DOUBLE_EQ(sample.value[1], 2.0);
  EXPECT_DOUBLE_EQ(sample.value[2], 3.0);

  field.sample(2, 0, sample);
  EXPECT_DOUBLE_EQ(sample.value[0], 7.0);
  EXPECT_DOUBLE_EQ(sample.value[1], 8.0);
  EXPECT_DOUBLE_EQ(sample.value[2], 9.0);
}

TEST(ConstantParameterField, SampleIgnoresQuadratureId)
{
  ES::VXd globalParams(6);
  globalParams << 10.0, 20.0, 30.0, 40.0, 50.0, 60.0;

  ConstantParameterField field(2, 3, globalParams.data());

  ParameterSample s0, s1, s2;
  field.sample(1, 0, s0);
  field.sample(1, 3, s1);
  field.sample(1, 7, s2);

  EXPECT_DOUBLE_EQ(s0.value[0], 30.0);
  EXPECT_DOUBLE_EQ(s0.value[1], 40.0);
  EXPECT_DOUBLE_EQ(s1.value[0], 30.0);
  EXPECT_DOUBLE_EQ(s1.value[1], 40.0);
  EXPECT_DOUBLE_EQ(s2.value[0], 30.0);
  EXPECT_DOUBLE_EQ(s2.value[1], 40.0);
}

TEST(ConstantParameterField, DValueDLocalIsIdentity)
{
  ES::VXd globalParams(6);
  globalParams << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  ConstantParameterField field(2, 3, globalParams.data());

  ParameterSample sample;
  field.sample(0, 0, sample);

  EXPECT_EQ(sample.dValueDLocal.rows(), 2);
  EXPECT_EQ(sample.dValueDLocal.cols(), 2);
  EXPECT_TRUE(sample.dValueDLocal.isIdentity());
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

  ParameterSample sample;
  field.sample(0, 0, sample);
  EXPECT_EQ(sample.value.size(), 0);
  EXPECT_EQ(sample.dValueDLocal.rows(), 0);
  EXPECT_EQ(sample.dValueDLocal.cols(), 0);

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

  ParameterSample sample;
  field.sample(0, 0, sample);
  EXPECT_DOUBLE_EQ(sample.value[0], 1.0);

  field.setGlobalData(data2.data());
  field.sample(0, 0, sample);
  EXPECT_DOUBLE_EQ(sample.value[0], 10.0);
}

TEST(ConstantParameterField, ExposeAsOptimizationVariableThrows)
{
  ES::VXd data(4);
  ConstantParameterField field(2, 2, data.data());

  EXPECT_THROW(field.setExposeAsOptimizationVariable(true), std::invalid_argument);
}

TEST(ConstantParameterField, MultipleElementsCorrectSlices)
{
  ES::VXd globalParams(8);
  globalParams << 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0;

  ConstantParameterField field(2, 4, globalParams.data());

  ParameterSample sample;
  field.sample(0, 0, sample);
  EXPECT_DOUBLE_EQ(sample.value[0], 10.0);
  EXPECT_DOUBLE_EQ(sample.value[1], 20.0);

  field.sample(1, 0, sample);
  EXPECT_DOUBLE_EQ(sample.value[0], 30.0);
  EXPECT_DOUBLE_EQ(sample.value[1], 40.0);

  field.sample(3, 0, sample);
  EXPECT_DOUBLE_EQ(sample.value[0], 70.0);
  EXPECT_DOUBLE_EQ(sample.value[1], 80.0);
}
