#include "gtest/gtest.h"

#include "material/fields/elementwiseParameterField.h"
#include "EigenSupport.h"

using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

TEST(ElementwiseParameterField, OwnsValuesAndGathersPerElement)
{
  ParameterFieldSpec spec;
  spec.domain = ParameterDomain::PLASTIC;
  spec.modelId = "volumetric_dof6";
  spec.numChannels = 6;
  spec.channelNames = { "Fxx", "Fxy", "Fxz", "Fyy", "Fyz", "Fzz" };

  ES::VXd values(12);
  values << 1.0, 0.0, 0.0, 1.0, 0.0, 1.0,
            1.2, 0.0, 0.0, 0.9, 0.0, 1.0;

  ElementwiseParameterField field(spec, 2, values);
  values.setConstant(9.0);

  EXPECT_EQ(field.spec().domain, ParameterDomain::PLASTIC);
  EXPECT_EQ(field.spec().modelId, "volumetric_dof6");
  EXPECT_EQ(field.kind(), ParameterFieldKind::ELEMENTWISE);
  EXPECT_EQ(field.numElements(), 2);
  EXPECT_EQ(field.numValueRows(), 2);
  EXPECT_EQ(field.numChannels(), 6);
  EXPECT_EQ(field.dofLayout()->numGlobalDofs(), 12);
  EXPECT_TRUE(field.dofLayout()->matchesParameterShape(6, 2));
  EXPECT_FALSE(field.dofLayout()->matchesParameterShape(6, 3));
  EXPECT_FALSE(field.dofLayout()->matchesParameterShape(5, 2));
  EXPECT_EQ(field.dofLayout()->globalDof(1, 3), 9);

  double out[6] = {};
  field.computeValue(1, 0, out);
  EXPECT_DOUBLE_EQ(out[0], 1.2);
  EXPECT_DOUBLE_EQ(out[3], 0.9);
  EXPECT_DOUBLE_EQ(out[5], 1.0);
}

TEST(ElementwiseParameterField, SetValuesCopiesReplacement)
{
  ParameterFieldSpec spec;
  spec.domain = ParameterDomain::ELASTIC;
  spec.modelId = "stable_neo";
  spec.numChannels = 2;

  ES::VXd values(4);
  values << 1.0, 0.3, 2.0, 0.4;
  ElementwiseParameterField field(spec, 2, values);

  ES::VXd replacement(4);
  replacement << 10.0, 0.1, 20.0, 0.2;
  field.setValues(replacement);
  replacement.setConstant(99.0);

  double out[2] = {};
  field.computeValue(1, 0, out);
  EXPECT_DOUBLE_EQ(out[0], 20.0);
  EXPECT_DOUBLE_EQ(out[1], 0.2);
}

TEST(ElementwiseParameterField, RejectsWrongValueSize)
{
  ParameterFieldSpec spec;
  spec.domain = ParameterDomain::PLASTIC;
  spec.modelId = "volumetric_dof3";
  spec.numChannels = 3;

  ES::VXd values(4);
  EXPECT_THROW(ElementwiseParameterField(spec, 2, values), std::invalid_argument);
}
