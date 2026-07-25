#include <gtest/gtest.h>

#include "material/core/materialFrameField.h"
#include "material/plastic/plasticModel3DDeformationGradient.h"
#include "material/plastic/plasticModel3D3DOF.h"

#include <limits>
#include <stdexcept>

namespace
{
using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

MaterialFrame obliqueFrame()
{
  constexpr double kInvSqrt2 = 0.7071067811865475244;
  MaterialFrame frame;
  frame.col(0) << kInvSqrt2, kInvSqrt2, 0.0;
  frame.col(1) << -kInvSqrt2, kInvSqrt2, 0.0;
  frame.col(2) << 0.0, 0.0, 1.0;
  return frame;
}
}  // namespace

TEST(MaterialFrameFieldGTest, GlobalAxesAndConstantSampleValidatedElements)
{
  GlobalAxesMaterialFrameField globalAxes(2);
  EXPECT_TRUE(globalAxes.materialToReferenceFrame(0, 0).isIdentity());
  EXPECT_TRUE(globalAxes.materialToReferenceFrame(1, 3).isIdentity());
  EXPECT_THROW(globalAxes.materialToReferenceFrame(-1, 0), std::out_of_range);
  EXPECT_THROW(globalAxes.materialToReferenceFrame(2, 0), std::out_of_range);
  EXPECT_THROW(globalAxes.materialToReferenceFrame(0, -1), std::out_of_range);

  const MaterialFrame frame = obliqueFrame();
  ConstantMaterialFrameField constant(3, frame);
  EXPECT_EQ(constant.numElements(), 3);
  EXPECT_TRUE(
    constant.materialToReferenceFrame(2, 7).isApprox(frame, 1e-12));
  EXPECT_TRUE(constant.primaryAxis(1, 0).isApprox(frame.col(0), 1e-12));
}

TEST(MaterialFrameFieldGTest, ElementwiseSamplesPerElement)
{
  const MaterialFrame frame = obliqueFrame();
  ElementwiseMaterialFrameField field(
    { MaterialFrame::Identity(), frame });
  EXPECT_EQ(field.numElements(), 2);
  EXPECT_TRUE(field.materialToReferenceFrame(0, 0).isIdentity());
  EXPECT_TRUE(field.materialToReferenceFrame(1, 0).isApprox(frame, 1e-12));
  EXPECT_THROW(field.materialToReferenceFrame(2, 0), std::out_of_range);
}

TEST(MaterialFrameFieldGTest, ExplicitFramesMustBeFiniteOrthonormalAndRightHanded)
{
  MaterialFrame nonFinite = MaterialFrame::Identity();
  nonFinite(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ConstantMaterialFrameField(1, nonFinite), std::invalid_argument);

  MaterialFrame nonOrthonormal = MaterialFrame::Identity();
  nonOrthonormal(0, 1) = 0.2;
  EXPECT_THROW(
    ConstantMaterialFrameField(1, nonOrthonormal),
    std::invalid_argument);

  MaterialFrame leftHanded = MaterialFrame::Identity();
  leftHanded.col(2) *= -1.0;
  EXPECT_THROW(
    ElementwiseMaterialFrameField({ leftHanded }),
    std::invalid_argument);
}

TEST(MaterialFrameFieldGTest, PrimarySecondaryDirectionsProduceRightHandedFrames)
{
  ES::M3Xd primary(3, 2);
  ES::M3Xd secondary(3, 2);
  primary.col(0) << 1.0, 1.0, 0.0;
  secondary.col(0) << 0.0, 1.0, 1.0;
  primary.col(1) << 1.0, -2.0, 3.0;
  secondary.col(1) << 2.0, 1.0, 0.5;

  auto field =
    ElementwiseMaterialFrameField::fromPrimarySecondaryDirections(
      primary, secondary);
  ASSERT_NE(field, nullptr);
  ASSERT_EQ(field->numElements(), 2);
  for (int elementId = 0; elementId < 2; elementId++) {
    const MaterialFrame frame =
      field->materialToReferenceFrame(elementId, 0);
    EXPECT_TRUE(
      (frame.transpose() * frame).isApprox(MaterialFrame::Identity(), 1e-12));
    EXPECT_NEAR(frame.determinant(), 1.0, 1e-12);
    EXPECT_TRUE(
      frame.col(0).isApprox(
        primary.col(elementId).normalized(), 1e-12));
  }
}

TEST(MaterialFrameFieldGTest, PrimarySecondaryDirectionsRejectInvalidInput)
{
  ES::M3Xd primary(3, 1);
  ES::M3Xd secondary(3, 1);

  primary.setZero();
  secondary.col(0) << 0.0, 1.0, 0.0;
  EXPECT_THROW(
    ElementwiseMaterialFrameField::fromPrimarySecondaryDirections(
      primary, secondary),
    std::invalid_argument);

  primary.col(0) << 1.0, 0.0, 0.0;
  secondary.col(0) << 2.0, 0.0, 0.0;
  EXPECT_THROW(
    ElementwiseMaterialFrameField::fromPrimarySecondaryDirections(
      primary, secondary),
    std::invalid_argument);

  secondary.col(0) << 0.0, 1.0, 0.0;
  primary(0, 0) = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
    ElementwiseMaterialFrameField::fromPrimarySecondaryDirections(
      primary, secondary),
    std::invalid_argument);
}

TEST(MaterialFrameFieldGTest, Dof3UsesMaterialToReferenceColumnConvention)
{
  const MaterialFrame frame = obliqueFrame();
  PlasticModel3D3DOF deformationGradientModel(frame.transpose());

  const ES::V3d parameters(0.8, 1.2, 1.5);
  double actualStorage[9];
  deformationGradientModel.computeA(
    parameters.data(), actualStorage);
  const ES::M3d actual =
    Eigen::Map<const ES::M3d>(actualStorage);
  const ES::M3d expected =
    frame * parameters.asDiagonal() * frame.transpose();
  EXPECT_TRUE(actual.isApprox(expected, 1e-12));
  EXPECT_FALSE(actual.isDiagonal());

  ES::V3d roundTrip;
  deformationGradientModel.toParam(
    actualStorage, roundTrip.data());
  EXPECT_TRUE(roundTrip.isApprox(parameters, 1e-12));
}

TEST(MaterialFrameFieldGTest, ReferenceRotationReturnsNewImmutableField)
{
  const MaterialFrame frame = obliqueFrame();
  ConstantMaterialFrameField field(1, frame);
  MaterialFrame rotation = MaterialFrame::Identity();
  rotation << 0.0, -1.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, 1.0;

  auto transformed =
    transformReferenceMaterialFrames(field, rotation);
  ASSERT_NE(transformed, nullptr);
  EXPECT_TRUE(
    transformed->materialToReferenceFrame(0, 0)
      .isApprox(rotation * frame, 1e-12));
  EXPECT_TRUE(
    field.materialToReferenceFrame(0, 0).isApprox(frame, 1e-12));
}
