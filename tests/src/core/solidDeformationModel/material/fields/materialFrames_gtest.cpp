#include <gtest/gtest.h>

#include "material/frame/materialFrames.h"
#include "material/plastic/plasticModel3DDeformationGradient.h"
#include "material/plastic/plasticModel3D3DOF.h"

#include <limits>
#include <span>
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

TEST(MaterialFramesGTest, StoresAndValidatesElementwiseFrames)
{
  const MaterialFrame frame = obliqueFrame();
  MaterialFrames frames({ MaterialFrame::Identity(), frame });
  EXPECT_EQ(frames.numElements(), 2);
  EXPECT_TRUE(frames[0].isIdentity());
  EXPECT_TRUE(frames[1].isApprox(frame, 1e-12));
  EXPECT_THROW(frames[-1], std::out_of_range);
  EXPECT_THROW(frames[2], std::out_of_range);

  MaterialFrame nonFinite = MaterialFrame::Identity();
  nonFinite(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(MaterialFrames({ nonFinite }), std::invalid_argument);

  MaterialFrame nonOrthonormal = MaterialFrame::Identity();
  nonOrthonormal(0, 1) = 0.2;
  EXPECT_THROW(MaterialFrames({ nonOrthonormal }), std::invalid_argument);

  MaterialFrame leftHanded = MaterialFrame::Identity();
  leftHanded.col(2) *= -1.0;
  EXPECT_THROW(MaterialFrames({ leftHanded }), std::invalid_argument);
}

TEST(MaterialFramesGTest, IdentityCreatesOneExplicitFramePerElement)
{
  const MaterialFrames frames = MaterialFrames::identity(3);
  ASSERT_EQ(frames.numElements(), 3);
  EXPECT_EQ(frames.values().size(), 3);
  for (int element = 0; element < frames.numElements(); ++element)
    EXPECT_TRUE(frames[element].isIdentity());
  EXPECT_THROW(MaterialFrames::identity(-1), std::invalid_argument);
}

TEST(MaterialFramesGTest, PrimaryAxesProduceDeterministicFullFrames)
{
  ES::M3Xd primary(3, 3);
  primary.col(0) << 1.0, 0.0, 0.0;
  primary.col(1) << 1.0, 2.0, 3.0;
  primary.col(2) << 0.0, 0.0, -2.0;

  const MaterialFrames first = materialFramesFromPrimaryAxes(primary);
  const MaterialFrames second = materialFramesFromPrimaryAxes(primary);
  ASSERT_EQ(first.numElements(), 3);
  for (int element = 0; element < 3; ++element) {
    const MaterialFrame &frame = first[element];
    EXPECT_TRUE(
      frame.col(0).isApprox(primary.col(element).normalized(), 1e-12));
    EXPECT_TRUE(
      (frame.transpose() * frame).isApprox(
        MaterialFrame::Identity(), 1e-12));
    EXPECT_NEAR(frame.determinant(), 1.0, 1e-12);
    EXPECT_TRUE(frame.isApprox(second[element], 1e-12));
  }

  ES::M3Xd invalid = ES::M3Xd::Zero(3, 1);
  EXPECT_THROW(
    materialFramesFromPrimaryAxes(invalid), std::invalid_argument);
}

TEST(MaterialFramesGTest, Dof3UsesMaterialToReferenceColumnConvention)
{
  const MaterialFrame frame = obliqueFrame();
  PlasticModel3D3DOF deformationGradientModel(frame.transpose());

  const ES::V3d parameters(0.8, 1.2, 1.5);
  const ES::M3d actual = deformationGradientModel.computeA(
    std::span<const double>(parameters.data(), 3));
  const ES::M3d expected =
    frame * parameters.asDiagonal() * frame.transpose();
  EXPECT_TRUE(actual.isApprox(expected, 1e-12));
  EXPECT_FALSE(actual.isDiagonal());

  ES::V3d roundTrip;
  deformationGradientModel.toParam(
    actual, std::span<double>(roundTrip.data(), 3));
  EXPECT_TRUE(roundTrip.isApprox(parameters, 1e-12));
}
