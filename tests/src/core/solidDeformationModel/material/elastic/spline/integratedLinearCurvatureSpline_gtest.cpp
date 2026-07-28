#include "material/elastic/spline/integratedLinearCurvatureSpline.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

namespace
{
namespace SDM = pgo::SolidDeformationModel;

double referenceCurvature(
  std::span<const double> knots,
  std::span<const double> curvature,
  double x)
{
  if (x <= knots.front())
    return curvature.front();
  if (x >= knots.back())
    return curvature.back();

  const auto rightIter = std::upper_bound(knots.begin(), knots.end(), x);
  const std::size_t right =
    static_cast<std::size_t>(rightIter - knots.begin());
  const std::size_t left = right - 1;
  const double t =
    (x - knots[left]) / (knots[right] - knots[left]);
  return curvature[left] +
    t * (curvature[right] - curvature[left]);
}

template<class Integrand>
double referencePiecewiseIntegral(
  std::span<const double> knots,
  double from,
  double to,
  Integrand integrand)
{
  if (from == to)
    return 0.0;

  std::vector<double> boundaries;
  boundaries.push_back(from);
  if (to > from) {
    for (double knot : knots) {
      if (knot > from && knot < to)
        boundaries.push_back(knot);
    }
  }
  else {
    for (auto iter = knots.rbegin(); iter != knots.rend(); ++iter) {
      if (*iter < from && *iter > to)
        boundaries.push_back(*iter);
    }
  }
  boundaries.push_back(to);

  constexpr double abscissa = 0.77459666924148337704;
  constexpr double sideWeight = 5.0 / 9.0;
  constexpr double centerWeight = 8.0 / 9.0;
  double integral = 0.0;
  for (std::size_t i = 0; i + 1 < boundaries.size(); ++i) {
    const double a = boundaries[i];
    const double b = boundaries[i + 1];
    const double center = 0.5 * (a + b);
    const double halfWidth = 0.5 * (b - a);
    integral += halfWidth *
      (sideWeight * integrand(center - abscissa * halfWidth) +
        centerWeight * integrand(center) +
        sideWeight * integrand(center + abscissa * halfWidth));
  }
  return integral;
}

double referenceSlope(
  std::span<const double> knots,
  std::span<const double> curvature,
  int anchorIndex,
  double anchorSlope,
  double x)
{
  const double anchorX = knots[static_cast<std::size_t>(anchorIndex)];
  return anchorSlope +
    referencePiecewiseIntegral(
      knots, anchorX, x,
      [&](double t) {
        return referenceCurvature(knots, curvature, t);
      });
}

double referenceValue(
  std::span<const double> knots,
  std::span<const double> curvature,
  int anchorIndex,
  double anchorValue,
  double anchorSlope,
  double x)
{
  const double anchorX = knots[static_cast<std::size_t>(anchorIndex)];
  return anchorValue + anchorSlope * (x - anchorX) +
    referencePiecewiseIntegral(
      knots, anchorX, x,
      [&](double t) {
        return (x - t) *
          referenceCurvature(knots, curvature, t);
      });
}

TEST(IntegratedLinearCurvatureSpline, PreservesAnchorAndMetadata)
{
  const std::array<double, 4> knots = { -1.0, 0.25, 0.9, 2.0 };
  const SDM::IntegratedLinearCurvatureSpline spline(
    knots, 1, 1.5, -0.75);
  const std::array<double, 4> curvature = { 2.0, 3.0, 4.0, 5.0 };

  EXPECT_EQ(spline.numKnots(), 4);
  EXPECT_EQ(spline.anchorIndex(), 1);
  EXPECT_TRUE(std::equal(
    spline.knots().begin(), spline.knots().end(), knots.begin()));
  EXPECT_DOUBLE_EQ(spline.y(curvature, knots[1]), 1.5);
  EXPECT_DOUBLE_EQ(spline.dy_dx(curvature, knots[1]), -0.75);
  EXPECT_DOUBLE_EQ(spline.d2y_dx2(curvature, knots[1]), curvature[1]);
}

TEST(IntegratedLinearCurvatureSpline, ConstantCurvatureRecoversQuadraticEverywhere)
{
  const std::array<double, 4> knots = { -1.2, 0.4, 0.9, 2.3 };
  constexpr int anchor = 1;
  constexpr double anchorValue = 1.2;
  constexpr double anchorSlope = -0.7;
  constexpr double curvatureValue = 2.5;
  const SDM::IntegratedLinearCurvatureSpline spline(
    knots, anchor, anchorValue, anchorSlope);
  const std::array<double, 4> curvature = {
    curvatureValue, curvatureValue, curvatureValue, curvatureValue
  };
  const std::array<double, 7> queries = {
    -3.0, -1.2, -0.25, 0.4, 1.4, 2.3, 4.0
  };

  for (double x : queries) {
    const double dx = x - knots[anchor];
    EXPECT_NEAR(
      spline.y(curvature, x),
      anchorValue + anchorSlope * dx +
        0.5 * curvatureValue * dx * dx,
      1e-12);
    EXPECT_NEAR(
      spline.dy_dx(curvature, x),
      anchorSlope + curvatureValue * dx,
      1e-12);
    EXPECT_DOUBLE_EQ(
      spline.d2y_dx2(curvature, x), curvatureValue);
  }
}

TEST(IntegratedLinearCurvatureSpline, LinearCurvatureRecoversCubicInsideKnotRange)
{
  const std::array<double, 5> knots = { -1.0, -0.1, 0.6, 1.7, 3.0 };
  constexpr int anchor = 2;
  constexpr double anchorValue = -0.3;
  constexpr double anchorSlope = 0.8;
  constexpr double curvatureIntercept = 1.1;
  constexpr double curvatureSlope = -0.35;
  std::array<double, 5> curvature;
  for (std::size_t i = 0; i < knots.size(); ++i)
    curvature[i] =
      curvatureIntercept + curvatureSlope * knots[i];

  const SDM::IntegratedLinearCurvatureSpline spline(
    knots, anchor, anchorValue, anchorSlope);
  const std::array<double, 7> queries = {
    -1.0, -0.7, -0.1, 0.6, 1.2, 1.7, 3.0
  };
  const double anchorX = knots[anchor];
  const double curvatureAtAnchor =
    curvatureIntercept + curvatureSlope * anchorX;

  for (double x : queries) {
    const double dx = x - anchorX;
    EXPECT_NEAR(
      spline.y(curvature, x),
      anchorValue + anchorSlope * dx +
        0.5 * curvatureAtAnchor * dx * dx +
        curvatureSlope * dx * dx * dx / 6.0,
      2e-12);
    EXPECT_NEAR(
      spline.dy_dx(curvature, x),
      anchorSlope + curvatureAtAnchor * dx +
        0.5 * curvatureSlope * dx * dx,
      2e-12);
    EXPECT_NEAR(
      spline.d2y_dx2(curvature, x),
      curvatureAtAnchor + curvatureSlope * dx,
      2e-12);
  }
}

TEST(IntegratedLinearCurvatureSpline, DerivativesMatchFiniteDifferences)
{
  const std::array<double, 5> knots = { -1.4, -0.2, 0.5, 1.8, 2.6 };
  const std::array<double, 5> curvature = { 1.2, 2.1, 0.7, 3.0, 1.6 };
  const SDM::IntegratedLinearCurvatureSpline spline(
    knots, 2, 0.4, -0.3);
  const std::array<double, 8> queries = {
    -2.0, -1.0, -0.05, 0.7, 1.1, 2.1, 2.4, 3.2
  };
  constexpr double h = 1e-6;

  for (double x : queries) {
    EXPECT_NEAR(
      spline.dy_dx(curvature, x),
      (spline.y(curvature, x + h) -
        spline.y(curvature, x - h)) /
        (2.0 * h),
      2e-8);
    EXPECT_NEAR(
      spline.d2y_dx2(curvature, x),
      (spline.dy_dx(curvature, x + h) -
        spline.dy_dx(curvature, x - h)) /
        (2.0 * h),
      2e-8);
  }
}

TEST(IntegratedLinearCurvatureSpline, IsC2AcrossKnotsAndExtrapolationBoundaries)
{
  const std::array<double, 5> knots = { -1.3, -0.4, 0.2, 1.5, 2.4 };
  const std::array<double, 5> curvature = { 0.8, 2.0, 1.1, 3.2, 1.7 };
  const SDM::IntegratedLinearCurvatureSpline spline(
    knots, 2, -0.2, 0.6);
  constexpr double epsilon = 1e-8;

  for (std::size_t i = 0; i < knots.size(); ++i) {
    const double knot = knots[i];
    EXPECT_NEAR(
      spline.y(curvature, knot - epsilon),
      spline.y(curvature, knot + epsilon),
      1e-6);
    EXPECT_NEAR(
      spline.dy_dx(curvature, knot - epsilon),
      spline.dy_dx(curvature, knot + epsilon),
      1e-6);
    EXPECT_NEAR(
      spline.d2y_dx2(curvature, knot - epsilon),
      spline.d2y_dx2(curvature, knot + epsilon),
      1e-6);
    EXPECT_DOUBLE_EQ(
      spline.d2y_dx2(curvature, knot), curvature[i]);
  }
}

TEST(IntegratedLinearCurvatureSpline, MatchesIndependentRandomizedIntegralOracle)
{
  std::mt19937 randomEngine(20230728);
  std::uniform_int_distribution<int> knotCountDistribution(2, 8);
  std::uniform_real_distribution<double> startDistribution(-2.0, 0.5);
  std::uniform_real_distribution<double> widthDistribution(0.1, 1.2);
  std::uniform_real_distribution<double> curvatureDistribution(-3.0, 4.0);
  std::uniform_real_distribution<double> anchorConditionDistribution(-1.5, 1.5);
  std::uniform_real_distribution<double> queryDistribution(-1.0, 1.0);

  for (int sample = 0; sample < 100; ++sample) {
    const int knotCount = knotCountDistribution(randomEngine);
    std::vector<double> knots(static_cast<std::size_t>(knotCount));
    knots[0] = startDistribution(randomEngine);
    for (int i = 1; i < knotCount; ++i) {
      knots[static_cast<std::size_t>(i)] =
        knots[static_cast<std::size_t>(i - 1)] +
        widthDistribution(randomEngine);
    }

    std::vector<double> curvature(static_cast<std::size_t>(knotCount));
    for (double &value : curvature)
      value = curvatureDistribution(randomEngine);

    std::uniform_int_distribution<int> anchorDistribution(
      0, knotCount - 1);
    const int anchorIndex = anchorDistribution(randomEngine);
    const double anchorValue =
      anchorConditionDistribution(randomEngine);
    const double anchorSlope =
      anchorConditionDistribution(randomEngine);
    const SDM::IntegratedLinearCurvatureSpline spline(
      knots, anchorIndex, anchorValue, anchorSlope);

    const double range = knots.back() - knots.front();
    for (int queryIndex = 0; queryIndex < 12; ++queryIndex) {
      const double x =
        0.5 * (knots.front() + knots.back()) +
        1.5 * range * queryDistribution(randomEngine);
      const double expectedValue = referenceValue(
        knots, curvature, anchorIndex,
        anchorValue, anchorSlope, x);
      const double expectedSlope = referenceSlope(
        knots, curvature, anchorIndex, anchorSlope, x);
      const double expectedCurvature =
        referenceCurvature(knots, curvature, x);
      const double valueScale =
        std::max(1.0, std::abs(expectedValue));
      const double slopeScale =
        std::max(1.0, std::abs(expectedSlope));
      const double curvatureScale =
        std::max(1.0, std::abs(expectedCurvature));

      EXPECT_NEAR(
        spline.y(curvature, x),
        expectedValue,
        2e-11 * valueScale);
      EXPECT_NEAR(
        spline.dy_dx(curvature, x),
        expectedSlope,
        2e-11 * slopeScale);
      EXPECT_NEAR(
        spline.d2y_dx2(curvature, x),
        expectedCurvature,
        2e-12 * curvatureScale);
    }
  }
}

TEST(IntegratedLinearCurvatureSpline, SupportsFirstAndLastKnotAnchors)
{
  const std::array<double, 4> knots = { -1.1, -0.3, 0.8, 2.2 };
  const std::array<double, 4> curvature = { 2.4, -0.5, 1.7, 3.1 };
  constexpr double anchorValue = 0.45;
  constexpr double anchorSlope = -0.8;
  const std::array<double, 7> queries = {
    -2.0, -1.1, -0.6, 0.2, 1.4, 2.2, 3.0
  };

  for (int anchorIndex : { 0, 3 }) {
    const SDM::IntegratedLinearCurvatureSpline spline(
      knots, anchorIndex, anchorValue, anchorSlope);
    for (double x : queries) {
      EXPECT_NEAR(
        spline.y(curvature, x),
        referenceValue(
          knots, curvature, anchorIndex,
          anchorValue, anchorSlope, x),
        1e-12);
      EXPECT_NEAR(
        spline.dy_dx(curvature, x),
        referenceSlope(
          knots, curvature, anchorIndex,
          anchorSlope, x),
        1e-12);
    }
  }
}

TEST(IntegratedLinearCurvatureSpline, UsesEndpointCurvaturesForQuadraticExtrapolation)
{
  const std::array<double, 4> knots = { -0.8, 0.1, 1.4, 2.0 };
  const std::array<double, 4> curvature = { 3.2, -0.4, 1.1, -2.3 };
  const SDM::IntegratedLinearCurvatureSpline spline(
    knots, 1, 0.7, -0.2);

  const double leftValue = spline.y(curvature, knots.front());
  const double leftSlope = spline.dy_dx(curvature, knots.front());
  constexpr double leftDx = -1.7;
  const double leftQuery = knots.front() + leftDx;
  EXPECT_NEAR(
    spline.y(curvature, leftQuery),
    leftValue + leftSlope * leftDx +
      0.5 * curvature.front() * leftDx * leftDx,
    1e-12);
  EXPECT_NEAR(
    spline.dy_dx(curvature, leftQuery),
    leftSlope + curvature.front() * leftDx,
    1e-12);
  EXPECT_DOUBLE_EQ(
    spline.d2y_dx2(curvature, leftQuery), curvature.front());

  const double rightValue = spline.y(curvature, knots.back());
  const double rightSlope = spline.dy_dx(curvature, knots.back());
  constexpr double rightDx = 1.3;
  const double rightQuery = knots.back() + rightDx;
  EXPECT_NEAR(
    spline.y(curvature, rightQuery),
    rightValue + rightSlope * rightDx +
      0.5 * curvature.back() * rightDx * rightDx,
    1e-12);
  EXPECT_NEAR(
    spline.dy_dx(curvature, rightQuery),
    rightSlope + curvature.back() * rightDx,
    1e-12);
  EXPECT_DOUBLE_EQ(
    spline.d2y_dx2(curvature, rightQuery), curvature.back());
}

TEST(IntegratedLinearCurvatureSpline, IsLinearInCurvatureValuesForZeroAnchorConditions)
{
  const std::array<double, 5> knots = { -1.2, -0.5, 0.3, 1.6, 2.1 };
  const std::array<double, 5> curvatureA = { 0.4, 2.1, -0.7, 1.8, 3.0 };
  const std::array<double, 5> curvatureB = { -1.2, 0.6, 2.4, -0.3, 1.1 };
  constexpr double alpha = 1.7;
  constexpr double beta = -0.45;
  std::array<double, 5> combined;
  for (std::size_t i = 0; i < combined.size(); ++i)
    combined[i] = alpha * curvatureA[i] + beta * curvatureB[i];

  const SDM::IntegratedLinearCurvatureSpline spline(
    knots, 2, 0.0, 0.0);
  const std::array<double, 8> queries = {
    -2.0, -1.2, -0.8, -0.1, 0.3, 1.0, 2.1, 3.0
  };
  for (double x : queries) {
    EXPECT_NEAR(
      spline.y(combined, x),
      alpha * spline.y(curvatureA, x) +
        beta * spline.y(curvatureB, x),
      2e-12);
    EXPECT_NEAR(
      spline.dy_dx(combined, x),
      alpha * spline.dy_dx(curvatureA, x) +
        beta * spline.dy_dx(curvatureB, x),
      2e-12);
    EXPECT_NEAR(
      spline.d2y_dx2(combined, x),
      alpha * spline.d2y_dx2(curvatureA, x) +
        beta * spline.d2y_dx2(curvatureB, x),
      2e-12);
  }
}

TEST(IntegratedLinearCurvatureSpline, CurvatureSensitivitiesMatchFiniteDifferences)
{
  const std::array<double, 5> knots = { -1.3, -0.4, 0.2, 1.4, 2.5 };
  const std::array<double, 5> curvature = { 0.7, 2.2, -0.8, 1.3, 3.1 };
  const SDM::IntegratedLinearCurvatureSpline spline(
    knots, 2, 0.6, -0.35);
  const std::array<double, 7> queries = {
    -2.0, -0.9, -0.1, 0.2, 0.8, 1.9, 3.0
  };
  constexpr double h = 1e-6;

  for (int parameter = 0; parameter < spline.numKnots(); ++parameter) {
    auto plus = curvature;
    auto minus = curvature;
    plus[static_cast<std::size_t>(parameter)] += h;
    minus[static_cast<std::size_t>(parameter)] -= h;

    for (double x : queries) {
      EXPECT_NEAR(
        spline.dy_dcurvature(parameter, x),
        (spline.y(plus, x) - spline.y(minus, x)) /
          (2.0 * h),
        2e-9);
      EXPECT_NEAR(
        spline.d2y_dx_dcurvature(parameter, x),
        (spline.dy_dx(plus, x) - spline.dy_dx(minus, x)) /
          (2.0 * h),
        2e-9);
    }
  }
}

TEST(IntegratedLinearCurvatureSpline, MixedSensitivitiesMatchSpatialFiniteDifferences)
{
  const std::array<double, 5> knots = { -1.1, -0.3, 0.4, 1.6, 2.2 };
  const SDM::IntegratedLinearCurvatureSpline spline(
    knots, 2, -0.4, 0.75);
  const std::array<double, 7> queries = {
    -1.8, -0.8, 0.0, 0.7, 1.2, 1.9, 2.8
  };
  constexpr double h = 1e-6;

  for (int parameter = 0; parameter < spline.numKnots(); ++parameter) {
    for (double x : queries) {
      EXPECT_NEAR(
        spline.d2y_dx_dcurvature(parameter, x),
        (spline.dy_dcurvature(parameter, x + h) -
          spline.dy_dcurvature(parameter, x - h)) /
          (2.0 * h),
        2e-10);
    }
  }
}

TEST(IntegratedLinearCurvatureSpline, CurvatureBasisReconstructsValueAndSlope)
{
  const std::array<double, 5> knots = { -1.4, -0.2, 0.5, 1.1, 2.7 };
  const std::array<double, 5> curvature = { 1.8, -0.6, 2.5, 0.4, 3.2 };
  constexpr int anchorIndex = 2;
  constexpr double anchorValue = 0.9;
  constexpr double anchorSlope = -0.45;
  const SDM::IntegratedLinearCurvatureSpline spline(
    knots, anchorIndex, anchorValue, anchorSlope);
  const std::array<double, 8> queries = {
    -2.2, -1.4, -0.7, 0.5, 0.9, 1.8, 2.7, 3.4
  };

  for (double x : queries) {
    const double dx = x - knots[anchorIndex];
    double reconstructedValue =
      anchorValue + anchorSlope * dx;
    double reconstructedSlope = anchorSlope;
    for (int parameter = 0; parameter < spline.numKnots(); ++parameter) {
      const double coefficient =
        curvature[static_cast<std::size_t>(parameter)];
      reconstructedValue += coefficient *
        spline.dy_dcurvature(parameter, x);
      reconstructedSlope += coefficient *
        spline.d2y_dx_dcurvature(parameter, x);
    }

    EXPECT_NEAR(
      spline.y(curvature, x), reconstructedValue, 2e-12);
    EXPECT_NEAR(
      spline.dy_dx(curvature, x), reconstructedSlope, 2e-12);
  }
}

TEST(IntegratedLinearCurvatureSpline, OwnsACopyOfKnots)
{
  std::vector<double> knots = { -0.5, 0.2, 1.4 };
  const std::array<double, 3> originalKnots = { -0.5, 0.2, 1.4 };
  const std::array<double, 3> curvature = { 1.0, 2.0, 4.0 };
  const SDM::IntegratedLinearCurvatureSpline spline(
    knots, 1, 0.3, -0.1);
  const double valueBefore = spline.y(curvature, 0.9);

  knots[0] = -10.0;
  knots[1] = -5.0;
  knots[2] = 8.0;

  EXPECT_TRUE(std::equal(
    spline.knots().begin(),
    spline.knots().end(),
    originalKnots.begin()));
  EXPECT_DOUBLE_EQ(spline.y(curvature, 0.9), valueBefore);
}

TEST(IntegratedLinearCurvatureSpline, RejectsInvalidConstructionInputs)
{
  EXPECT_THROW(
    SDM::IntegratedLinearCurvatureSpline(
      std::array<double, 1>{ 0.0 }, 0, 0.0, 0.0),
    std::invalid_argument);
  EXPECT_THROW(
    SDM::IntegratedLinearCurvatureSpline(
      std::array<double, 3>{ 0.0, 0.0, 1.0 }, 0, 0.0, 0.0),
    std::invalid_argument);
  EXPECT_THROW(
    SDM::IntegratedLinearCurvatureSpline(
      std::array<double, 3>{ 0.0, -1.0, 1.0 }, 0, 0.0, 0.0),
    std::invalid_argument);
  EXPECT_THROW(
    SDM::IntegratedLinearCurvatureSpline(
      std::array<double, 2>{
        0.0, std::numeric_limits<double>::infinity() },
      0, 0.0, 0.0),
    std::invalid_argument);
  EXPECT_THROW(
    SDM::IntegratedLinearCurvatureSpline(
      std::array<double, 2>{ 0.0, 1.0 }, -1, 0.0, 0.0),
    std::out_of_range);
  EXPECT_THROW(
    SDM::IntegratedLinearCurvatureSpline(
      std::array<double, 2>{ 0.0, 1.0 }, 2, 0.0, 0.0),
    std::out_of_range);
  EXPECT_THROW(
    SDM::IntegratedLinearCurvatureSpline(
      std::array<double, 2>{ 0.0, 1.0 }, 0,
      std::numeric_limits<double>::quiet_NaN(), 0.0),
    std::invalid_argument);
}

TEST(IntegratedLinearCurvatureSpline, RejectsInvalidEvaluationInputs)
{
  const SDM::IntegratedLinearCurvatureSpline spline(
    std::array<double, 3>{ 0.0, 1.0, 2.0 }, 1, 0.0, 0.0);
  const std::array<double, 3> curvature = { 1.0, 2.0, 3.0 };

  EXPECT_THROW(
    spline.y(std::array<double, 2>{ 1.0, 2.0 }, 0.5),
    std::invalid_argument);
  EXPECT_THROW(
    spline.dy_dx(
      std::array<double, 3>{
        1.0, std::numeric_limits<double>::infinity(), 3.0 },
      0.5),
    std::invalid_argument);
  EXPECT_THROW(
    spline.d2y_dx2(
      curvature, std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
  EXPECT_THROW(
    spline.dy_dcurvature(-1, 0.5),
    std::out_of_range);
  EXPECT_THROW(
    spline.d2y_dx_dcurvature(3, 0.5),
    std::out_of_range);
  EXPECT_THROW(
    spline.dy_dcurvature(
      1, std::numeric_limits<double>::infinity()),
    std::invalid_argument);
}

}  // namespace
