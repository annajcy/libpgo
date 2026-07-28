#include <gtest/gtest.h>

#include "material/elastic/elasticModel1D.h"
#include "material/elastic/elasticModel1DCubicSpline.h"
#include "material/elastic/elasticModel1DIntegratedLinearCurvatureSpline.h"
#include "material/elastic/elasticModel1DLogSquared.h"
#include "material/elastic/elasticModel1DQuadratic.h"
#include "material/elastic/elasticModel1DZero.h"
#include "material/elastic/spline/integratedLinearCurvatureSpline.h"
#include "naturalCubicSplineDerivatives.h"

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <vector>

namespace
{
namespace SDM = pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;
using pgo::NonlinearOptimization::NaturalCubicSpline2DWithParameterDerivatives;

TEST(NaturalCubicSpline2DWithParameterDerivatives, EvaluatesValueAndParameterDerivatives)
{
  const ES::VXd x((ES::V3d() << 0.0, 1.0, 2.0).finished());
  const ES::VXd y((ES::V3d() << 0.0, 1.0, 4.0).finished());
  NaturalCubicSpline2DWithParameterDerivatives spline(x, y);
  constexpr double query = 0.75;
  constexpr double h = 1e-6;

  EXPECT_NEAR(spline.y(0.0), 0.0, 1e-12);
  EXPECT_NEAR(spline.y(1.0), 1.0, 1e-12);
  EXPECT_NEAR(spline.y(2.0), 4.0, 1e-12);
  EXPECT_NEAR(
    spline.dy_dx(query),
    (spline.y(query + h) - spline.y(query - h)) / (2.0 * h),
    1e-8);

  ES::VXd parameterGradient(3);
  spline.dy_dparam(query, parameterGradient);
  for (int i = 0; i < 3; ++i) {
    ES::VXd yPlus = y;
    ES::VXd yMinus = y;
    yPlus[i] += h;
    yMinus[i] -= h;
    EXPECT_NEAR(
      parameterGradient[i],
      (spline.y(query, yPlus) - spline.y(query, yMinus)) / (2.0 * h),
      1e-8);
  }

  ES::MXd parameterHessian(3, 3);
  spline.d2y_dparam2(query, parameterHessian);
  EXPECT_NEAR(parameterHessian.norm(), 0.0, 1e-15);
  EXPECT_THROW(
    NaturalCubicSpline2DWithParameterDerivatives(
      (ES::V2d() << 0.0, 0.0).finished()),
    std::invalid_argument);
}

TEST(NaturalCubicSpline2DWithParameterDerivatives, MixedDerivativeAndExtrapolation)
{
  const ES::VXd x((ES::V3d() << 0.0, 1.0, 2.0).finished());
  const ES::VXd y((ES::V3d() << 0.0, 1.0, 4.0).finished());
  NaturalCubicSpline2DWithParameterDerivatives spline(x, y);
  constexpr double query = 0.75;
  constexpr double h = 1e-6;

  ES::VXd gradientPlus(3);
  ES::VXd gradientMinus(3);
  spline.dy_dparam(query + h, gradientPlus);
  spline.dy_dparam(query - h, gradientMinus);
  ES::VXd mixedDerivative(3);
  spline.d2y_dparam_dx(query, mixedDerivative);
  EXPECT_TRUE(mixedDerivative.isApprox((gradientPlus - gradientMinus) / (2.0 * h), 1e-8));

  EXPECT_NEAR(spline.d2y_dx2(-0.5), 0.0, 1e-15);
  EXPECT_NEAR(spline.d2y_dx2(2.5), 0.0, 1e-15);
  EXPECT_THROW(spline.y(0.5, ES::V2d::Zero()), std::invalid_argument);
}

TEST(ElasticModel1D, QuadraticUsesScalarConstitutiveAPI)
{
  SDM::ElasticModel1DQuadratic model(2.0);
  const std::array<double, 1> parameters = { 1.5 };
  constexpr double x = 1.2;
  constexpr double h = 1e-6;

  EXPECT_NEAR(model.compute_psi(parameters, x), 2.16, 1e-12);
  EXPECT_NEAR(model.compute_dpsi_dx(parameters, x), 3.6, 1e-12);
  EXPECT_NEAR(model.compute_d2psi_dx2(parameters, x), 3.0, 1e-12);
  EXPECT_NEAR(model.compute_dpsi_dparam(parameters, 0, x), 1.44, 1e-12);
  EXPECT_NEAR(model.compute_d2psi_dx_dparam(parameters, 0, x), 2.4, 1e-12);
  EXPECT_NEAR(model.compute_d2psi_dparam2(parameters, 0, 0, x), 0.0, 1e-15);

  EXPECT_NEAR(
    model.compute_dpsi_dx(parameters, x),
    (model.compute_psi(parameters, x + h) - model.compute_psi(parameters, x - h)) / (2.0 * h),
    1e-8);

  const std::array<double, 1> plus = { parameters[0] + h };
  const std::array<double, 1> minus = { parameters[0] - h };
  EXPECT_NEAR(
    model.compute_dpsi_dparam(parameters, 0, x),
    (model.compute_psi(plus, x) - model.compute_psi(minus, x)) / (2.0 * h),
    1e-8);
}

TEST(ElasticModel1D, ZeroIsParameterFreeAndReturnsZero)
{
  SDM::ElasticModel1DZero model;
  const std::span<const double> parameters;

  EXPECT_EQ(model.getNumParameters(), 0);
  for (double x : { -3.0, 0.0, 1.0, 4.5 }) {
    EXPECT_DOUBLE_EQ(model.compute_psi(parameters, x), 0.0);
    EXPECT_DOUBLE_EQ(model.compute_dpsi_dx(parameters, x), 0.0);
    EXPECT_DOUBLE_EQ(model.compute_d2psi_dx2(parameters, x), 0.0);
  }
}

TEST(ElasticModel1D, ZeroRejectsParametersIndicesAndNonFiniteQueries)
{
  SDM::ElasticModel1DZero model;
  const std::span<const double> parameters;
  const std::array<double, 1> nonemptyParameters = { 0.0 };

  EXPECT_THROW(
    model.compute_psi(nonemptyParameters, 1.0),
    std::invalid_argument);
  EXPECT_THROW(
    model.compute_psi(
      parameters, std::numeric_limits<double>::infinity()),
    std::invalid_argument);
  EXPECT_THROW(
    model.compute_dpsi_dparam(parameters, 0, 1.0),
    std::out_of_range);
  EXPECT_THROW(
    model.compute_d2psi_dx_dparam(parameters, -1, 1.0),
    std::out_of_range);
  EXPECT_THROW(
    model.compute_d2psi_dparam2(parameters, 0, 0, 1.0),
    std::out_of_range);
}

TEST(ElasticModel1D, LogSquaredMatchesAnalyticValues)
{
  SDM::ElasticModel1DLogSquared model;
  const std::array<double, 1> parameters = { 2.4 };

  EXPECT_EQ(model.getNumParameters(), 1);
  EXPECT_DOUBLE_EQ(model.compute_psi(parameters, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(model.compute_dpsi_dx(parameters, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(
    model.compute_d2psi_dx2(parameters, 1.0),
    parameters[0]);
  EXPECT_DOUBLE_EQ(
    model.compute_dpsi_dparam(parameters, 0, 1.0),
    0.0);
  EXPECT_DOUBLE_EQ(
    model.compute_d2psi_dx_dparam(parameters, 0, 1.0),
    0.0);
  EXPECT_DOUBLE_EQ(
    model.compute_d2psi_dparam2(parameters, 0, 0, 1.0),
    0.0);

  const double e = std::exp(1.0);
  EXPECT_NEAR(
    model.compute_psi(parameters, e),
    0.5 * parameters[0],
    1e-14);
  EXPECT_NEAR(
    model.compute_dpsi_dx(parameters, e),
    parameters[0] / e,
    1e-14);
  EXPECT_NEAR(
    model.compute_d2psi_dx2(parameters, e),
    0.0,
    1e-14);
  EXPECT_NEAR(
    model.compute_dpsi_dparam(parameters, 0, e),
    0.5,
    1e-14);
  EXPECT_NEAR(
    model.compute_d2psi_dx_dparam(parameters, 0, e),
    1.0 / e,
    1e-14);
}

TEST(ElasticModel1D, LogSquaredRandomizedDerivativesMatchFiniteDifferences)
{
  SDM::ElasticModel1DLogSquared model;
  std::mt19937 randomEngine(0x1065A);
  std::uniform_real_distribution<double> logXDistribution(-4.0, 4.0);
  std::uniform_real_distribution<double> lambdaDistribution(-5.0, 8.0);
  constexpr double relativeXStep = 1e-6;

  for (int sample = 0; sample < 200; ++sample) {
    const double x = std::exp(logXDistribution(randomEngine));
    const std::array<double, 1> parameters = {
      lambdaDistribution(randomEngine)
    };
    const double xStep = relativeXStep * x;
    const double parameterStep =
      1e-6 * (1.0 + std::abs(parameters[0]));
    const std::array<double, 1> plus = {
      parameters[0] + parameterStep
    };
    const std::array<double, 1> minus = {
      parameters[0] - parameterStep
    };

    const double first =
      model.compute_dpsi_dx(parameters, x);
    const double second =
      model.compute_d2psi_dx2(parameters, x);
    const double parameterDerivative =
      model.compute_dpsi_dparam(parameters, 0, x);
    const double mixedDerivative =
      model.compute_d2psi_dx_dparam(parameters, 0, x);

    EXPECT_NEAR(
      first,
      (model.compute_psi(parameters, x + xStep) -
        model.compute_psi(parameters, x - xStep)) /
        (2.0 * xStep),
      2e-8 * (1.0 + std::abs(first)));
    EXPECT_NEAR(
      second,
      (model.compute_dpsi_dx(parameters, x + xStep) -
        model.compute_dpsi_dx(parameters, x - xStep)) /
        (2.0 * xStep),
      2e-8 * (1.0 + std::abs(second)));
    EXPECT_NEAR(
      parameterDerivative,
      (model.compute_psi(plus, x) -
        model.compute_psi(minus, x)) /
        (2.0 * parameterStep),
      2e-8 * (1.0 + std::abs(parameterDerivative)));
    EXPECT_NEAR(
      mixedDerivative,
      (model.compute_dpsi_dx(plus, x) -
        model.compute_dpsi_dx(minus, x)) /
        (2.0 * parameterStep),
      2e-8 * (1.0 + std::abs(mixedDerivative)));
    EXPECT_NEAR(
      mixedDerivative,
      (model.compute_dpsi_dparam(
         parameters, 0, x + xStep) -
        model.compute_dpsi_dparam(
          parameters, 0, x - xStep)) /
        (2.0 * xStep),
      2e-8 * (1.0 + std::abs(mixedDerivative)));
    EXPECT_DOUBLE_EQ(
      model.compute_d2psi_dparam2(parameters, 0, 0, x),
      0.0);
    EXPECT_DOUBLE_EQ(
      (model.compute_dpsi_dparam(plus, 0, x) -
        model.compute_dpsi_dparam(minus, 0, x)) /
        (2.0 * parameterStep),
      0.0);
  }
}

TEST(ElasticModel1D, LogSquaredIsLinearInItsParameter)
{
  SDM::ElasticModel1DLogSquared model;
  const std::array<double, 1> lambdaA = { -1.7 };
  const std::array<double, 1> lambdaB = { 3.2 };
  const std::array<double, 1> lambdaSum = {
    lambdaA[0] + lambdaB[0]
  };

  for (double x : { 0.1, 0.5, 1.0, 2.0, 10.0 }) {
    const double energySum =
      model.compute_psi(lambdaA, x) +
      model.compute_psi(lambdaB, x);
    const double slopeSum =
      model.compute_dpsi_dx(lambdaA, x) +
      model.compute_dpsi_dx(lambdaB, x);
    const double curvatureSum =
      model.compute_d2psi_dx2(lambdaA, x) +
      model.compute_d2psi_dx2(lambdaB, x);
    EXPECT_NEAR(
      model.compute_psi(lambdaSum, x),
      energySum,
      2e-14 * (1.0 + std::abs(energySum)));
    EXPECT_NEAR(
      model.compute_dpsi_dx(lambdaSum, x),
      slopeSum,
      2e-14 * (1.0 + std::abs(slopeSum)));
    EXPECT_NEAR(
      model.compute_d2psi_dx2(lambdaSum, x),
      curvatureSum,
      2e-14 * (1.0 + std::abs(curvatureSum)));
  }
}

TEST(ElasticModel1D, LogSquaredRejectsInvalidInputs)
{
  SDM::ElasticModel1DLogSquared model;
  const std::array<double, 1> parameters = { 2.0 };

  EXPECT_THROW(
    model.compute_psi(std::span<const double>{}, 1.0),
    std::invalid_argument);
  EXPECT_THROW(
    model.compute_psi(
      std::array<double, 2>{ 1.0, 2.0 }, 1.0),
    std::invalid_argument);
  EXPECT_THROW(
    model.compute_psi(
      std::array<double, 1>{
        std::numeric_limits<double>::quiet_NaN() },
      1.0),
    std::invalid_argument);
  EXPECT_THROW(
    model.compute_psi(parameters, 0.0),
    std::domain_error);
  EXPECT_THROW(
    model.compute_dpsi_dx(parameters, -1.0),
    std::domain_error);
  EXPECT_THROW(
    model.compute_d2psi_dx2(
      parameters, std::numeric_limits<double>::infinity()),
    std::domain_error);
  EXPECT_THROW(
    model.compute_dpsi_dparam(
      parameters,
      0,
      std::numeric_limits<double>::quiet_NaN()),
    std::domain_error);
  EXPECT_THROW(
    model.compute_dpsi_dparam(parameters, -1, 1.0),
    std::out_of_range);
  EXPECT_THROW(
    model.compute_d2psi_dx_dparam(parameters, 1, 1.0),
    std::out_of_range);
  EXPECT_THROW(
    model.compute_d2psi_dparam2(parameters, 0, 1, 1.0),
    std::out_of_range);
}

TEST(ElasticModel1D, IntegratedCurvatureSplineMatchesMathematicalPrimitive)
{
  const std::array<double, 5> knots = {
    -1.2, -0.3, 0.4, 1.5, 2.3
  };
  const std::array<double, 5> parameters = {
    0.8, 2.1, -0.7, 1.4, 3.0
  };
  constexpr int anchorIndex = 2;
  constexpr double anchorValue = 0.6;
  constexpr double anchorSlope = -0.35;
  SDM::ElasticModel1DIntegratedLinearCurvatureSpline model(
    knots, anchorIndex, anchorValue, anchorSlope);
  SDM::IntegratedLinearCurvatureSpline spline(
    knots, anchorIndex, anchorValue, anchorSlope);

  EXPECT_EQ(model.getNumParameters(), 5);
  for (double x : { -2.0, -0.8, 0.4, 0.9, 1.9, 3.0 }) {
    EXPECT_DOUBLE_EQ(
      model.compute_psi(parameters, x),
      spline.y(parameters, x));
    EXPECT_DOUBLE_EQ(
      model.compute_dpsi_dx(parameters, x),
      spline.dy_dx(parameters, x));
    EXPECT_DOUBLE_EQ(
      model.compute_d2psi_dx2(parameters, x),
      spline.d2y_dx2(parameters, x));
    for (int i = 0; i < model.getNumParameters(); ++i) {
      EXPECT_DOUBLE_EQ(
        model.compute_dpsi_dparam(parameters, i, x),
        spline.dy_dcurvature(i, x));
      EXPECT_DOUBLE_EQ(
        model.compute_d2psi_dx_dparam(parameters, i, x),
        spline.d2y_dx_dcurvature(i, x));
      EXPECT_DOUBLE_EQ(
        model.compute_d2psi_dparam2(parameters, i, 4 - i, x),
        0.0);
    }
  }
}

TEST(ElasticModel1D, IntegratedCurvatureSplineDerivativesMatchFiniteDifferences)
{
  const std::array<double, 5> knots = {
    -1.4, -0.2, 0.5, 1.2, 2.6
  };
  const std::array<double, 5> parameters = {
    1.7, -0.4, 2.3, 0.6, 3.1
  };
  SDM::ElasticModel1DIntegratedLinearCurvatureSpline model(
    knots, 2, 0.9, -0.45);
  const std::array<double, 5> queries = {
    -1.9, -0.8, 0.1, 0.9, 3.0
  };
  constexpr double h = 1e-6;

  for (double x : queries) {
    EXPECT_NEAR(
      model.compute_dpsi_dx(parameters, x),
      (model.compute_psi(parameters, x + h) -
        model.compute_psi(parameters, x - h)) /
        (2.0 * h),
      2e-8);
    EXPECT_NEAR(
      model.compute_d2psi_dx2(parameters, x),
      (model.compute_dpsi_dx(parameters, x + h) -
        model.compute_dpsi_dx(parameters, x - h)) /
        (2.0 * h),
      2e-8);

    for (int i = 0; i < model.getNumParameters(); ++i) {
      auto plus = parameters;
      auto minus = parameters;
      plus[static_cast<std::size_t>(i)] += h;
      minus[static_cast<std::size_t>(i)] -= h;

      EXPECT_NEAR(
        model.compute_dpsi_dparam(parameters, i, x),
        (model.compute_psi(plus, x) -
          model.compute_psi(minus, x)) /
          (2.0 * h),
        2e-8);
      EXPECT_NEAR(
        model.compute_d2psi_dx_dparam(parameters, i, x),
        (model.compute_dpsi_dx(plus, x) -
          model.compute_dpsi_dx(minus, x)) /
          (2.0 * h),
        2e-8);
      EXPECT_NEAR(
        model.compute_d2psi_dx_dparam(parameters, i, x),
        (model.compute_dpsi_dparam(parameters, i, x + h) -
          model.compute_dpsi_dparam(parameters, i, x - h)) /
          (2.0 * h),
        2e-8);
    }
  }
}

TEST(ElasticModel1D, IntegratedCurvatureSplineRejectsInvalidInputs)
{
  const std::array<double, 3> knots = { 0.2, 1.0, 1.8 };
  const std::array<double, 3> parameters = { 0.5, 1.0, 1.5 };
  SDM::ElasticModel1DIntegratedLinearCurvatureSpline model(
    knots, 1, 0.0, 0.0);

  EXPECT_THROW(
    model.compute_psi(
      std::array<double, 2>{ 0.5, 1.0 }, 1.0),
    std::invalid_argument);
  EXPECT_THROW(
    model.compute_dpsi_dx(
      std::array<double, 3>{
        0.5, std::numeric_limits<double>::quiet_NaN(), 1.5 },
      1.0),
    std::invalid_argument);
  EXPECT_THROW(
    model.compute_d2psi_dx2(
      parameters, std::numeric_limits<double>::infinity()),
    std::invalid_argument);
  EXPECT_THROW(
    model.compute_dpsi_dparam(parameters, -1, 1.0),
    std::out_of_range);
  EXPECT_THROW(
    model.compute_d2psi_dx_dparam(parameters, 3, 1.0),
    std::out_of_range);
  EXPECT_THROW(
    model.compute_d2psi_dparam2(parameters, 0, 3, 1.0),
    std::out_of_range);
}

TEST(ElasticModel1D, IntegratedCurvatureSplineRandomizedFiniteDifferences)
{
  std::mt19937 randomEngine(0x51A17E);
  std::uniform_int_distribution<int> knotCountDistribution(2, 8);
  std::uniform_real_distribution<double> startDistribution(-2.0, 0.0);
  std::uniform_real_distribution<double> spacingDistribution(0.2, 1.4);
  std::uniform_real_distribution<double> valueDistribution(-3.0, 3.0);
  std::uniform_real_distribution<double> unitDistribution(0.0, 1.0);
  constexpr double h = 1e-6;

  for (int sample = 0; sample < 60; ++sample) {
    const int knotCount = knotCountDistribution(randomEngine);
    std::vector<double> knots(
      static_cast<std::size_t>(knotCount));
    knots[0] = startDistribution(randomEngine);
    for (int i = 1; i < knotCount; ++i) {
      knots[static_cast<std::size_t>(i)] =
        knots[static_cast<std::size_t>(i - 1)] +
        spacingDistribution(randomEngine);
    }

    std::vector<double> parameters(
      static_cast<std::size_t>(knotCount));
    for (double &parameter : parameters)
      parameter = valueDistribution(randomEngine);

    std::uniform_int_distribution<int> anchorDistribution(
      0, knotCount - 1);
    const int anchorIndex = anchorDistribution(randomEngine);
    SDM::ElasticModel1DIntegratedLinearCurvatureSpline model(
      knots,
      anchorIndex,
      valueDistribution(randomEngine),
      valueDistribution(randomEngine));

    const double width = knots.back() - knots.front();
    for (int query = 0; query < 8; ++query) {
      const double x = knots.front() - 0.35 * width +
        1.7 * width * unitDistribution(randomEngine);

      const double first = model.compute_dpsi_dx(parameters, x);
      const double second =
        model.compute_d2psi_dx2(parameters, x);
      EXPECT_NEAR(
        first,
        (model.compute_psi(parameters, x + h) -
          model.compute_psi(parameters, x - h)) /
          (2.0 * h),
        2e-7 * (1.0 + std::abs(first)));
      EXPECT_NEAR(
        second,
        (model.compute_dpsi_dx(parameters, x + h) -
          model.compute_dpsi_dx(parameters, x - h)) /
          (2.0 * h),
        2e-7 * (1.0 + std::abs(second)));

      for (int i = 0; i < knotCount; ++i) {
        auto plus = parameters;
        auto minus = parameters;
        plus[static_cast<std::size_t>(i)] += h;
        minus[static_cast<std::size_t>(i)] -= h;

        const double parameterDerivative =
          model.compute_dpsi_dparam(parameters, i, x);
        const double mixedDerivative =
          model.compute_d2psi_dx_dparam(parameters, i, x);
        EXPECT_NEAR(
          parameterDerivative,
          (model.compute_psi(plus, x) -
            model.compute_psi(minus, x)) /
            (2.0 * h),
          2e-7 * (1.0 + std::abs(parameterDerivative)));
        EXPECT_NEAR(
          mixedDerivative,
          (model.compute_dpsi_dx(plus, x) -
            model.compute_dpsi_dx(minus, x)) /
            (2.0 * h),
          2e-7 * (1.0 + std::abs(mixedDerivative)));
        EXPECT_NEAR(
          mixedDerivative,
          (model.compute_dpsi_dparam(
             parameters, i, x + h) -
            model.compute_dpsi_dparam(
              parameters, i, x - h)) /
            (2.0 * h),
          2e-7 * (1.0 + std::abs(mixedDerivative)));
      }
    }
  }
}

TEST(ElasticModel1D, IntegratedCurvatureSplineParameterHessianIsZero)
{
  const std::array<double, 5> knots = {
    -1.0, -0.1, 0.6, 1.7, 2.4
  };
  const std::array<double, 5> parameters = {
    1.3, -0.8, 2.1, 0.4, 2.7
  };
  SDM::ElasticModel1DIntegratedLinearCurvatureSpline model(
    knots, 2, 0.7, -0.2);
  constexpr double x = 1.1;
  constexpr double h = 1e-6;

  for (int i = 0; i < model.getNumParameters(); ++i) {
    for (int j = 0; j < model.getNumParameters(); ++j) {
      auto plus = parameters;
      auto minus = parameters;
      plus[static_cast<std::size_t>(j)] += h;
      minus[static_cast<std::size_t>(j)] -= h;

      EXPECT_DOUBLE_EQ(
        model.compute_d2psi_dparam2(parameters, i, j, x),
        0.0);
      EXPECT_DOUBLE_EQ(
        (model.compute_dpsi_dparam(plus, i, x) -
          model.compute_dpsi_dparam(minus, i, x)) /
          (2.0 * h),
        0.0);
    }
  }
}

TEST(ElasticModel1D, IntegratedCurvatureSplineSupportsEndpointAnchors)
{
  const std::array<double, 4> knots = {
    -0.8, 0.1, 1.3, 2.0
  };
  const std::array<double, 4> parameters = {
    2.4, -0.6, 1.7, 3.2
  };
  constexpr double anchorValue = 0.85;
  constexpr double anchorSlope = -0.4;

  for (int anchorIndex : { 0, 3 }) {
    SDM::ElasticModel1DIntegratedLinearCurvatureSpline model(
      knots, anchorIndex, anchorValue, anchorSlope);
    const double anchorX =
      knots[static_cast<std::size_t>(anchorIndex)];

    EXPECT_DOUBLE_EQ(
      model.compute_psi(parameters, anchorX),
      anchorValue);
    EXPECT_DOUBLE_EQ(
      model.compute_dpsi_dx(parameters, anchorX),
      anchorSlope);
    EXPECT_DOUBLE_EQ(
      model.compute_d2psi_dx2(parameters, anchorX),
      parameters[static_cast<std::size_t>(anchorIndex)]);
  }
}

TEST(ElasticModel1D, IntegratedCurvatureSplineRejectsInvalidConstruction)
{
  const std::array<double, 3> validKnots = { 0.2, 1.0, 1.8 };

  EXPECT_THROW(
    (SDM::ElasticModel1DIntegratedLinearCurvatureSpline(
      std::array<double, 1>{ 1.0 }, 0, 0.0, 0.0)),
    std::invalid_argument);
  EXPECT_THROW(
    (SDM::ElasticModel1DIntegratedLinearCurvatureSpline(
      std::array<double, 3>{ 0.2, 1.0, 1.0 },
      1,
      0.0,
      0.0)),
    std::invalid_argument);
  EXPECT_THROW(
    (SDM::ElasticModel1DIntegratedLinearCurvatureSpline(
      std::array<double, 3>{
        0.2, std::numeric_limits<double>::quiet_NaN(), 1.8 },
      1,
      0.0,
      0.0)),
    std::invalid_argument);
  EXPECT_THROW(
    (SDM::ElasticModel1DIntegratedLinearCurvatureSpline(
      validKnots, -1, 0.0, 0.0)),
    std::out_of_range);
  EXPECT_THROW(
    (SDM::ElasticModel1DIntegratedLinearCurvatureSpline(
      validKnots, 3, 0.0, 0.0)),
    std::out_of_range);
  EXPECT_THROW(
    (SDM::ElasticModel1DIntegratedLinearCurvatureSpline(
      validKnots,
      1,
      std::numeric_limits<double>::infinity(),
      0.0)),
    std::invalid_argument);
  EXPECT_THROW(
    (SDM::ElasticModel1DIntegratedLinearCurvatureSpline(
      validKnots,
      1,
      0.0,
      std::numeric_limits<double>::quiet_NaN())),
    std::invalid_argument);
}

TEST(ElasticModel1D, ModelsShareScalarBaseInterface)
{
  std::unique_ptr<SDM::ElasticModel1D> model =
    std::make_unique<SDM::ElasticModel1DQuadratic>(1.0);
  const std::array<double, 1> parameters = { 2.0 };
  EXPECT_NEAR(model->compute_dpsi_dx(parameters, 0.5), 1.0, 1e-12);

  model = std::make_unique<SDM::ElasticModel1DZero>();
  EXPECT_DOUBLE_EQ(
    model->compute_psi(std::span<const double>{}, 0.5),
    0.0);

  model = std::make_unique<SDM::ElasticModel1DLogSquared>();
  const std::array<double, 1> logSquaredParameters = { 2.0 };
  EXPECT_DOUBLE_EQ(
    model->compute_d2psi_dx2(logSquaredParameters, 1.0),
    2.0);

  const std::array<double, 3> knots = { 0.5, 1.0, 1.5 };
  const std::array<double, 3> curvatures = { 1.0, 2.0, 3.0 };
  model =
    std::make_unique<
      SDM::ElasticModel1DIntegratedLinearCurvatureSpline>(
      knots, 1, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(
    model->compute_d2psi_dx2(curvatures, 1.0),
    2.0);
}

TEST(ElasticModel1D, CubicSplineDerivativesMatchFiniteDifferences)
{
  SDM::ElasticModel1DCubicSpline model(1.25, 3, 0.0, 2.0);
  const std::array<double, 3> parameters = { 0.0, 1.0, 4.0 };
  constexpr double x = 1.2;
  constexpr double h = 1e-6;

  EXPECT_NEAR(
    model.compute_dpsi_dx(parameters, x),
    (model.compute_psi(parameters, x + h) - model.compute_psi(parameters, x - h)) / (2.0 * h),
    1e-7);
  EXPECT_NEAR(
    model.compute_d2psi_dx2(parameters, x),
    (model.compute_dpsi_dx(parameters, x + h) - model.compute_dpsi_dx(parameters, x - h)) / (2.0 * h),
    1e-6);

  for (int i = 0; i < model.getNumParameters(); ++i) {
    auto plus = parameters;
    auto minus = parameters;
    plus[i] += h;
    minus[i] -= h;

    EXPECT_NEAR(
      model.compute_dpsi_dparam(parameters, i, x),
      (model.compute_psi(plus, x) - model.compute_psi(minus, x)) / (2.0 * h),
      1e-7);
    EXPECT_NEAR(
      model.compute_d2psi_dx_dparam(parameters, i, x),
      (model.compute_dpsi_dparam(parameters, i, x + h) -
        model.compute_dpsi_dparam(parameters, i, x - h)) /
        (2.0 * h),
      1e-6);
  }

  EXPECT_NEAR(model.compute_d2psi_dparam2(parameters, 0, 2, x), 0.0, 1e-15);
  EXPECT_THROW(model.compute_psi(std::array<double, 2>{ 0.0, 1.0 }, x), std::invalid_argument);
  EXPECT_THROW(model.compute_dpsi_dparam(parameters, 3, x), std::out_of_range);
}
}  // namespace
