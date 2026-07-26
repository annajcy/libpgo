#include <gtest/gtest.h>

#include "material/elastic/elasticModel1D.h"
#include "material/elastic/elasticModel1DCubicSpline.h"
#include "material/elastic/elasticModel1DQuadratic.h"
#include "naturalCubicSplineDerivatives.h"

#include <array>
#include <memory>
#include <stdexcept>

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
  const std::array<double, 1> parameters = {1.5};
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

  const std::array<double, 1> plus = {parameters[0] + h};
  const std::array<double, 1> minus = {parameters[0] - h};
  EXPECT_NEAR(
    model.compute_dpsi_dparam(parameters, 0, x),
    (model.compute_psi(plus, x) - model.compute_psi(minus, x)) / (2.0 * h),
    1e-8);
}

TEST(ElasticModel1D, ModelsShareScalarBaseInterface)
{
  std::unique_ptr<SDM::ElasticModel1D> model =
    std::make_unique<SDM::ElasticModel1DQuadratic>(1.0);
  const std::array<double, 1> parameters = {2.0};
  EXPECT_NEAR(model->compute_dpsi_dx(parameters, 0.5), 1.0, 1e-12);
}

TEST(ElasticModel1D, CubicSplineDerivativesMatchFiniteDifferences)
{
  SDM::ElasticModel1DCubicSpline model(1.25, 3, 0.0, 2.0);
  const std::array<double, 3> parameters = {0.0, 1.0, 4.0};
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
        model.compute_dpsi_dparam(parameters, i, x - h)) / (2.0 * h),
      1e-6);
  }

  EXPECT_NEAR(model.compute_d2psi_dparam2(parameters, 0, 2, x), 0.0, 1e-15);
  EXPECT_THROW(model.compute_psi(std::array<double, 2>{0.0, 1.0}, x), std::invalid_argument);
  EXPECT_THROW(model.compute_dpsi_dparam(parameters, 3, x), std::out_of_range);
}
}  // namespace
