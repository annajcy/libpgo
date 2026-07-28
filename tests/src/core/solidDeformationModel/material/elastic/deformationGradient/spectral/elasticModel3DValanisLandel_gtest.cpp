#include <gtest/gtest.h>

#include "material/elastic/deformationGradient/spectral/elasticModel3DValanisLandel.h"
#include "material/elastic/elasticModel1DIntegratedLinearCurvatureSpline.h"
#include "material/elastic/elasticModel1DLogSquared.h"
#include "material/elastic/elasticModel1DQuadratic.h"
#include "material/elastic/elasticModel1DZero.h"

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <span>
#include <stdexcept>

namespace
{
namespace ES = pgo::EigenSupport;
namespace SDM = pgo::SolidDeformationModel;

class TestableValanisLandel final : public SDM::ElasticModel3DValanisLandel
{
public:
  using ElasticModel3DValanisLandel::
    ElasticModel3DValanisLandel;
  using ElasticModel3DValanisLandel::compute_d2psi_ds2;
  using ElasticModel3DValanisLandel::compute_dpsi_ds;
  using ElasticModel3DValanisLandel::compute_psi_s;
};

class ParameterCountOnlyModel final : public SDM::ElasticModel1D
{
public:
  explicit ParameterCountOnlyModel(int parameterCount):
    parameterCount_(parameterCount)
  {
  }

  int getNumParameters() const override { return parameterCount_; }

  double compute_psi(
    std::span<const double>, double) const override
  {
    return 0.0;
  }

  double compute_dpsi_dx(
    std::span<const double>, double) const override
  {
    return 0.0;
  }

  double compute_d2psi_dx2(
    std::span<const double>, double) const override
  {
    return 0.0;
  }

  double compute_dpsi_dparam(
    std::span<const double>, int, double) const override
  {
    return 0.0;
  }

  double compute_d2psi_dx_dparam(
    std::span<const double>, int, double) const override
  {
    return 0.0;
  }

  double compute_d2psi_dparam2(
    std::span<const double>, int, int, double) const override
  {
    return 0.0;
  }

private:
  int parameterCount_;
};

struct QuadraticValanisResult
{
  double energy;
  ES::V3d gradient;
  ES::M3d hessian;
};

QuadraticValanisResult evaluateQuadraticValanis(
  const ES::V3d &s,
  double fStiffness,
  double gStiffness,
  double hStiffness)
{
  QuadraticValanisResult result;
  result.energy = 0.5 * fStiffness * s.squaredNorm();
  result.gradient = fStiffness * s;
  result.hessian = fStiffness * ES::M3d::Identity();

  static constexpr std::array<std::array<int, 2>, 3> pairs = {
    std::array<int, 2>{ 0, 1 },
    std::array<int, 2>{ 1, 2 },
    std::array<int, 2>{ 2, 0 }
  };
  for (const auto &pair : pairs) {
    const int i = pair[0];
    const int j = pair[1];
    const double area = s(i) * s(j);
    result.energy += 0.5 * gStiffness * area * area;
    result.gradient(i) += gStiffness * area * s(j);
    result.gradient(j) += gStiffness * area * s(i);
    result.hessian(i, i) +=
      gStiffness * s(j) * s(j);
    result.hessian(j, j) +=
      gStiffness * s(i) * s(i);
    const double mixed = 2.0 * gStiffness * area;
    result.hessian(i, j) += mixed;
    result.hessian(j, i) += mixed;
  }

  const double J = s.prod();
  const ES::V3d dJds(
    s(1) * s(2),
    s(0) * s(2),
    s(0) * s(1));
  result.energy += 0.5 * hStiffness * J * J;
  result.gradient.noalias() += hStiffness * J * dJds;
  result.hessian.noalias() +=
    hStiffness * dJds * dJds.transpose();
  for (const auto &pair : pairs) {
    const int i = pair[0];
    const int j = pair[1];
    const int remaining = 3 - i - j;
    const double mixed = hStiffness * J * s(remaining);
    result.hessian(i, j) += mixed;
    result.hessian(j, i) += mixed;
  }
  return result;
}

SDM::SpectralState diagonalState(const ES::V3d &s)
{
  SDM::SpectralState state;
  state.U = ES::M3d::Identity();
  state.V = ES::M3d::Identity();
  state.stretches = s;
  state.F = s.asDiagonal();
  return state;
}

TEST(ElasticModel3DValanisLandel, QuadraticComponentsMatchClosedForm)
{
  constexpr double fCoefficient = 1.3;
  constexpr double gCoefficient = -0.8;
  constexpr double hCoefficient = 2.1;
  const std::array<double, 3> parameters = {
    1.7, -0.6, 0.9
  };
  const double fStiffness = fCoefficient * parameters[0];
  const double gStiffness = gCoefficient * parameters[1];
  const double hStiffness = hCoefficient * parameters[2];
  const ES::V3d s(0.7, 1.2, 1.8);

  TestableValanisLandel model(
    std::make_shared<SDM::ElasticModel1DQuadratic>(
      fCoefficient),
    std::make_shared<SDM::ElasticModel1DQuadratic>(
      gCoefficient),
    std::make_shared<SDM::ElasticModel1DQuadratic>(
      hCoefficient));
  const QuadraticValanisResult expected =
    evaluateQuadraticValanis(
      s, fStiffness, gStiffness, hStiffness);

  EXPECT_EQ(model.getNumParameters(), 3);
  EXPECT_NEAR(
    model.compute_psi_s(parameters, s),
    expected.energy,
    1e-13);
  EXPECT_TRUE(
    model.compute_dpsi_ds(parameters, s)
      .isApprox(expected.gradient, 1e-13));
  EXPECT_TRUE(
    model.compute_d2psi_ds2(parameters, s)
      .isApprox(expected.hessian, 1e-13));
}

TEST(ElasticModel3DValanisLandel, RandomStretchDerivativesMatchFiniteDifferences)
{
  TestableValanisLandel model(
    std::make_shared<SDM::ElasticModel1DQuadratic>(1.2),
    std::make_shared<SDM::ElasticModel1DQuadratic>(-0.7),
    std::make_shared<SDM::ElasticModel1DQuadratic>(0.9));
  std::mt19937 randomEngine(0x3D71A);
  std::uniform_real_distribution<double> stretchDistribution(
    0.35, 2.2);
  std::uniform_real_distribution<double> parameterDistribution(
    -2.0, 3.0);

  for (int sample = 0; sample < 200; ++sample) {
    ES::V3d s;
    for (int i = 0; i < 3; ++i)
      s(i) = stretchDistribution(randomEngine);
    const std::array<double, 3> parameters = {
      parameterDistribution(randomEngine),
      parameterDistribution(randomEngine),
      parameterDistribution(randomEngine)
    };

    const ES::V3d gradient =
      model.compute_dpsi_ds(parameters, s);
    const ES::M3d hessian =
      model.compute_d2psi_ds2(parameters, s);
    EXPECT_TRUE(hessian.isApprox(hessian.transpose(), 1e-14));

    for (int i = 0; i < 3; ++i) {
      const double step = 1e-6 * s(i);
      ES::V3d plus = s;
      ES::V3d minus = s;
      plus(i) += step;
      minus(i) -= step;

      const double finiteDifferenceGradient =
        (model.compute_psi_s(parameters, plus) -
          model.compute_psi_s(parameters, minus)) /
        (2.0 * step);
      EXPECT_NEAR(
        gradient(i),
        finiteDifferenceGradient,
        3e-7 * (1.0 + std::abs(gradient(i))));

      const ES::V3d finiteDifferenceHessianColumn =
        (model.compute_dpsi_ds(parameters, plus) -
          model.compute_dpsi_ds(parameters, minus)) /
        (2.0 * step);
      for (int j = 0; j < 3; ++j) {
        EXPECT_NEAR(
          hessian(j, i),
          finiteDifferenceHessianColumn(j),
          3e-7 * (1.0 + std::abs(hessian(j, i))));
      }
    }
  }
}

TEST(ElasticModel3DValanisLandel, SplitsDifferentParameterBlocksCorrectly)
{
  const std::array<double, 4> knots = {
    0.4, 0.8, 1.0, 1.7
  };
  auto f = std::make_shared<
    SDM::ElasticModel1DIntegratedLinearCurvatureSpline>(
    knots, 2, 0.0, 0.0);
  auto g =
    std::make_shared<SDM::ElasticModel1DQuadratic>(1.6);
  auto h = std::make_shared<SDM::ElasticModel1DLogSquared>();
  TestableValanisLandel model(f, g, h);
  const std::array<double, 6> parameters = {
    0.7, 1.1, 1.8, 2.4,
    -0.5,
    3.2
  };
  const std::span<const double> fParameters(parameters.data(), 4);
  const std::span<const double> gParameters(
    parameters.data() + 4, 1);
  const std::span<const double> hParameters(
    parameters.data() + 5, 1);
  const ES::V3d s(0.65, 1.15, 1.6);
  const double J = s.prod();

  double expectedEnergy = h->compute_psi(hParameters, J);
  for (int i = 0; i < 3; ++i)
    expectedEnergy += f->compute_psi(fParameters, s(i));
  expectedEnergy +=
    g->compute_psi(gParameters, s(0) * s(1));
  expectedEnergy +=
    g->compute_psi(gParameters, s(1) * s(2));
  expectedEnergy +=
    g->compute_psi(gParameters, s(2) * s(0));

  EXPECT_EQ(model.getNumParameters(), 6);
  EXPECT_NEAR(
    model.compute_psi_s(parameters, s),
    expectedEnergy,
    1e-13);

  const ES::V3d gradient =
    model.compute_dpsi_ds(parameters, s);
  const ES::M3d hessian =
    model.compute_d2psi_ds2(parameters, s);
  for (int i = 0; i < 3; ++i) {
    const double step = 1e-6 * s(i);
    ES::V3d plus = s;
    ES::V3d minus = s;
    plus(i) += step;
    minus(i) -= step;
    EXPECT_NEAR(
      gradient(i),
      (model.compute_psi_s(parameters, plus) -
        model.compute_psi_s(parameters, minus)) /
        (2.0 * step),
      2e-7 * (1.0 + std::abs(gradient(i))));
    const ES::V3d finiteDifferenceColumn =
      (model.compute_dpsi_ds(parameters, plus) -
        model.compute_dpsi_ds(parameters, minus)) /
      (2.0 * step);
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(
        hessian(j, i),
        finiteDifferenceColumn(j),
        2e-7 * (1.0 + std::abs(hessian(j, i))));
    }
  }
}

TEST(ElasticModel3DValanisLandel, BatchParameterDerivativesMatchFiniteDifferences)
{
  const std::array<double, 4> knots = {
    0.4, 0.8, 1.0, 1.7
  };
  TestableValanisLandel model(
    std::make_shared<
      SDM::ElasticModel1DIntegratedLinearCurvatureSpline>(
      knots, 2, 0.0, 0.0),
    std::make_shared<SDM::ElasticModel1DQuadratic>(1.6),
    std::make_shared<SDM::ElasticModel1DLogSquared>());
  std::array<double, 6> parameters = {
    0.7, 1.1, 1.8, 2.4, -0.5, 3.2
  };
  const SDM::SpectralState state =
    diagonalState(ES::V3d(0.65, 1.15, 1.6));
  const int parameterCount = model.getNumParameters();

  ES::VXd gradient(parameterCount);
  ES::MXd hessian(parameterCount, parameterCount);
  ES::MXd pJacobian(9, parameterCount);
  model.compute_dpsi_dparams(parameters, state, gradient);
  model.compute_d2psi_dparams2(parameters, state, hessian);
  model.compute_dP_dparams(parameters, state, pJacobian);

  EXPECT_TRUE(hessian.isApprox(hessian.transpose(), 1e-14));
  EXPECT_TRUE(hessian.isZero(0.0));

  for (int a = 0; a < parameterCount; ++a) {
    const double step =
      1e-6 * std::max(1.0, std::abs(parameters[a]));
    std::array<double, 6> plus = parameters;
    std::array<double, 6> minus = parameters;
    plus[a] += step;
    minus[a] -= step;

    const double finiteDifferenceGradient =
      (model.compute_psi(plus, state) -
        model.compute_psi(minus, state)) /
      (2.0 * step);
    EXPECT_NEAR(
      gradient(a),
      finiteDifferenceGradient,
      2e-8 * (1.0 + std::abs(gradient(a))));

    const ES::M3d finiteDifferenceP =
      (model.compute_P(plus, state) -
        model.compute_P(minus, state)) /
      (2.0 * step);
    const ES::V9d finiteDifferencePVector =
      Eigen::Map<const ES::V9d>(
        finiteDifferenceP.data());
    EXPECT_TRUE(
      pJacobian.col(a).isApprox(
        finiteDifferencePVector, 2e-8));

    ES::VXd plusGradient(parameterCount);
    ES::VXd minusGradient(parameterCount);
    model.compute_dpsi_dparams(
      plus, state, plusGradient);
    model.compute_dpsi_dparams(
      minus, state, minusGradient);
    const ES::VXd finiteDifferenceHessianColumn =
      (plusGradient - minusGradient) / (2.0 * step);
    EXPECT_TRUE(
      hessian.col(a).isApprox(
        finiteDifferenceHessianColumn, 2e-9));
  }
}

TEST(ElasticModel3DValanisLandel, BatchParameterApiRejectsWrongShapes)
{
  auto quadratic =
    std::make_shared<SDM::ElasticModel1DQuadratic>(1.0);
  TestableValanisLandel model(
    quadratic, quadratic, quadratic);
  const std::array<double, 3> parameters = {
    1.0, 2.0, 3.0
  };
  const SDM::SpectralState state =
    diagonalState(ES::V3d(0.8, 1.1, 1.5));

  ES::VXd wrongGradient(2);
  ES::MXd wrongHessian(3, 2);
  ES::MXd wrongPJacobian(8, 3);
  EXPECT_THROW(
    model.compute_dpsi_dparams(
      parameters, state, wrongGradient),
    std::invalid_argument);
  EXPECT_THROW(
    model.compute_d2psi_dparams2(
      parameters, state, wrongHessian),
    std::invalid_argument);
  EXPECT_THROW(
    model.compute_dP_dparams(
      parameters, state, wrongPJacobian),
    std::invalid_argument);

  ES::VXd gradient(3);
  EXPECT_THROW(
    model.compute_dpsi_dparams(
      std::array<double, 2>{ 1.0, 2.0 },
      state,
      gradient),
    std::invalid_argument);
  EXPECT_THROW(
    model.compute_dpsi_dparams(
      parameters,
      diagonalState(ES::V3d(1.0, 0.0, 1.0)),
      gradient),
    std::domain_error);
}

TEST(ElasticModel3DValanisLandel, SupportsZeroDimensionalComponents)
{
  auto zero = std::make_shared<SDM::ElasticModel1DZero>();
  TestableValanisLandel model(zero, zero, zero);
  const ES::V3d s(0.6, 1.0, 1.9);

  EXPECT_EQ(model.getNumParameters(), 0);
  EXPECT_DOUBLE_EQ(model.compute_psi_s({}, s), 0.0);
  EXPECT_TRUE(
    model.compute_dpsi_ds({}, s).isZero(0.0));
  EXPECT_TRUE(
    model.compute_d2psi_ds2({}, s).isZero(0.0));

  const SDM::SpectralState state = diagonalState(s);
  EXPECT_DOUBLE_EQ(model.compute_psi({}, state), 0.0);
  EXPECT_TRUE(model.compute_P({}, state).isZero(0.0));
  EXPECT_TRUE(model.compute_dPdF({}, state).isZero(0.0));
}

TEST(ElasticModel3DValanisLandel, IsInvariantUnderStretchPermutation)
{
  TestableValanisLandel model(
    std::make_shared<SDM::ElasticModel1DQuadratic>(1.1),
    std::make_shared<SDM::ElasticModel1DQuadratic>(0.8),
    std::make_shared<SDM::ElasticModel1DLogSquared>());
  const std::array<double, 3> parameters = {
    1.4, -0.6, 2.3
  };
  const ES::V3d s(0.55, 1.25, 1.9);
  ES::M3d permutation = ES::M3d::Zero();
  permutation(0, 2) = 1.0;
  permutation(1, 0) = 1.0;
  permutation(2, 1) = 1.0;
  const ES::V3d permuted = permutation * s;

  EXPECT_NEAR(
    model.compute_psi_s(parameters, permuted),
    model.compute_psi_s(parameters, s),
    1e-13);
  EXPECT_TRUE(
    model.compute_dpsi_ds(parameters, permuted)
      .isApprox(
        permutation *
          model.compute_dpsi_ds(parameters, s),
        1e-13));
  EXPECT_TRUE(
    model.compute_d2psi_ds2(parameters, permuted)
      .isApprox(
        permutation *
          model.compute_d2psi_ds2(parameters, s) *
          permutation.transpose(),
        1e-13));
}

TEST(ElasticModel3DValanisLandel, RejectsInvalidModelsAndParameterCounts)
{
  auto quadratic =
    std::make_shared<SDM::ElasticModel1DQuadratic>(1.0);
  EXPECT_THROW(
    TestableValanisLandel(nullptr, quadratic, quadratic),
    std::invalid_argument);
  EXPECT_THROW(
    TestableValanisLandel(quadratic, nullptr, quadratic),
    std::invalid_argument);
  EXPECT_THROW(
    TestableValanisLandel(quadratic, quadratic, nullptr),
    std::invalid_argument);

  auto negativeCount =
    std::make_shared<ParameterCountOnlyModel>(-1);
  EXPECT_THROW(
    TestableValanisLandel(
      negativeCount, quadratic, quadratic),
    std::invalid_argument);

  auto hugeCount = std::make_shared<ParameterCountOnlyModel>(
    std::numeric_limits<int>::max());
  auto zeroCount =
    std::make_shared<ParameterCountOnlyModel>(0);
  EXPECT_THROW(
    TestableValanisLandel(
      hugeCount, hugeCount, zeroCount),
    std::overflow_error);

  TestableValanisLandel model(
    quadratic, quadratic, quadratic);
  const ES::V3d s(0.8, 1.1, 1.5);
  EXPECT_THROW(
    model.compute_psi_s(
      std::array<double, 2>{ 1.0, 2.0 }, s),
    std::invalid_argument);
  EXPECT_THROW(
    model.compute_dpsi_ds(
      std::array<double, 4>{ 1.0, 2.0, 3.0, 4.0 },
      s),
    std::invalid_argument);
  EXPECT_THROW(
    model.compute_d2psi_ds2(
      std::array<double, 3>{
        1.0,
        std::numeric_limits<double>::quiet_NaN(),
        3.0 },
      s),
    std::invalid_argument);
}

TEST(ElasticModel3DValanisLandel, PublicApiRejectsInvalidStretches)
{
  auto quadratic =
    std::make_shared<SDM::ElasticModel1DQuadratic>(1.0);
  TestableValanisLandel model(
    quadratic, quadratic, quadratic);
  const std::array<double, 3> parameters = {
    1.0, 1.0, 1.0
  };

  EXPECT_THROW(
    model.compute_psi(
      parameters, diagonalState(ES::V3d(1.0, 0.0, 1.0))),
    std::domain_error);
  EXPECT_THROW(
    model.compute_P(
      parameters, diagonalState(ES::V3d(1.0, -0.2, 1.0))),
    std::domain_error);
  EXPECT_THROW(
    model.compute_dPdF(
      parameters,
      diagonalState(ES::V3d(
        1.0,
        std::numeric_limits<double>::infinity(),
        1.0))),
    std::invalid_argument);
}

}  // namespace
