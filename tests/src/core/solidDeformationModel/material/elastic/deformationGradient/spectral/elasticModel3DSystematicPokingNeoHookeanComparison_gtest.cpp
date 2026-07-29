#include <gtest/gtest.h>

#include "material/elastic/deformationGradient/spectral/elasticModel3DSystematicPoking.h"
#include "material/elastic/deformationGradient/spectral/elasticModel3DValanisLandel.h"
#include "material/elastic/elasticModel1DFixedParameters.h"
#include "material/elastic/elasticModel1DIntegratedLinearCurvatureSpline.h"
#include "material/elastic/elasticModel1DLogSquared.h"
#include "material/elastic/elasticModel1DZero.h"
#include "material/elastic/elasticModel3DNeoHookean.h"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
namespace SDM = pgo::SolidDeformationModel;

constexpr double youngsModulus = 2.0e5;
constexpr double poissonRatio = 0.35;
constexpr double mu =
  youngsModulus / (2.0 * (1.0 + poissonRatio));
constexpr double lambda =
  youngsModulus * poissonRatio /
  ((1.0 + poissonRatio) * (1.0 - 2.0 * poissonRatio));

const std::array<double, 9> stretchKnots = {
  0.5,
  std::exp(-3.0 * std::log(2.0) / 4.0),
  std::exp(-std::log(2.0) / 2.0),
  std::exp(-std::log(2.0) / 4.0),
  1.0,
  std::exp(std::log(2.0) / 4.0),
  std::exp(std::log(2.0) / 2.0),
  std::exp(3.0 * std::log(2.0) / 4.0),
  2.0
};

const std::array<double, 9> volumeKnots = {
  std::exp(-1.0),
  std::exp(-0.75),
  std::exp(-0.5),
  std::exp(-0.25),
  1.0,
  std::exp(0.25),
  std::exp(0.5),
  std::exp(0.75),
  std::exp(1.0)
};

std::array<double, 10> neoHookeanSplineParameters()
{
  std::array<double, 10> parameters;
  for (std::size_t i = 0; i < stretchKnots.size(); ++i) {
    const double stretch = stretchKnots[i];
    parameters[i] =
      mu * (1.0 + 1.0 / (stretch * stretch));
  }
  parameters.back() = lambda;
  return parameters;
}

SDM::SpectralState stateFromF(const ES::M3d &F)
{
  const Eigen::JacobiSVD<ES::M3d> svd(
    F, Eigen::ComputeFullU | Eigen::ComputeFullV);
  SDM::SpectralState state;
  state.F = F;
  state.U = svd.matrixU();
  state.V = svd.matrixV();
  state.stretches = svd.singularValues();
  return state;
}

struct ApproximationError
{
  double psi = 0.0;
  double stress = 0.0;
  double tangent = 0.0;
  std::string psiState;
  std::string stressState;
  std::string tangentState;

  template<class ApproximateModel>
  void include(
    const ApproximateModel &approximate,
    std::span<const double> approximateParameters,
    const SDM::ElasticModel3DNeoHookean &neoHookean,
    const ES::M3d &F,
    const std::string &label)
  {
    const SDM::SpectralState state = stateFromF(F);
    const double psiError =
      std::abs(
        approximate.compute_psi(approximateParameters, state) -
        neoHookean.compute_psi({}, state)) /
      youngsModulus;
    const double stressError =
      (approximate.compute_P(approximateParameters, state) -
        neoHookean.compute_P({}, state))
        .norm() /
      youngsModulus;
    const double tangentError =
      (approximate.compute_dPdF(approximateParameters, state) -
        neoHookean.compute_dPdF({}, state))
        .norm() /
      youngsModulus;

    if (psiError > psi) {
      psi = psiError;
      psiState = label;
    }
    if (stressError > stress) {
      stress = stressError;
      stressState = label;
    }
    if (tangentError > tangent) {
      tangent = tangentError;
      tangentState = label;
    }
  }
};

ES::M3d diagonal(double x, double y, double z)
{
  return ES::V3d(x, y, z).asDiagonal();
}

std::vector<double> logarithmicKnots(
  double minimum,
  double maximum,
  int count)
{
  std::vector<double> knots(static_cast<std::size_t>(count));
  const double logMinimum = std::log(minimum);
  const double logMaximum = std::log(maximum);
  for (int i = 0; i < count; ++i) {
    const double t =
      static_cast<double>(i) /
      static_cast<double>(count - 1);
    knots[static_cast<std::size_t>(i)] =
      std::exp((1.0 - t) * logMinimum + t * logMaximum);
  }
  return knots;
}

std::vector<double> stretchCurvatures(
  std::span<const double> knots)
{
  std::vector<double> curvatures;
  curvatures.reserve(knots.size());
  for (double stretch : knots) {
    curvatures.push_back(
      mu * (1.0 + 1.0 / (stretch * stretch)));
  }
  return curvatures;
}

template<class ApproximateModel>
ApproximationError fullDomainPathError(
  const ApproximateModel &approximate,
  std::span<const double> approximateParameters,
  const SDM::ElasticModel3DNeoHookean &neoHookean)
{
  ApproximationError error;
  constexpr int sampleCount = 81;
  for (int sample = 0; sample < sampleCount; ++sample) {
    const double t =
      static_cast<double>(sample) /
      static_cast<double>(sampleCount - 1);

    const double uniaxialStretch =
      0.52 + t * (1.95 - 0.52);
    error.include(
      approximate, approximateParameters, neoHookean,
      diagonal(uniaxialStretch, 1.0, 1.0),
      "uniaxial");

    const double biaxialStretch =
      std::exp(-0.45 + 0.9 * t);
    error.include(
      approximate, approximateParameters, neoHookean,
      diagonal(biaxialStretch, biaxialStretch, 1.0),
      "biaxial");

    const double volumetricStretch =
      std::exp(-0.3 + 0.6 * t);
    error.include(
      approximate, approximateParameters, neoHookean,
      volumetricStretch * ES::M3d::Identity(),
      "volumetric");

    ES::M3d shear = ES::M3d::Identity();
    shear(0, 1) = -1.0 + 2.0 * t;
    error.include(
      approximate, approximateParameters, neoHookean,
      shear,
      "simple shear");
  }
  return error;
}

class ExactNeoHookeanStretch final : public SDM::ElasticModel1D
{
public:
  int getNumParameters() const override
  {
    return 0;
  }

  double compute_psi(
    std::span<const double>, double stretch) const override
  {
    return 0.5 * mu * (stretch * stretch - 1.0) -
      mu * std::log(stretch);
  }

  double compute_dpsi_dx(
    std::span<const double>, double stretch) const override
  {
    return mu * (stretch - 1.0 / stretch);
  }

  double compute_d2psi_dx2(
    std::span<const double>, double stretch) const override
  {
    return mu * (1.0 + 1.0 / (stretch * stretch));
  }

  double compute_dpsi_dparam(
    std::span<const double>, int, double) const override
  {
    throw std::out_of_range(
      "ExactNeoHookeanStretch has no parameters");
  }

  double compute_d2psi_dx_dparam(
    std::span<const double>, int, double) const override
  {
    throw std::out_of_range(
      "ExactNeoHookeanStretch has no parameters");
  }

  double compute_d2psi_dparam2(
    std::span<const double>, int, int, double) const override
  {
    throw std::out_of_range(
      "ExactNeoHookeanStretch has no parameters");
  }
};

void expectStrictlyDecreasing(
  std::span<const ApproximationError> errors)
{
  for (std::size_t i = 1; i < errors.size(); ++i) {
    EXPECT_LT(errors[i].psi, errors[i - 1].psi);
    EXPECT_LT(errors[i].stress, errors[i - 1].stress);
    EXPECT_LT(errors[i].tangent, errors[i - 1].tangent);
  }
}

TEST(
  ElasticModel3DSystematicPokingNeoHookeanComparison,
  ConstitutiveSweepQuantifiesModerateAndFullDomainError)
{
  SDM::ElasticModel3DSystematicPoking systematic(
    stretchKnots, 4, volumeKnots, 4);
  const std::array<double, 10> parameters =
    neoHookeanSplineParameters();
  SDM::ElasticModel3DNeoHookean neoHookean(mu, lambda);
  ApproximationError fullDomainError;
  ApproximationError moderateError;

  constexpr int sampleCount = 41;
  for (int sample = 0; sample < sampleCount; ++sample) {
    const double t =
      static_cast<double>(sample) /
      static_cast<double>(sampleCount - 1);

    const double uniaxialStretch = 0.52 + t * (1.95 - 0.52);
    fullDomainError.include(
      systematic, parameters, neoHookean,
      diagonal(uniaxialStretch, 1.0, 1.0),
      "uniaxial");

    const double biaxialStretch =
      std::exp(-0.45 + 0.9 * t);
    fullDomainError.include(
      systematic, parameters, neoHookean,
      diagonal(biaxialStretch, biaxialStretch, 1.0),
      "biaxial");

    const double volumetricStretch =
      std::exp(-0.3 + 0.6 * t);
    fullDomainError.include(
      systematic, parameters, neoHookean,
      volumetricStretch * ES::M3d::Identity(),
      "volumetric");

    ES::M3d shear = ES::M3d::Identity();
    shear(0, 1) = -1.0 + 2.0 * t;
    fullDomainError.include(
      systematic, parameters, neoHookean,
      shear,
      "simple shear");
  }

  for (int sample = 0; sample < sampleCount; ++sample) {
    const double t =
      static_cast<double>(sample) /
      static_cast<double>(sampleCount - 1);
    const double uniaxialStretch =
      std::exp(-0.25 + 0.5 * t);
    moderateError.include(
      systematic, parameters, neoHookean,
      diagonal(uniaxialStretch, 1.0, 1.0),
      "moderate uniaxial");

    const double biaxialStretch =
      std::exp(-0.2 + 0.4 * t);
    moderateError.include(
      systematic, parameters, neoHookean,
      diagonal(biaxialStretch, biaxialStretch, 1.0),
      "moderate biaxial");

    const double volumetricStretch =
      std::exp(-0.15 + 0.3 * t);
    moderateError.include(
      systematic, parameters, neoHookean,
      volumetricStretch * ES::M3d::Identity(),
      "moderate volumetric");

    ES::M3d shear = ES::M3d::Identity();
    shear(0, 1) = -0.5 + t;
    moderateError.include(
      systematic, parameters, neoHookean,
      shear,
      "moderate simple shear");
  }

  std::mt19937 randomEngine(20260729);
  std::uniform_real_distribution<double> logStretch(-0.45, 0.45);
  std::uniform_real_distribution<double> angle(-1.0, 1.0);
  for (int sample = 0; sample < 80; ++sample) {
    ES::V3d logStretches;
    do {
      for (double &entry : logStretches)
        entry = logStretch(randomEngine);
    } while (std::abs(logStretches.sum()) > 0.9);

    const ES::M3d leftRotation = Eigen::AngleAxisd(
      angle(randomEngine),
      ES::V3d(1.0, 2.0, -1.0).normalized())
                                      .toRotationMatrix();
    const ES::M3d rightRotation = Eigen::AngleAxisd(
      angle(randomEngine),
      ES::V3d(-2.0, 1.0, 3.0).normalized())
                                       .toRotationMatrix();
    const ES::M3d F =
      leftRotation *
      logStretches.array().exp().matrix().asDiagonal() *
      rightRotation.transpose();
    fullDomainError.include(
      systematic, parameters, neoHookean, F, "random rotated");
  }

  std::cout
    << "Systematic Poking moderate-range approximation: "
    << "max psi/E=" << moderateError.psi
    << " (" << moderateError.psiState << "), "
    << "max |P|/E=" << moderateError.stress
    << " (" << moderateError.stressState << "), "
    << "max |dP/dF|/E=" << moderateError.tangent
    << " (" << moderateError.tangentState << ")\n"
    << "Systematic Poking full-knot-domain approximation: "
    << "max psi/E=" << fullDomainError.psi
    << " (" << fullDomainError.psiState << "), "
    << "max |P|/E=" << fullDomainError.stress
    << " (" << fullDomainError.stressState << "), "
    << "max |dP/dF|/E=" << fullDomainError.tangent
    << " (" << fullDomainError.tangentState << ")\n";

  EXPECT_LT(moderateError.psi, 6.0e-3);
  EXPECT_LT(moderateError.stress, 5.0e-2);
  EXPECT_LT(moderateError.tangent, 3.5e-1);
  EXPECT_LT(fullDomainError.psi, 3.0e-2);
  EXPECT_LT(fullDomainError.stress, 1.2e-1);
  EXPECT_LT(fullDomainError.tangent, 6.5e-1);
}

TEST(
  ElasticModel3DSystematicPokingNeoHookeanComparison,
  IncreasingNestedKnotCountsReducesApproximationError)
{
  constexpr std::array<int, 5> knotCounts = {
    5, 9, 17, 33, 65
  };
  SDM::ElasticModel3DNeoHookean neoHookean(mu, lambda);
  std::vector<ApproximationError> combinedErrors;
  std::vector<ApproximationError> stretchErrors;
  std::vector<ApproximationError> volumeErrors;

  std::cout
    << "knot_count"
    << " | combined psi/E P/E C/E"
    << " | stretch-only psi/E P/E C/E"
    << " | volume-only psi/E P/E C/E\n";

  for (int count : knotCounts) {
    const std::vector<double> currentStretchKnots =
      logarithmicKnots(0.5, 2.0, count);
    const std::vector<double> currentVolumeKnots =
      logarithmicKnots(std::exp(-1.0), std::exp(1.0), count);
    const std::vector<double> currentStretchCurvatures =
      stretchCurvatures(currentStretchKnots);

    std::vector<double> combinedParameters =
      currentStretchCurvatures;
    combinedParameters.push_back(lambda);
    SDM::ElasticModel3DSystematicPoking combined(
      currentStretchKnots,
      count / 2,
      currentVolumeKnots,
      count / 2);
    combinedErrors.push_back(
      fullDomainPathError(
        combined, combinedParameters, neoHookean));

    auto stretchSpline = std::make_shared<
      SDM::ElasticModel1DIntegratedLinearCurvatureSpline>(
      currentStretchKnots, count / 2, 0.0, 0.0);
    auto exactVolume =
      std::make_shared<SDM::ElasticModel1DFixedParameters>(
        std::make_shared<SDM::ElasticModel1DLogSquared>(),
        std::array<double, 1>{ lambda });
    SDM::ElasticModel3DValanisLandel stretchOnly(
      stretchSpline,
      std::make_shared<SDM::ElasticModel1DZero>(),
      exactVolume);
    stretchErrors.push_back(
      fullDomainPathError(
        stretchOnly, currentStretchCurvatures, neoHookean));

    auto volumeSpline = std::make_shared<
      SDM::ElasticModel1DIntegratedLinearCurvatureSpline>(
      currentVolumeKnots, count / 2, 0.0, 0.0);
    auto approximateVolume =
      std::make_shared<SDM::ElasticModel1DFixedParameters>(
        volumeSpline,
        SDM::sampleLogSquaredVolumeCurvatures(
          currentVolumeKnots, lambda));
    SDM::ElasticModel3DValanisLandel volumeOnly(
      std::make_shared<ExactNeoHookeanStretch>(),
      std::make_shared<SDM::ElasticModel1DZero>(),
      approximateVolume);
    volumeErrors.push_back(
      fullDomainPathError(
        volumeOnly, std::span<const double>{}, neoHookean));

    const ApproximationError &combinedError =
      combinedErrors.back();
    const ApproximationError &stretchError =
      stretchErrors.back();
    const ApproximationError &volumeError =
      volumeErrors.back();
    std::cout
      << count
      << " | "
      << combinedError.psi << " "
      << combinedError.stress << " "
      << combinedError.tangent
      << " | "
      << stretchError.psi << " "
      << stretchError.stress << " "
      << stretchError.tangent
      << " | "
      << volumeError.psi << " "
      << volumeError.stress << " "
      << volumeError.tangent
      << "\n";
  }

  expectStrictlyDecreasing(combinedErrors);
  expectStrictlyDecreasing(stretchErrors);
  expectStrictlyDecreasing(volumeErrors);

  for (std::size_t i = 0; i < knotCounts.size(); ++i) {
    EXPECT_GT(volumeErrors[i].psi, 10.0 * stretchErrors[i].psi);
    EXPECT_GT(
      volumeErrors[i].stress,
      10.0 * stretchErrors[i].stress);
    EXPECT_GT(
      volumeErrors[i].tangent,
      10.0 * stretchErrors[i].tangent);
  }

  EXPECT_LT(
    combinedErrors.back().psi,
    combinedErrors.front().psi / 100.0);
  EXPECT_LT(
    combinedErrors.back().stress,
    combinedErrors.front().stress / 100.0);
  EXPECT_LT(
    combinedErrors.back().tangent,
    combinedErrors.front().tangent / 100.0);
}

TEST(
  ElasticModel3DSystematicPokingNeoHookeanComparison,
  MaterialTangentIsContinuousAcrossSplineKnots)
{
  SDM::ElasticModel3DSystematicPoking systematic(
    stretchKnots, 4, volumeKnots, 4);
  const std::array<double, 10> parameters =
    neoHookeanSplineParameters();

  for (double knot : stretchKnots) {
    const double epsilon = 1.0e-7 * knot;
    const SDM::SpectralState left =
      stateFromF(diagonal(knot - epsilon, 1.13, 0.91));
    const SDM::SpectralState right =
      stateFromF(diagonal(knot + epsilon, 1.13, 0.91));
    EXPECT_LT(
      std::abs(
        systematic.compute_psi(parameters, right) -
        systematic.compute_psi(parameters, left)) /
        youngsModulus,
      2.0e-6);
    EXPECT_LT(
      (systematic.compute_P(parameters, right) -
        systematic.compute_P(parameters, left))
          .norm() /
        youngsModulus,
      2.0e-6);
    EXPECT_LT(
      (systematic.compute_dPdF(parameters, right) -
        systematic.compute_dPdF(parameters, left))
          .norm() /
        youngsModulus,
      2.0e-5);
  }

  for (double knot : volumeKnots) {
    const double epsilon = 1.0e-7 * knot;
    const SDM::SpectralState left =
      stateFromF(
        std::cbrt(knot - epsilon) * ES::M3d::Identity());
    const SDM::SpectralState right =
      stateFromF(
        std::cbrt(knot + epsilon) * ES::M3d::Identity());
    EXPECT_LT(
      std::abs(
        systematic.compute_psi(parameters, right) -
        systematic.compute_psi(parameters, left)) /
        youngsModulus,
      2.0e-6);
    EXPECT_LT(
      (systematic.compute_P(parameters, right) -
        systematic.compute_P(parameters, left))
          .norm() /
        youngsModulus,
      2.0e-6);
    EXPECT_LT(
      (systematic.compute_dPdF(parameters, right) -
        systematic.compute_dPdF(parameters, left))
          .norm() /
        youngsModulus,
      2.0e-5);
  }
}

TEST(
  ElasticModel3DSystematicPokingNeoHookeanComparison,
  ExtrapolationRemainsFiniteNearKnotDomain)
{
  SDM::ElasticModel3DSystematicPoking systematic(
    stretchKnots, 4, volumeKnots, 4);
  const std::array<double, 10> parameters =
    neoHookeanSplineParameters();
  const std::array<ES::M3d, 4> deformationGradients = {
    diagonal(0.45, 1.0, 1.0),
    diagonal(2.1, 1.0, 1.0),
    std::cbrt(0.9 * volumeKnots.front()) *
      ES::M3d::Identity(),
    std::cbrt(1.1 * volumeKnots.back()) *
      ES::M3d::Identity()
  };

  for (const ES::M3d &F : deformationGradients) {
    const SDM::SpectralState state = stateFromF(F);
    EXPECT_TRUE(
      std::isfinite(systematic.compute_psi(parameters, state)));
    EXPECT_TRUE(
      systematic.compute_P(parameters, state).allFinite());
    EXPECT_TRUE(
      systematic.compute_dPdF(parameters, state).allFinite());
  }
}

}  // namespace
