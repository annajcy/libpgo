#include <gtest/gtest.h>

#include "material/elastic/deformationGradient/spectral/elasticModel3DSystematicPoking.h"
#include "material/elastic/elasticModel3DNeoHookean.h"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <span>
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

  void include(
    const SDM::ElasticModel3DSystematicPoking &systematic,
    std::span<const double> systematicParameters,
    const SDM::ElasticModel3DNeoHookean &neoHookean,
    const ES::M3d &F,
    const std::string &label)
  {
    const SDM::SpectralState state = stateFromF(F);
    const double psiError =
      std::abs(
        systematic.compute_psi(systematicParameters, state) -
        neoHookean.compute_psi({}, state)) /
      youngsModulus;
    const double stressError =
      (systematic.compute_P(systematicParameters, state) -
        neoHookean.compute_P({}, state))
        .norm() /
      youngsModulus;
    const double tangentError =
      (systematic.compute_dPdF(systematicParameters, state) -
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
