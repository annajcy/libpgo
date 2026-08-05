#include <gtest/gtest.h>

#include "material/elastic/deformationGradient/spectral/elasticModel3DSystematicPoking.h"
#include "material/elastic/deformationGradient/spectral/elasticModel3DValanisLandel.h"
#include "material/elastic/elasticModel1DFixedParameters.h"
#include "material/elastic/elasticModel1DIntegratedLinearCurvatureSpline.h"
#include "material/elastic/elasticModel1DScaled.h"
#include "material/elastic/elasticModel1DZero.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <span>
#include <stdexcept>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
namespace SDM = pgo::SolidDeformationModel;

struct SpectralData
{
  ES::M3d F;
  ES::M3d U;
  ES::M3d V;
  ES::V3d stretches;
};

SpectralData makeSpectralData(const ES::V3d &stretches)
{
  const ES::M3d U = Eigen::AngleAxisd(
    0.43,
    ES::V3d(1.0, 2.0, -1.0).normalized())
                      .toRotationMatrix();
  const ES::M3d V = Eigen::AngleAxisd(
    -0.67,
    ES::V3d(-2.0, 1.0, 3.0).normalized())
                      .toRotationMatrix();
  return {
    U * stretches.asDiagonal() * V.transpose(),
    U,
    V,
    stretches
  };
}

SpectralData decomposePositiveF(const ES::M3d &F)
{
  const Eigen::JacobiSVD<ES::M3d> svd(
    F, Eigen::ComputeFullU | Eigen::ComputeFullV);
  return {
    F,
    svd.matrixU(),
    svd.matrixV(),
    svd.singularValues()
  };
}

SDM::SpectralState toState(const SpectralData &data)
{
  SDM::SpectralState state;
  state.F = data.F;
  state.U = data.U;
  state.V = data.V;
  state.stretches = data.stretches;
  return state;
}

SDM::SpectralState identityState()
{
  return {};
}

double evaluatePsi(
  const SDM::ElasticModel3DSystematicPoking &model,
  std::span<const double> parameters,
  const ES::M3d &F)
{
  return model.compute_psi(
    parameters, toState(decomposePositiveF(F)));
}

ES::M3d evaluateP(
  const SDM::ElasticModel3DSystematicPoking &model,
  std::span<const double> parameters,
  const ES::M3d &F)
{
  return model.compute_P(
    parameters, toState(decomposePositiveF(F)));
}

ES::M9d evaluateTangent(
  const SDM::ElasticModel3DSystematicPoking &model,
  std::span<const double> parameters,
  const ES::M3d &F)
{
  return model.compute_dPdF(
    parameters, toState(decomposePositiveF(F)));
}

template<class DerivedA, class DerivedB>
void expectRelativeNear(
  const Eigen::MatrixBase<DerivedA> &a,
  const Eigen::MatrixBase<DerivedB> &b,
  double tolerance)
{
  const double scale =
    std::max({ 1.0, a.norm(), b.norm() });
  EXPECT_LT((a - b).norm() / scale, tolerance);
}

std::shared_ptr<const SDM::ElasticModel1D>
makePaperVolumeModel(
  std::span<const double> volumeKnots,
  int volumeRestKnotIndex)
{
  auto spline = std::make_shared<
    SDM::ElasticModel1DIntegratedLinearCurvatureSpline>(
    volumeKnots,
    volumeRestKnotIndex,
    0.0,
    0.0);
  auto fixedShape =
    std::make_shared<SDM::ElasticModel1DFixedParameters>(
      spline,
      SDM::sampleLogSquaredVolumeCurvatures(
        volumeKnots, 1.0));
  return std::make_shared<SDM::ElasticModel1DScaled>(
    fixedShape);
}

TEST(ElasticModel3DSystematicPoking, SamplesPaperVolumeCurvatureShape)
{
  const std::array<double, 7> volumeKnots = {
    std::exp(-1.0),
    std::exp(-2.0 / 3.0),
    std::exp(-1.0 / 3.0),
    1.0,
    std::exp(1.0 / 3.0),
    std::exp(2.0 / 3.0),
    std::exp(1.0)
  };
  constexpr double lambda = 3.2;
  const std::vector<double> curvatures =
    SDM::sampleLogSquaredVolumeCurvatures(
      volumeKnots, lambda);
  const std::vector<double> unitCurvatures =
    SDM::sampleLogSquaredVolumeCurvatures(
      volumeKnots, 1.0);

  ASSERT_EQ(curvatures.size(), volumeKnots.size());
  for (std::size_t i = 0; i < volumeKnots.size(); ++i) {
    const double J = volumeKnots[i];
    const double expected =
      lambda * (1.0 - std::log(J)) / (J * J);
    EXPECT_NEAR(curvatures[i], expected, 2e-14);
    EXPECT_NEAR(
      curvatures[i],
      lambda * unitCurvatures[i],
      2e-14);
  }
  EXPECT_DOUBLE_EQ(curvatures.back(), 0.0);
}

TEST(ElasticModel3DSystematicPoking, MatchesExplicitPaperComposition)
{
  const std::array<double, 5> stretchKnots = {
    0.45, 0.75, 1.0, 1.35, 1.8
  };
  const std::array<double, 7> volumeKnots = {
    std::exp(-1.0),
    std::exp(-2.0 / 3.0),
    std::exp(-1.0 / 3.0),
    1.0,
    std::exp(1.0 / 3.0),
    std::exp(2.0 / 3.0),
    std::exp(1.0)
  };
  const std::array<double, 6> parameters = {
    0.8, 1.2, 2.0, 1.5, 2.7,
    3.4
  };
  SDM::ElasticModel3DSystematicPoking systematic(
    stretchKnots, 2, volumeKnots, 3);
  SDM::ElasticModel3DValanisLandel explicitComposition(
    std::make_shared<
      SDM::ElasticModel1DIntegratedLinearCurvatureSpline>(
      stretchKnots, 2, 0.0, 0.0),
    std::make_shared<SDM::ElasticModel1DZero>(),
    makePaperVolumeModel(volumeKnots, 3));
  const SDM::SpectralState state =
    toState(makeSpectralData(
      ES::V3d(0.68, 1.17, 1.62)));

  EXPECT_EQ(systematic.getNumParameters(), 6);
  EXPECT_NEAR(
    systematic.compute_psi(parameters, state),
    explicitComposition.compute_psi(parameters, state),
    1e-14);
  EXPECT_TRUE(
    systematic.compute_P(parameters, state)
      .isApprox(
        explicitComposition.compute_P(parameters, state),
        1e-13));
  EXPECT_TRUE(
    systematic.compute_dPdF(parameters, state)
      .isApprox(
        explicitComposition.compute_dPdF(parameters, state),
        1e-12));
}

TEST(ElasticModel3DSystematicPoking, ParameterBlocksAddLinearly)
{
  const std::array<double, 5> stretchKnots = {
    0.5, 0.8, 1.0, 1.3, 1.7
  };
  const std::array<double, 5> volumeKnots = {
    std::exp(-1.0), std::exp(-0.5), 1.0,
    std::exp(0.5), std::exp(1.0)
  };
  SDM::ElasticModel3DSystematicPoking model(
    stretchKnots, 2, volumeKnots, 2);
  const std::array<double, 6> fOnly = {
    0.7, 1.1, 1.8, 2.3, 2.9,
    0.0
  };
  const std::array<double, 6> hOnly = {
    0.0, 0.0, 0.0, 0.0, 0.0,
    3.6
  };
  std::array<double, 6> combined;
  for (std::size_t i = 0; i < combined.size(); ++i)
    combined[i] = fOnly[i] + hOnly[i];

  const SDM::SpectralState state =
    toState(makeSpectralData(
      ES::V3d(0.72, 1.08, 1.55)));
  EXPECT_NEAR(
    model.compute_psi(combined, state),
    model.compute_psi(fOnly, state) +
      model.compute_psi(hOnly, state),
    1e-13);
  EXPECT_TRUE(
    model.compute_P(combined, state)
      .isApprox(
        model.compute_P(fOnly, state) +
          model.compute_P(hOnly, state),
        1e-12));
  EXPECT_TRUE(
    model.compute_dPdF(combined, state)
      .isApprox(
        model.compute_dPdF(fOnly, state) +
          model.compute_dPdF(hOnly, state),
        1e-11));
}

TEST(ElasticModel3DSystematicPoking, BatchParameterDerivativesCoverRepeatedStretches)
{
  const std::array<double, 5> stretchKnots = {
    0.5, 0.8, 1.0, 1.3, 1.7
  };
  const std::array<double, 5> volumeKnots = {
    std::exp(-1.0), std::exp(-0.5), 1.0,
    std::exp(0.5), std::exp(1.0)
  };
  std::array<double, 6> parameters = {
    0.7, 1.1, 1.8, 2.3, 2.9, 3.6
  };
  SDM::ElasticModel3DSystematicPoking model(
    stretchKnots, 2, volumeKnots, 2);
  const int parameterCount = model.getNumParameters();
  const std::array<ES::V3d, 3> testStretches = {
    ES::V3d(0.72, 1.08, 1.55),
    ES::V3d(1.25, 1.25, 0.72),
    ES::V3d(1.1, 1.1, 1.1)
  };

  for (const ES::V3d &stretches : testStretches) {
    const SDM::SpectralState state =
      toState(makeSpectralData(stretches));
    ES::VXd gradient(parameterCount);
    ES::MXd hessian(parameterCount, parameterCount);
    ES::MXd pJacobian(9, parameterCount);
    model.compute_dpsi_dparams(
      parameters, state, gradient);
    model.compute_d2psi_dparams2(
      parameters, state, hessian);
    model.compute_dP_dparams(
      parameters, state, pJacobian);

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
      expectRelativeNear(
        pJacobian.col(a),
        finiteDifferencePVector,
        2e-8);
    }
  }
}

TEST(ElasticModel3DSystematicPoking, BatchParameterApiRejectsWrongShapes)
{
  const std::array<double, 5> stretchKnots = {
    0.5, 0.8, 1.0, 1.3, 1.7
  };
  const std::array<double, 5> volumeKnots = {
    std::exp(-1.0), std::exp(-0.5), 1.0,
    std::exp(0.5), std::exp(1.0)
  };
  const std::array<double, 6> parameters = {
    0.7, 1.1, 1.8, 2.3, 2.9, 3.6
  };
  SDM::ElasticModel3DSystematicPoking model(
    stretchKnots, 2, volumeKnots, 2);
  const SDM::SpectralState state =
    toState(makeSpectralData(
      ES::V3d(0.72, 1.08, 1.55)));

  ES::VXd wrongGradient(5);
  ES::MXd wrongHessian(6, 5);
  ES::MXd wrongPJacobian(8, 6);
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
}

TEST(ElasticModel3DSystematicPoking, RestTangentMatchesLameLinearization)
{
  const std::array<double, 5> stretchKnots = {
    0.5, 0.8, 1.0, 1.3, 1.8
  };
  const std::array<double, 5> volumeKnots = {
    std::exp(-1.0), std::exp(-0.5), 1.0,
    std::exp(0.5), std::exp(1.0)
  };
  constexpr double mu = 2.4;
  constexpr double lambda = 3.7;
  const std::array<double, 6> parameters = {
    3.2, 4.1, 2.0 * mu, 5.3, 6.2,
    lambda
  };
  SDM::ElasticModel3DSystematicPoking model(
    stretchKnots, 2, volumeKnots, 2);
  const SDM::SpectralState rest = identityState();

  EXPECT_DOUBLE_EQ(
    model.compute_psi(parameters, rest), 0.0);
  EXPECT_TRUE(
    model.compute_P(parameters, rest).isZero(0.0));

  const ES::M9d tangent =
    model.compute_dPdF(parameters, rest);
  for (int dof = 0; dof < 9; ++dof) {
    ES::M3d dF = ES::M3d::Zero();
    dF.data()[dof] = 1.0;
    const ES::M3d expected =
      mu * (dF + dF.transpose()) +
      lambda * dF.trace() * ES::M3d::Identity();
    const ES::M3d actual =
      Eigen::Map<const ES::M3d>(
        tangent.col(dof).data());
    EXPECT_TRUE(actual.isApprox(expected, 1e-12));
  }
}

TEST(ElasticModel3DSystematicPoking, RandomFDerivativesMatchFiniteDifferences)
{
  const std::array<double, 7> stretchKnots = {
    0.35, 0.6, 0.8, 1.0, 1.25, 1.55, 2.0
  };
  const std::array<double, 7> volumeKnots = {
    std::exp(-1.0),
    std::exp(-2.0 / 3.0),
    std::exp(-1.0 / 3.0),
    1.0,
    std::exp(1.0 / 3.0),
    std::exp(2.0 / 3.0),
    std::exp(1.0)
  };
  const std::array<double, 8> parameters = {
    0.8, 1.1, 1.5, 2.0, 1.7, 2.4, 3.0,
    3.5
  };
  SDM::ElasticModel3DSystematicPoking model(
    stretchKnots, 3, volumeKnots, 3);
  std::mt19937 randomEngine(0x2023A);
  std::uniform_real_distribution<double> distribution(
    -1.0, 1.0);
  constexpr double step = 1e-6;

  for (int sample = 0; sample < 20; ++sample) {
    ES::M3d F = ES::M3d::Identity();
    for (double &entry : F.reshaped())
      entry += 0.22 * distribution(randomEngine);
    ASSERT_GT(F.determinant(), 0.15);

    const ES::M3d analyticP =
      evaluateP(model, parameters, F);
    const ES::M9d analyticTangent =
      evaluateTangent(model, parameters, F);
    ES::V9d finiteDifferenceP;
    ES::M9d finiteDifferenceTangent;

    for (int dof = 0; dof < 9; ++dof) {
      ES::M3d plus = F;
      ES::M3d minus = F;
      plus.data()[dof] += step;
      minus.data()[dof] -= step;

      finiteDifferenceP(dof) =
        (evaluatePsi(model, parameters, plus) -
          evaluatePsi(model, parameters, minus)) /
        (2.0 * step);
      const ES::M3d difference =
        (evaluateP(model, parameters, plus) -
          evaluateP(model, parameters, minus)) /
        (2.0 * step);
      finiteDifferenceTangent.col(dof) =
        Eigen::Map<const ES::V9d>(difference.data());
    }

    const ES::V9d analyticPVector =
      Eigen::Map<const ES::V9d>(analyticP.data());
    expectRelativeNear(
      analyticPVector, finiteDifferenceP, 3e-8);
    expectRelativeNear(
      analyticTangent, finiteDifferenceTangent, 8e-7);
  }
}

TEST(ElasticModel3DSystematicPoking, RepeatedStretchResultsAreStable)
{
  const std::array<double, 5> stretchKnots = {
    0.4, 0.7, 1.0, 1.4, 2.0
  };
  const std::array<double, 5> volumeKnots = {
    std::exp(-1.0), std::exp(-0.5), 1.0,
    std::exp(0.5), std::exp(1.0)
  };
  const std::array<double, 6> parameters = {
    0.8, 1.2, 2.1, 1.9, 2.8,
    3.7
  };
  SDM::ElasticModel3DSystematicPoking model(
    stretchKnots, 2, volumeKnots, 2);
  constexpr double step = 1e-6;

  const std::array<ES::V3d, 2> testStretches = {
    ES::V3d(1.25, 1.25, 0.72),
    ES::V3d(1.1, 1.1, 1.1)
  };
  for (const ES::V3d &stretches : testStretches) {
    const SpectralData data = makeSpectralData(stretches);
    SDM::SpectralState first = toState(data);
    SDM::SpectralState second = first;
    const ES::M3d rotation =
      stretches(0) == stretches(2) ?
      Eigen::AngleAxisd(
        0.39,
        ES::V3d(1.0, -2.0, 3.0).normalized())
        .toRotationMatrix() :
      Eigen::AngleAxisd(
        -0.52, ES::V3d::UnitZ())
        .toRotationMatrix();
    second.U = first.U * rotation;
    second.V = first.V * rotation;

    EXPECT_TRUE(
      model.compute_P(parameters, first)
        .isApprox(
          model.compute_P(parameters, second),
          1e-10));
    expectRelativeNear(
      model.compute_dPdF(parameters, first),
      model.compute_dPdF(parameters, second),
      1e-9);

    const ES::M3d F = data.F;
    const ES::M9d analyticTangent =
      evaluateTangent(model, parameters, F);
    ES::M9d finiteDifferenceTangent;
    for (int dof = 0; dof < 9; ++dof) {
      ES::M3d plus = F;
      ES::M3d minus = F;
      plus.data()[dof] += step;
      minus.data()[dof] -= step;
      const ES::M3d difference =
        (evaluateP(model, parameters, plus) -
          evaluateP(model, parameters, minus)) /
        (2.0 * step);
      finiteDifferenceTangent.col(dof) =
        Eigen::Map<const ES::V9d>(difference.data());
    }
    expectRelativeNear(
      analyticTangent, finiteDifferenceTangent, 2e-5);
  }
}

TEST(ElasticModel3DSystematicPoking, VolumeSplineRemainsFiniteNearCollapse)
{
  const std::array<double, 5> stretchKnots = {
    0.4, 0.7, 1.0, 1.4, 2.0
  };
  const std::array<double, 5> volumeKnots = {
    std::exp(-1.0), std::exp(-0.5), 1.0,
    std::exp(0.5), std::exp(1.0)
  };
  const std::array<double, 6> parameters = {
    0.8, 1.2, 2.1, 1.9, 2.8,
    3.7
  };
  SDM::ElasticModel3DSystematicPoking model(
    stretchKnots, 2, volumeKnots, 2);

  const std::array<ES::V3d, 3> nearCollapseStretches = {
    ES::V3d(1e-5, 0.7, 1.3),
    ES::V3d(1e-3, 1e-3, 1.0),
    ES::V3d::Constant(1e-3)
  };
  for (const ES::V3d &stretches :
    nearCollapseStretches) {
    const SDM::SpectralState state =
      toState(makeSpectralData(stretches));
    EXPECT_TRUE(
      std::isfinite(
        model.compute_psi(parameters, state)));
    EXPECT_TRUE(
      model.compute_P(parameters, state).allFinite());
    EXPECT_TRUE(
      model.compute_dPdF(parameters, state).allFinite());
  }
}

TEST(ElasticModel3DSystematicPoking, RejectsInvalidConstructionAndParameters)
{
  const std::array<double, 5> stretchKnots = {
    0.4, 0.7, 1.0, 1.4, 2.0
  };
  const std::array<double, 5> volumeKnots = {
    std::exp(-1.0), std::exp(-0.5), 1.0,
    std::exp(0.5), std::exp(1.0)
  };

  EXPECT_THROW(
    SDM::ElasticModel3DSystematicPoking(
      stretchKnots, -1, volumeKnots, 2),
    std::out_of_range);
  EXPECT_THROW(
    SDM::ElasticModel3DSystematicPoking(
      stretchKnots, 2, volumeKnots, 5),
    std::out_of_range);
  EXPECT_THROW(
    SDM::ElasticModel3DSystematicPoking(
      stretchKnots, 1, volumeKnots, 2),
    std::invalid_argument);
  EXPECT_THROW(
    SDM::ElasticModel3DSystematicPoking(
      stretchKnots, 2, volumeKnots, 1),
    std::invalid_argument);
  EXPECT_THROW(
    SDM::ElasticModel3DSystematicPoking(
      stretchKnots,
      2,
      std::array<double, 3>{ 0.0, 1.0, 2.0 },
      1),
    std::domain_error);

  SDM::ElasticModel3DSystematicPoking model(
    stretchKnots, 2, volumeKnots, 2);
  const SDM::SpectralState state =
    toState(makeSpectralData(
      ES::V3d(0.8, 1.1, 1.5)));
  EXPECT_THROW(
    model.compute_psi(
      std::array<double, 5>{
        1.0, 1.0, 1.0, 1.0, 1.0 },
      state),
    std::invalid_argument);
  EXPECT_THROW(
    model.compute_P(
      std::array<double, 6>{
        1.0,
        1.0,
        std::numeric_limits<double>::quiet_NaN(),
        1.0,
        1.0,
        2.0 },
      state),
    std::invalid_argument);

  EXPECT_THROW(
    SDM::sampleLogSquaredVolumeCurvatures(
      std::array<double, 2>{ -0.5, 1.0 }, 2.0),
    std::domain_error);
  EXPECT_THROW(
    SDM::sampleLogSquaredVolumeCurvatures(
      volumeKnots,
      std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
  EXPECT_THROW(
    SDM::sampleLogSquaredVolumeCurvatures(
      std::array<double, 1>{
        std::numeric_limits<double>::min() },
      2.0),
    std::overflow_error);
}

TEST(ElasticModel3DSystematicPoking, DefinitionCreatesMaterialFieldEvaluator)
{
  const std::array<double, 5> stretchKnots = {
    0.4, 0.7, 1.0, 1.4, 2.0
  };
  const std::array<double, 5> volumeKnots = {
    std::exp(-1.0), std::exp(-0.5), 1.0,
    std::exp(0.5), std::exp(1.0)
  };
  const std::array<double, 6> parameters = {
    0.8, 1.2, 2.1, 1.9, 2.8,
    3.7
  };
  SDM::SystematicPokingDefinition definition(
    stretchKnots, 2, volumeKnots, 2);

  EXPECT_EQ(definition.id(), "systematic_poking");
  EXPECT_EQ(definition.numFixedChannels(), 0);
  EXPECT_EQ(definition.numOptimizableChannels(), 6);
  EXPECT_TRUE(std::equal(
    definition.stretchKnots().begin(),
    definition.stretchKnots().end(),
    stretchKnots.begin(), stretchKnots.end()));
  EXPECT_EQ(definition.stretchRestKnotIndex(), 2);
  EXPECT_TRUE(std::equal(
    definition.volumeKnots().begin(),
    definition.volumeKnots().end(),
    volumeKnots.begin(), volumeKnots.end()));
  EXPECT_EQ(definition.volumeRestKnotIndex(), 2);

  std::unique_ptr<SDM::ElasticModel> created =
    definition.createModel({}, SDM::MaterialFrame::Identity());
  ASSERT_NE(created, nullptr);
  EXPECT_EQ(created->getNumParameters(), 6);
  auto *createdSystematic =
    dynamic_cast<SDM::ElasticModel3DSystematicPoking *>(
      created.get());
  ASSERT_NE(createdSystematic, nullptr);

  SDM::ElasticModel3DSystematicPoking direct(
    stretchKnots, 2, volumeKnots, 2);
  const SDM::SpectralState state =
    toState(makeSpectralData(
      ES::V3d(0.72, 1.08, 1.55)));
  EXPECT_NEAR(
    createdSystematic->compute_psi(parameters, state),
    direct.compute_psi(parameters, state),
    1e-14);
  EXPECT_TRUE(
    createdSystematic->compute_P(parameters, state).isApprox(
      direct.compute_P(parameters, state), 1e-13));

  EXPECT_THROW(
    definition.createModel(
      std::array<double, 1>{ 1.0 },
      SDM::MaterialFrame::Identity()),
    std::invalid_argument);
  EXPECT_THROW(
    SDM::SystematicPokingDefinition(
      stretchKnots, 1, volumeKnots, 2),
    std::invalid_argument);
}

}  // namespace
