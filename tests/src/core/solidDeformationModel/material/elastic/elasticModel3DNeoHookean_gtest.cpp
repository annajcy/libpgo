#include <gtest/gtest.h>

#include "material/elastic/elasticModel3DNeoHookean.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <random>

namespace
{
namespace ES = pgo::EigenSupport;
namespace SDM = pgo::SolidDeformationModel;

SDM::SpectralState stateFromF(const ES::M3d &F)
{
  SDM::SpectralState state;
  state.F = F;
  return state;
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

TEST(ElasticModel3DNeoHookean, RestStateMatchesLinearElasticity)
{
  constexpr double mu = 2.3;
  constexpr double lambda = 4.7;
  SDM::ElasticModel3DNeoHookean model(mu, lambda);
  const SDM::SpectralState state =
    stateFromF(ES::M3d::Identity());

  EXPECT_NEAR(model.compute_psi({}, state), 0.0, 1e-14);
  EXPECT_TRUE(
    model.compute_P({}, state).isZero(1e-14));

  ES::M9d expected;
  for (int column = 0; column < 9; ++column) {
    ES::M3d dF = ES::M3d::Zero();
    dF.data()[column] = 1.0;
    const ES::M3d dP =
      mu * (dF + dF.transpose()) +
      lambda * dF.trace() * ES::M3d::Identity();
    expected.col(column) =
      Eigen::Map<const ES::V9d>(dP.data());
  }
  expectRelativeNear(
    model.compute_dPdF({}, state), expected, 1e-14);
}

TEST(ElasticModel3DNeoHookean, RandomDerivativesMatchFiniteDifferences)
{
  SDM::ElasticModel3DNeoHookean model(2.3, 4.7);
  std::mt19937 randomEngine(20260728);
  std::uniform_real_distribution<double> distribution(
    -1.0, 1.0);
  constexpr double step = 1e-6;

  for (int sample = 0; sample < 20; ++sample) {
    ES::M3d F = ES::M3d::Identity();
    for (double &entry : F.reshaped())
      entry += 0.2 * distribution(randomEngine);
    ASSERT_GT(F.determinant(), 0.2);

    const SDM::SpectralState state = stateFromF(F);
    const ES::M3d analyticP = model.compute_P({}, state);
    const ES::M9d analyticTangent =
      model.compute_dPdF({}, state);
    ES::V9d finiteDifferenceP;
    ES::M9d finiteDifferenceTangent;

    for (int dof = 0; dof < 9; ++dof) {
      ES::M3d plus = F;
      ES::M3d minus = F;
      plus.data()[dof] += step;
      minus.data()[dof] -= step;
      finiteDifferenceP(dof) =
        (model.compute_psi({}, stateFromF(plus)) -
          model.compute_psi({}, stateFromF(minus))) /
        (2.0 * step);
      const ES::M3d difference =
        (model.compute_P({}, stateFromF(plus)) -
          model.compute_P({}, stateFromF(minus))) /
        (2.0 * step);
      finiteDifferenceTangent.col(dof) =
        Eigen::Map<const ES::V9d>(difference.data());
    }

    expectRelativeNear(
      Eigen::Map<const ES::V9d>(analyticP.data()),
      finiteDifferenceP,
      2e-9);
    expectRelativeNear(
      analyticTangent,
      finiteDifferenceTangent,
      2e-9);
    expectRelativeNear(
      analyticTangent,
      analyticTangent.transpose(),
      2e-14);
  }
}

TEST(ElasticModel3DNeoHookean, PSDProjectionIsPositiveSemidefinite)
{
  SDM::ElasticModel3DNeoHookean model(2.3, 4.7);
  ES::M3d F;
  F <<
    0.45, 0.18, 0.0,
    0.02, 1.25, 0.12,
    0.0, 0.04, 0.72;
  const ES::M9d projected =
    model.compute_dPdF_psd({}, stateFromF(F));
  const Eigen::SelfAdjointEigenSolver<ES::M9d> solver(
    projected);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-12);
}

TEST(ElasticModel3DNeoHookean, DefinitionAndDomainValidation)
{
  SDM::NeoHookeanDefinition definition;
  EXPECT_EQ(definition.id(), "neo_hookean");
  EXPECT_EQ(definition.numFixedChannels(), 2);
  EXPECT_EQ(definition.numOptimizableChannels(), 0);

  std::unique_ptr<SDM::ElasticModel> created =
    definition.createModel(
      std::array<double, 2>{ 200000.0, 0.35 },
      SDM::MaterialFrame::Identity());
  auto *neo =
    dynamic_cast<SDM::ElasticModel3DNeoHookean *>(
      created.get());
  ASSERT_NE(neo, nullptr);
  EXPECT_NEAR(
    neo->mu(),
    200000.0 / (2.0 * 1.35),
    1e-12);

  EXPECT_THROW(
    definition.createModel(
      std::array<double, 1>{ 200000.0 },
      SDM::MaterialFrame::Identity()),
    std::invalid_argument);
  EXPECT_THROW(
    definition.createModel(
      std::array<double, 2>{ 200000.0, 0.5 },
      SDM::MaterialFrame::Identity()),
    std::invalid_argument);
  EXPECT_THROW(
    SDM::ElasticModel3DNeoHookean(0.0, 1.0),
    std::invalid_argument);

  const ES::M3d inverted =
    (ES::V3d(-1.0, 1.0, 1.0)).asDiagonal();
  EXPECT_THROW(
    neo->compute_psi({}, stateFromF(inverted)),
    std::domain_error);
  EXPECT_THROW(
    neo->compute_P(
      std::array<double, 1>{ 1.0 },
      stateFromF(ES::M3d::Identity())),
    std::invalid_argument);
}

}  // namespace
