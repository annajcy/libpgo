#include <gtest/gtest.h>

#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/deformationGradient/spectral/elasticModelStableNeoHookeanPrincipalStretch.h"
#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModelHillTypeMaterial.h"
#include "material/elastic/elasticModel3DMooneyRivlin.h"
#include "material/plastic/plasticModel3D3DOF.h"

#include "EigenSupport.h"

#include <array>
#include <memory>
#include <span>
#include <vector>

using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

TEST(ElasticModelDefinition, StableNeoHasStableIdentity)
{
  StableNeoDefinition config;
  EXPECT_EQ(config.id(), "stable_neo");
  EXPECT_EQ(config.numOptimizableChannels(), 0);
}

TEST(ElasticModelDefinition, StableNeoPrincipalStretchHasDistinctIdentity)
{
  StableNeoPrincipalStretchDefinition config;
  EXPECT_EQ(config.id(), "stable_neo_principal_stretch");
  EXPECT_EQ(config.numOptimizableChannels(), 0);
}

TEST(ElasticModelDefinition, HillRequiresActivation)
{
  HillStableNeoDefinition config;
  EXPECT_EQ(config.numOptimizableChannels(), 1);
}

TEST(ElasticModelDefinition, CombinedHillBatchParameterDerivativesMatchFiniteDifferences)
{
  ElasticModelCombinedMaterial<2> model(
    std::make_unique<ElasticModelHillTypeMaterial>(
      0.7, 2.4, 1.1, ES::V3d::UnitX()),
    std::make_unique<ElasticModelHillTypeMaterial>(
      1.2, 1.8, 0.9, ES::V3d::UnitY()));
  std::array<double, 2> parameters = { 0.4, 0.8 };
  SpectralState state;
  state.F <<
    1.1, 0.2, 0.0,
    0.1, 0.9, 0.1,
    0.0, 0.1, 1.2;

  ES::VXd gradient(2);
  ES::MXd hessian(2, 2);
  ES::MXd pJacobian(9, 2);
  model.compute_dpsi_dparams(
    parameters, state, gradient);
  model.compute_d2psi_dparams2(
    parameters, state, hessian);
  model.compute_dP_dparams(
    parameters, state, pJacobian);

  EXPECT_TRUE(hessian.isZero(0.0));
  constexpr double step = 1e-6;
  for (int parameter = 0; parameter < 2; ++parameter) {
    std::array<double, 2> plus = parameters;
    std::array<double, 2> minus = parameters;
    plus[parameter] += step;
    minus[parameter] -= step;

    EXPECT_NEAR(
      gradient(parameter),
      (model.compute_psi(plus, state) -
        model.compute_psi(minus, state)) /
        (2.0 * step),
      1e-9);

    const ES::M3d finiteDifferenceP =
      (model.compute_P(plus, state) -
        model.compute_P(minus, state)) /
      (2.0 * step);
    EXPECT_TRUE(
      pJacobian.col(parameter).isApprox(
        Eigen::Map<const ES::V9d>(
          finiteDifferenceP.data()),
        1e-9));
  }
}

TEST(ElasticModelDefinition, DynamicCombinedMaterialBatchDerivativesMatchFiniteDifferences)
{
  std::vector<
    std::unique_ptr<ElasticModel3DDeformationGradient>>
    materials;
  materials.emplace_back(
    std::make_unique<ElasticModelHillTypeMaterial>(
      0.7, 2.4, 1.1, ES::V3d::UnitX()));
  materials.emplace_back(
    std::make_unique<ElasticModelStableNeoHookeanMaterial>(
      1.3, 2.1));
  materials.emplace_back(
    std::make_unique<ElasticModelHillTypeMaterial>(
      1.2, 1.8, 0.9, ES::V3d::UnitY()));
  ElasticModelDynamicCombinedMaterial model(
    std::move(materials));

  std::array<double, 2> parameters = { 0.4, 0.8 };
  SpectralState state;
  state.F <<
    1.1, 0.2, 0.0,
    0.1, 0.9, 0.1,
    0.0, 0.1, 1.2;

  ASSERT_EQ(model.getNumParameters(), 2);
  ES::VXd gradient(2);
  ES::MXd hessian(2, 2);
  ES::MXd pJacobian(9, 2);
  model.compute_dpsi_dparams(
    parameters, state, gradient);
  model.compute_d2psi_dparams2(
    parameters, state, hessian);
  model.compute_dP_dparams(
    parameters, state, pJacobian);

  constexpr double step = 1e-6;
  for (int parameter = 0; parameter < 2; ++parameter) {
    std::array<double, 2> plus = parameters;
    std::array<double, 2> minus = parameters;
    plus[parameter] += step;
    minus[parameter] -= step;

    const double finiteDifferenceGradient =
      (model.compute_psi(plus, state) -
        model.compute_psi(minus, state)) /
      (2.0 * step);
    EXPECT_NEAR(
      gradient(parameter),
      finiteDifferenceGradient,
      1e-9);

    const ES::M3d finiteDifferenceP =
      (model.compute_P(plus, state) -
        model.compute_P(minus, state)) /
      (2.0 * step);
    EXPECT_TRUE(
      pJacobian.col(parameter).isApprox(
        Eigen::Map<const ES::V9d>(
          finiteDifferenceP.data()),
        1e-9));

    ES::VXd plusGradient(2);
    ES::VXd minusGradient(2);
    model.compute_dpsi_dparams(
      plus, state, plusGradient);
    model.compute_dpsi_dparams(
      minus, state, minusGradient);
    const ES::VXd finiteDifferenceHessianColumn =
      (plusGradient - minusGradient) /
      (2.0 * step);
    EXPECT_TRUE(
      hessian.col(parameter).isApprox(
        finiteDifferenceHessianColumn,
        1e-10));
  }

  EXPECT_TRUE(hessian.isZero(0.0));
}

TEST(PlasticModelDefinition, DofVariantsExposeExplicitConfigs)
{
  VolumetricPlasticity3Definition config;
  EXPECT_EQ(config.id(), "volumetric_dof3");
  EXPECT_EQ(config.numOptimizableChannels(), 3);
}

TEST(ElasticModelDefinition, MooneyRivlinDerivativesMatchFiniteDifferences)
{
  ElasticModel3DMooneyRivlin model(0.7, 0.4, 2.0);
  const std::array<double, 9> F = {
    1.10, 0.05, 0.00,
    0.02, 0.92, 0.03,
    0.00, 0.04, 1.15};
  const std::array<double, 9> I = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  const std::array<double, 3> S = {1.0, 1.0, 1.0};
  constexpr double h = 1e-6;

  SpectralState state;
  state.F = Eigen::Map<const ES::M3d>(F.data());
  state.U = Eigen::Map<const ES::M3d>(I.data());
  state.V = Eigen::Map<const ES::M3d>(I.data());
  state.stretches = Eigen::Map<const ES::V3d>(S.data());
  const ES::M3d P = model.compute_P({}, state);

  auto plusF = F;
  auto minusF = F;
  plusF[0] += h;
  minusF[0] -= h;
  SpectralState plusState = state;
  SpectralState minusState = state;
  plusState.F = Eigen::Map<const ES::M3d>(plusF.data());
  minusState.F = Eigen::Map<const ES::M3d>(minusF.data());
  const double psiPlus = model.compute_psi({}, plusState);
  const double psiMinus = model.compute_psi({}, minusState);
  EXPECT_NEAR(P.data()[0], (psiPlus - psiMinus) / (2.0 * h), 2e-7);

  const ES::M9d dPdF = model.compute_dPdF({}, state);
  const ES::M9d &tangent = dPdF;

  for (int column = 0; column < 9; ++column) {
    auto plus = F;
    auto minus = F;
    plus[column] += h;
    minus[column] -= h;
    SpectralState plusState = state;
    SpectralState minusState = state;
    plusState.F = Eigen::Map<const ES::M3d>(plus.data());
    minusState.F = Eigen::Map<const ES::M3d>(minus.data());
    const ES::M3d PPlus = model.compute_P({}, plusState);
    const ES::M3d PMinus = model.compute_P({}, minusState);
    for (int row = 0; row < 9; ++row) {
      const double finiteDifference = (PPlus.data()[row] - PMinus.data()[row]) / (2.0 * h);
      EXPECT_NEAR(tangent(row, column), finiteDifference, 3e-5)
        << "row=" << row << ", column=" << column;
    }
  }
}

TEST(ElasticModelDefinition, MooneyRivlinPSDPathProjectsOnlyTangent)
{
  ElasticModel3DMooneyRivlin model(0.7, 0.4, 2.0);
  const std::array<double, 9> F = {
    1.10, 0.05, 0.00,
    0.02, 0.92, 0.03,
    0.00, 0.04, 1.15};
  const std::array<double, 9> I = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  const std::array<double, 3> S = {1.0, 1.0, 1.0};
  SpectralState state;
  state.F = Eigen::Map<const ES::M3d>(F.data());
  state.U = Eigen::Map<const ES::M3d>(I.data());
  state.V = Eigen::Map<const ES::M3d>(I.data());
  state.stretches = Eigen::Map<const ES::V3d>(S.data());
  const ES::M9d tangent = model.compute_dPdF_psd({}, state);
  EXPECT_TRUE(tangent.isApprox(tangent.transpose(), 1e-10));
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> solver(tangent);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-10);
}
