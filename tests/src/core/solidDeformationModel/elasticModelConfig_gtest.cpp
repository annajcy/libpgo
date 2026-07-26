#include <gtest/gtest.h>

#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/deformationGradient/spectral/elasticModelStableNeoHookeanPrincipalStretch.h"
#include "material/elastic/elasticModelHillTypeMaterial.h"
#include "material/elastic/elasticModel3DMooneyRivlin.h"
#include "material/plastic/plasticModel3D3DOF.h"

#include "EigenSupport.h"

#include <array>
#include <span>

using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

TEST(ElasticModelConfig, StableNeoHasStableIdentity)
{
  StableNeoConfig config;
  EXPECT_EQ(config.id(), "stable_neo");
  EXPECT_TRUE(config.parameterChannelNames().empty());
  EXPECT_EQ(config.frameRequirement(), MaterialFrameRequirement::None);
}

TEST(ElasticModelConfig, StableNeoPrincipalStretchHasDistinctIdentity)
{
  StableNeoPrincipalStretchConfig config;
  EXPECT_EQ(config.id(), "stable_neo_principal_stretch");
  EXPECT_TRUE(config.parameterChannelNames().empty());
  EXPECT_EQ(config.frameRequirement(), MaterialFrameRequirement::None);
}

TEST(ElasticModelConfig, HillRequiresActivationAndPrimaryAxis)
{
  HillStableNeoConfig config;
  EXPECT_EQ(config.parameterChannelNames().size(), 1);
  EXPECT_EQ(config.parameterChannelNames().front(), "activation");
  EXPECT_EQ(config.frameRequirement(), MaterialFrameRequirement::PrimaryAxis);
}

TEST(PlasticModelConfig, DofVariantsExposeExplicitConfigs)
{
  VolumetricPlasticity3Config config;
  EXPECT_EQ(config.id(), "volumetric_dof3");
  EXPECT_EQ(config.parameterChannelNames().size(), 3);
  EXPECT_EQ(config.frameRequirement(), MaterialFrameRequirement::FullFrame);
}

TEST(ElasticModelConfig, MooneyRivlinDerivativesMatchFiniteDifferences)
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

TEST(ElasticModelConfig, MooneyRivlinPSDPathProjectsOnlyTangent)
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
