#include <gtest/gtest.h>

#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/elasticModelHillTypeMaterial.h"
#include "material/elastic/elasticModel3DMooneyRivlin.h"
#include "material/plastic/plasticModel3D3DOF.h"

#include "EigenSupport.h"

#include <array>

using namespace pgo::SolidDeformationModel;

TEST(ElasticModelConfig, StableNeoHasStableIdentity)
{
  StableNeoConfig config;
  EXPECT_EQ(config.id(), "stable_neo");
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
  constexpr double h = 1e-6;

  std::array<double, 9> P{};
  model.compute_P(nullptr, F.data(), nullptr, nullptr, nullptr, P.data());

  auto plusF = F;
  auto minusF = F;
  plusF[0] += h;
  minusF[0] -= h;
  const double psiPlus = model.compute_psi(
    nullptr, plusF.data(), nullptr, nullptr, nullptr);
  const double psiMinus = model.compute_psi(
    nullptr, minusF.data(), nullptr, nullptr, nullptr);
  EXPECT_NEAR(P[0], (psiPlus - psiMinus) / (2.0 * h), 2e-7);

  std::array<double, 81> dPdF{};
  model.compute_dPdF(nullptr, F.data(), nullptr, nullptr, nullptr, dPdF.data());
  const Eigen::Map<const Eigen::Matrix<double, 9, 9>> tangent(dPdF.data());

  for (int column = 0; column < 9; ++column) {
    auto plus = F;
    auto minus = F;
    plus[column] += h;
    minus[column] -= h;
    std::array<double, 9> PPlus{}, PMinus{};
    model.compute_P(nullptr, plus.data(), nullptr, nullptr, nullptr, PPlus.data());
    model.compute_P(nullptr, minus.data(), nullptr, nullptr, nullptr, PMinus.data());
    for (int row = 0; row < 9; ++row) {
      const double finiteDifference = (PPlus[row] - PMinus[row]) / (2.0 * h);
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
  std::array<double, 81> dPdF{};
  model.compute_dPdF_psd(nullptr, F.data(), nullptr, nullptr, nullptr, dPdF.data());

  const Eigen::Map<const Eigen::Matrix<double, 9, 9>> tangent(dPdF.data());
  EXPECT_TRUE(tangent.isApprox(tangent.transpose(), 1e-10));
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> solver(tangent);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-10);
}
