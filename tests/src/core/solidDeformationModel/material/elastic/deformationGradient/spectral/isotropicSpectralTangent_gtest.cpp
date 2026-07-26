#include <gtest/gtest.h>

#include "material/elastic/deformationGradient/spectral/elasticModel3DIsotropicPrincipalStretch.h"
#include "material/elastic/deformationGradient/spectral/elasticModelStableNeoHookeanPrincipalStretch.h"
#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <span>
#include <stdexcept>

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

enum class NonFiniteDerivative
{
  Gradient,
  Hessian
};

class NonFinitePrincipalStretchModel final
  : public ElasticModel3DIsotropicPrincipalStretch
{
public:
  explicit NonFinitePrincipalStretchModel(NonFiniteDerivative derivative):
    derivative_(derivative)
  {
  }

  int getNumParameters() const override { return 0; }

protected:
  double compute_psi_s(std::span<const double>, const ES::V3d &s) const override
  {
    return 0.5 * s.squaredNorm();
  }

  ES::V3d compute_dpsi_ds(std::span<const double>, const ES::V3d &s) const override
  {
    ES::V3d gradient = s;
    if (derivative_ == NonFiniteDerivative::Gradient)
      gradient(0) = std::numeric_limits<double>::quiet_NaN();
    return gradient;
  }

  ES::M3d compute_d2psi_ds2(std::span<const double>, const ES::V3d &) const override
  {
    ES::M3d hessian = ES::M3d::Identity();
    if (derivative_ == NonFiniteDerivative::Hessian)
      hessian(0, 0) = std::numeric_limits<double>::infinity();
    return hessian;
  }

private:
  NonFiniteDerivative derivative_;
};

struct SpectralData
{
  ES::M3d F;
  ES::M3d U;
  ES::M3d V;
  ES::V3d S;
};

SpectralData makeSpectralData(const ES::V3d &s)
{
  const Eigen::AngleAxisd first(0.37, ES::V3d(1.0, 2.0, 3.0).normalized());
  const Eigen::AngleAxisd second(-0.61, ES::V3d(-2.0, 1.0, 0.5).normalized());
  SpectralData data;
  data.U = (first * second).toRotationMatrix();
  data.V = Eigen::AngleAxisd(
    0.83, ES::V3d(2.0, -1.0, 1.0).normalized()).toRotationMatrix();
  data.S = s;
  data.F = data.U * s.asDiagonal() * data.V.transpose();
  return data;
}

SpectralData decomposePositiveF(const ES::M3d &F)
{
  const Eigen::JacobiSVD<ES::M3d> svd(
    F, Eigen::ComputeFullU | Eigen::ComputeFullV);
  SpectralData data;
  data.F = F;
  data.U = svd.matrixU();
  data.V = svd.matrixV();
  data.S = svd.singularValues();
  return data;
}

SpectralState toState(const SpectralData &data)
{
  SpectralState state;
  state.F = data.F;
  state.U = data.U;
  state.V = data.V;
  state.stretches = data.S;
  return state;
}

double evaluatePsi(
  const ElasticModel3DIsotropicPrincipalStretch &model,
  const ES::M3d &F)
{
  const SpectralData data = decomposePositiveF(F);
  return model.compute_psi({}, toState(data));
}

ES::M3d evaluateP(
  const ElasticModel3DIsotropicPrincipalStretch &model,
  const ES::M3d &F)
{
  const SpectralData data = decomposePositiveF(F);
  return model.compute_P({}, toState(data));
}

ES::M9d evaluate_dPdF(
  const ElasticModel3DIsotropicPrincipalStretch &model,
  const ES::M3d &F)
{
  const SpectralData data = decomposePositiveF(F);
  return model.compute_dPdF({}, toState(data));
}

void expectRelativeNear(const ES::M9d &a, const ES::M9d &b, double tolerance)
{
  const double scale = std::max({1.0, a.norm(), b.norm()});
  EXPECT_LT((a - b).norm() / scale, tolerance);
}

TEST(IsotropicSpectralTangent, ApplyMatchesAssembledMatrix)
{
  IsotropicSpectralTangentBlocks blocks;
  blocks.d2psi_ds2 << 2.0, 0.3, -0.2,
    0.3, 1.7, 0.4,
    -0.2, 0.4, 3.1;
  blocks.beta << 0.8, 1.2, 0.5;
  blocks.alpha << 1.1, 0.7, 1.4;
  const auto data = makeSpectralData(ES::V3d(1.3, 0.9, 0.6));
  const ES::M9d assembled = IsotropicSpectralTangent::assemble_dPdF(
    data.U, data.V, blocks);
  const ES::M3d dF = (ES::M3d() <<
    0.2, -0.4, 0.1,
    0.5, 0.3, -0.2,
    -0.1, 0.6, 0.7).finished();
  const ES::M3d applied = IsotropicSpectralTangent::apply_dPdF(
    data.U, data.V, blocks, dF);
  const ES::V9d expected = assembled * Eigen::Map<const ES::V9d>(dF.data());
  EXPECT_TRUE(applied.isApprox(Eigen::Map<const ES::M3d>(expected.data()), 1e-12));
}

TEST(IsotropicSpectralTangent, StableNeoMatchesDirectFSpaceModel)
{
  constexpr double mu = 0.7;
  constexpr double lambda = 1.2;
  ElasticModelStableNeoHookeanPrincipalStretch spectral(mu, lambda);
  ElasticModelStableNeoHookeanMaterial direct(mu, lambda);
  const auto data = makeSpectralData(ES::V3d(1.3, 0.9, 0.6));

  const SpectralState state = toState(data);
  const ES::M3d spectralP = spectral.compute_P({}, state);
  const ES::M9d spectralH = spectral.compute_dPdF({}, state);
  const ES::M3d directP = direct.compute_P({}, state);
  const ES::M9d directH = direct.compute_dPdF({}, state);

  EXPECT_TRUE(spectralP.isApprox(directP, 1e-10));
  expectRelativeNear(spectralH, directH, 1e-10);
}

TEST(IsotropicSpectralTangent, EigenSpectralStateApiWorksThroughBase)
{
  ElasticModelStableNeoHookeanPrincipalStretch model(0.7, 1.2);
  const auto data = makeSpectralData(ES::V3d(1.3, 0.9, 0.6));
  SpectralState state;
  state.F = data.F;
  state.U = data.U;
  state.V = data.V;
  state.stretches = data.S;

  const ElasticModel3DDeformationGradient &base = model;
  EXPECT_TRUE(base.compute_P({}, state).isApprox(model.compute_P({}, state), 1e-12));
  EXPECT_TRUE(base.compute_dPdF({}, state).isApprox(model.compute_dPdF({}, state), 1e-12));
}

TEST(IsotropicSpectralTangent, StableNeoMatchesDirectFSpaceAtRepeatedStretches)
{
  constexpr double mu = 0.7;
  constexpr double lambda = 1.2;
  ElasticModelStableNeoHookeanPrincipalStretch spectral(mu, lambda);
  ElasticModelStableNeoHookeanMaterial direct(mu, lambda);

  const std::array<ES::V3d, 2> stretchCases = {
    ES::V3d(1.2, 1.2, 0.7), ES::V3d(1.0, 1.0, 1.0)};
  for (const ES::V3d &s : stretchCases) {
    const auto data = makeSpectralData(s);
    const SpectralState state = toState(data);
    const double spectralPsi = spectral.compute_psi({}, state);
    const double directPsi = direct.compute_psi({}, state);
    const ES::M3d spectralP = spectral.compute_P({}, state);
    const ES::M9d spectralH = spectral.compute_dPdF({}, state);
    const ES::M3d directP = direct.compute_P({}, state);
    const ES::M9d directH = direct.compute_dPdF({}, state);

    EXPECT_NEAR(spectralPsi, directPsi, 1e-12);
    EXPECT_LT((spectralP - directP).norm(), 1e-10);
    expectRelativeNear(spectralH, directH, 1e-10);
  }
}

TEST(IsotropicSpectralTangent, StableNeoMatchesDirectFSpaceNearRepeatedStretches)
{
  constexpr double mu = 0.7;
  constexpr double lambda = 1.2;
  ElasticModelStableNeoHookeanPrincipalStretch spectral(mu, lambda);
  ElasticModelStableNeoHookeanMaterial direct(mu, lambda);

  const std::array<double, 3> separations = {1e-10, 5e-7, 2e-6};
  for (double separation : separations) {
    const auto data =
      makeSpectralData(ES::V3d(1.2 + separation, 1.2, 0.7));
    const SpectralState state = toState(data);
    expectRelativeNear(spectral.compute_dPdF({}, state),
      direct.compute_dPdF({}, state), 2e-6);
  }
}

TEST(IsotropicSpectralTangent, RandomFiniteDifferencesRecoverPAndTangent)
{
  ElasticModelStableNeoHookeanPrincipalStretch model(0.7, 1.2);
  std::mt19937 randomEngine(20230726);
  std::uniform_real_distribution<double> distribution(-1.0, 1.0);
  constexpr double step = 1e-6;

  for (int sample = 0; sample < 8; sample++) {
    ES::M3d F = ES::M3d::Identity();
    for (double &entry : F.reshaped())
      entry += 0.18 * distribution(randomEngine);
    ASSERT_GT(F.determinant(), 0.2);

    const ES::M3d analyticP = evaluateP(model, F);
    const ES::M9d analyticTangent = evaluate_dPdF(model, F);
    ES::V9d finiteDifferenceP;
    ES::M9d finiteDifferenceTangent;

    for (int dof = 0; dof < 9; dof++) {
      ES::M3d FPlus = F;
      ES::M3d FMinus = F;
      FPlus.data()[dof] += step;
      FMinus.data()[dof] -= step;

      finiteDifferenceP(dof) =
        (evaluatePsi(model, FPlus) - evaluatePsi(model, FMinus)) /
        (2.0 * step);
      const ES::M3d PPlus = evaluateP(model, FPlus);
      const ES::M3d PMinus = evaluateP(model, FMinus);
      const ES::M3d dP = (PPlus - PMinus) / (2.0 * step);
      finiteDifferenceTangent.col(dof) =
        Eigen::Map<const ES::V9d>(dP.data());
    }

    const ES::V9d analyticPVector =
      Eigen::Map<const ES::V9d>(analyticP.data());
    const double pScale =
      std::max({1.0, analyticPVector.norm(), finiteDifferenceP.norm()});
    const double tangentScale =
      std::max({1.0, analyticTangent.norm(), finiteDifferenceTangent.norm()});
    EXPECT_LT(
      (analyticPVector - finiteDifferenceP).norm() / pScale, 1e-8);
    EXPECT_LT(
      (analyticTangent - finiteDifferenceTangent).norm() / tangentScale,
      1e-8);
  }
}

TEST(IsotropicSpectralTangent, PairEqualIsIndependentOfDegenerateBasis)
{
  ElasticModelStableNeoHookeanPrincipalStretch model(0.7, 1.2);
  const auto data = makeSpectralData(ES::V3d(1.2, 1.2, 0.7));
  const Eigen::AngleAxisd rotation(0.47, ES::V3d::UnitZ());
  const ES::M3d R = rotation.toRotationMatrix();
  const ES::M3d U2 = data.U * R;
  const ES::M3d V2 = data.V * R;

  SpectralState state1 = toState(data);
  SpectralState state2 = state1;
  state2.U = U2;
  state2.V = V2;
  const ES::M3d P1 = model.compute_P({}, state1);
  const ES::M3d P2 = model.compute_P({}, state2);
  const ES::M9d H1 = model.compute_dPdF({}, state1);
  const ES::M9d H2 = model.compute_dPdF({}, state2);

  EXPECT_TRUE(P1.isApprox(P2, 1e-10));
  expectRelativeNear(H1, H2, 1e-10);
}

TEST(IsotropicSpectralTangent, AllEqualIsIndependentOfDegenerateBasis)
{
  ElasticModelStableNeoHookeanPrincipalStretch model(0.7, 1.2);
  const auto data = makeSpectralData(ES::V3d::Ones());
  const ES::M3d R = Eigen::AngleAxisd(
    -0.51, ES::V3d(1.0, -3.0, 2.0).normalized()).toRotationMatrix();
  const ES::M3d U2 = data.U * R;
  const ES::M3d V2 = data.V * R;

  SpectralState state1 = toState(data);
  SpectralState state2 = state1;
  state2.U = U2;
  state2.V = V2;
  expectRelativeNear(model.compute_dPdF({}, state1),
    model.compute_dPdF({}, state2), 1e-10);
}

TEST(IsotropicSpectralTangent, PSDProjectionClampsSpectralBlocks)
{
  IsotropicSpectralTangentBlocks blocks;
  blocks.d2psi_ds2 << 2.0, 0.0, 0.0,
    0.0, -1.0, 0.0,
    0.0, 0.0, 3.0;
  blocks.beta << -0.5, 1.2, 0.0;
  blocks.alpha << 0.3, -0.2, 0.0;
  const auto projected =
    IsotropicSpectralTangent::project_dPdF_blocks_psd(blocks);
  const Eigen::SelfAdjointEigenSolver<ES::M3d> solver(projected.d2psi_ds2);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-12);
  EXPECT_GE(projected.beta.minCoeff(), 0.0);
  EXPECT_GE(projected.alpha.minCoeff(), 0.0);
}

TEST(IsotropicSpectralTangent, PrincipalStretchPSDPathProducesPSDMatrix)
{
  ElasticModelStableNeoHookeanPrincipalStretch model(0.7, 1.2);
  const auto data = makeSpectralData(ES::V3d(1.3, 0.9, 0.6));
  const ES::M9d matrix = model.compute_dPdF_psd({}, toState(data));
  const Eigen::SelfAdjointEigenSolver<ES::M9d> solver(matrix);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-10);
}

TEST(IsotropicSpectralTangent, RejectsNonPositiveStretches)
{
  ElasticModelStableNeoHookeanPrincipalStretch model(0.7, 1.2);
  const auto data = makeSpectralData(ES::V3d(1.2, 0.9, 0.6));
  SpectralState state = toState(data);
  state.stretches(2) = 0.0;
  EXPECT_THROW(
    model.compute_P({}, state),
    std::domain_error);
}

TEST(IsotropicSpectralTangent, RejectsNonFiniteStretchDerivatives)
{
  const auto data = makeSpectralData(ES::V3d(1.2, 0.9, 0.6));
  NonFinitePrincipalStretchModel badGradient(NonFiniteDerivative::Gradient);
  NonFinitePrincipalStretchModel badHessian(NonFiniteDerivative::Hessian);
  const SpectralState state = toState(data);

  EXPECT_THROW(
    badGradient.compute_P({}, state),
    std::invalid_argument);
  EXPECT_THROW(
    badHessian.compute_dPdF({}, state),
    std::invalid_argument);
}

}  // namespace
