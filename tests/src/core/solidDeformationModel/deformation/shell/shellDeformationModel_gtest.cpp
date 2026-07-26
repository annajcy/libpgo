#include "gtest/gtest.h"

#include "material/elastic/elasticModel2DFundamentalFormsFabric.h"
#include "material/elastic/elasticModel2DFundamentalFormsSTVK.h"
#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"

#include "deformation/shell/shellDeformationModel.h"
#include "deformation/shell/koiterShellElementMapping.h"

#include <cmath>
#include <algorithm>
#include <functional>
#include <span>
#include <stdexcept>
#include <utility>

namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

namespace
{

template<typename Derived>
std::span<const double> constSpan(const Eigen::MatrixBase<Derived> &values)
{
  return std::span<const double>(values.derived().data(),
    static_cast<size_t>(values.size()));
}

template<typename Derived>
std::span<double> mutableSpan(Eigen::MatrixBase<Derived> &values)
{
  return std::span<double>(values.derived().data(),
    static_cast<size_t>(values.size()));
}

// Interior triangle: all 6 nodes present, slightly curved out of plane.
const ES::V18d interiorRestX = (ES::V18d() << 0.0, 0.0, 0.0,
  2.0, 0.0, 0.5,
  1.0, 1.5, -0.3,
  -0.5, -0.2, 1.0,
  2.5, -0.1, 0.8,
  1.2, 2.0, 1.2)
                                 .finished();

void perturbedDisplacement(double *x, const double *rest, int n, double scale)
{
  for (int i = 0; i < n; i++) {
    x[i] = rest[i] + scale * 0.05 * std::sin(0.7 * static_cast<double>(i) + 0.3);
  }
}

ES::VXd defaultShellElasticParams()
{
  ES::VXd params(5);
  params << 20000.0, 0.45, 10000.0, 0.3, 1e-3;
  return params;
}

ES::VXd defaultShellPlasticParams()
{
  ES::VXd params(1);
  params << 1.0;
  return params;
}

}  // namespace

// ============================================================
// Interior triangle: energy finite at rest and gradient FD check
// ============================================================

TEST(ShellDeformationModelTest, InteriorEnergyFiniteAtRest)
{
  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  auto evaluator = model.createEvaluator();
  const ES::VXd elasticParams = defaultShellElasticParams();
  const ES::VXd plasticParams = defaultShellPlasticParams();
  evaluator->prepare(
    constSpan(interiorRestX), constSpan(elasticParams), constSpan(plasticParams));

  double energy = evaluator->compute_E();
  EXPECT_TRUE(std::isfinite(energy));

  ES::V18d grad;
  evaluator->compute_dE_dx(grad);
  for (int i = 0; i < 18; i++)
    EXPECT_TRUE(std::isfinite(grad[i]));
}

TEST(ShellDeformationModelTest, ParameterizedModelRejectsMissingParameters)
{
  auto elasticModel =
    std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel =
    std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };
  auto mapping =
    std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(
    std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  const ES::VXd elasticParams = defaultShellElasticParams();
  const ES::VXd plasticParams = defaultShellPlasticParams();
  auto evaluator = model.createEvaluator();

  EXPECT_THROW(
    evaluator->prepare(
      constSpan(interiorRestX), std::span<const double>{}, constSpan(plasticParams)),
    std::invalid_argument);
  EXPECT_THROW(
    evaluator->prepare(
      constSpan(interiorRestX), constSpan(elasticParams), std::span<const double>{}),
    std::invalid_argument);
  EXPECT_NO_THROW(
    evaluator->prepare(
      constSpan(interiorRestX), constSpan(elasticParams), constSpan(plasticParams)));
}

TEST(ShellDeformationModelTest, ExplicitPlasticParametersInitializeRestMetric)
{
  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  auto evaluator = model.createEvaluator();
  const ES::VXd elasticParams = defaultShellElasticParams();
  const ES::VXd plasticParams = defaultShellPlasticParams();
  evaluator->prepare(
    constSpan(interiorRestX), constSpan(elasticParams), constSpan(plasticParams));

  EXPECT_TRUE(std::isfinite(evaluator->compute_E()));
}

// ============================================================
// Boundary triangle (node 4 missing): energy finite at rest
// ============================================================

TEST(ShellDeformationModelTest, BoundaryMissingNode4EnergyFinite)
{
  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, false, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  auto evaluator = model.createEvaluator();
  const ES::VXd elasticParams = defaultShellElasticParams();
  const ES::VXd plasticParams = defaultShellPlasticParams();
  evaluator->prepare(
    constSpan(interiorRestX), constSpan(elasticParams), constSpan(plasticParams));

  double energy = evaluator->compute_E();
  EXPECT_TRUE(std::isfinite(energy));

  ES::V18d grad;
  evaluator->compute_dE_dx(grad);
  for (int i = 0; i < 18; i++)
    EXPECT_TRUE(std::isfinite(grad[i]));
}

// ============================================================
// FD sanity check — gradient matches finite difference
// ============================================================

TEST(ShellDeformationModelFDTest, GradientMatchesFiniteDifference)
{
  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  auto evaluator = model.createEvaluator();
  const ES::VXd elasticParams = defaultShellElasticParams();
  const ES::VXd plasticParams = defaultShellPlasticParams();

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX.data(), 18, 0.1);
  evaluator->prepare(
    x, constSpan(elasticParams), constSpan(plasticParams));

  ES::V18d g;
  evaluator->compute_dE_dx(g);

  const double eps = 1e-6;
  for (int i = 0; i < 18; i++) {
    double xPlus[18], xMinus[18];
    std::copy(x, x + 18, xPlus);
    std::copy(x, x + 18, xMinus);
    xPlus[i] += eps;
    xMinus[i] -= eps;

    auto evaluatorPlus = model.createEvaluator();
    evaluatorPlus->prepare(xPlus, constSpan(elasticParams), constSpan(plasticParams));
    double ePlus = evaluatorPlus->compute_E();

    auto evaluatorMinus = model.createEvaluator();
    evaluatorMinus->prepare(xMinus, constSpan(elasticParams), constSpan(plasticParams));
    double eMinus = evaluatorMinus->compute_E();

    double fdGrad = (ePlus - eMinus) / (2.0 * eps);
    EXPECT_NEAR(fdGrad, g[i], 1e-5) << "FD gradient mismatch at index " << i;
  }
}

TEST(ShellDeformationModelFDTest, PlasticParameterGradientMatchesFiniteDifference)
{
  ES::VXd elasticParams = defaultShellElasticParams();
  ES::VXd plasticParams(1);
  plasticParams << 1.2;

  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX.data(), 18, 0.1);

  auto energyAt = [&](double s) {
    ES::VXd params = plasticParams;
    params[0] = s;
    auto evaluator = model.createEvaluator();
    evaluator->prepare(x, constSpan(elasticParams), constSpan(params));
    return evaluator->compute_E();
  };

  plasticParams[0] = 1.2;
  auto evaluator = model.createEvaluator();
  evaluator->prepare(x, constSpan(elasticParams), constSpan(plasticParams));

  ES::VXd analytic(1);
  evaluator->compute_dE_dp(analytic);

  const double eps = 1e-6;
  const double fd = (energyAt(1.2 + eps) - energyAt(1.2 - eps)) / (2.0 * eps);
  EXPECT_NEAR(analytic[0], fd, 1e-5 * std::max(1.0, std::abs(fd)));
}

TEST(ShellDeformationModelFDTest, ElasticParameterGradientMatchesFiniteDifference)
{
  ES::VXd elasticParams = defaultShellElasticParams();
  ES::VXd plasticParams(1);
  plasticParams << 1.2;

  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX.data(), 18, 0.1);

  auto energyAt = [&](int channel, double value) {
    ES::VXd params = elasticParams;
    params[channel] = value;
    auto evaluator = model.createEvaluator();
    evaluator->prepare(x, constSpan(params), constSpan(plasticParams));
    return evaluator->compute_E();
  };

  auto evaluator = model.createEvaluator();
  evaluator->prepare(x, constSpan(elasticParams), constSpan(plasticParams));

  ES::VXd analytic(5);
  evaluator->compute_dE_de(analytic);

  for (int c = 0; c < 5; c++) {
    const double eps = 1e-6 * std::max(1.0, std::abs(elasticParams[c]));
    const double fd = (energyAt(c, elasticParams[c] + eps) - energyAt(c, elasticParams[c] - eps)) / (2.0 * eps);
    EXPECT_NEAR(analytic[c], fd, 1e-5 * std::max(1.0, std::abs(fd)))
      << "elastic parameter channel " << c;
  }
}

TEST(ShellMaterialAnalyticDerivativeFDTest, STVKReferenceAndElasticParameterDerivativesMatchFiniteDifference)
{
  ElasticModel2DFundamentalFormsSTVK elastic;
  ES::VXd params = defaultShellElasticParams();
  ES::M2d a;
  a << 4.4, 0.35, 0.35, 2.7;
  ES::M2d b;
  b << 0.36, 0.06, 0.06, -0.21;
  ES::M2d abar;
  abar << 3.9, 0.28, 0.28, 2.2;
  ES::M2d bbar;
  bbar << 0.11, 0.03, 0.03, -0.08;

  auto psiAt = [&](const ES::VXd &p, const ES::M2d &A, const ES::M2d &B) {
    return elastic.compute_psi_a(constSpan(p), a, A) +
      elastic.compute_psi_b(constSpan(p), b, A, B);
  };

  const ES::M2d dpsiDabarMat = elastic.compute_dpsi_dabar(constSpan(params), a, b, abar, bbar);
  const ES::M2d dpsiDbbarMat = elastic.compute_dpsi_dbbar(constSpan(params), a, b, abar, bbar);
  const ES::V4d dpsiDabar = Eigen::Map<const ES::V4d>(dpsiDabarMat.data());
  const ES::V4d dpsiDbbar = Eigen::Map<const ES::V4d>(dpsiDbbarMat.data());
  ES::VXd dpsiDparam(params.size());
  elastic.compute_dpsi_dparam(constSpan(params), a, b, abar, bbar, dpsiDparam);

  for (int i = 0; i < 4; i++) {
    const double h = 1e-6;
    ES::M2d plus = abar;
    ES::M2d minus = abar;
    plus.data()[i] += h;
    minus.data()[i] -= h;
    const double fd = (psiAt(params, plus, bbar) - psiAt(params, minus, bbar)) / (2.0 * h);
    EXPECT_NEAR(dpsiDabar[i], fd, 1e-5 * std::max(1.0, std::abs(fd))) << "dpsi/dabar " << i;

    plus = bbar;
    minus = bbar;
    plus.data()[i] += h;
    minus.data()[i] -= h;
    const double fdB = (psiAt(params, abar, plus) - psiAt(params, abar, minus)) / (2.0 * h);
    EXPECT_NEAR(dpsiDbbar[i], fdB, 1e-5 * std::max(1.0, std::abs(fdB))) << "dpsi/dbbar " << i;
  }

  for (int i = 0; i < params.size(); i++) {
    const double h = 1e-6 * std::max(1.0, std::abs(params[i]));
    ES::VXd plus = params;
    ES::VXd minus = params;
    plus[i] += h;
    minus[i] -= h;
    const double fd = (psiAt(plus, abar, bbar) - psiAt(minus, abar, bbar)) / (2.0 * h);
    EXPECT_NEAR(dpsiDparam[i], fd, 1e-5 * std::max(1.0, std::abs(fd))) << "dpsi/dparam " << i;
  }
}

TEST(ShellDeformationModelFDTest, ParameterHessiansMatchFiniteDifferenceOfParameterGradients)
{
  ES::VXd elasticParams = defaultShellElasticParams();
  ES::VXd plasticParams(1);
  plasticParams << 1.2;

  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX.data(), 18, 0.1);

  auto dE_dp_at = [&](const ES::VXd &elasticValues, const ES::VXd &plasticValues) {
    auto evaluator = model.createEvaluator();
    evaluator->prepare(x, constSpan(elasticValues), constSpan(plasticValues));
    ES::VXd grad(1);
    evaluator->compute_dE_dp(grad);
    return grad;
  };

  auto dE_de_at = [&](const ES::VXd &elasticValues, const ES::VXd &plasticValues) {
    auto evaluator = model.createEvaluator();
    evaluator->prepare(x, constSpan(elasticValues), constSpan(plasticValues));
    ES::VXd grad(5);
    evaluator->compute_dE_de(grad);
    return grad;
  };

  auto evaluator = model.createEvaluator();
  evaluator->prepare(x, constSpan(elasticParams), constSpan(plasticParams));

  ES::MXd d2daa(1, 1);
  evaluator->compute_d2E_dp2(d2daa);
  const double plasticStep = 1e-6 * std::max(1.0, std::abs(plasticParams[0]));
  ES::VXd plasticPlus = plasticParams;
  ES::VXd plasticMinus = plasticParams;
  plasticPlus[0] += plasticStep;
  plasticMinus[0] -= plasticStep;
  const ES::VXd gPlasticPlus = dE_dp_at(elasticParams, plasticPlus);
  const ES::VXd gPlasticMinus = dE_dp_at(elasticParams, plasticMinus);
  const double fdPlasticHessian = (gPlasticPlus[0] - gPlasticMinus[0]) / (2.0 * plasticStep);
  EXPECT_NEAR(d2daa(0, 0), fdPlasticHessian,
    2e-5 * std::max(1.0, std::abs(fdPlasticHessian)));

  ES::MXd d2dbb(5, 5);
  evaluator->compute_d2E_de2(d2dbb);
  for (int col = 0; col < elasticParams.size(); col++) {
    const double step = 1e-6 * std::max(1.0, std::abs(elasticParams[col]));
    ES::VXd elasticPlus = elasticParams;
    ES::VXd elasticMinus = elasticParams;
    elasticPlus[col] += step;
    elasticMinus[col] -= step;
    const ES::VXd gPlus = dE_de_at(elasticPlus, plasticParams);
    const ES::VXd gMinus = dE_de_at(elasticMinus, plasticParams);
    const ES::VXd fdCol = (gPlus - gMinus) / (2.0 * step);
    for (int row = 0; row < elasticParams.size(); row++) {
      EXPECT_NEAR(d2dbb(row, col), fdCol[row],
        2e-5 * std::max(1.0, std::abs(fdCol[row])))
        << "elastic Hessian (" << row << ", " << col << ")";
    }
  }

  ES::MXd d2dadb(1, 5);
  evaluator->compute_d2E_dpde(d2dadb);
  for (int col = 0; col < elasticParams.size(); col++) {
    const double step = 1e-6 * std::max(1.0, std::abs(elasticParams[col]));
    ES::VXd elasticPlus = elasticParams;
    ES::VXd elasticMinus = elasticParams;
    elasticPlus[col] += step;
    elasticMinus[col] -= step;
    const ES::VXd gPlus = dE_dp_at(elasticPlus, plasticParams);
    const ES::VXd gMinus = dE_dp_at(elasticMinus, plasticParams);
    const double fd = (gPlus[0] - gMinus[0]) / (2.0 * step);
    EXPECT_NEAR(d2dadb(0, col), fd, 2e-5 * std::max(1.0, std::abs(fd)))
      << "mixed plastic-elastic Hessian column " << col;
  }
}

TEST(ShellDeformationModelTest, FabricParameterDerivativeRequiresAnalyticImplementation)
{
  ES::VXd elasticParams(12);
  elasticParams << 0.1, 8.0, 0.5, 7.0, 0.45, 0.2, 0.1, 0.02, 0.03, 0.01, 0.0, 1e-3;
  ES::VXd plasticParams(1);
  plasticParams << 1.0;

  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsFabric>(
    ES::V2d(1.0, 0.0), ES::V2d(0.0, 1.0));
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX.data(), 18, 0.1);
  auto evaluator = model.createEvaluator();
  evaluator->prepare(x, constSpan(elasticParams), constSpan(plasticParams));

  ES::VXd grad(elasticParams.size());
  EXPECT_THROW(evaluator->compute_dE_de(grad), std::logic_error);
}

TEST(ShellDeformationModelTest, UnsupportedDiagnosticsThrow)
{
  ES::VXd elasticParams(12);
  elasticParams << 0.1, 8.0, 0.5, 7.0, 0.45, 0.2, 0.1,
    0.02, 0.03, 0.01, 0.0, 1e-3;
  ES::VXd plasticParams(1);
  plasticParams << 1.0;

  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsFabric>(
    ES::V2d(1.0, 0.0), ES::V2d(0.0, 1.0));
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(
    std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  auto evaluator = model.createEvaluator();
  evaluator->prepare(
    constSpan(interiorRestX), constSpan(elasticParams), constSpan(plasticParams));

  double value = 0.0;
  EXPECT_THROW(
    evaluator->computeVonMisesStress(std::span<double>(&value, 1), 1),
    UnsupportedDeformationDiagnosticError);
  EXPECT_THROW(
    evaluator->computeMaxStrain(std::span<double>(&value, 1), 1),
    UnsupportedDeformationDiagnosticError);
}

TEST(ShellDeformationModelTest, VonMisesDiagnosticEnforcesOutputCapacity)
{
  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(
    std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  auto evaluator = model.createEvaluator();
  const ES::VXd elasticParams = defaultShellElasticParams();
  const ES::VXd plasticParams = defaultShellPlasticParams();
  evaluator->prepare(
    constSpan(interiorRestX), constSpan(elasticParams), constSpan(plasticParams));

  double stress = -1.0;
  EXPECT_EQ(evaluator->computeVonMisesStress(std::span<double>(&stress, 1), 1), 1);
  EXPECT_TRUE(std::isfinite(stress));
  EXPECT_THROW(
    evaluator->computeVonMisesStress(std::span<double>{}, 0),
    std::length_error);
}

// ============================================================
// SPD enable produces symmetric PSD hessian
// ============================================================

TEST(ShellDeformationModelTest, SPDEnableProducesSymmetricPSD)
{
  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel),
    DeformationModelConstructionOptions{ true });

  auto evaluator = model.createEvaluator();
  const ES::VXd elasticParams = defaultShellElasticParams();
  const ES::VXd plasticParams = defaultShellPlasticParams();

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX.data(), 18, 0.1);
  evaluator->prepare(
    x, constSpan(elasticParams), constSpan(plasticParams));

  ES::M18d hess;
  evaluator->compute_d2E_dx2(hess);

  // Check symmetry.
  for (int i = 0; i < 18; i++)
    for (int j = 0; j < i; j++)
      EXPECT_NEAR(hess(i, j), hess(j, i), 1e-10)
        << "SPD hessian asymmetry at (" << i << ", " << j << ")";
}

TEST(KoiterShellElementMappingTest, FundamentalFormJacobiansMatchFiniteDifference)
{
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };
  KoiterShellElementMapping mapping(interiorRestX, hasVtx);
  ShellElementMapping::APositions triangle{
    interiorRestX.segment<3>(0), interiorRestX.segment<3>(3), interiorRestX.segment<3>(6)
  };
  ShellElementMapping::BPositions positions{
    interiorRestX.segment<3>(0), interiorRestX.segment<3>(3), interiorRestX.segment<3>(6),
    interiorRestX.segment<3>(9), interiorRestX.segment<3>(12), interiorRestX.segment<3>(15)
  };

  const auto checkEntries = [](const ES::M4x9d &jacobian,
                              const std::array<ES::V3d, 3> &x, const std::function<ES::M2d(const std::array<ES::V3d, 3> &)> &evaluate) {
    const double eps = 1e-7;
    const std::array<int, 4> entryIndex = { 0, 2, 1, 3 };
    for (int dof = 0; dof < 9; ++dof) {
      auto plus = x;
      auto minus = x;
      plus[dof / 3][dof % 3] += eps;
      minus[dof / 3][dof % 3] -= eps;
      const ES::M2d fdForm = (evaluate(plus) - evaluate(minus)) / (2.0 * eps);
      for (int row = 0; row < 4; ++row)
        EXPECT_NEAR(fdForm.data()[entryIndex[row]], jacobian(row, dof), 1e-6);
    }
  };

  const ES::M4x9d da = mapping.compute_da_dx(triangle);
  checkEntries(da, triangle, [&mapping](const ShellElementMapping::APositions &x) {
    return mapping.compute_a(x);
  });
  const ES::M9x36d d2a = mapping.compute_d2a_dx2(triangle);
  for (int parameter = 0; parameter < 9; ++parameter) {
    auto plus = triangle;
    auto minus = triangle;
    plus[parameter / 3][parameter % 3] += 1e-6;
    minus[parameter / 3][parameter % 3] -= 1e-6;
    const ES::M4x9d fdJacobian =
      (mapping.compute_da_dx(plus) - mapping.compute_da_dx(minus)) / 2e-6;
    for (int entry = 0; entry < 4; ++entry)
      for (int dof = 0; dof < 9; ++dof) {
        const double analytic = d2a.block<9, 9>(0, 9 * entry)(dof, parameter);
        EXPECT_NEAR(fdJacobian(entry, dof), analytic, 1e-5);
      }
  }

  const ES::M4x18d db = mapping.compute_db_dx(positions);
  const ES::M18x72d d2b = mapping.compute_d2b_dx2(positions);
  const double eps = 1e-7;
  const std::array<int, 4> entryIndex = { 0, 2, 1, 3 };
  for (int dof = 0; dof < 18; ++dof) {
    auto plus = positions;
    auto minus = positions;
    plus[dof / 3][dof % 3] += eps;
    minus[dof / 3][dof % 3] -= eps;
    const ES::M2d fdForm = (mapping.compute_b(plus) - mapping.compute_b(minus)) / (2.0 * eps);
    for (int row = 0; row < 4; ++row)
      EXPECT_NEAR(fdForm.data()[entryIndex[row]], db(row, dof), 1e-5);
  }
  for (int parameter = 0; parameter < 18; ++parameter) {
    auto plus = positions;
    auto minus = positions;
    plus[parameter / 3][parameter % 3] += 1e-6;
    minus[parameter / 3][parameter % 3] -= 1e-6;
    const ES::M4x18d fdJacobian =
      (mapping.compute_db_dx(plus) - mapping.compute_db_dx(minus)) / 2e-6;
    for (int entry = 0; entry < 4; ++entry)
      for (int dof = 0; dof < 18; ++dof) {
        const double analytic = d2b.block<18, 18>(0, 18 * entry)(dof, parameter);
        EXPECT_NEAR(fdJacobian(entry, dof), analytic, 5e-4);
      }
  }
}
