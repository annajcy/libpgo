#include "gtest/gtest.h"

#include "material/elastic/elasticModel2DFundamentalFormsFabric.h"
#include "material/elastic/elasticModel2DFundamentalFormsSTVK.h"
#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"

#include "deformation/shell/koiterShellDeformationElement.h"

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

TEST(ShellDeformationElementTest, InteriorEnergyFiniteAtRest)
{
  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  KoiterShellDeformationElement model(interiorRestX, hasVtx, std::move(elasticModel), std::move(plasticModel));

  const ES::VXd elasticParams = defaultShellElasticParams();
  const ES::VXd plasticParams = defaultShellPlasticParams();
  double energy = model.computeEnergy(
    constSpan(interiorRestX), constSpan(elasticParams), constSpan(plasticParams));
  EXPECT_TRUE(std::isfinite(energy));

  ES::V18d grad;
  model.computeDisplacementGradient(
    constSpan(interiorRestX), constSpan(elasticParams),
    constSpan(plasticParams), grad);
  for (int i = 0; i < 18; i++)
    EXPECT_TRUE(std::isfinite(grad[i]));
}

TEST(ShellDeformationElementTest, ParameterizedModelRejectsMissingParameters)
{
  auto elasticModel =
    std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel =
    std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };
  KoiterShellDeformationElement model(
    interiorRestX, hasVtx, std::move(elasticModel), std::move(plasticModel));

  const ES::VXd elasticParams = defaultShellElasticParams();
  const ES::VXd plasticParams = defaultShellPlasticParams();
  EXPECT_THROW(
    model.computeEnergy(
      constSpan(interiorRestX), {}, constSpan(plasticParams)),
    std::invalid_argument);
  EXPECT_THROW(
    model.computeEnergy(
      constSpan(interiorRestX), constSpan(elasticParams), {}),
    std::invalid_argument);
  EXPECT_NO_THROW(
    model.computeEnergy(
      constSpan(interiorRestX), constSpan(elasticParams), constSpan(plasticParams)));
}

TEST(ShellDeformationElementTest, ExplicitPlasticParametersInitializeRestMetric)
{
  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  KoiterShellDeformationElement model(interiorRestX, hasVtx, std::move(elasticModel), std::move(plasticModel));

  const ES::VXd elasticParams = defaultShellElasticParams();
  const ES::VXd plasticParams = defaultShellPlasticParams();
  EXPECT_TRUE(std::isfinite(model.computeEnergy(
    constSpan(interiorRestX), constSpan(elasticParams), constSpan(plasticParams))));
}

// ============================================================
// Boundary triangle (node 4 missing): energy finite at rest
// ============================================================

TEST(ShellDeformationElementTest, BoundaryMissingNode4EnergyFinite)
{
  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, false, true };

  KoiterShellDeformationElement model(interiorRestX, hasVtx, std::move(elasticModel), std::move(plasticModel));

  const ES::VXd elasticParams = defaultShellElasticParams();
  const ES::VXd plasticParams = defaultShellPlasticParams();
  double energy = model.computeEnergy(
    constSpan(interiorRestX), constSpan(elasticParams), constSpan(plasticParams));
  EXPECT_TRUE(std::isfinite(energy));

  ES::V18d grad;
  model.computeDisplacementGradient(
    constSpan(interiorRestX), constSpan(elasticParams),
    constSpan(plasticParams), grad);
  for (int i = 0; i < 18; i++)
    EXPECT_TRUE(std::isfinite(grad[i]));
}

// ============================================================
// FD sanity check — gradient matches finite difference
// ============================================================

TEST(ShellDeformationElementFDTest, GradientMatchesFiniteDifference)
{
  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  KoiterShellDeformationElement model(interiorRestX, hasVtx, std::move(elasticModel), std::move(plasticModel));

  const ES::VXd elasticParams = defaultShellElasticParams();
  const ES::VXd plasticParams = defaultShellPlasticParams();

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX.data(), 18, 0.1);
  ES::V18d g;
  model.computeDisplacementGradient(
    x, constSpan(elasticParams), constSpan(plasticParams), g);

  const double eps = 1e-6;
  for (int i = 0; i < 18; i++) {
    double xPlus[18], xMinus[18];
    std::copy(x, x + 18, xPlus);
    std::copy(x, x + 18, xMinus);
    xPlus[i] += eps;
    xMinus[i] -= eps;

    double ePlus = model.computeEnergy(
      xPlus, constSpan(elasticParams), constSpan(plasticParams));
    double eMinus = model.computeEnergy(
      xMinus, constSpan(elasticParams), constSpan(plasticParams));

    double fdGrad = (ePlus - eMinus) / (2.0 * eps);
    EXPECT_NEAR(fdGrad, g[i], 1e-5) << "FD gradient mismatch at index " << i;
  }
}

TEST(ShellDeformationElementFDTest, PlasticParameterGradientMatchesFiniteDifference)
{
  ES::VXd elasticParams = defaultShellElasticParams();
  ES::VXd plasticParams(1);
  plasticParams << 1.2;

  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  KoiterShellDeformationElement model(interiorRestX, hasVtx, std::move(elasticModel), std::move(plasticModel));

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX.data(), 18, 0.1);

  auto energyAt = [&](double s) {
    ES::VXd params = plasticParams;
    params[0] = s;
    return model.computeEnergy(x, constSpan(elasticParams), constSpan(params));
  };

  plasticParams[0] = 1.2;
  ES::VXd analytic(1);
  model.computePlasticGradient(
    x, constSpan(elasticParams), constSpan(plasticParams), analytic);

  const double eps = 1e-6;
  const double fd = (energyAt(1.2 + eps) - energyAt(1.2 - eps)) / (2.0 * eps);
  EXPECT_NEAR(analytic[0], fd, 1e-5 * std::max(1.0, std::abs(fd)));
}

TEST(ShellDeformationElementFDTest, ElasticParameterGradientMatchesFiniteDifference)
{
  ES::VXd elasticParams = defaultShellElasticParams();
  ES::VXd plasticParams(1);
  plasticParams << 1.2;

  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  KoiterShellDeformationElement model(interiorRestX, hasVtx, std::move(elasticModel), std::move(plasticModel));

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX.data(), 18, 0.1);

  auto energyAt = [&](int channel, double value) {
    ES::VXd params = elasticParams;
    params[channel] = value;
    return model.computeEnergy(x, constSpan(params), constSpan(plasticParams));
  };

  ES::VXd analytic(5);
  model.computeElasticGradient(
    x, constSpan(elasticParams), constSpan(plasticParams), analytic);

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

TEST(ShellDeformationElementTest, FabricParameterDerivativeRequiresAnalyticImplementation)
{
  ES::VXd elasticParams(12);
  elasticParams << 0.1, 8.0, 0.5, 7.0, 0.45, 0.2, 0.1, 0.02, 0.03, 0.01, 0.0, 1e-3;
  ES::VXd plasticParams(1);
  plasticParams << 1.0;

  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsFabric>(
    ES::V2d(1.0, 0.0), ES::V2d(0.0, 1.0));
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  KoiterShellDeformationElement model(interiorRestX, hasVtx, std::move(elasticModel), std::move(plasticModel));

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX.data(), 18, 0.1);
  ES::VXd grad(elasticParams.size());
  EXPECT_THROW(model.computeElasticGradient(
                 x, constSpan(elasticParams), constSpan(plasticParams), grad),
    std::logic_error);
}

TEST(ShellDeformationElementTest, UnsupportedDiagnosticsThrow)
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

  KoiterShellDeformationElement model(
    interiorRestX, hasVtx, std::move(elasticModel), std::move(plasticModel));

  double value = 0.0;
  EXPECT_THROW(
    model.computeVonMisesStress(
      constSpan(interiorRestX), constSpan(elasticParams),
      constSpan(plasticParams), std::span<double>(&value, 1)),
    UnsupportedDeformationDiagnosticError);
  EXPECT_THROW(
    model.computeMaxStrain(
      constSpan(interiorRestX), constSpan(elasticParams),
      constSpan(plasticParams), std::span<double>(&value, 1)),
    UnsupportedDeformationDiagnosticError);
}

TEST(ShellDeformationElementTest, VonMisesDiagnosticEnforcesOutputCapacity)
{
  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  KoiterShellDeformationElement model(
    interiorRestX, hasVtx, std::move(elasticModel), std::move(plasticModel));

  const ES::VXd elasticParams = defaultShellElasticParams();
  const ES::VXd plasticParams = defaultShellPlasticParams();
  double stress = -1.0;
  EXPECT_EQ(model.computeVonMisesStress(
              constSpan(interiorRestX), constSpan(elasticParams),
              constSpan(plasticParams), std::span<double>(&stress, 1)),
    1);
  EXPECT_TRUE(std::isfinite(stress));
  EXPECT_THROW(
    model.computeVonMisesStress(
      constSpan(interiorRestX), constSpan(elasticParams),
      constSpan(plasticParams), std::span<double>{}),
    std::length_error);
}

// ============================================================
// SPD enable produces symmetric PSD hessian
// ============================================================

TEST(ShellDeformationElementTest, SPDEnableProducesSymmetricPSD)
{
  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };

  KoiterShellDeformationElement model(interiorRestX, hasVtx, std::move(elasticModel), std::move(plasticModel),
    DeformationElementConstructionOptions{ true });

  const ES::VXd elasticParams = defaultShellElasticParams();
  const ES::VXd plasticParams = defaultShellPlasticParams();

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX.data(), 18, 0.1);
  ES::M18d hess;
  model.computeDisplacementHessian(
    x, constSpan(elasticParams), constSpan(plasticParams), hess);

  // Check symmetry.
  for (int i = 0; i < 18; i++)
    for (int j = 0; j < i; j++)
      EXPECT_NEAR(hess(i, j), hess(j, i), 1e-10)
        << "SPD hessian asymmetry at (" << i << ", " << j << ")";
}

TEST(KoiterShellGeometryTest, FundamentalFormJacobiansMatchFiniteDifference)
{
  const std::array<bool, 6> hasVtx = { true, true, true, true, true, true };
  KoiterShellDeformationElement model(
    interiorRestX, hasVtx,
    std::make_unique<ElasticModel2DFundamentalFormsSTVK>(),
    std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>());
  KoiterShellDeformationElement::APositions triangle{
    interiorRestX.segment<3>(0), interiorRestX.segment<3>(3), interiorRestX.segment<3>(6)
  };
  KoiterShellDeformationElement::BPositions positions{
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

  const ES::M4x9d da = model.computeFirstFundamentalFormDerivative(triangle);
  checkEntries(da, triangle, [&model](const KoiterShellDeformationElement::APositions &x) {
    return model.computeFirstFundamentalForm(x);
  });
  const ES::M9x36d d2a = model.computeFirstFundamentalFormHessian(triangle);
  for (int parameter = 0; parameter < 9; ++parameter) {
    auto plus = triangle;
    auto minus = triangle;
    plus[parameter / 3][parameter % 3] += 1e-6;
    minus[parameter / 3][parameter % 3] -= 1e-6;
    const ES::M4x9d fdJacobian =
      (model.computeFirstFundamentalFormDerivative(plus) -
        model.computeFirstFundamentalFormDerivative(minus)) / 2e-6;
    for (int entry = 0; entry < 4; ++entry)
      for (int dof = 0; dof < 9; ++dof) {
        const double analytic = d2a.block<9, 9>(0, 9 * entry)(dof, parameter);
        EXPECT_NEAR(fdJacobian(entry, dof), analytic, 1e-5);
      }
  }

  const ES::M4x18d db = model.computeSecondFundamentalFormDerivative(positions);
  const ES::M18x72d d2b = model.computeSecondFundamentalFormHessian(positions);
  const double eps = 1e-7;
  const std::array<int, 4> entryIndex = { 0, 2, 1, 3 };
  for (int dof = 0; dof < 18; ++dof) {
    auto plus = positions;
    auto minus = positions;
    plus[dof / 3][dof % 3] += eps;
    minus[dof / 3][dof % 3] -= eps;
    const ES::M2d fdForm =
      (model.computeSecondFundamentalForm(plus) -
        model.computeSecondFundamentalForm(minus)) / (2.0 * eps);
    for (int row = 0; row < 4; ++row)
      EXPECT_NEAR(fdForm.data()[entryIndex[row]], db(row, dof), 1e-5);
  }
  for (int parameter = 0; parameter < 18; ++parameter) {
    auto plus = positions;
    auto minus = positions;
    plus[parameter / 3][parameter % 3] += 1e-6;
    minus[parameter / 3][parameter % 3] -= 1e-6;
    const ES::M4x18d fdJacobian =
      (model.computeSecondFundamentalFormDerivative(plus) -
        model.computeSecondFundamentalFormDerivative(minus)) / 2e-6;
    for (int entry = 0; entry < 4; ++entry)
      for (int dof = 0; dof < 18; ++dof) {
        const double analytic = d2b.block<18, 18>(0, 18 * entry)(dof, parameter);
        EXPECT_NEAR(fdJacobian(entry, dof), analytic, 5e-4);
      }
  }
}
