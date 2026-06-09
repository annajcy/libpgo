#include "gtest/gtest.h"

#include "material/elastic/elasticModel2DFundamentalFormsFabric.h"
#include "material/elastic/elasticModel2DFundamentalFormsSTVK.h"
#include "material/fields/constantParameterField.h"
#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"

#include "deformation/shell/shellDeformationModelCacheData.h"
#include "deformation/shell/shellDeformationModel.h"
#include "deformation/shell/koiterShellElementMapping.h"

#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <utility>

namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

namespace
{

// Interior triangle: all 6 nodes present, slightly curved out of plane.
const double interiorRestX[18] = {
  0.0, 0.0, 0.0,
  2.0, 0.0, 0.5,
  1.0, 1.5, -0.3,
  -0.5, -0.2, 1.0,  // node 3
  2.5, -0.1, 0.8,   // node 4
  1.2, 2.0, 1.2,    // node 5
};

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

class ScaledConstantParameterField : public OptimizableField
{
public:
  ScaledConstantParameterField(ParameterDomain domain, ES::VXd values, ES::VXd scales):
    values_(std::move(values)),
    scales_(std::move(scales)),
    dofLayout_(static_cast<int>(values_.size()))
  {
    spec_.domain = domain;
    spec_.numChannels = static_cast<int>(values_.size());
  }

  const ParameterFieldSpec &spec() const override { return spec_; }
  ParameterFieldKind kind() const override { return ParameterFieldKind::EXTERNAL_PROCEDURAL; }
  int numChannels() const override { return spec_.numChannels; }
  int numLocalDofs() const override { return spec_.numChannels; }

  void computeValue(int ele, int quadratureId, double *out) const override
  {
    (void)ele;
    (void)quadratureId;
    Eigen::Map<ES::VXd>(out, spec_.numChannels) = scales_.cwiseProduct(values_);
  }

  void setGlobalData(const double *data) override
  {
    values_ = Eigen::Map<const ES::VXd>(data, spec_.numChannels);
  }

  const ParameterDofLayout *dofLayout() const override { return &dofLayout_; }
  const double *globalData() const override { return values_.data(); }
  int numValueRows() const override { return spec_.numChannels == 0 ? 0 : 1; }

  void computeDerivative(int ele, int quadratureId, double *derivOut) const override
  {
    (void)ele;
    (void)quadratureId;
    Eigen::Map<ES::MXd> deriv(derivOut, spec_.numChannels, spec_.numChannels);
    deriv.setZero();
    deriv.diagonal() = scales_;
  }

private:
  class Layout : public OptimizableField::ParameterDofLayout
  {
  public:
    explicit Layout(int n): n_(n) {}
    int numGlobalDofs() const override { return n_; }
    int numLocalDofs() const override { return n_; }
    bool matchesParameterShape(int numChannels, int numElements) const override
    {
      (void)numElements;
      return numChannels == n_;
    }
    int globalDof(int ele, int localDof) const override
    {
      (void)ele;
      return localDof;
    }
    void gather(int ele, const double *global, double *local) const override
    {
      (void)ele;
      std::copy(global, global + n_, local);
    }

  private:
    int n_ = 0;
  };

  ParameterFieldSpec spec_;
  ES::VXd values_;
  ES::VXd scales_;
  Layout dofLayout_;
};

}  // namespace

// ============================================================
// Interior triangle: energy finite at rest and gradient FD check
// ============================================================

TEST(ShellDeformationModelTest, InteriorEnergyFiniteAtRest)
{
  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const bool hasVtx[6] = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  auto cd = model.allocateCacheData();
  model.prepareData(interiorRestX, cd.get());

  double energy = model.computeEnergy(cd.get());
  EXPECT_TRUE(std::isfinite(energy));

  ES::V18d grad;
  model.compute_dE_dx(cd.get(), grad.data());
  for (int i = 0; i < 18; i++)
    EXPECT_TRUE(std::isfinite(grad[i]));
}

TEST(ShellDeformationModelTest, DefaultPlasticParametersInitializeRestMetric)
{
  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const bool hasVtx[6] = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  const ES::M2d restI = mapping->restI();
  const ES::M2d restII = mapping->restII();
  const double restArea = mapping->restArea();
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  auto cd = model.allocateCacheData();
  model.prepareData(interiorRestX, cd.get());

  const auto *shellCache = static_cast<const ShellDeformationModelCacheData *>(cd.get());
  ASSERT_EQ(shellCache->numPlasticParams, 1);
  EXPECT_NEAR(shellCache->plasticParamsValue[0], 1.0, 1e-12);
  EXPECT_TRUE(shellCache->abar.isApprox(restI, 1e-12));
  EXPECT_TRUE(shellCache->bbar.isApprox(restII, 1e-12));
  EXPECT_NEAR(shellCache->area, restArea, 1e-12);
}

// ============================================================
// Boundary triangle (node 4 missing): energy finite at rest
// ============================================================

TEST(ShellDeformationModelTest, BoundaryMissingNode4EnergyFinite)
{
  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const bool hasVtx[6] = { true, true, true, true, false, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  auto cd = model.allocateCacheData();
  model.prepareData(interiorRestX, cd.get());

  double energy = model.computeEnergy(cd.get());
  EXPECT_TRUE(std::isfinite(energy));

  ES::V18d grad;
  model.compute_dE_dx(cd.get(), grad.data());
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
  const bool hasVtx[6] = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  auto cd = model.allocateCacheData();

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX, 18, 0.1);
  model.prepareData(x, cd.get());
  double e0 = model.computeEnergy(cd.get());

  ES::V18d g;
  model.compute_dE_dx(cd.get(), g.data());

  const double eps = 1e-6;
  for (int i = 0; i < 18; i++) {
    double xPlus[18];
    std::copy(x, x + 18, xPlus);
    xPlus[i] += eps;

    auto cdP = model.allocateCacheData();
    model.prepareData(xPlus, cdP.get());
    double ePlus = model.computeEnergy(cdP.get());

    double fdGrad = (ePlus - e0) / eps;
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
  const bool hasVtx[6] = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX, 18, 0.1);

  auto energyAt = [&](double s) {
    ES::VXd params = plasticParams;
    params[0] = s;
    auto cd = model.allocateCacheData();
    model.prepareData(x, elasticParams.data(), params.data(), cd.get());
    return model.computeEnergy(cd.get());
  };

  plasticParams[0] = 1.2;
  auto cd = model.allocateCacheData();
  model.prepareData(x, elasticParams.data(), plasticParams.data(), cd.get());

  double analytic[1] = {};
  model.compute_dE_da(cd.get(), analytic);

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
  const bool hasVtx[6] = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX, 18, 0.1);

  auto energyAt = [&](int channel, double value) {
    ES::VXd params = elasticParams;
    params[channel] = value;
    auto cd = model.allocateCacheData();
    model.prepareData(x, params.data(), plasticParams.data(), cd.get());
    return model.computeEnergy(cd.get());
  };

  auto cd = model.allocateCacheData();
  model.prepareData(x, elasticParams.data(), plasticParams.data(), cd.get());

  ES::VXd analytic(5);
  model.compute_dE_db(cd.get(), analytic.data());

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
    return elastic.compute_psi_a(p.data(), a.data(), A.data()) +
      elastic.compute_psi_b(p.data(), b.data(), A.data(), B.data());
  };

  ES::V4d dpsiDabar;
  ES::V4d dpsiDbbar;
  ES::VXd dpsiDparam(params.size());
  elastic.compute_dpsi_dabar(params.data(), a.data(), b.data(), abar.data(), bbar.data(), dpsiDabar.data());
  elastic.compute_dpsi_dbbar(params.data(), a.data(), b.data(), abar.data(), bbar.data(), dpsiDbbar.data());
  elastic.compute_dpsi_dparam(params.data(), a.data(), b.data(), abar.data(), bbar.data(), dpsiDparam.data());

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
  const bool hasVtx[6] = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX, 18, 0.1);

  auto plasticGradientAt = [&](const ES::VXd &elasticValues, const ES::VXd &plasticValues) {
    auto cd = model.allocateCacheData();
    model.prepareData(x, elasticValues.data(), plasticValues.data(), cd.get());
    ES::VXd grad(1);
    model.compute_dE_da(cd.get(), grad.data());
    return grad;
  };

  auto elasticGradientAt = [&](const ES::VXd &elasticValues, const ES::VXd &plasticValues) {
    auto cd = model.allocateCacheData();
    model.prepareData(x, elasticValues.data(), plasticValues.data(), cd.get());
    ES::VXd grad(5);
    model.compute_dE_db(cd.get(), grad.data());
    return grad;
  };

  auto cd = model.allocateCacheData();
  model.prepareData(x, elasticParams.data(), plasticParams.data(), cd.get());

  ES::MXd d2daa(1, 1);
  model.compute_d2E_da2(cd.get(), d2daa.data());
  const double plasticStep = 1e-6 * std::max(1.0, std::abs(plasticParams[0]));
  ES::VXd plasticPlus = plasticParams;
  ES::VXd plasticMinus = plasticParams;
  plasticPlus[0] += plasticStep;
  plasticMinus[0] -= plasticStep;
  const ES::VXd gPlasticPlus = plasticGradientAt(elasticParams, plasticPlus);
  const ES::VXd gPlasticMinus = plasticGradientAt(elasticParams, plasticMinus);
  const double fdPlasticHessian = (gPlasticPlus[0] - gPlasticMinus[0]) / (2.0 * plasticStep);
  EXPECT_NEAR(d2daa(0, 0), fdPlasticHessian,
    2e-5 * std::max(1.0, std::abs(fdPlasticHessian)));

  ES::MXd d2dbb(5, 5);
  model.compute_d2E_db2(cd.get(), d2dbb.data());
  for (int col = 0; col < elasticParams.size(); col++) {
    const double step = 1e-6 * std::max(1.0, std::abs(elasticParams[col]));
    ES::VXd elasticPlus = elasticParams;
    ES::VXd elasticMinus = elasticParams;
    elasticPlus[col] += step;
    elasticMinus[col] -= step;
    const ES::VXd gPlus = elasticGradientAt(elasticPlus, plasticParams);
    const ES::VXd gMinus = elasticGradientAt(elasticMinus, plasticParams);
    const ES::VXd fdCol = (gPlus - gMinus) / (2.0 * step);
    for (int row = 0; row < elasticParams.size(); row++) {
      EXPECT_NEAR(d2dbb(row, col), fdCol[row],
        2e-5 * std::max(1.0, std::abs(fdCol[row])))
        << "elastic Hessian (" << row << ", " << col << ")";
    }
  }

  ES::MXd d2dadb(1, 5);
  model.compute_d2E_dadb(cd.get(), d2dadb.data());
  for (int col = 0; col < elasticParams.size(); col++) {
    const double step = 1e-6 * std::max(1.0, std::abs(elasticParams[col]));
    ES::VXd elasticPlus = elasticParams;
    ES::VXd elasticMinus = elasticParams;
    elasticPlus[col] += step;
    elasticMinus[col] -= step;
    const ES::VXd gPlus = plasticGradientAt(elasticPlus, plasticParams);
    const ES::VXd gMinus = plasticGradientAt(elasticMinus, plasticParams);
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
  const bool hasVtx[6] = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX, 18, 0.1);
  auto cd = model.allocateCacheData();
  model.prepareData(x, elasticParams.data(), plasticParams.data(), cd.get());

  ES::VXd grad(elasticParams.size());
  EXPECT_THROW(model.compute_dE_db(cd.get(), grad.data()), std::logic_error);
}

TEST(ShellDeformationModelFDTest, ScaledParameterFieldLeavesModelDerivativeRaw)
{
  ES::VXd elasticScales(5);
  elasticScales << 1.2, 0.75, 1.4, 0.6, 1.8;
  ES::VXd elasticGlobal = defaultShellElasticParams().cwiseQuotient(elasticScales);
  ScaledConstantParameterField elasticField(ParameterDomain::ELASTIC, elasticGlobal, elasticScales);

  ES::VXd plasticScales(1);
  plasticScales << 1.5;
  ES::VXd plasticGlobal(1);
  plasticGlobal << 0.8;
  ScaledConstantParameterField plasticField(ParameterDomain::PLASTIC, plasticGlobal, plasticScales);

  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const bool hasVtx[6] = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX, 18, 0.1);

  auto energyAt = [&](const ES::VXd &elasticValues, const ES::VXd &plasticValues) {
    elasticField.setGlobalData(elasticValues.data());
    plasticField.setGlobalData(plasticValues.data());
    ES::VXd elasticMaterial(5);
    ES::VXd plasticMaterial(1);
    elasticField.computeValue(0, 0, elasticMaterial.data());
    plasticField.computeValue(0, 0, plasticMaterial.data());
    auto cd = model.allocateCacheData();
    model.prepareData(x, elasticMaterial.data(), plasticMaterial.data(), cd.get());
    return model.computeEnergy(cd.get());
  };

  ES::VXd elasticMaterial(5);
  ES::VXd plasticMaterial(1);
  elasticField.computeValue(0, 0, elasticMaterial.data());
  plasticField.computeValue(0, 0, plasticMaterial.data());
  auto cd = model.allocateCacheData();
  model.prepareData(x, elasticMaterial.data(), plasticMaterial.data(), cd.get());

  ES::VXd analyticPlastic(1);
  model.compute_dE_da(cd.get(), analyticPlastic.data());
  const double plasticStep = 1e-6 * std::max(1.0, std::abs(plasticGlobal[0]));
  ES::VXd plasticPlus = plasticGlobal;
  ES::VXd plasticMinus = plasticGlobal;
  plasticPlus[0] += plasticStep;
  plasticMinus[0] -= plasticStep;
  const double plasticFD =
    (energyAt(elasticGlobal, plasticPlus) - energyAt(elasticGlobal, plasticMinus)) / (2.0 * plasticStep);
  EXPECT_NEAR(analyticPlastic[0], plasticFD / plasticScales[0],
    1e-5 * std::max(1.0, std::abs(plasticFD / plasticScales[0])));

  elasticField.setGlobalData(elasticGlobal.data());
  plasticField.setGlobalData(plasticGlobal.data());
  elasticField.computeValue(0, 0, elasticMaterial.data());
  plasticField.computeValue(0, 0, plasticMaterial.data());
  cd = model.allocateCacheData();
  model.prepareData(x, elasticMaterial.data(), plasticMaterial.data(), cd.get());

  ES::VXd analyticElastic(5);
  model.compute_dE_db(cd.get(), analyticElastic.data());
  for (int c = 0; c < elasticGlobal.size(); c++) {
    const double step = 1e-6 * std::max(1.0, std::abs(elasticGlobal[c]));
    ES::VXd elasticPlus = elasticGlobal;
    ES::VXd elasticMinus = elasticGlobal;
    elasticPlus[c] += step;
    elasticMinus[c] -= step;
    const double fd =
      (energyAt(elasticPlus, plasticGlobal) - energyAt(elasticMinus, plasticGlobal)) / (2.0 * step);
    EXPECT_NEAR(analyticElastic[c], fd / elasticScales[c],
      1e-5 * std::max(1.0, std::abs(fd / elasticScales[c])))
      << "scaled elastic parameter channel " << c;
  }
}

// ============================================================
// SPD enable produces symmetric PSD hessian
// ============================================================

TEST(ShellDeformationModelTest, SPDEnableProducesSymmetricPSD)
{
  auto elasticModel = std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
  auto plasticModel = std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  const bool hasVtx[6] = { true, true, true, true, true, true };

  auto mapping = std::make_unique<KoiterShellElementMapping>(interiorRestX, hasVtx);
  ShellDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  auto cd = model.allocateCacheData();

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX, 18, 0.1);
  model.prepareData(x, cd.get());

  model.enableSPD(1);
  ES::M18d hess;
  model.compute_d2E_dx2(cd.get(), hess.data());

  // Check symmetry.
  for (int i = 0; i < 18; i++)
    for (int j = 0; j < i; j++)
      EXPECT_NEAR(hess(i, j), hess(j, i), 1e-10)
        << "SPD hessian asymmetry at (" << i << ", " << j << ")";
}
