#include "koiterShellDeformationElement.h"
#include "deformation/hessianProjection.h"

#include <stdexcept>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{
namespace
{
void validateMaterialLocation(int materialLocation)
{
  if (materialLocation > 0)
    throw std::out_of_range("Shell material location is out of range.");
}

std::span<const double> parameterView(const ES::VXd &parameters)
{
  return { parameters.data(), static_cast<std::size_t>(parameters.size()) };
}

KoiterShellDeformationElement::APositions trianglePositions(
  const std::array<ES::V3d, 6> &x)
{
  return { x[0], x[1], x[2] };
}
}  // namespace

KoiterShellDeformationElement::KoiterShellDeformationElement(
  const ES::V18d &restPositions, const std::array<bool, 6> &hasVertex,
  std::unique_ptr<ElasticModel2DFundamentalForms> elasticModel,
  std::unique_ptr<PlasticModel2DFundamentalForms> plasticModel,
  DeformationElementConstructionOptions options):
  DeformationElement(), hasVertex_(hasVertex),
  elastic2D_(std::move(elasticModel)), plastic2D_(std::move(plasticModel)),
  projectHessianPSD_(options.projectHessianPSD)
{
  if (!elastic2D_) {
    throw std::logic_error("KoiterShellDeformationElement requires non-null ElasticModel2DFundamentalForms");
  }
  if (!plastic2D_) {
    throw std::logic_error("KoiterShellDeformationElement requires non-null PlasticModel2DFundamentalForms");
  }

  for (int i = 0; i < numNodes; ++i)
    restPositions_[i] = restPositions.segment<3>(3 * i);
  const APositions restTriangle{
    restPositions_[0], restPositions_[1], restPositions_[2]
  };
  restFirstFundamentalForm_ =
    computeFirstFundamentalForm(restTriangle);
  restSecondFundamentalForm_ =
    computeSecondFundamentalForm(restPositions_);
  restArea_ = 0.5 *
    (restPositions_[1] - restPositions_[0])
      .cross(restPositions_[2] - restPositions_[0]).norm();

  plastic2D_->set_abar(restFirstFundamentalForm_);
  plastic2D_->set_bbar(restSecondFundamentalForm_);
  plastic2D_->setArea(restArea_);

  numPlasticParams_ = plastic2D_->getNumParameters();
  numElasticParams_ = elastic2D_->getNumParameters();
  cache_ = KoiterShellDeformationElementCache(numPlasticParams_, numElasticParams_);
}

double KoiterShellDeformationElement::computeEnergy(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  return computeEnergy(cache_);
}

void KoiterShellDeformationElement::computeDisplacementGradient(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, ES::RefVecXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  computeDisplacementGradient(cache_, output);
}

void KoiterShellDeformationElement::computeDisplacementHessian(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, ES::RefMatXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  computeDisplacementHessian(cache_, output);
}

double KoiterShellDeformationElement::computeEnergyGradient(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams,
  ES::RefVecXd displacementGradient) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  const double energy = computeEnergy(cache_);
  computeDisplacementGradient(cache_, displacementGradient);
  return energy;
}

double KoiterShellDeformationElement::computeEnergyGradientHessian(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams,
  ES::RefVecXd displacementGradient, ES::RefMatXd displacementHessian) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  const double energy = computeEnergy(cache_);
  computeDisplacementGradient(cache_, displacementGradient);
  computeDisplacementHessian(cache_, displacementHessian);
  return energy;
}

void KoiterShellDeformationElement::computeElasticGradient(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, ES::RefVecXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  output.setZero();
  computeElasticGradient(cache_, output, 0);
}

void KoiterShellDeformationElement::computePlasticGradient(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, ES::RefVecXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  output.setZero();
  computePlasticGradient(cache_, output, 0);
}

void KoiterShellDeformationElement::computeElasticVJP(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams,
  std::span<const double> displacementAdjoint, ES::RefVecXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  auto mixed = cache_.mixedDerivativeScratch.leftCols(numElasticParams_);
  computeDisplacementElasticHessian(cache_, mixed, 0);
  output.noalias() = mixed.transpose() *
    Eigen::Map<const ES::VXd>(displacementAdjoint.data(), getNumDOFs());
}

void KoiterShellDeformationElement::computePlasticVJP(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams,
  std::span<const double> displacementAdjoint, ES::RefVecXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  auto mixed = cache_.mixedDerivativeScratch.leftCols(numPlasticParams_);
  computeDisplacementPlasticHessian(cache_, mixed, 0);
  output.noalias() = mixed.transpose() *
    Eigen::Map<const ES::VXd>(displacementAdjoint.data(), getNumDOFs());
}

int KoiterShellDeformationElement::computeVonMisesStress(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, std::span<double> stresses) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  return computeVonMisesStress(
    cache_, stresses, static_cast<int>(stresses.size()));
}

void KoiterShellDeformationElement::prepareData(
  std::span<const double> x, std::span<const double> elasticParams, std::span<const double> plasticParams,
  KoiterShellDeformationElementCache &cacheDataBase) const
{
  if (x.size() != static_cast<std::size_t>(getNumDOFs()))
    throw std::invalid_argument(
      "Shell deformation local-position buffer has the wrong size.");
  if (elasticParams.size() != static_cast<std::size_t>(numElasticParams_))
    throw std::invalid_argument(
      "Elastic parameters are required by this shell deformation model.");
  if (plasticParams.size() != static_cast<std::size_t>(numPlasticParams_))
    throw std::invalid_argument(
      "Plastic parameters are required by this shell deformation model.");

  KoiterShellDeformationElementCache &cacheData = this->cacheData(cacheDataBase);
  cacheData.x[0] = ES::V3d(x[0], x[1], x[2]);
  cacheData.x[1] = ES::V3d(x[3], x[4], x[5]);
  cacheData.x[2] = ES::V3d(x[6], x[7], x[8]);
  cacheData.x[3] = ES::V3d(x[9], x[10], x[11]);
  cacheData.x[4] = ES::V3d(x[12], x[13], x[14]);
  cacheData.x[5] = ES::V3d(x[15], x[16], x[17]);

  if (numPlasticParams_ > 0) {
    cacheData.plasticParamsValue =
      Eigen::Map<const ES::VXd>(plasticParams.data(), numPlasticParams_);
  }

  if (numElasticParams_ > 0) {
    cacheData.elasticParamsValue =
      Eigen::Map<const ES::VXd>(elasticParams.data(), numElasticParams_);
  }

  const auto plasticParamView = parameterView(cacheData.plasticParamsValue);
  cacheData.abar = plastic2D_->compute_abar(plasticParamView);
  cacheData.bbar = plastic2D_->compute_bbar(plasticParamView);
  cacheData.area = plastic2D_->computeArea(plasticParamView);

  cacheData.a = computeFirstFundamentalForm(trianglePositions(cacheData.x));
  cacheData.b = computeSecondFundamentalForm(cacheData.x);
}

double KoiterShellDeformationElement::computeEnergy(const KoiterShellDeformationElementCache &cacheDataBase) const
{
  const KoiterShellDeformationElementCache &cacheData = this->cacheData(cacheDataBase);
  return computeEnergy(cacheData, parameterView(cacheData.plasticParamsValue), parameterView(cacheData.elasticParamsValue));
}

void KoiterShellDeformationElement::computeDisplacementGradient(const KoiterShellDeformationElementCache &cacheDataBase,
  ES::RefVecXd grad) const
{
  if (grad.size() != getNumDOFs())
    throw std::invalid_argument("Shell deformation gradient has unexpected size.");
  const KoiterShellDeformationElementCache &cacheData = this->cacheData(cacheDataBase);

  ES::M4x9d dadx;
  ES::M4x18d dbdx;

  dadx = computeFirstFundamentalFormDerivative(trianglePositions(cacheData.x));
  dbdx = computeSecondFundamentalFormDerivative(cacheData.x);

  const auto elasticParamView = parameterView(cacheData.elasticParamsValue);
  const ES::M2d dEda = elastic2D_->compute_dpsi_da(
    elasticParamView, cacheData.a, cacheData.abar);
  const ES::M2d dEdb = elastic2D_->compute_dpsi_db(
    elasticParamView, cacheData.b, cacheData.abar, cacheData.bbar);

  (ES::Mp<ES::V18d>(grad.data())).setZero();
  (ES::Mp<ES::V9d>(grad.data())) =
    dadx.transpose() * Eigen::Map<const ES::V4d>(dEda.data()) * cacheData.area;
  (ES::Mp<ES::V18d>(grad.data())) +=
    dbdx.transpose() * Eigen::Map<const ES::V4d>(dEdb.data()) * cacheData.area;
}

void KoiterShellDeformationElement::computeDisplacementHessian(const KoiterShellDeformationElementCache &cacheDataBase,
  ES::RefMatXd hess) const
{
  if (hess.rows() != getNumDOFs() || hess.cols() != getNumDOFs())
    throw std::invalid_argument("Shell deformation Hessian has unexpected shape.");
  const KoiterShellDeformationElementCache &cacheData = this->cacheData(cacheDataBase);

  ES::M4x9d dadx;
  ES::M4x18d dbdx;

  ES::M9x36d d2adx2;
  ES::M18x72d d2bdx2;

  dadx = computeFirstFundamentalFormDerivative(trianglePositions(cacheData.x));
  dbdx = computeSecondFundamentalFormDerivative(cacheData.x);
  d2adx2 = computeFirstFundamentalFormHessian(trianglePositions(cacheData.x));
  d2bdx2 = computeSecondFundamentalFormHessian(cacheData.x);

  const auto elasticParamView = parameterView(cacheData.elasticParamsValue);
  const ES::M2d dEda = elastic2D_->compute_dpsi_da(
    elasticParamView, cacheData.a, cacheData.abar);
  const ES::M2d dEdb = elastic2D_->compute_dpsi_db(
    elasticParamView, cacheData.b, cacheData.abar, cacheData.bbar);

  const ES::M4d d2Eda2 = elastic2D_->compute_d2psi_da2(
    elasticParamView, cacheData.a, cacheData.abar);
  const ES::M4d d2Edb2 = elastic2D_->compute_d2psi_db2(
    elasticParamView, cacheData.b, cacheData.abar, cacheData.bbar);

  ES::Mp<ES::M18d> hessMap(hess.data());
  hessMap.setZero();

  hessMap.block<9, 9>(0, 0) +=
    dadx.transpose() * d2Eda2 * dadx * cacheData.area;
  for (int j = 0; j < 4; j++) {
    hessMap.block<9, 9>(0, 0) +=
      dEda.data()[j] * d2adx2.block<9, 9>(0, 9 * j) * cacheData.area;
  }

  hessMap += dbdx.transpose() * d2Edb2 * dbdx * cacheData.area;
  for (int j = 0; j < 4; j++) {
    hessMap += dEdb.data()[j] * d2bdx2.block<18, 18>(0, 18 * j) * cacheData.area;
  }

  if (projectHessianPSD_)
    hessMap = projectSymmetricPSD(hessMap);
}

void KoiterShellDeformationElement::computeDisplacementPlasticHessian(const KoiterShellDeformationElementCache &cacheDataBase,
  ES::RefMatXd hess, int materialLocation) const
{
  validateMaterialLocation(materialLocation);
  const KoiterShellDeformationElementCache &cacheData = this->cacheData(cacheDataBase);
  if (numPlasticParams_ == 0)
    return;

  const auto elasticParamView = parameterView(cacheData.elasticParamsValue);
  const auto plasticParamView = parameterView(cacheData.plasticParamsValue);
  const ES::M2d dpsi_da = elastic2D_->compute_dpsi_da(elasticParamView, cacheData.a, cacheData.abar);
  const ES::M2d dpsi_db = elastic2D_->compute_dpsi_db(elasticParamView, cacheData.b, cacheData.abar, cacheData.bbar);

  const ES::M4d d2psi_da_dabar = elastic2D_->compute_d2psi_dadabar(
    elasticParamView, cacheData.a, cacheData.abar);
  const ES::M4d d2psi_db_dabar = elastic2D_->compute_d2psi_db_dabar(
    elasticParamView, cacheData.b, cacheData.abar, cacheData.bbar);
  const ES::M4d d2psi_db_dbbar = elastic2D_->compute_d2psi_db_dbbar(
    elasticParamView, cacheData.b, cacheData.abar, cacheData.bbar);

  ES::M4x9d dadx;
  ES::M4x18d dbdx;
  dadx = computeFirstFundamentalFormDerivative(trianglePositions(cacheData.x));
  dbdx = computeSecondFundamentalFormDerivative(cacheData.x);

  ES::MXd &dabar_dF = cacheData.plasticDAbarDparamScratch;
  ES::MXd &dbbar_dF = cacheData.plasticDBbarDparamScratch;
  ES::VXd &darea_dF = cacheData.plasticDAreaDparamScratch;
  dabar_dF.setZero();
  dbbar_dF.setZero();
  darea_dF.setZero();
  plastic2D_->compute_dabar_dparam(plasticParamView, dabar_dF);
  plastic2D_->compute_dbbar_dparam(plasticParamView, dbbar_dF);
  plastic2D_->compute_darea_dparam(plasticParamView, darea_dF);

  const ES::V9d dpsi_a_dx = Eigen::Map<const ES::V4d>(dpsi_da.data()).transpose() * dadx;
  const ES::V18d dpsi_b_dx = Eigen::Map<const ES::V4d>(dpsi_db.data()).transpose() * dbdx;

  int np = numPlasticParams_;
  ES::MXd &mixed = cacheData.mixedDerivativeScratch;
  mixed.block(0, 0, 18, np).setZero();

  mixed.block(0, 0, 9, np) +=
    dadx.transpose() * d2psi_da_dabar * dabar_dF * cacheData.area;
  mixed.block(0, 0, 9, np) +=
    dpsi_a_dx * darea_dF.transpose();

  mixed.block(0, 0, 18, np) +=
    dbdx.transpose() * d2psi_db_dabar * dabar_dF * cacheData.area;
  mixed.block(0, 0, 18, np) +=
    dbdx.transpose() * d2psi_db_dbbar * dbbar_dF * cacheData.area;
  mixed.block(0, 0, 18, np) +=
    dpsi_b_dx * darea_dF.transpose();

  ES::Mp<ES::MXd>(hess.data(), 18, np) = mixed.block(0, 0, 18, np);
}

void KoiterShellDeformationElement::computeDisplacementElasticHessian(const KoiterShellDeformationElementCache &cacheDataBase,
  ES::RefMatXd hess, int materialLocation) const
{
  validateMaterialLocation(materialLocation);
  const KoiterShellDeformationElementCache &cacheData = this->cacheData(cacheDataBase);
  if (numElasticParams_ == 0)
    return;

  ES::MXd &d2psi_da_dparam = cacheData.elasticDpsiDaDparamScratch;
  ES::MXd &d2psi_db_dparam = cacheData.elasticDpsiDbDparamScratch;
  d2psi_da_dparam.setZero();
  d2psi_db_dparam.setZero();
  const auto elasticParamView = parameterView(cacheData.elasticParamsValue);
  elastic2D_->compute_d2psi_da_dparam(
    elasticParamView, cacheData.a, cacheData.abar, d2psi_da_dparam);
  elastic2D_->compute_d2psi_db_dparam(
    elasticParamView, cacheData.b, cacheData.abar, cacheData.bbar, d2psi_db_dparam);

  ES::M4x9d dadx;
  ES::M4x18d dbdx;
  dadx = computeFirstFundamentalFormDerivative(trianglePositions(cacheData.x));
  dbdx = computeSecondFundamentalFormDerivative(cacheData.x);

  int np = numElasticParams_;
  ES::MXd &mixed = cacheData.mixedDerivativeScratch;
  mixed.block(0, 0, 18, np).setZero();

  mixed.block(0, 0, 9, np) +=
    dadx.transpose() * d2psi_da_dparam * cacheData.area;
  mixed.block(0, 0, 18, np) +=
    dbdx.transpose() * d2psi_db_dparam * cacheData.area;

  ES::Mp<ES::MXd>(hess.data(), 18, np) = mixed.block(0, 0, 18, np);
}

void KoiterShellDeformationElement::computePlasticGradient(const KoiterShellDeformationElementCache &cacheDataBase,
  ES::RefVecXd grad, int materialLocation) const
{
  validateMaterialLocation(materialLocation);
  const KoiterShellDeformationElementCache &cacheData = this->cacheData(cacheDataBase);
  if (numPlasticParams_ == 0)
    return;

  if (grad.size() != static_cast<Eigen::Index>(numPlasticParams_))
    throw std::invalid_argument("Shell plastic gradient has unexpected size.");
  grad.setZero();

  const auto plasticParamView = parameterView(cacheData.plasticParamsValue);
  const auto elasticParamView = parameterView(cacheData.elasticParamsValue);
  const double psi = computeEnergy(cacheData, plasticParamView, elasticParamView) / cacheData.area;

  const ES::M2d dpsiDabarMat = elastic2D_->compute_dpsi_dabar(
    elasticParamView, cacheData.a, cacheData.b, cacheData.abar, cacheData.bbar);
  const ES::M2d dpsiDbbarMat = elastic2D_->compute_dpsi_dbbar(
    elasticParamView, cacheData.a, cacheData.b, cacheData.abar, cacheData.bbar);
  const ES::V4d dpsiDabar = Eigen::Map<const ES::V4d>(dpsiDabarMat.data());
  const ES::V4d dpsiDbbar = Eigen::Map<const ES::V4d>(dpsiDbbarMat.data());

  plastic2D_->compute_dabar_dparam(plasticParamView, cacheData.plasticDAbarDparamScratch);
  plastic2D_->compute_dbbar_dparam(plasticParamView, cacheData.plasticDBbarDparamScratch);
  plastic2D_->compute_darea_dparam(plasticParamView, cacheData.plasticDAreaDparamScratch);

  const ES::MXd &dabarDp = cacheData.plasticDAbarDparamScratch;
  const ES::MXd &dbbarDp = cacheData.plasticDBbarDparamScratch;
  for (int i = 0; i < numPlasticParams_; i++) {
    grad[i] = cacheData.plasticDAreaDparamScratch[i] * psi +
      cacheData.area * (dpsiDabar.dot(dabarDp.col(i)) + dpsiDbbar.dot(dbbarDp.col(i)));
  }
}

void KoiterShellDeformationElement::computeElasticGradient(const KoiterShellDeformationElementCache &cacheDataBase,
  ES::RefVecXd grad, int materialLocation) const
{
  validateMaterialLocation(materialLocation);
  const KoiterShellDeformationElementCache &cacheData = this->cacheData(cacheDataBase);
  if (numElasticParams_ == 0)
    return;

  if (grad.size() != static_cast<Eigen::Index>(numElasticParams_))
    throw std::invalid_argument("Shell elastic gradient has unexpected size.");
  grad.setZero();
  const auto elasticParamView = parameterView(cacheData.elasticParamsValue);
  elastic2D_->compute_dpsi_dparam(
    elasticParamView, cacheData.a, cacheData.b,
    cacheData.abar, cacheData.bbar, grad);
  grad *= cacheData.area;
}

int KoiterShellDeformationElement::computeVonMisesStress(
  const KoiterShellDeformationElementCache &cacheDataBase,
  std::span<double> stresses, int capacity) const
{
  if (capacity < 1)
    throw std::length_error(
      "Shell von Mises stress output capacity is too small.");
  if (stresses.size() < 1)
    throw std::invalid_argument(
      "Shell von Mises stress output must not be null.");
  if (numElasticParams_ == 0)
    throw UnsupportedDeformationDiagnosticError(
      "Von Mises stress requires shell elastic parameters.");

  const KoiterShellDeformationElementCache &cd = cacheData(cacheDataBase);
  double value = 0.0;
  const auto elasticParamView = parameterView(cd.elasticParamsValue);
  bool ok = elastic2D_->computeVonMisesStress(
    elasticParamView, cd.a, cd.b, cd.abar, cd.bbar,
    value);

  if (!ok) {
    throw UnsupportedDeformationDiagnosticError(
      "Von Mises stress is not implemented by this shell elastic model.");
  }

  stresses[0] = value;
  return 1;
}

const KoiterShellDeformationElementCache &KoiterShellDeformationElement::cacheData(
  const KoiterShellDeformationElementCache &cacheDataBase) const
{
  return cacheDataBase;
}

KoiterShellDeformationElementCache &KoiterShellDeformationElement::cacheData(
  KoiterShellDeformationElementCache &cacheDataBase) const
{
  return cacheDataBase;
}

double KoiterShellDeformationElement::computeEnergy(
  const KoiterShellDeformationElementCache &cacheData, std::span<const double> plasticParams, std::span<const double> elasticParams) const
{
  ES::M2d abar;
  ES::M2d bbar;
  abar = plastic2D_->compute_abar(plasticParams);
  bbar = plastic2D_->compute_bbar(plasticParams);
  const double area = plastic2D_->computeArea(plasticParams);

  const double E1 = elastic2D_->compute_psi_a(
    elasticParams, cacheData.a, abar);
  const double E2 = elastic2D_->compute_psi_b(
    elasticParams, cacheData.b, abar, bbar);

  return (E1 + E2) * area;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
