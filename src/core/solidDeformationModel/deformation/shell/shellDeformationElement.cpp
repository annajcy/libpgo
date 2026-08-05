#include "shellDeformationElement.h"
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

ShellElementMapping::APositions trianglePositions(const std::array<ES::V3d, 6> &x)
{
  return { x[0], x[1], x[2] };
}
}  // namespace

ShellDeformationElement::ShellDeformationElement(
  std::unique_ptr<ShellElementMapping> mapping,
  std::unique_ptr<ElasticModel2DFundamentalForms> elasticModel,
  std::unique_ptr<PlasticModel2DFundamentalForms> plasticModel,
  DeformationElementConstructionOptions options): DeformationElement(), elementMapping_(std::move(mapping)), elastic2D_(std::move(elasticModel)), plastic2D_(std::move(plasticModel)), projectHessianPSD_(options.projectHessianPSD)
{
  if (!elementMapping_) {
    throw std::logic_error("ShellDeformationElement requires non-null shell mapping");
  }
  if (!elastic2D_) {
    throw std::logic_error("ShellDeformationElement requires non-null ElasticModel2DFundamentalForms");
  }
  if (!plastic2D_) {
    throw std::logic_error("ShellDeformationElement requires non-null PlasticModel2DFundamentalForms");
  }

  plastic2D_->set_abar(elementMapping_->restI());
  plastic2D_->set_bbar(elementMapping_->restII());
  plastic2D_->setArea(elementMapping_->restArea());

  numPlasticParams_ = plastic2D_->getNumParameters();
  numElasticParams_ = elastic2D_->getNumParameters();
  cache_ = ShellDeformationElementCache(numPlasticParams_, numElasticParams_);
}

double ShellDeformationElement::computeEnergy(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  return compute_E(cache_);
}

void ShellDeformationElement::computeDisplacementGradient(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, ES::RefVecXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  compute_dE_dx(cache_, output);
}

void ShellDeformationElement::computeDisplacementHessian(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, ES::RefMatXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  compute_d2E_dx2(cache_, output);
}

void ShellDeformationElement::computeElasticGradient(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, ES::RefVecXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  output.setZero();
  compute_dE_de(cache_, output, 0);
}

void ShellDeformationElement::computePlasticGradient(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, ES::RefVecXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  output.setZero();
  compute_dE_dp(cache_, output, 0);
}

void ShellDeformationElement::computeElasticVJP(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams,
  std::span<const double> displacementAdjoint, ES::RefVecXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  auto mixed = cache_.mixedDerivativeScratch.leftCols(numElasticParams_);
  compute_d2E_dude(cache_, mixed, 0);
  output.noalias() = mixed.transpose() *
    Eigen::Map<const ES::VXd>(displacementAdjoint.data(), getNumDOFs());
}

void ShellDeformationElement::computePlasticVJP(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams,
  std::span<const double> displacementAdjoint, ES::RefVecXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  auto mixed = cache_.mixedDerivativeScratch.leftCols(numPlasticParams_);
  compute_d2E_dudp(cache_, mixed, 0);
  output.noalias() = mixed.transpose() *
    Eigen::Map<const ES::VXd>(displacementAdjoint.data(), getNumDOFs());
}

int ShellDeformationElement::computeVonMisesStress(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, std::span<double> stresses) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  return computeVonMisesStress(
    cache_, stresses, static_cast<int>(stresses.size()));
}

void ShellDeformationElement::prepareData(
  std::span<const double> x, std::span<const double> elasticParams, std::span<const double> plasticParams,
  ShellDeformationElementCache &cacheDataBase) const
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

  ShellDeformationElementCache &cacheData = this->cacheData(cacheDataBase);
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

  cacheData.a = elementMapping_->compute_a(trianglePositions(cacheData.x));
  cacheData.b = elementMapping_->compute_b(cacheData.x);
}

double ShellDeformationElement::compute_E(const ShellDeformationElementCache &cacheDataBase) const
{
  const ShellDeformationElementCache &cacheData = this->cacheData(cacheDataBase);
  return compute_E(cacheData, parameterView(cacheData.plasticParamsValue), parameterView(cacheData.elasticParamsValue));
}

void ShellDeformationElement::compute_dE_dx(const ShellDeformationElementCache &cacheDataBase,
  ES::RefVecXd grad) const
{
  if (grad.size() != getNumDOFs())
    throw std::invalid_argument("Shell deformation gradient has unexpected size.");
  const ShellDeformationElementCache &cacheData = this->cacheData(cacheDataBase);

  ES::M4x9d dadx;
  ES::M4x18d dbdx;

  dadx = elementMapping_->compute_da_dx(trianglePositions(cacheData.x));
  dbdx = elementMapping_->compute_db_dx(cacheData.x);

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

void ShellDeformationElement::compute_d2E_dx2(const ShellDeformationElementCache &cacheDataBase,
  ES::RefMatXd hess) const
{
  if (hess.rows() != getNumDOFs() || hess.cols() != getNumDOFs())
    throw std::invalid_argument("Shell deformation Hessian has unexpected shape.");
  const ShellDeformationElementCache &cacheData = this->cacheData(cacheDataBase);

  ES::M4x9d dadx;
  ES::M4x18d dbdx;

  ES::M9x36d d2adx2;
  ES::M18x72d d2bdx2;

  dadx = elementMapping_->compute_da_dx(trianglePositions(cacheData.x));
  dbdx = elementMapping_->compute_db_dx(cacheData.x);
  d2adx2 = elementMapping_->compute_d2a_dx2(trianglePositions(cacheData.x));
  d2bdx2 = elementMapping_->compute_d2b_dx2(cacheData.x);

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

void ShellDeformationElement::compute_d2E_dudp(const ShellDeformationElementCache &cacheDataBase,
  ES::RefMatXd hess, int materialLocation) const
{
  validateMaterialLocation(materialLocation);
  const ShellDeformationElementCache &cacheData = this->cacheData(cacheDataBase);
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
  dadx = elementMapping_->compute_da_dx(trianglePositions(cacheData.x));
  dbdx = elementMapping_->compute_db_dx(cacheData.x);

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

void ShellDeformationElement::compute_d2E_dude(const ShellDeformationElementCache &cacheDataBase,
  ES::RefMatXd hess, int materialLocation) const
{
  validateMaterialLocation(materialLocation);
  const ShellDeformationElementCache &cacheData = this->cacheData(cacheDataBase);
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
  dadx = elementMapping_->compute_da_dx(trianglePositions(cacheData.x));
  dbdx = elementMapping_->compute_db_dx(cacheData.x);

  int np = numElasticParams_;
  ES::MXd &mixed = cacheData.mixedDerivativeScratch;
  mixed.block(0, 0, 18, np).setZero();

  mixed.block(0, 0, 9, np) +=
    dadx.transpose() * d2psi_da_dparam * cacheData.area;
  mixed.block(0, 0, 18, np) +=
    dbdx.transpose() * d2psi_db_dparam * cacheData.area;

  ES::Mp<ES::MXd>(hess.data(), 18, np) = mixed.block(0, 0, 18, np);
}

void ShellDeformationElement::compute_dE_dp(const ShellDeformationElementCache &cacheDataBase,
  ES::RefVecXd grad, int materialLocation) const
{
  validateMaterialLocation(materialLocation);
  const ShellDeformationElementCache &cacheData = this->cacheData(cacheDataBase);
  if (numPlasticParams_ == 0)
    return;

  if (grad.size() != static_cast<Eigen::Index>(numPlasticParams_))
    throw std::invalid_argument("Shell plastic gradient has unexpected size.");
  grad.setZero();

  const auto plasticParamView = parameterView(cacheData.plasticParamsValue);
  const auto elasticParamView = parameterView(cacheData.elasticParamsValue);
  const double psi = compute_E(cacheData, plasticParamView, elasticParamView) / cacheData.area;

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

void ShellDeformationElement::compute_dE_de(const ShellDeformationElementCache &cacheDataBase,
  ES::RefVecXd grad, int materialLocation) const
{
  validateMaterialLocation(materialLocation);
  const ShellDeformationElementCache &cacheData = this->cacheData(cacheDataBase);
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

int ShellDeformationElement::computeVonMisesStress(
  const ShellDeformationElementCache &cacheDataBase,
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

  const ShellDeformationElementCache &cd = cacheData(cacheDataBase);
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

int ShellDeformationElement::getNumVertices() const
{
  return elementMapping_->getNumNodes();
}

int ShellDeformationElement::getNumDOFs() const
{
  return elementMapping_->getLocalDofs();
}

DeformationElement::LocalMaxStepResult ShellDeformationElement::computeLocalMaxStepSize(
  std::span<const double> x_local, std::span<const double> dx_local) const
{
  (void)x_local;
  (void)dx_local;
  return LocalMaxStepResult{};
}

const ShellDeformationElementCache &ShellDeformationElement::cacheData(
  const ShellDeformationElementCache &cacheDataBase) const
{
  return cacheDataBase;
}

ShellDeformationElementCache &ShellDeformationElement::cacheData(
  ShellDeformationElementCache &cacheDataBase) const
{
  return cacheDataBase;
}

double ShellDeformationElement::compute_E(
  const ShellDeformationElementCache &cacheData, std::span<const double> plasticParams, std::span<const double> elasticParams) const
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
