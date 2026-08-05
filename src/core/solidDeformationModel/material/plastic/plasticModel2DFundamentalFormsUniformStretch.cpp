#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"
#include <array>

#include "EigenSupport.h"

pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::PlasticModel2DFundamentalFormsUniformStretch()
{
}

pgo::EigenSupport::M2d pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_abar(std::span<const double> params) const
{
  double s = params[0];
  EigenSupport::M2d S;
  S << s, 0, 0, s;

  return S * abar * S;
}

pgo::EigenSupport::M2d pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_Fp(std::span<const double> params) const
{
  return EigenSupport::M2d::Identity() * params[0];
}

pgo::EigenSupport::M2d pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_bbar(std::span<const double> params) const
{
  (void)params;
  return bbar;
}

pgo::EigenSupport::M3d pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_tbar(std::span<const double> params) const
{
  double s = params[0];
  EigenSupport::M2d S;
  S << s, 0, 0, s;

  EigenSupport::M3d result = tbar;
  result.leftCols<2>() *= S;
  return result;
}

double pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::computeArea(std::span<const double> params) const
{
  return areaRest * params[0] * params[0];
}

pgo::EigenSupport::M3d pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_dtbar_inv_dparam(std::span<const double> params, int j) const
{
  EigenSupport::M3d S;
  S << 1, 0, 0,
    0, 1, 0,
    0, 0, 0;

  (void)params; (void)j;
  return tbar * S;
}

pgo::EigenSupport::M3d pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_dqbar_dparam(std::span<const double> params, int j) const
{
  (void)params; (void)j;
  return EigenSupport::M3d::Zero();
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_dK_dparam(std::span<const double> params, EigenSupport::RefVecXd dK_da) const
{
  // K = det(L) = det(I_inv * II) = det(I_inv) * det(II) = det(S^-1 * Ibar^-1 * S^-1) * det(IIbar) = (1/s^4) * det(Ibar^-1) * det(IIbar)
  // H = trace(L) / 2

  double detAbar = abar.determinant();
  double detBbar = bbar.determinant();
  double s = params[0];

  double dK_ds = -4.0 * (1.0 / (s * s * s * s * s)) / detAbar * detBbar;
  dK_da[0] = dK_ds;
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_dH_dparam(std::span<const double> params, EigenSupport::RefVecXd dH_da) const
{
  // K = det(L) = det(I_inv * II) = det(I_inv) * det(II) = det(S^-1 * Ibar^-1 * S^-1) * det(IIbar) = (1/s^4) * det(Ibar^-1) * det(IIbar)
  // H = trace(L) / 2

  double s = params[0];
  EigenSupport::M2d I_inv = abar.inverse();
  EigenSupport::M2d II = bbar;
  EigenSupport::M2d L = I_inv * II;
  double dH_ds = -2.0 / (s * s * s) * L.trace() * 0.5;
  dH_da[0] = dH_ds;
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_darea_dparam(std::span<const double> params, EigenSupport::RefVecXd darea_da) const
{
  double s = params[0];
  double dA_ds = 2.0 * areaRest * s;
  darea_da[0] = dA_ds;
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_dabar_dparam(std::span<const double> params, EigenSupport::RefMatXd dabar_dparam) const
{
  double s = params[0];
  EigenSupport::M2d S;
  S << s, 0, 0, s;

  EigenSupport::M2d dS_ds;
  dS_ds << 1, 0, 0, 1;

  EigenSupport::M2d dAbar_ds = dS_ds * abar * S + S * abar * dS_ds;

  dabar_dparam.setZero();
  dabar_dparam.col(0) = Eigen::Map<const EigenSupport::V4d>(dAbar_ds.data());
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_dbbar_dparam(std::span<const double> params, EigenSupport::RefMatXd dbbar_dparam) const
{
  (void)params;
  dbbar_dparam.setZero();
}


#include <stdexcept>

namespace pgo::SolidDeformationModel {
int ShellPlasticity1Definition::numOptimizableChannels() const { return 1; }
std::unique_ptr<PlasticModel> ShellPlasticity1Definition::createModel(std::span<const double> values, const MaterialFrame &) const
{
  if (!values.empty()) throw std::invalid_argument("shell_ff_dof1 has no fixed channels");
  return std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
}
}  // namespace pgo::SolidDeformationModel
