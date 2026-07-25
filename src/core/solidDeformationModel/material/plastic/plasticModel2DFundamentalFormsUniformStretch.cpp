#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"
#include <array>

#include "EigenSupport.h"

pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::PlasticModel2DFundamentalFormsUniformStretch()
{
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_abar(const double *params, double *a) const
{
  double s = params[0];
  EigenSupport::M2d S;
  S << s, 0, 0, s;

  (EigenSupport::Mp<EigenSupport::M2d>(a)) = S * abar * S;
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_Fp(const double *params, double *Fp) const
{
  Fp[0] = params[0];
  Fp[1] = 0.0;
  Fp[2] = 0.0;
  Fp[3] = params[0];
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_bbar(const double *params, double *b) const
{
  (EigenSupport::Mp<EigenSupport::M2d>(b)) = bbar;
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_tbar(const double *params, double *t) const
{
  double s = params[0];
  EigenSupport::M2d S;
  S << s, 0, 0, s;

  (EigenSupport::Mp<EigenSupport::M3d>(t)) = tbar;
  (EigenSupport::Mp<EigenSupport::M3d>(t)).leftCols<2>() *= S;
}

double pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::computeArea(const double *params) const
{
  return areaRest * params[0] * params[0];
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_dtbar_inv_dparam(const double *params, int j, double *dtbar_da) const
{
  EigenSupport::M3d S;
  S << 1, 0, 0,
    0, 1, 0,
    0, 0, 0;

  (EigenSupport::Mp<EigenSupport::M3d>(dtbar_da)) = tbar * S;
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_dqbar_dparam(const double *params, int j, double *dqbar_da) const
{
  EigenSupport::M3d zero = EigenSupport::M3d::Zero();
  (EigenSupport::Mp<EigenSupport::M3d>(dqbar_da)) = zero;
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_dK_dparam(const double *params, double *dK_da) const
{
  // K = det(L) = det(I_inv * II) = det(I_inv) * det(II) = det(S^-1 * Ibar^-1 * S^-1) * det(IIbar) = (1/s^4) * det(Ibar^-1) * det(IIbar)
  // H = trace(L) / 2

  double detAbar = abar.determinant();
  double detBbar = bbar.determinant();
  double s = params[0];

  double dK_ds = -4.0 * (1.0 / (s * s * s * s * s)) / detAbar * detBbar;
  *(dK_da) = dK_ds;
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_dH_dparam(const double *params, double *dH_da) const
{
  // K = det(L) = det(I_inv * II) = det(I_inv) * det(II) = det(S^-1 * Ibar^-1 * S^-1) * det(IIbar) = (1/s^4) * det(Ibar^-1) * det(IIbar)
  // H = trace(L) / 2

  double s = params[0];
  EigenSupport::M2d I_inv = abar.inverse();
  EigenSupport::M2d II = bbar;
  EigenSupport::M2d L = I_inv * II;
  double dH_ds = -2.0 / (s * s * s) * L.trace() * 0.5;
  *(dH_da) = dH_ds;
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_darea_dparam(const double *params, double *darea_da) const
{
  double s = params[0];
  double dA_ds = 2.0 * areaRest * s;
  *(darea_da) = dA_ds;
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_dabar_dparam(const double *params, double *dabar_dparam) const
{
  double s = params[0];
  EigenSupport::M2d S;
  S << s, 0, 0, s;

  EigenSupport::M2d dS_ds;
  dS_ds << 1, 0, 0, 1;

  EigenSupport::M2d dAbar_ds = dS_ds * abar * S + S * abar * dS_ds;

  (EigenSupport::Mp<EigenSupport::M2d>(dabar_dparam)) = dAbar_ds;
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_dbbar_dparam(const double *params, double *dbbar_dparam) const
{
  EigenSupport::M2d zero = EigenSupport::M2d::Zero();
  (EigenSupport::Mp<EigenSupport::M2d>(dbbar_dparam)) = zero;
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_d2abar_dparam2(
  const double *, int pi, int pj, double *d2abar_dparam2) const
{
  EigenSupport::Mp<EigenSupport::M2d> d2abar(d2abar_dparam2);
  if (pi == 0 && pj == 0) {
    d2abar = 2.0 * abar;
  }
  else {
    d2abar.setZero();
  }
}

void pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_d2dbbar_dparam2(
  const double *, int, int, double *d2dbbar_dparam2) const
{
  EigenSupport::Mp<EigenSupport::M2d>(d2dbbar_dparam2).setZero();
}

double pgo::SolidDeformationModel::PlasticModel2DFundamentalFormsUniformStretch::compute_d2area_dparam2(
  const double *, int pi, int pj) const
{
  return (pi == 0 && pj == 0) ? 2.0 * areaRest : 0.0;
}


#include "simulation/simulationMesh.h"
#include <algorithm>
#include <initializer_list>
#include <stdexcept>

namespace pgo::SolidDeformationModel {
namespace {
void expectSize(std::span<double> output, std::size_t expected) {
  if (output.size() != expected) throw std::invalid_argument("plastic config default parameter buffer has the wrong size");
}
}
std::span<const std::string_view> ShellPlasticity1Config::parameterChannelNames() const { static constexpr std::array<std::string_view, 1> names{"stretch"}; return names; }
void ShellPlasticity1Config::initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double> output) const { expectSize(output, 1); output[0] = 1.0; }
std::unique_ptr<PlasticModel> ShellPlasticity1Config::createModel(const SimulationMesh &, int, const MaterialFrame &) const
{
  return std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
}
}  // namespace pgo::SolidDeformationModel
