#include "material/elastic/elasticModel2DFundamentalFormsSTVK.h"

#include "EigenDef.h"

#include <cmath>

using namespace pgo;
using namespace SolidDeformationModel;

namespace ES = pgo::EigenSupport;

static inline ES::M4d KroneckerProduct2(const ES::M2d &A, const ES::M2d &B)
{
  ES::M4d ret;

  for (int col = 0; col < 2; col++) {
    for (int row = 0; row < 2; row++) {
      ret.block<2, 2>(row * 2, col * 2) = A(row, col) * B;
    }
  }

  return ret;
}

static inline ES::V4d vecCM(const ES::M2d &X)
{  // column-major vec
  ES::V4d v;
  v << X(0, 0), X(1, 0), X(0, 1), X(1, 1);
  return v;
}

struct Lame2D
{
  double alpha = 0.0;
  double beta = 0.0;
  double dalpha_dE = 0.0;
  double dbeta_dE = 0.0;
  double dalpha_dnu = 0.0;
  double dbeta_dnu = 0.0;
  double d2alpha_dEdnu = 0.0;
  double d2beta_dEdnu = 0.0;
  double d2alpha_dnu2 = 0.0;
  double d2beta_dnu2 = 0.0;
};

static inline Lame2D computeLame2D(double E, double nu)
{
  const double onePlusNu = 1.0 + nu;
  const double oneMinus2Nu = 1.0 - 2.0 * nu;
  const double denom = onePlusNu * oneMinus2Nu;
  const double denom2 = denom * denom;
  const double denom3 = denom2 * denom;

  Lame2D l;
  l.alpha = E * nu / denom;
  l.beta = E / (2.0 * onePlusNu);
  l.dalpha_dE = nu / denom;
  l.dbeta_dE = 1.0 / (2.0 * onePlusNu);
  l.dalpha_dnu = E * (1.0 + 2.0 * nu * nu) / denom2;
  l.dbeta_dnu = -E / (2.0 * onePlusNu * onePlusNu);
  l.d2alpha_dEdnu = (1.0 + 2.0 * nu * nu) / denom2;
  l.d2beta_dEdnu = -1.0 / (2.0 * onePlusNu * onePlusNu);
  l.d2alpha_dnu2 = E * (2.0 + 12.0 * nu + 8.0 * nu * nu * nu) / denom3;
  l.d2beta_dnu2 = E / (onePlusNu * onePlusNu * onePlusNu);
  return l;
}

static inline double stvkCoreEnergy(double alpha, double beta, const ES::M2d &M)
{
  const double traceM = M.trace();
  return 0.5 * alpha * traceM * traceM + beta * (M * M).trace();
}

static inline ES::M2d stvkCoreStress(double alpha, double beta, const ES::M2d &M)
{
  return alpha * M.trace() * ES::M2d::Identity() + 2.0 * beta * M;
}

static inline ES::M2d stvkDpsiDabarCore(
  double alpha, double beta, const ES::M2d &Z, const ES::M2d &X, const ES::M2d &M)
{
  const ES::M2d G = stvkCoreStress(alpha, beta, M);
  return -Z.transpose() * G.transpose() * X.transpose() * Z.transpose();
}

static inline ES::M2d stvkDpsiDbbarCore(
  double alpha, double beta, const ES::M2d &Z, const ES::M2d &M)
{
  const ES::M2d G = stvkCoreStress(alpha, beta, M);
  return -Z.transpose() * G.transpose();
}

static inline double stvkCoreSecondDirectionalDerivative(
  double alpha, double beta, const ES::M2d &M,
  const ES::M2d &dMi, const ES::M2d &dMj, const ES::M2d &d2Mij)
{
  return alpha * (dMi.trace() * dMj.trace() + M.trace() * d2Mij.trace()) +
    2.0 * beta * ((dMi * dMj).trace() + (M * d2Mij).trace());
}

bool ElasticModel2DFundamentalFormsSTVK::computeVonMisesStress(
  const double *param,
  const double a_[4], const double b_[4],
  const double abar_[4], const double bbar_[4], double &stress) const
{
  const double E_m = param[0];
  const double nu_m = param[1];
  const double E_b = param[2];
  const double nu_b = param[3];
  const double h = param[4];

  const double alpha_m = E_m * nu_m / ((1.0 + nu_m) * (1.0 - 2.0 * nu_m));
  const double beta_m = E_m / (2.0 * (1.0 + nu_m));
  const double alpha_b = E_b * nu_b / ((1.0 + nu_b) * (1.0 - 2.0 * nu_b));
  const double beta_b = E_b / (2.0 * (1.0 + nu_b));

  const ES::M2d a = ES::Mp<const ES::M2d>(a_);
  const ES::M2d b = ES::Mp<const ES::M2d>(b_);
  const ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  const ES::M2d bbar = ES::Mp<const ES::M2d>(bbar_);

  const ES::M2d abar_inv = abar.fullPivHouseholderQr().inverse();

  // Membrane strain: E_mem = 0.5 * (abar^{-1} * a - I)
  const ES::M2d E_mem = 0.5 * (abar_inv * a - ES::M2d::Identity());
  // Bending curvature: kappa = abar^{-1} * (b - bbar)
  const ES::M2d kappa = abar_inv * (b - bbar);

  // Stress resultants (2x2 mixed tensors, do NOT symmetrize)
  // sigma_mem = alpha_m * tr(E_mem) * I + 2 * beta_m * E_mem
  const ES::M2d sigma_mem = alpha_m * E_mem.trace() * ES::M2d::Identity() + 2.0 * beta_m * E_mem;
  // sigma_bnd = alpha_b * tr(kappa) * I + 2 * beta_b * kappa
  const ES::M2d sigma_bnd = alpha_b * kappa.trace() * ES::M2d::Identity() + 2.0 * beta_b * kappa;

  // Evaluate at z = +h/2 and z = -h/2; take the max von Mises value
  double vm_max = 0.0;
  for (int sign : { 1, -1 }) {
    const double z = sign * 0.5 * h;
    // sigma(z) = sigma_mem - z * sigma_bnd  (fiber stress at depth z)
    const ES::M2d sigma = sigma_mem - z * sigma_bnd;
    // Plane-stress von Mises: sqrt(max(tr^2 - 3*det, 0))
    const double tr_s = sigma.trace();
    const double det_s = sigma.determinant();
    const double val = tr_s * tr_s - 3.0 * det_s;
    const double vm = std::sqrt(std::max(val, 0.0));
    if (vm > vm_max)
      vm_max = vm;
  }

  stress = vm_max;
  return true;
}

double ElasticModel2DFundamentalFormsSTVK::compute_psi_a(const double *param, const double a_[4], const double abar_[4]) const
{
  double Es = param[0];
  double nu_s = param[1];
  double h = param[4];

  double lameAlpha = Es * nu_s / ((1 + nu_s) * (1 - 2 * nu_s));
  double lameBeta = Es / (2 * (1 + nu_s));

  ES::M2d a = ES::Mp<const ES::M2d>(a_);
  ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  ES::M2d a_bar_inv = abar.fullPivHouseholderQr().inverse();
  ES::M2d M = a_bar_inv * a - ES::M2d::Identity();

  double trace_M = M.trace();
  double Wc = 0.5 * lameAlpha * (trace_M * trace_M) + lameBeta * (M * M).trace();

  return Wc * h;
}

void ElasticModel2DFundamentalFormsSTVK::compute_dpsi_da(const double *param, const double a_[4], const double abar_[4], double da_[4]) const
{
  double Es = param[0];
  double nu_s = param[1];
  double h = param[4];

  double lameAlpha = Es * nu_s / ((1 + nu_s) * (1 - 2 * nu_s));
  double lameBeta = Es / (2 * (1 + nu_s));

  ES::M2d a = ES::Mp<const ES::M2d>(a_);
  ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  ES::M2d a_bar_inv = abar.fullPivHouseholderQr().inverse();
  ES::M2d M = a_bar_inv * a - ES::M2d::Identity();

  double trace_M = M.trace();
  ES::M2d I = ES::M2d::Identity();
  ES::M2d S = (lameAlpha * trace_M * I + 2.0 * lameBeta * M) * a_bar_inv;

  (ES::Mp<ES::M2d>(da_)) = S * h;
}

void ElasticModel2DFundamentalFormsSTVK::compute_d2psi_da2(const double *param, const double a_[4], const double abar_[4], double da2_[16]) const
{
  double Es = param[0];
  double nu_s = param[1];
  double h = param[4];

  double lameAlpha = Es * nu_s / ((1 + nu_s) * (1 - 2 * nu_s));
  double lameBeta = Es / (2 * (1 + nu_s));

  ES::M2d a = ES::Mp<const ES::M2d>(a_);
  ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  ES::M2d a_bar_inv = abar.fullPivHouseholderQr().inverse();
  ES::M2d M = a_bar_inv * a - ES::M2d::Identity();

  // E = 0.5 alpha (trace(M))^2 + beta trace(M^2)
  // dE/dM = alpha trace(M) I + 2 beta M
  // d2E/dM2 = alpha I x I + 2 beta I

  // dE/da = dE/dM : dM/da
  // d2E/da2 = (dM/da)^T : d2E/dM2 : dM/da + dE/dM : d2M/da2

  ES::M2d I;
  I.setIdentity();

  ES::M4d d2EdM2;
  d2EdM2 << (lameAlpha + 2 * lameBeta), 0, 0, lameAlpha,
    0, 0, 2 * lameBeta, 0,
    0, 2 * lameBeta, 0, 0,
    lameAlpha, 0, 0, lameAlpha + 2 * lameBeta;

  ES::M4d dMda = KroneckerProduct2(I, a_bar_inv);
  ES::M4d d2Eda2 = dMda.transpose() * d2EdM2 * dMda;

  (ES::Mp<ES::M4d>(da2_)) = d2Eda2 * h;
}

double ElasticModel2DFundamentalFormsSTVK::compute_psi_b(const double *param, const double b_[4], const double abar_[4], const double bbar_[4]) const
{
  double Es = param[2];
  double nu_s = param[3];
  double h = param[4];

  double lameAlpha = Es * nu_s / ((1 + nu_s) * (1 - 2 * nu_s));
  double lameBeta = Es / (2 * (1 + nu_s));

  ES::M2d b = ES::Mp<const ES::M2d>(b_);
  ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  ES::M2d bbar = ES::Mp<const ES::M2d>(bbar_);
  ES::M2d a_bar_inv = abar.fullPivHouseholderQr().inverse();
  ES::M2d M = a_bar_inv * (b - bbar);

  double trace_M = M.trace();
  double Wc = 0.5 * lameAlpha * (trace_M * trace_M) + lameBeta * (M * M).trace();

  return Wc * h * h * h / 12;
}

void ElasticModel2DFundamentalFormsSTVK::compute_dpsi_db(const double *param, const double b_[4], const double abar_[4], const double bbar_[4], double db_[4]) const
{
  double Es = param[2];
  double nu_s = param[3];
  double h = param[4];

  double lameAlpha = Es * nu_s / ((1 + nu_s) * (1 - 2 * nu_s));
  double lameBeta = Es / (2 * (1 + nu_s));

  ES::M2d b = ES::Mp<const ES::M2d>(b_);
  ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  ES::M2d bbar = ES::Mp<const ES::M2d>(bbar_);
  ES::M2d a_bar_inv = abar.fullPivHouseholderQr().inverse();
  ES::M2d M = a_bar_inv * (b - bbar);

  double trace_M = M.trace();
  ES::M2d S = lameAlpha * trace_M * a_bar_inv + 2.0 * lameBeta * M * a_bar_inv;
  (ES::Mp<ES::M2d>(db_)) = S * h * h * h / 12;
}

void ElasticModel2DFundamentalFormsSTVK::compute_d2psi_db2(const double *param, const double b_[4], const double abar_[4], const double bbar_[4], double db2_[16]) const
{
  double Es = param[2];
  double nu_s = param[3];
  double h = param[4];

  double lameAlpha = Es * nu_s / ((1 + nu_s) * (1 - 2 * nu_s));
  double lameBeta = Es / (2 * (1 + nu_s));

  ES::M2d b = ES::Mp<const ES::M2d>(b_);
  ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  ES::M2d bbar = ES::Mp<const ES::M2d>(bbar_);
  ES::M2d a_bar_inv = abar.fullPivHouseholderQr().inverse();
  ES::M2d M = a_bar_inv * (b - bbar);

  ES::M2d I;
  I.setIdentity();

  ES::M4d d2EdM2;
  d2EdM2 << (lameAlpha + 2 * lameBeta), 0, 0, lameAlpha,
    0, 0, 2 * lameBeta, 0,
    0, 2 * lameBeta, 0, 0,
    lameAlpha, 0, 0, lameAlpha + 2 * lameBeta;

  ES::M4d dMdb = KroneckerProduct2(I, a_bar_inv);
  ES::M4d d2Edb2 = dMdb.transpose() * d2EdM2 * dMdb;

  (ES::Mp<ES::M4d>(db2_)) = d2Edb2 * h * h * h / 12;
}

void ElasticModel2DFundamentalFormsSTVK::compute_d2psi_dadabar(const double *param, const double a_[4], const double abar_[4], double dadabar_[16]) const
{
  double Es = param[0];
  double nu_s = param[1];
  double h = param[4];

  double lameAlpha = Es * nu_s / ((1 + nu_s) * (1 - 2 * nu_s));
  double lameBeta = Es / (2 * (1 + nu_s));

  // vec (ABX) = (X^T ⊗ A) vec(B)

  // d(abar^{-1]}) abar + abar^{-1} d(abar) = 0
  // d(abar^{-1)}) = - abar^{-1} d(abar) abar^{-1}

  // dM/dabar = d(abar^{-1} a - I) /dabar = d(z a - I) / dz * dz/dabar
  //          = (a^T ⊗ I) * d(abar^{-1})/dabar
  //          = - (a^T ⊗ I) * (abar^{-T} ⊗ abar^{-1})
  //

  // ES::M2d S = (lameAlpha * trace_M * I + 2.0 * lameBeta * M) * a_bar_inv;
  // dS/dabar = (c1 d trace(M)/dM * dM/dabar * I + c2 dM/dabar) * a_bar_inv + S * d(a_bar_inv)/dabar
  // dS/dabar = (c1 * I : dM/dabar + c2 dM/dabar) * a_bar_inv + S * d(a_bar_inv)/dabar

  ES::M2d a = ES::Mp<const ES::M2d>(a_);
  ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  ES::M2d a_bar_inv = abar.fullPivHouseholderQr().inverse();
  ES::M2d M = a_bar_inv * a - ES::M2d::Identity();
  ES::M2d I = ES::M2d::Identity();

  ES::M4d dadabar;
  dadabar.setZero();
  for (int i = 0; i < 4; i++) {
    ES::M2d dabar_i;
    dabar_i.setZero();
    dabar_i.data()[i] = 1.0;

    ES::M2d dabar_inv_i = -a_bar_inv * dabar_i * a_bar_inv;
    ES::M2d dM_dabar_i = dabar_inv_i * a;

    ES::M2d dS_dabar_i = (lameAlpha * dM_dabar_i.trace() * I + 2.0 * lameBeta * dM_dabar_i) * a_bar_inv +
      (lameAlpha * M.trace() * I + 2.0 * lameBeta * M) * dabar_inv_i;
    dadabar.col(i) = vecCM(dS_dabar_i);
  }

  (ES::Mp<ES::M4d>(dadabar_)) = dadabar * h;
}

void ElasticModel2DFundamentalFormsSTVK::compute_d2psi_db_dabar(const double *param, const double b_[4], const double abar_[4], const double bbar_[4], double dbdabar_[16]) const
{
  double Es = param[2];
  double nu_s = param[3];
  double h = param[4];

  double lameAlpha = Es * nu_s / ((1 + nu_s) * (1 - 2 * nu_s));
  double lameBeta = Es / (2 * (1 + nu_s));

  ES::M2d b = ES::Mp<const ES::M2d>(b_);
  ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  ES::M2d bbar = ES::Mp<const ES::M2d>(bbar_);
  ES::M2d a_bar_inv = abar.fullPivHouseholderQr().inverse();
  ES::M2d a_bar_inv_T = a_bar_inv.transpose();
  ES::M2d M = a_bar_inv * (b - bbar);
  ES::M2d I = ES::M2d::Identity();

  ES::M4d dbdabar;
  dbdabar.setZero();

  for (int i = 0; i < 4; i++) {
    ES::M2d dabar_i;
    dabar_i.setZero();
    dabar_i.data()[i] = 1.0;

    ES::M2d dabar_inv_i = -a_bar_inv * dabar_i * a_bar_inv;
    ES::M2d dM_dabar_i = dabar_inv_i * (b - bbar);

    ES::M2d dS_dabar_i = (lameAlpha * dM_dabar_i.trace() * I + 2.0 * lameBeta * dM_dabar_i) * a_bar_inv +
      (lameAlpha * M.trace() * I + 2.0 * lameBeta * M) * dabar_inv_i;
    dbdabar.col(i) = vecCM(dS_dabar_i);
  }

  (ES::Mp<ES::M4d>(dbdabar_)) = dbdabar * h * h * h / 12;
}

void ElasticModel2DFundamentalFormsSTVK::compute_d2psi_db_dbbar(const double *param, const double b_[4], const double abar_[4], const double bbar_[4], double dbdbbar_[16]) const
{
  double Es = param[2];
  double nu_s = param[3];
  double h = param[4];

  double lameAlpha = Es * nu_s / ((1 + nu_s) * (1 - 2 * nu_s));
  double lameBeta = Es / (2 * (1 + nu_s));

  ES::M2d b = ES::Mp<const ES::M2d>(b_);
  ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  ES::M2d bbar = ES::Mp<const ES::M2d>(bbar_);
  ES::M2d a_bar_inv = abar.fullPivHouseholderQr().inverse();
  ES::M2d a_bar_inv_T = a_bar_inv.transpose();
  ES::M2d M = a_bar_inv * (b - bbar);
  ES::M2d I = ES::M2d::Identity();

  ES::M4d dbdbbar;
  dbdbbar.setZero();
  for (int i = 0; i < 4; i++) {
    ES::M2d dbbar_i;
    dbbar_i.setZero();
    dbbar_i.data()[i] = 1.0;

    ES::M2d dM_dbbar_i = -a_bar_inv * dbbar_i;
    ES::M2d dS_dbbar_i = (lameAlpha * dM_dbbar_i.trace() * I + 2.0 * lameBeta * dM_dbbar_i) * a_bar_inv;

    dbdbbar.col(i) = vecCM(dS_dbbar_i);
  }

  (ES::Mp<ES::M4d>(dbdbbar_)) = dbdbbar * h * h * h / 12;
  // this route is not executed yet
}

void ElasticModel2DFundamentalFormsSTVK::compute_d2psi_da_dparam(const double *param, const double a_[4], const double abar_[4], double d2psi_dadparam[/*4 x numParams*/]) const
{
  double Es = param[0];
  double nu_s = param[1];
  double h = param[4];

  double lameAlpha = Es * nu_s / ((1 + nu_s) * (1 - 2 * nu_s));
  double lameBeta = Es / (2 * (1 + nu_s));

  ES::M2d a = ES::Mp<const ES::M2d>(a_);
  ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  ES::M2d a_bar_inv = abar.fullPivHouseholderQr().inverse();
  ES::M2d M = a_bar_inv * a - ES::M2d::Identity();

  double trace_M = M.trace();
  ES::M2d I = ES::M2d::Identity();
  ES::M2d S = (lameAlpha * trace_M * I + 2.0 * lameBeta * M) * a_bar_inv;

  Eigen::Map<ES::MXd> d2psi_dadparam_map(d2psi_dadparam, 4, 5);
  d2psi_dadparam_map.setZero();

  ES::M2d dS_dalpha = (trace_M * I) * a_bar_inv;
  ES::M2d dS_dbeta = (2.0 * M) * a_bar_inv;

  ES::M2d dS_dE = dS_dalpha * (nu_s / ((1 + nu_s) * (1 - 2 * nu_s))) + dS_dbeta * (1.0 / (2 * (1 + nu_s)));
  ES::M2d dS_dnu = dS_dalpha * (Es * (1 + 2 * nu_s * nu_s) / ((1 + nu_s) * (1 + nu_s) * (1 - 2 * nu_s) * (1 - 2 * nu_s)));
  dS_dnu += dS_dbeta * (-Es / (2 * (1 + nu_s) * (1 + nu_s)));

  d2psi_dadparam_map.col(0) = vecCM(dS_dE) * h;
  d2psi_dadparam_map.col(1) = vecCM(dS_dnu) * h;
  d2psi_dadparam_map.col(2).setZero();
  d2psi_dadparam_map.col(3).setZero();
  d2psi_dadparam_map.col(4) = vecCM(S);
}

void ElasticModel2DFundamentalFormsSTVK::compute_d2psi_db_dparam(const double *param, const double b_[4], const double abar_[4], const double bbar_[4], double d2psi_dbdparam[/*4 x numParams*/]) const
{
  double Es = param[2];
  double nu_s = param[3];
  double h = param[4];

  double lameAlpha = Es * nu_s / ((1 + nu_s) * (1 - 2 * nu_s));
  double lameBeta = Es / (2 * (1 + nu_s));

  ES::M2d b = ES::Mp<const ES::M2d>(b_);
  ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  ES::M2d bbar = ES::Mp<const ES::M2d>(bbar_);
  ES::M2d a_bar_inv = abar.fullPivHouseholderQr().inverse();
  ES::M2d M = a_bar_inv * (b - bbar);

  double trace_M = M.trace();
  ES::M2d I = ES::M2d::Identity();
  ES::M2d S = (lameAlpha * trace_M * I + 2.0 * lameBeta * M) * a_bar_inv;

  Eigen::Map<ES::MXd> d2psi_dbdparam_map(d2psi_dbdparam, 4, 5);
  d2psi_dbdparam_map.setZero();

  ES::M2d dS_dalpha = (trace_M * I) * a_bar_inv;
  ES::M2d dS_dbeta = (2.0 * M) * a_bar_inv;

  ES::M2d dS_dE = dS_dalpha * (nu_s / ((1 + nu_s) * (1 - 2 * nu_s))) + dS_dbeta * (1.0 / (2 * (1 + nu_s)));
  ES::M2d dS_dnu = dS_dalpha * (Es * (1 + 2 * nu_s * nu_s) / ((1 + nu_s) * (1 + nu_s) * (1 - 2 * nu_s) * (1 - 2 * nu_s)));
  dS_dnu += dS_dbeta * (-Es / (2 * (1 + nu_s) * (1 + nu_s)));

  d2psi_dbdparam_map.col(0).setZero();
  d2psi_dbdparam_map.col(1).setZero();
  d2psi_dbdparam_map.col(2) = vecCM(dS_dE) * h * h * h / 12;
  d2psi_dbdparam_map.col(3) = vecCM(dS_dnu) * h * h * h / 12;
  d2psi_dbdparam_map.col(4) = vecCM(S) * 3 * h * h / 12;
}

void ElasticModel2DFundamentalFormsSTVK::compute_dpsi_dabar(
  const double *param, const double a_[4], const double b_[4],
  const double abar_[4], const double bbar_[4], double dpsi_dabar_[4]) const
{
  const double h = param[4];
  const ES::M2d a = ES::Mp<const ES::M2d>(a_);
  const ES::M2d b = ES::Mp<const ES::M2d>(b_);
  const ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  const ES::M2d bbar = ES::Mp<const ES::M2d>(bbar_);
  const ES::M2d Z = abar.fullPivHouseholderQr().inverse();

  const Lame2D mem = computeLame2D(param[0], param[1]);
  const ES::M2d Mmem = Z * a - ES::M2d::Identity();
  ES::M2d grad = h * stvkDpsiDabarCore(mem.alpha, mem.beta, Z, a, Mmem);

  const Lame2D bend = computeLame2D(param[2], param[3]);
  const ES::M2d Xbend = b - bbar;
  const ES::M2d Mbend = Z * Xbend;
  grad += (h * h * h / 12.0) *
    stvkDpsiDabarCore(bend.alpha, bend.beta, Z, Xbend, Mbend);

  Eigen::Map<ES::M2d> gradMap(dpsi_dabar_);
  gradMap = grad;
}

void ElasticModel2DFundamentalFormsSTVK::compute_dpsi_dbbar(
  const double *param, const double[4], const double b_[4],
  const double abar_[4], const double bbar_[4], double dpsi_dbbar_[4]) const
{
  const double h = param[4];
  const ES::M2d b = ES::Mp<const ES::M2d>(b_);
  const ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  const ES::M2d bbar = ES::Mp<const ES::M2d>(bbar_);
  const ES::M2d Z = abar.fullPivHouseholderQr().inverse();

  const Lame2D bend = computeLame2D(param[2], param[3]);
  const ES::M2d Mbend = Z * (b - bbar);
  Eigen::Map<ES::M2d> gradMap(dpsi_dbbar_);
  gradMap = (h * h * h / 12.0) * stvkDpsiDbbarCore(bend.alpha, bend.beta, Z, Mbend);
}

void ElasticModel2DFundamentalFormsSTVK::compute_dpsi_dparam(
  const double *param, const double a_[4], const double b_[4],
  const double abar_[4], const double bbar_[4], double dpsi_dparam_[/*numParams*/]) const
{
  const double h = param[4];
  const ES::M2d a = ES::Mp<const ES::M2d>(a_);
  const ES::M2d b = ES::Mp<const ES::M2d>(b_);
  const ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  const ES::M2d bbar = ES::Mp<const ES::M2d>(bbar_);
  const ES::M2d Z = abar.fullPivHouseholderQr().inverse();

  const ES::M2d Mmem = Z * a - ES::M2d::Identity();
  const ES::M2d Mbend = Z * (b - bbar);
  const double trMem2 = Mmem.trace() * Mmem.trace();
  const double trBend2 = Mbend.trace() * Mbend.trace();
  const double memM2 = (Mmem * Mmem).trace();
  const double bendM2 = (Mbend * Mbend).trace();

  const Lame2D mem = computeLame2D(param[0], param[1]);
  const Lame2D bend = computeLame2D(param[2], param[3]);
  const double Wmem = stvkCoreEnergy(mem.alpha, mem.beta, Mmem);
  const double Wbend = stvkCoreEnergy(bend.alpha, bend.beta, Mbend);

  Eigen::Map<ES::VXd> out(dpsi_dparam_, getNumParameters());
  out.setZero();
  out[0] = h * (0.5 * mem.dalpha_dE * trMem2 + mem.dbeta_dE * memM2);
  out[1] = h * (0.5 * mem.dalpha_dnu * trMem2 + mem.dbeta_dnu * memM2);
  out[2] = (h * h * h / 12.0) * (0.5 * bend.dalpha_dE * trBend2 + bend.dbeta_dE * bendM2);
  out[3] = (h * h * h / 12.0) * (0.5 * bend.dalpha_dnu * trBend2 + bend.dbeta_dnu * bendM2);
  out[4] = Wmem + (3.0 * h * h / 12.0) * Wbend;
}

void ElasticModel2DFundamentalFormsSTVK::compute_d2psi_dparam2(
  const double *param, const double a_[4], const double b_[4],
  const double abar_[4], const double bbar_[4], double d2psi_dparam2_[/*numParams x numParams*/]) const
{
  const double h = param[4];
  const ES::M2d a = ES::Mp<const ES::M2d>(a_);
  const ES::M2d b = ES::Mp<const ES::M2d>(b_);
  const ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  const ES::M2d bbar = ES::Mp<const ES::M2d>(bbar_);
  const ES::M2d Z = abar.fullPivHouseholderQr().inverse();

  const ES::M2d Mmem = Z * a - ES::M2d::Identity();
  const ES::M2d Mbend = Z * (b - bbar);
  const double trMem2 = Mmem.trace() * Mmem.trace();
  const double trBend2 = Mbend.trace() * Mbend.trace();
  const double memM2 = (Mmem * Mmem).trace();
  const double bendM2 = (Mbend * Mbend).trace();

  const Lame2D mem = computeLame2D(param[0], param[1]);
  const Lame2D bend = computeLame2D(param[2], param[3]);
  const double Wbend = stvkCoreEnergy(bend.alpha, bend.beta, Mbend);

  Eigen::Map<ES::MXd> H(d2psi_dparam2_, getNumParameters(), getNumParameters());
  H.setZero();

  const double dWmem_dE = 0.5 * mem.dalpha_dE * trMem2 + mem.dbeta_dE * memM2;
  const double dWmem_dnu = 0.5 * mem.dalpha_dnu * trMem2 + mem.dbeta_dnu * memM2;
  H(0, 1) = H(1, 0) = h * (0.5 * mem.d2alpha_dEdnu * trMem2 + mem.d2beta_dEdnu * memM2);
  H(1, 1) = h * (0.5 * mem.d2alpha_dnu2 * trMem2 + mem.d2beta_dnu2 * memM2);
  H(0, 4) = H(4, 0) = dWmem_dE;
  H(1, 4) = H(4, 1) = dWmem_dnu;

  const double bendScale = h * h * h / 12.0;
  const double dWbend_dE = 0.5 * bend.dalpha_dE * trBend2 + bend.dbeta_dE * bendM2;
  const double dWbend_dnu = 0.5 * bend.dalpha_dnu * trBend2 + bend.dbeta_dnu * bendM2;
  H(2, 3) = H(3, 2) = bendScale * (0.5 * bend.d2alpha_dEdnu * trBend2 + bend.d2beta_dEdnu * bendM2);
  H(3, 3) = bendScale * (0.5 * bend.d2alpha_dnu2 * trBend2 + bend.d2beta_dnu2 * bendM2);
  H(2, 4) = H(4, 2) = (3.0 * h * h / 12.0) * dWbend_dE;
  H(3, 4) = H(4, 3) = (3.0 * h * h / 12.0) * dWbend_dnu;
  H(4, 4) = (6.0 * h / 12.0) * Wbend;
}

void ElasticModel2DFundamentalFormsSTVK::compute_d2psi_dabar_dparam(
  const double *param, const double a_[4], const double b_[4],
  const double abar_[4], const double bbar_[4], double d2psi_dabar_dparam_[/*4 x numParams*/]) const
{
  const double h = param[4];
  const ES::M2d a = ES::Mp<const ES::M2d>(a_);
  const ES::M2d b = ES::Mp<const ES::M2d>(b_);
  const ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  const ES::M2d bbar = ES::Mp<const ES::M2d>(bbar_);
  const ES::M2d Z = abar.fullPivHouseholderQr().inverse();

  const ES::M2d Mmem = Z * a - ES::M2d::Identity();
  const ES::M2d Xbend = b - bbar;
  const ES::M2d Mbend = Z * Xbend;
  const Lame2D mem = computeLame2D(param[0], param[1]);
  const Lame2D bend = computeLame2D(param[2], param[3]);

  Eigen::Map<ES::MXd> out(d2psi_dabar_dparam_, 4, getNumParameters());
  out.setZero();
  out.col(0) = vecCM(h * stvkDpsiDabarCore(mem.dalpha_dE, mem.dbeta_dE, Z, a, Mmem));
  out.col(1) = vecCM(h * stvkDpsiDabarCore(mem.dalpha_dnu, mem.dbeta_dnu, Z, a, Mmem));
  out.col(2) = vecCM((h * h * h / 12.0) *
    stvkDpsiDabarCore(bend.dalpha_dE, bend.dbeta_dE, Z, Xbend, Mbend));
  out.col(3) = vecCM((h * h * h / 12.0) *
    stvkDpsiDabarCore(bend.dalpha_dnu, bend.dbeta_dnu, Z, Xbend, Mbend));
  out.col(4) = vecCM(stvkDpsiDabarCore(mem.alpha, mem.beta, Z, a, Mmem) +
    (3.0 * h * h / 12.0) * stvkDpsiDabarCore(bend.alpha, bend.beta, Z, Xbend, Mbend));
}

void ElasticModel2DFundamentalFormsSTVK::compute_d2psi_dbbar_dparam(
  const double *param, const double[4], const double b_[4],
  const double abar_[4], const double bbar_[4], double d2psi_dbbar_dparam_[/*4 x numParams*/]) const
{
  const double h = param[4];
  const ES::M2d b = ES::Mp<const ES::M2d>(b_);
  const ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  const ES::M2d bbar = ES::Mp<const ES::M2d>(bbar_);
  const ES::M2d Z = abar.fullPivHouseholderQr().inverse();
  const ES::M2d Mbend = Z * (b - bbar);
  const Lame2D bend = computeLame2D(param[2], param[3]);

  Eigen::Map<ES::MXd> out(d2psi_dbbar_dparam_, 4, getNumParameters());
  out.setZero();
  out.col(2) = vecCM((h * h * h / 12.0) *
    stvkDpsiDbbarCore(bend.dalpha_dE, bend.dbeta_dE, Z, Mbend));
  out.col(3) = vecCM((h * h * h / 12.0) *
    stvkDpsiDbbarCore(bend.dalpha_dnu, bend.dbeta_dnu, Z, Mbend));
  out.col(4) = vecCM((3.0 * h * h / 12.0) *
    stvkDpsiDbbarCore(bend.alpha, bend.beta, Z, Mbend));
}

void ElasticModel2DFundamentalFormsSTVK::compute_d2psi_dabar2(
  const double *param, const double a_[4], const double b_[4],
  const double abar_[4], const double bbar_[4], double d2psi_dabar2_[16]) const
{
  const double h = param[4];
  const ES::M2d a = ES::Mp<const ES::M2d>(a_);
  const ES::M2d b = ES::Mp<const ES::M2d>(b_);
  const ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  const ES::M2d bbar = ES::Mp<const ES::M2d>(bbar_);
  const ES::M2d Z = abar.fullPivHouseholderQr().inverse();
  const ES::M2d Xbend = b - bbar;

  const Lame2D mem = computeLame2D(param[0], param[1]);
  const Lame2D bend = computeLame2D(param[2], param[3]);
  const ES::M2d Mmem = Z * a - ES::M2d::Identity();
  const ES::M2d Mbend = Z * Xbend;
  const double bendScale = h * h * h / 12.0;

  Eigen::Map<ES::M4d> H(d2psi_dabar2_);
  H.setZero();
  for (int i = 0; i < 4; i++) {
    ES::M2d Di = ES::M2d::Zero();
    Di.data()[i] = 1.0;
    const ES::M2d dZi = -Z * Di * Z;
    const ES::M2d dMmemI = dZi * a;
    const ES::M2d dMbendI = dZi * Xbend;

    for (int j = 0; j < 4; j++) {
      ES::M2d Dj = ES::M2d::Zero();
      Dj.data()[j] = 1.0;
      const ES::M2d dZj = -Z * Dj * Z;
      const ES::M2d dMmemJ = dZj * a;
      const ES::M2d dMbendJ = dZj * Xbend;
      const ES::M2d d2Zij = Z * Dj * Z * Di * Z + Z * Di * Z * Dj * Z;

      H(i, j) =
        h * stvkCoreSecondDirectionalDerivative(mem.alpha, mem.beta, Mmem,
              dMmemI, dMmemJ, d2Zij * a) +
        bendScale * stvkCoreSecondDirectionalDerivative(bend.alpha, bend.beta, Mbend,
              dMbendI, dMbendJ, d2Zij * Xbend);
    }
  }
}

void ElasticModel2DFundamentalFormsSTVK::compute_d2psi_dabar_dbbar(
  const double *param, const double[4], const double b_[4],
  const double abar_[4], const double bbar_[4], double d2psi_dabar_dbbar_[16]) const
{
  const double h = param[4];
  const ES::M2d b = ES::Mp<const ES::M2d>(b_);
  const ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  const ES::M2d bbar = ES::Mp<const ES::M2d>(bbar_);
  const ES::M2d Z = abar.fullPivHouseholderQr().inverse();
  const ES::M2d Xbend = b - bbar;

  const Lame2D bend = computeLame2D(param[2], param[3]);
  const ES::M2d Mbend = Z * Xbend;
  const double bendScale = h * h * h / 12.0;

  Eigen::Map<ES::M4d> H(d2psi_dabar_dbbar_);
  H.setZero();
  for (int i = 0; i < 4; i++) {
    ES::M2d Di = ES::M2d::Zero();
    Di.data()[i] = 1.0;
    const ES::M2d dZi = -Z * Di * Z;
    const ES::M2d dMbendI = dZi * Xbend;

    for (int j = 0; j < 4; j++) {
      ES::M2d Ej = ES::M2d::Zero();
      Ej.data()[j] = 1.0;
      const ES::M2d dMbbarJ = -Z * Ej;
      const ES::M2d d2Mij = -dZi * Ej;

      H(i, j) = bendScale *
        stvkCoreSecondDirectionalDerivative(bend.alpha, bend.beta, Mbend,
          dMbendI, dMbbarJ, d2Mij);
    }
  }
}

void ElasticModel2DFundamentalFormsSTVK::compute_d2psi_dbbar2(
  const double *param, const double[4], const double b_[4],
  const double abar_[4], const double bbar_[4], double d2psi_dbbar2_[16]) const
{
  const double h = param[4];
  const ES::M2d b = ES::Mp<const ES::M2d>(b_);
  const ES::M2d abar = ES::Mp<const ES::M2d>(abar_);
  const ES::M2d bbar = ES::Mp<const ES::M2d>(bbar_);
  const ES::M2d Z = abar.fullPivHouseholderQr().inverse();

  const Lame2D bend = computeLame2D(param[2], param[3]);
  const ES::M2d Mbend = Z * (b - bbar);
  const double bendScale = h * h * h / 12.0;

  Eigen::Map<ES::M4d> H(d2psi_dbbar2_);
  H.setZero();
  for (int i = 0; i < 4; i++) {
    ES::M2d Ei = ES::M2d::Zero();
    Ei.data()[i] = 1.0;
    const ES::M2d dMi = -Z * Ei;

    for (int j = 0; j < 4; j++) {
      ES::M2d Ej = ES::M2d::Zero();
      Ej.data()[j] = 1.0;
      const ES::M2d dMj = -Z * Ej;
      H(i, j) = bendScale *
        stvkCoreSecondDirectionalDerivative(bend.alpha, bend.beta, Mbend,
          dMi, dMj, ES::M2d::Zero());
    }
  }
}


#include "simulation/simulationMesh.h"
#include <initializer_list>
#include <stdexcept>

namespace pgo::SolidDeformationModel {
namespace {
void expectSize(std::span<double> output, std::size_t expected) {
  if (output.size() != expected) throw std::invalid_argument("elastic config default parameter buffer has the wrong size");
}
MaterialParameterSpec channels(std::initializer_list<const char *> names) {
  MaterialParameterSpec spec;
  for (const char *name : names) spec.channelNames.emplace_back(name);
  return spec;
}
}
MaterialParameterSpec KoiterStVKConfig::parameterSpec() const
{
  return channels({"E_membrane", "nu_membrane", "E_bending", "nu_bending", "thickness"});
}
void KoiterStVKConfig::initializeDefaultParameters(const SimulationMesh &mesh, int element, std::span<double> output) const
{
  expectSize(output, 5);
  const auto *mat = dynamic_cast<const SimulationMeshENuhMaterial *>(mesh.getElementMaterial(element, 0));
  if (!mat) throw std::invalid_argument("KoiterStVKConfig requires SimulationMeshENuhMaterial");
  output[0] = mat->getE(); output[1] = mat->getNu(); output[2] = mat->getE(); output[3] = mat->getNu(); output[4] = mat->geth();
}
std::unique_ptr<ElasticModel> KoiterStVKConfig::createModel(const SimulationMesh &, int, const MaterialFrame &) const
{
  return std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
}
}  // namespace pgo::SolidDeformationModel
