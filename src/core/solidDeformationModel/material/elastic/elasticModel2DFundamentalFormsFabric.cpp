#include "material/elastic/elasticModel2DFundamentalFormsFabric.h"
#include <array>

namespace pgo
{
namespace ES = pgo::EigenSupport;

namespace SolidDeformationModel
{

inline ES::V2d normalizeWithMetric(const ES::V2d &v, const ES::M2d &A0)
{
  const double n = std::sqrt(v.dot(A0 * v));

  if (!(n > 0.0))
    throw std::runtime_error("Direction has nonpositive metric length.");

  return v / n;
}

inline ES::V3d symToMandel(const ES::M2d &S)
{
  ES::V3d v;

  v << S(0, 0), S(1, 1), std::sqrt(2.0) * S(0, 1);

  return v;
}

inline ES::V4d vecCM(const ES::M2d &X)
{
  ES::V4d v;
  v << X(0, 0), X(1, 0), X(0, 1), X(1, 1);
  return v;
}

inline ES::M2d mandelToSym(const ES::V3d &v)
{
  ES::M2d S;

  S.setZero();
  S(0, 0) = v(0);
  S(1, 1) = v(1);
  S(0, 1) = S(1, 0) = v(2) / std::sqrt(2.0);

  return S;
}

inline ES::M4d pack_Mandel3To4(const ES::M3d &M3)
{
  // int index[4] = { 0, 2, 2, 1 };
  // ES::M4d M4;
  // M4.setZero();
  // for (int i = 0; i < 4; i++) {
  //   for (int j = 0; j < 4; j++) {
  //     M4(i, j) = M3(index[i], index[j]);
  //   }
  // }

  ES::M4d M4;
  Eigen::Matrix<double, 3, 4> S;
  S << 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
    0.0, 1 / std::sqrt(2.0), 1 / std::sqrt(2.0), 0.0;

  M4 = S.transpose() * M3 * S;

  return M4;
}

inline ES::M2d symmetrize(const ES::M2d &M)
{
  return 0.5 * (M + M.transpose());
}

// ===================== Smooth ramp (C1) =====================

inline double smoothRamp(double x, double eps)
{
  const double e = std::max(eps, 1e-16);  // guard
  const double srt = std::sqrt(x * x + e * e);
  return 0.5 * (x + srt);
}

inline double smoothRampGrad(double x, double eps)
{
  const double e = std::max(eps, 1e-16);  // guard
  const double srt = std::sqrt(x * x + e * e);
  return 0.5 * (1.0 + x / srt);
}

inline double smoothRampHess(double x, double eps)
{
  const double e = std::max(eps, 1e-16);  // guard
  const double srt = std::sqrt(x * x + e * e);
  return 0.5 * (e * e) / (srt * srt * srt);
}

double ElasticModel2DFundamentalFormsFabric::compute_psi_a(std::span<const double> param, const ES::M2d &a_, const ES::M2d &abar_) const
{
  // Unpack parameters
  // Tiny isotropic matrix term (optional)
  double mu0 = param[0];

  // Warp (I4) exponential fiber law
  double k1_4 = param[1];
  double k2_4 = param[2];

  // Weft (I6) exponential fiber law
  double k1_6 = param[3];
  double k2_6 = param[4];

  // Shear trellising with saturation: (ks/2) * d8^2 / (1 + alpha d8^2)
  double ks = param[5];
  double alpha = param[6];

  // Bending (orthotropic)
  double kappa11 = param[7];
  double kappa22 = param[8];
  double kappa12 = param[9];

  double I8_0 = param[10];

  double h = param[11];  // shell thickness

  const ES::M2d &a = a_;
  const ES::M2d &abar = abar_;
  ES::M2d abarInv = abar.inverse();
  ES::M2d C = abarInv * a;

  ES::V2d e1, e2;
  e1 = normalizeWithMetric(warpDir, abar);
  e2 = normalizeWithMetric(weftDir, abar);

  double I1 = C.trace();
  // Fiber stretch / shear invariants use the current metric a (not C = abarInv*a)
  // so that, with e1/e2 normalized in the rest metric abar, they equal 1 / the rest
  // shear at the undeformed configuration (a == abar) for ANY mesh scale. Using C
  // here made I4 = e1.dot(e1) at rest, which for a sub-unit-scale mesh blows up the
  // exp() fiber law to +inf.
  double I4 = e1.dot(a * e1);
  double I6 = e2.dot(a * e2);
  double I8 = e1.dot(a * e2);

  // --- Membrane energy pieces (C1 fibers + shear saturation) ---
  // Isotropic matrix (tiny)
  const double psi_iso = 0.5 * mu0 * (I1 - 2.0);

  // Warp fiber (I4)
  const double r4 = smoothRamp(I4 - 1.0, eps4);  // s4, ds4, d2s4
  const double E4 = std::exp(k2_4 * r4 * r4);
  const double psi4 = (k1_4 / (2.0 * k2_4)) * (E4 - 1.0);

  // Weft fiber (I6)
  const double r6 = smoothRamp(I6 - 1.0, eps6);
  const double E6 = std::exp(k2_6 * r6 * r6);
  const double psi6 = (k1_6 / (2.0 * k2_6)) * (E6 - 1.0);

  // Shear trellising (I8) with saturation
  const double d8 = I8 - I8_0;
  const double denom = 1.0 + alpha * d8 * d8;
  const double psi8 = 0.5 * ks * (d8 * d8) / denom;

  // Total membrane energy
  const double psi_mem = psi_iso + psi4 + psi6 + psi8;

  return h * psi_mem;
}

double ElasticModel2DFundamentalFormsFabric::compute_psi_b(std::span<const double> param, const ES::M2d &b_, const ES::M2d &abar_, const ES::M2d &bbar_) const
{
  // Unpack parameters
  // Tiny isotropic matrix term (optional)
  double mu0 = param[0];

  // Warp (I4) exponential fiber law
  double k1_4 = param[1];
  double k2_4 = param[2];

  // Weft (I6) exponential fiber law
  double k1_6 = param[3];
  double k2_6 = param[4];

  // Shear trellising with saturation: (ks/2) * d8^2 / (1 + alpha d8^2)
  double ks = param[5];
  double alpha = param[6];

  // Bending (orthotropic)
  double kappa11 = param[7];
  double kappa22 = param[8];
  double kappa12 = param[9];

  double I8_0 = param[10];

  double h = param[11];  // shell thickness

  const ES::M2d &b = b_;
  const ES::M2d &abar = abar_;
  const ES::M2d &bbar = bbar_;

  ES::M2d abarInv = abar.inverse();
  ES::M2d b_eff = abarInv * (b - bbar);

  // -------- Bending (correct Mandel scaling) --------
  // ψ_b = 0.5*(k11 b11^2 + k22 b22^2 + 2 k12 b12^2)
  // δψ_b = k11 b11 δb11 + k22 b22 δb22 + 2 k12 b12 δb12

  const double psi_bend =
    0.5 * (kappa11 * b_eff(0, 0) * b_eff(0, 0) + kappa22 * b_eff(1, 1) * b_eff(1, 1) + 2.0 * kappa12 * b_eff(0, 1) * b_eff(0, 1));

  return h * h * h / 12 * psi_bend;
}

ES::M2d ElasticModel2DFundamentalFormsFabric::compute_dpsi_da(std::span<const double> param, const ES::M2d &a_, const ES::M2d &abar_) const
{
  // Unpack parameters
  // Tiny isotropic matrix term (optional)
  double mu0 = param[0];

  // Warp (I4) exponential fiber law
  double k1_4 = param[1];
  double k2_4 = param[2];

  // Weft (I6) exponential fiber law
  double k1_6 = param[3];
  double k2_6 = param[4];

  // Shear trellising with saturation: (ks/2) * d8^2 / (1 + alpha d8^2)
  double ks = param[5];
  double alpha = param[6];

  // Bending (orthotropic)
  double kappa11 = param[7];
  double kappa22 = param[8];
  double kappa12 = param[9];

  double I8_0 = param[10];

  double h = param[11];  // shell thickness

  const ES::M2d &a = a_;
  const ES::M2d &abar = abar_;
  ES::M2d abarInv = abar.inverse();
  ES::M2d C = abarInv * a;

  ES::V2d e1, e2;
  e1 = normalizeWithMetric(warpDir, abar);
  e2 = normalizeWithMetric(weftDir, abar);

  // Fiber stretch / shear invariants use the current metric a (not C = abarInv*a)
  // so that, with e1/e2 normalized in the rest metric abar, they equal 1 / the rest
  // shear at the undeformed configuration (a == abar) for ANY mesh scale. Using C
  // here made I4 = e1.dot(e1) at rest, which for a sub-unit-scale mesh blows up the
  // exp() fiber law to +inf.
  double I4 = e1.dot(a * e1);
  double I6 = e2.dot(a * e2);
  double I8 = e1.dot(a * e2);

  ES::M2d dI1_da = symmetrize(abarInv);
  // dI4/da = e1 e1^T (etc.): I4 = e1^T a e1 is linear in a, so no abarInv weighting
  // and no second-order term. Must stay consistent with the value formulas above.
  ES::M2d dI4_da = symmetrize(e1 * e1.transpose());
  ES::M2d dI6_da = symmetrize(e2 * e2.transpose());
  ES::M2d dI8_da = symmetrize(e2 * e1.transpose());

  // --- Membrane energy pieces (C1 fibers + shear saturation) ---
  // Isotropic matrix (tiny)
  const double dpsi_dI1 = 0.5 * mu0;

  // Warp fiber (I4)
  const double r4 = smoothRamp(I4 - 1.0, eps4);  // s4, ds4, d2s4
  const double dr4 = smoothRampGrad(I4 - 1.0, eps4);
  const double E4 = std::exp(k2_4 * r4 * r4);
  const double dpsi_dI4 = k1_4 * r4 * E4 * dr4;

  // Weft fiber (I6)
  const double r6 = smoothRamp(I6 - 1.0, eps6);
  const double dr6 = smoothRampGrad(I6 - 1.0, eps6);
  const double E6 = std::exp(k2_6 * r6 * r6);
  const double dpsi_dI6 = k1_6 * r6 * E6 * dr6;

  // Shear trellising (I8) with saturation
  const double d8 = I8 - I8_0;
  const double denom = 1.0 + alpha * d8 * d8;
  const double dpsi_dI8 = ks * d8 / (denom * denom);

  // -------- Gradient wrt 'a' (2x2), Hessian Haa (3x3 Mandel) --------
  ES::M2d grad_a_mat = dpsi_dI1 * dI1_da + dpsi_dI4 * dI4_da + dpsi_dI6 * dI6_da + dpsi_dI8 * dI8_da;
  return grad_a_mat * h;
}

ES::M2d ElasticModel2DFundamentalFormsFabric::compute_dpsi_db(std::span<const double> param, const ES::M2d &b_, const ES::M2d &abar_, const ES::M2d &bbar_) const
{
  // Unpack parameters
  // Tiny isotropic matrix term (optional)
  double mu0 = param[0];

  // Warp (I4) exponential fiber law
  double k1_4 = param[1];
  double k2_4 = param[2];

  // Weft (I6) exponential fiber law
  double k1_6 = param[3];
  double k2_6 = param[4];

  // Shear trellising with saturation: (ks/2) * d8^2 / (1 + alpha d8^2)
  double ks = param[5];
  double alpha = param[6];

  // Bending (orthotropic)
  double kappa11 = param[7];
  double kappa22 = param[8];
  double kappa12 = param[9];

  double I8_0 = param[10];

  double h = param[11];  // shell thickness

  const ES::M2d &abar = abar_;
  const ES::M2d &bbar = bbar_;
  const ES::M2d &b = b_;

  ES::M2d abarInv = abar.inverse();
  ES::M2d b_eff = abarInv * (b - bbar);

  // -------- Bending (correct Mandel scaling) --------
  // ψ_b = 0.5*(k11 b11^2 + k22 b22^2 + 2 k12 b12^2)
  // δψ_b = k11 b11 δb11 + k22 b22 δb22 + 2 k12 b12 δb12
  ES::M2d grad_b_mat = ES::M2d::Zero();
  grad_b_mat(0, 0) = kappa11 * b_eff(0, 0);
  grad_b_mat(1, 1) = kappa22 * b_eff(1, 1);
  grad_b_mat(0, 1) = grad_b_mat(1, 0) = kappa12 * b_eff(0, 1);  // NOTE: not 2*kappa12

  grad_b_mat *= h * h * h / 12;
  return grad_b_mat;
}

ES::M4d ElasticModel2DFundamentalFormsFabric::compute_d2psi_da2(std::span<const double> param, const ES::M2d &a_, const ES::M2d &abar_) const
{
  // Unpack parameters
  // Tiny isotropic matrix term (optional)
  // double mu0 = param[0];

  // Warp (I4) exponential fiber law
  double k1_4 = param[1];
  double k2_4 = param[2];

  // Weft (I6) exponential fiber law
  double k1_6 = param[3];
  double k2_6 = param[4];

  // Shear trellising with saturation: (ks/2) * d8^2 / (1 + alpha d8^2)
  double ks = param[5];
  double alpha = param[6];

  // Bending (orthotropic)
  double kappa11 = param[7];
  double kappa22 = param[8];
  double kappa12 = param[9];

  double I8_0 = param[10];

  double h = param[11];  // shell thickness

  const ES::M2d &a = a_;
  const ES::M2d &abar = abar_;
  ES::M2d abarInv = abar.inverse();
  ES::M2d C = abarInv * a;

  ES::V2d e1, e2;
  e1 = normalizeWithMetric(warpDir, abar);
  e2 = normalizeWithMetric(weftDir, abar);

  // double I1 = C.trace();
  // Fiber stretch / shear invariants use the current metric a (not C = abarInv*a)
  // so that, with e1/e2 normalized in the rest metric abar, they equal 1 / the rest
  // shear at the undeformed configuration (a == abar) for ANY mesh scale. Using C
  // here made I4 = e1.dot(e1) at rest, which for a sub-unit-scale mesh blows up the
  // exp() fiber law to +inf.
  double I4 = e1.dot(a * e1);
  double I6 = e2.dot(a * e2);
  double I8 = e1.dot(a * e2);

  ES::M2d dI1_da = symmetrize(abarInv);
  // dI4/da = e1 e1^T (etc.): I4 = e1^T a e1 is linear in a, so no abarInv weighting
  // and no second-order term. Must stay consistent with the value formulas above.
  ES::M2d dI4_da = symmetrize(e1 * e1.transpose());
  ES::M2d dI6_da = symmetrize(e2 * e2.transpose());
  ES::M2d dI8_da = symmetrize(e2 * e1.transpose());

  ES::V3d dI1_m = symToMandel(dI1_da);
  ES::V3d dI4_m = symToMandel(dI4_da);
  ES::V3d dI6_m = symToMandel(dI6_da);
  ES::V3d dI8_m = symToMandel(dI8_da);

  // --- Membrane energy pieces (C1 fibers + shear saturation) ---
  // Isotropic matrix (tiny)
  const double d2psi_dI1 = 0.0;

  // Warp fiber (I4)
  const double r4 = smoothRamp(I4 - 1.0, eps4);  // s4, ds4, d2s4
  const double dr4 = smoothRampGrad(I4 - 1.0, eps4);
  const double d2r4 = smoothRampHess(I4 - 1.0, eps4);

  const double E4 = std::exp(k2_4 * r4 * r4);
  const double d2psi_dI4 = k1_4 * ((dr4 * dr4) * E4 * (1.0 + 2.0 * k2_4 * r4 * r4) + r4 * E4 * d2r4);

  // Weft fiber (I6)
  const double r6 = smoothRamp(I6 - 1.0, eps6);
  const double dr6 = smoothRampGrad(I6 - 1.0, eps6);
  const double d2r6 = smoothRampHess(I6 - 1.0, eps6);

  const double E6 = std::exp(k2_6 * r6 * r6);
  const double d2psi_dI6 =
    k1_6 * ((dr6 * dr6) * E6 * (1.0 + 2.0 * k2_6 * r6 * r6) + r6 * E6 * d2r6);

  // Shear trellising (I8) with saturation
  const double d8 = I8 - I8_0;
  const double denom = 1.0 + alpha * d8 * d8;
  const double d2psi_dI8 = ks * (1.0 - 3.0 * alpha * d8 * d8) / std::pow(denom, 3);

  ES::M3d Haa = ES::M3d::Zero();
  Haa.noalias() += d2psi_dI1 * (dI1_m * dI1_m.transpose());  // zero anyway
  Haa.noalias() += d2psi_dI4 * (dI4_m * dI4_m.transpose());
  Haa.noalias() += d2psi_dI6 * (dI6_m * dI6_m.transpose());
  Haa.noalias() += d2psi_dI8 * (dI8_m * dI8_m.transpose());

  ES::M4d Haa4;
  Haa4 = pack_Mandel3To4(Haa) * h;

  return Haa4;
}

ES::M4d ElasticModel2DFundamentalFormsFabric::compute_d2psi_db2(std::span<const double> param, const ES::M2d &b_, const ES::M2d &abar_, const ES::M2d &bbar_) const
{
  // Unpack parameters
  // Tiny isotropic matrix term (optional)
  // double mu0 = param[0];

  // Warp (I4) exponential fiber law
  double k1_4 = param[1];
  double k2_4 = param[2];

  // Weft (I6) exponential fiber law
  double k1_6 = param[3];
  double k2_6 = param[4];

  // Shear trellising with saturation: (ks/2) * d8^2 / (1 + alpha d8^2)
  double ks = param[5];
  double alpha = param[6];

  // Bending (orthotropic)
  double kappa11 = param[7];
  double kappa22 = param[8];
  double kappa12 = param[9];

  double I8_0 = param[10];

  double h = param[11];  // shell thickness

  const ES::M2d &abar = abar_;
  const ES::M2d &bbar = bbar_;
  const ES::M2d &b = b_;

  ES::M2d abarInv = abar.inverse();
  ES::M2d b_eff = abarInv * (b - bbar);  ///???

  // In Mandel, the Hbb diag must be [k11, k22, k12] to match energy correctly.
  ES::M3d Hbb = ES::M3d::Zero();
  Hbb(0, 0) = kappa11;
  Hbb(1, 1) = kappa22;
  Hbb(2, 2) = kappa12;  // NOTE: not 2*kappa12

  ES::M4d Hbb4;
  Hbb4 = pack_Mandel3To4(Hbb);

  Hbb4 *= h * h * h / 12;

  return Hbb4;
}

void ElasticModel2DFundamentalFormsFabric::compute_d2psi_da_dparam(
  std::span<const double> param, const ES::M2d &a_, const ES::M2d &abar_,
  ES::RefMatXd d2psi_dadparam) const
{
  double mu0 = param[0];
  double k1_4 = param[1];
  double k2_4 = param[2];
  double k1_6 = param[3];
  double k2_6 = param[4];
  double ks = param[5];
  double alpha = param[6];
  double I8_0 = param[10];
  double h = param[11];

  const ES::M2d &a = a_;
  const ES::M2d &abar = abar_;
  ES::M2d abarInv = abar.inverse();

  ES::V2d e1 = normalizeWithMetric(warpDir, abar);
  ES::V2d e2 = normalizeWithMetric(weftDir, abar);

  const double I4 = e1.dot(a * e1);
  const double I6 = e2.dot(a * e2);
  const double I8 = e1.dot(a * e2);

  const ES::M2d dI1_da = symmetrize(abarInv);
  const ES::M2d dI4_da = symmetrize(e1 * e1.transpose());
  const ES::M2d dI6_da = symmetrize(e2 * e2.transpose());
  const ES::M2d dI8_da = symmetrize(e2 * e1.transpose());

  const double dpsi_dI1 = 0.5 * mu0;

  const double r4 = smoothRamp(I4 - 1.0, eps4);
  const double dr4 = smoothRampGrad(I4 - 1.0, eps4);
  const double E4 = std::exp(k2_4 * r4 * r4);
  const double dpsi_dI4 = k1_4 * r4 * E4 * dr4;

  const double r6 = smoothRamp(I6 - 1.0, eps6);
  const double dr6 = smoothRampGrad(I6 - 1.0, eps6);
  const double E6 = std::exp(k2_6 * r6 * r6);
  const double dpsi_dI6 = k1_6 * r6 * E6 * dr6;

  const double d8 = I8 - I8_0;
  const double denom = 1.0 + alpha * d8 * d8;
  const double dpsi_dI8 = ks * d8 / (denom * denom);
  const double d2psi_dI8 = ks * (1.0 - 3.0 * alpha * d8 * d8) / std::pow(denom, 3);

  const ES::M2d grad_a_mat =
    dpsi_dI1 * dI1_da + dpsi_dI4 * dI4_da + dpsi_dI6 * dI6_da + dpsi_dI8 * dI8_da;

  d2psi_dadparam.setZero();

  d2psi_dadparam.col(0) = vecCM(dI1_da) * (0.5 * h);
  d2psi_dadparam.col(1) = vecCM(dI4_da) * (r4 * E4 * dr4 * h);
  d2psi_dadparam.col(2) = vecCM(dI4_da) * (k1_4 * r4 * r4 * r4 * E4 * dr4 * h);
  d2psi_dadparam.col(3) = vecCM(dI6_da) * (r6 * E6 * dr6 * h);
  d2psi_dadparam.col(4) = vecCM(dI6_da) * (k1_6 * r6 * r6 * r6 * E6 * dr6 * h);
  d2psi_dadparam.col(5) = vecCM(dI8_da) * (d8 / (denom * denom) * h);
  d2psi_dadparam.col(6) = vecCM(dI8_da) * (-2.0 * ks * d8 * d8 * d8 / std::pow(denom, 3) * h);
  d2psi_dadparam.col(10) = vecCM(dI8_da) * (-d2psi_dI8 * h);
  d2psi_dadparam.col(11) = vecCM(grad_a_mat);
}

void ElasticModel2DFundamentalFormsFabric::compute_d2psi_db_dparam(
  std::span<const double> param, const ES::M2d &b_, const ES::M2d &abar_, const ES::M2d &bbar_,
  ES::RefMatXd d2psi_dbdparam) const
{
  double kappa11 = param[7];
  double kappa22 = param[8];
  double kappa12 = param[9];
  double h = param[11];

  const ES::M2d &abar = abar_;
  const ES::M2d &bbar = bbar_;
  const ES::M2d &b = b_;

  ES::M2d abarInv = abar.inverse();
  ES::M2d b_eff = abarInv * (b - bbar);

  d2psi_dbdparam.setZero();

  const double h3_over_12 = h * h * h / 12.0;

  ES::M2d col = ES::M2d::Zero();
  col(0, 0) = b_eff(0, 0);
  d2psi_dbdparam.col(7) = vecCM(col) * h3_over_12;

  col.setZero();
  col(1, 1) = b_eff(1, 1);
  d2psi_dbdparam.col(8) = vecCM(col) * h3_over_12;

  col.setZero();
  col(0, 1) = col(1, 0) = b_eff(0, 1);
  d2psi_dbdparam.col(9) = vecCM(col) * h3_over_12;

  ES::M2d grad_b_mat = ES::M2d::Zero();
  grad_b_mat(0, 0) = kappa11 * b_eff(0, 0);
  grad_b_mat(1, 1) = kappa22 * b_eff(1, 1);
  grad_b_mat(0, 1) = grad_b_mat(1, 0) = kappa12 * b_eff(0, 1);
  d2psi_dbdparam.col(11) = vecCM(grad_b_mat) * (3.0 * h * h / 12.0);
}

}  // namespace SolidDeformationModel
}  // namespace pgo


#include <algorithm>
#include <stdexcept>

namespace pgo::SolidDeformationModel {
MaterialChannelSchema KoiterFabricDefinition::optimizableChannelSchema() const
{
  static constexpr std::array<std::string_view, 12> names{
    "mu0", "k1_4", "k2_4", "k1_6", "k2_6", "ks",
    "alpha", "kappa11", "kappa22", "kappa12", "I8_0", "h"};
  return MaterialChannelSchema(names);
}
std::unique_ptr<ElasticModel> KoiterFabricDefinition::createModel(std::span<const double> values, const MaterialFrame &) const
{
  if (!values.empty()) throw std::invalid_argument("koiter_fabric has no fixed channels");
  return std::make_unique<ElasticModel2DFundamentalFormsFabric>(EigenSupport::V2d(1, 0), EigenSupport::V2d(0, 1));
}
}  // namespace pgo::SolidDeformationModel
