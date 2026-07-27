#include "material/elastic/elasticModel3DMooneyRivlin.h"

using namespace pgo;
using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

namespace pgo::SolidDeformationModel
{
// Flatten a 3x3 into a 9x1 (column-major vec)
inline ES::V9d vec(const ES::M3d &A)
{
  ES::V9d v;
  int k = 0;
  for (int j = 0; j < 3; ++j)
    for (int i = 0; i < 3; ++i)
      v(k++) = A(i, j);
  return v;
}

inline double inner(const ES::M3d &A, const ES::M3d &B)
{
  return (A.cwiseProduct(B)).sum();
}

// Action of the Hessian of I2 on dF (linear operator form)
// I2 = 0.5 * (I1^2 - tr(C^2)), with C = F^T F, I1 = tr(C)
// ∇_F I2 = 2 (I1 F - F C)
inline ES::M3d H_I2_apply(const ES::M3d &F, const ES::M3d &C, double I1, const ES::M3d &dF)
{
  const double s = 2.0 * inner(F, dF);  // δI1 = 2 F : dF
  ES::M3d HI2 = ES::M3d::Zero();

  // δ{ ∇I2 } = 2[ (δI1)F + I1 δF - δF C - F(δF^T F + F^T δF) ]
  HI2 = 2.0 * (s * F + I1 * dF - dF * C - F * (dF.transpose() * F + F.transpose() * dF));
  return HI2;
}

// Action of the Hessian of J (i.e., δ(J F^{-T})) on dF
inline ES::M3d H_J_apply(const ES::M3d &FinvT, double J, const ES::M3d &dF)
{
  const double trTerm = inner(FinvT, dF);                  // tr(F^{-1} δF) = F^{-T} : δF
  const double dJ = J * trTerm;                            // δJ
  const ES::M3d dFinvT = -FinvT * dF.transpose() * FinvT;  // δ(F^{-T}) = -F^{-T} δF^T F^{-T}
  return dJ * FinvT + J * dFinvT;                          // δ(JF^{-T})
}

// Action of the Hessian of I1bar and I2bar on dF
inline ES::M3d H_I1bar_apply(const ES::M3d &F, const ES::M3d &FinvT,
  double Jm23, double I1, const ES::M3d &dF)
{
  // Precomputations
  const ES::M3d gradI1 = 2.0 * F;
  const double s = inner(FinvT, dF);             // tr(F^{-1} δF)
  const double dJm23 = (-2.0 / 3.0) * Jm23 * s;  // δ( J^{-2/3} )
  const double dI1 = inner(gradI1, dF);          // δI1 = 2 F : δF
  const ES::M3d dFinvT = -FinvT * dF.transpose() * FinvT;

  // ∇I1bar = J^{-2/3} ∇I1 + I1 * (-2/3) J^{-2/3} F^{-T}
  // δ(∇I1bar) = δ(J^{-2/3}) ∇I1 + J^{-2/3} δ(∇I1)
  //           + δI1 * (-2/3) J^{-2/3} F^{-T}
  //           + I1 * (-2/3) δ(J^{-2/3}) F^{-T}
  //           + I1 * (-2/3) J^{-2/3} δ(F^{-T})
  return dJm23 * gradI1 + Jm23 * (2.0 * dF) + dI1 * (-2.0 / 3.0) * Jm23 * FinvT + I1 * (-2.0 / 3.0) * dJm23 * FinvT + I1 * (-2.0 / 3.0) * Jm23 * dFinvT;
}

inline ES::M3d H_I2bar_apply(const ES::M3d &F, const ES::M3d &C,
  const ES::M3d &FinvT, double Jm43, double I1, double I2,
  const ES::M3d &dF)
{
  const ES::M3d gradI2 = 2.0 * (I1 * F - F * C);
  const double s = inner(FinvT, dF);             // tr(F^{-1} δF)
  const double dJm43 = (-4.0 / 3.0) * Jm43 * s;  // δ(J^{-4/3})
  const double dI2 = inner(gradI2, dF);
  const ES::M3d HI2 = H_I2_apply(F, C, I1, dF);
  const ES::M3d dFinvT = -FinvT * dF.transpose() * FinvT;

  // ∇I2bar = J^{-4/3} ∇I2 + I2 * (-4/3) J^{-4/3} F^{-T}
  return dJm43 * gradI2 + Jm43 * HI2 + dI2 * (-4.0 / 3.0) * Jm43 * FinvT + I2 * (-4.0 / 3.0) * dJm43 * FinvT + I2 * (-4.0 / 3.0) * Jm43 * dFinvT;
}
}  // namespace pgo::SolidDeformationModel

ElasticModel3DMooneyRivlin::ElasticModel3DMooneyRivlin(
  double mu01, double mu10, double v1):
  mu01_(mu01), mu10_(mu10), v1_(v1)
{
}

double ElasticModel3DMooneyRivlin::compute_psi(std::span<const double>, const SpectralState &state) const
{
  const ES::M3d &F = state.F;

  // Kinematics and invariants
  const ES::M3d C = F.transpose() * F;
  const double I1 = C.trace();
  const double I2 = 0.5 * (I1 * I1 - (C * C).trace());
  const double J = F.determinant();
  const double Jm23 = std::pow(J * J, -1.0 / 3.0);
  const double Jm43 = std::pow(J * J * J * J, -1.0 / 3.0);
  const double I1bar = Jm23 * I1;
  const double I2bar = Jm43 * I2;
  const double f1 = I1bar - 3.0;
  const double f2 = I2bar - 3.0;

  return mu10_ * f1 + mu01_ * f2 + v1_ * (J - 1.0) * (J - 1.0);
}

ES::M3d ElasticModel3DMooneyRivlin::compute_P(std::span<const double>, const SpectralState &state) const
{
  const ES::M3d &F = state.F;

  // Kinematics and invariants
  const ES::M3d C = F.transpose() * F;
  const double I1 = C.trace();
  const double I2 = 0.5 * (I1 * I1 - (C * C).trace());
  const double J = F.determinant();
  const ES::M3d Finv = F.fullPivHouseholderQr().inverse();
  const ES::M3d FinvT = Finv.transpose();

  const double Jm23 = std::pow(J * J, -1.0 / 3.0);
  const double Jm43 = std::pow(J * J * J * J, -1.0 / 3.0);

  // Gradient building blocks
  const ES::M3d gradI1 = 2.0 * F;
  const ES::M3d gradI2 = 2.0 * (I1 * F - F * C);
  const ES::M3d gradJ = J * FinvT;  // ∇_F J
  const ES::M3d g1bar = Jm23 * gradI1 + I1 * (-2.0 / 3.0) * Jm23 * FinvT;
  const ES::M3d g2bar = Jm43 * gradI2 + I2 * (-4.0 / 3.0) * Jm43 * FinvT;

  ES::M3d dW_dF = mu10_ * g1bar + mu01_ * g2bar;
  dW_dF += 2.0 * v1_ * (J - 1.0) * gradJ;

  return dW_dF;
}

ES::M9d ElasticModel3DMooneyRivlin::compute_dPdF(std::span<const double>, const SpectralState &state) const
{
  const ES::M3d &F = state.F;

  // Kinematics and invariants
  const ES::M3d C = F.transpose() * F;
  const double I1 = C.trace();
  const double I2 = 0.5 * (I1 * I1 - (C * C).trace());
  const double J = F.determinant();
  const ES::M3d Finv = F.fullPivHouseholderQr().inverse();
  const ES::M3d FinvT = Finv.transpose();

  const double Jm23 = std::pow(J * J, -1.0 / 3.0);
  const double Jm43 = std::pow(J * J * J * J, -1.0 / 3.0);
  // Gradient building blocks
  const ES::M3d gradI1 = 2.0 * F;
  const ES::M3d gradI2 = 2.0 * (I1 * F - F * C);
  const ES::M3d gradJ = J * FinvT;  // ∇_F J
  const ES::M3d g1bar = Jm23 * gradI1 + I1 * (-2.0 / 3.0) * Jm23 * FinvT;
  const ES::M3d g2bar = Jm43 * gradI2 + I2 * (-4.0 / 3.0) * Jm43 * FinvT;

  auto H_apply = [&](const ES::M3d &dF) -> ES::M3d {
    ES::M3d out = ES::M3d::Zero();

    out += mu10_ * H_I1bar_apply(F, FinvT, Jm23, I1, dF);
    out += mu01_ * H_I2bar_apply(F, C, FinvT, Jm43, I1, I2, dF);
    out += 2.0 * v1_ * (
      inner(gradJ, dF) * gradJ + (J - 1.0) * H_J_apply(FinvT, J, dF));

    return out;
  };

  // Assemble 9x9 Hessian by applying H to each basis perturbation δF = E_ij
  ES::M9d d2W_dF2;

  d2W_dF2.setZero();
  for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 3; ++i) {
      ES::M3d dF = ES::M3d::Zero();
      dF(i, j) = 1.0;

      const ES::V9d col = vec(H_apply(dF));

      const int k = i + 3 * j;  // column-major index of (i,j)
      d2W_dF2.col(k) = col;
    }
  }

  return d2W_dF2;
}


#include <stdexcept>
#include <string>

namespace pgo::SolidDeformationModel {
namespace {
// This model has no optimization channels.
}
MaterialChannelSchema MooneyRivlinDefinition::optimizableChannelSchema() const { return {}; }
MaterialChannelSchema MooneyRivlinDefinition::fixedChannelSchema() const { static constexpr std::array<std::string_view, 3> names{"mu01", "mu10", "v1"}; return MaterialChannelSchema(names); }
std::unique_ptr<ElasticModel> MooneyRivlinDefinition::createModel(std::span<const double> values, const MaterialFrame &) const
{
  if (values.size() != 3) throw std::invalid_argument("mooney_rivlin requires fixed channels mu01, mu10, v1");
  return std::make_unique<ElasticModel3DMooneyRivlin>(values[0], values[1], values[2]);
}
}  // namespace pgo::SolidDeformationModel
