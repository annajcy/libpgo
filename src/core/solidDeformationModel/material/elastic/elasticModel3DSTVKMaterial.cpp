#include "material/elastic/elasticModel3DSTVKMaterial.h"

#include "polarDecompositionDerivatives.h"

#include <cmath>
#include <cstring>
#include <iostream>

namespace ES = pgo::EigenSupport;

using namespace pgo::SolidDeformationModel;

inline void crossProductMatrix(const ES::V3d &v, ES::M3d &K)
{
  K << 0, -v(2), v(1),
    v(2), 0, -v(0),
    -v(1), v(0), 0;
}

ElasticModel3DSTVKMaterial::ElasticModel3DSTVKMaterial(double _mu, double _lambda):
  mu(_mu), lambda(_lambda)
{
  C[0] << 0, -1, 0,
    1, 0, 0,
    0, 0, 0;
  C[1] << 0, 0, 0,
    0, 0, 1,
    0, -1, 0;
  C[2] << 0, 0, 1,
    0, 0, 0,
    -1, 0, 0;
}

ElasticModel3DSTVKMaterial::~ElasticModel3DSTVKMaterial()
{
}

ElasticModel3DSTVKMaterial::LowerInvarianceResult
ElasticModel3DSTVKMaterial::computeLowerInvariance(const ES::M3d &F) const
{
  Eigen::JacobiSVD<ES::M3d, Eigen::NoQRPreconditioner> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);

  ES::M3d U = svd.matrixU();
  ES::M3d V = svd.matrixV();
  ES::V3d S = svd.singularValues();

  if (U.determinant() < 0.0) {
    U.col(2) *= -1.0;
    S(2) *= -1.0;
  }
  if (V.determinant() < 0.0) {
    V.col(2) *= -1.0;
    S(2) *= -1.0;
  }

  // U V^T and the signed stretch tensor V S V^T are the lower-invariance factors.
  ES::M3d SS = V * S.asDiagonal() * V.transpose();

  LowerInvarianceResult result;
  result.invariants = ES::V3d(SS.trace(), (SS * SS).trace(), SS.determinant());
  result.rotation = U * V.transpose();
  result.signedStretches = S;
  result.U = U;
  result.V = V;
  return result;
}

double ElasticModel3DSTVKMaterial::compute_psi(std::span<const double>, const SpectralState &state) const
{
  ES::M3d F = state.F;
  const LowerInvarianceResult invariance = computeLowerInvariance(F);
  const ES::V3d &i = invariance.invariants;

  double I1 = i[0];
  double I2 = i[1];
  double I3 = i[2];

  double energy = lambda / 8.0 * (I2 - 3) * (I2 - 3) + mu / 8.0 * (8 * I1 * I3 + I2 * I2 + 2 * I1 * I1 * I2 - 4 * I2 - I1 * I1 * I1 * I1 + 6);

  return energy;
}

ES::M3d ElasticModel3DSTVKMaterial::compute_P(std::span<const double>, const SpectralState &state) const
{
  ES::M3d F = state.F;
  const LowerInvarianceResult invariance = computeLowerInvariance(F);
  const ES::M3d &R = invariance.rotation;
  const ES::V3d &i = invariance.invariants;

  double I1 = i[0];
  double I2 = i[1];
  double I3 = i[2];

  double dpsi_dI1 = mu * I3 + 0.5 * mu * I1 * I2 - 0.5 * mu * I1 * I1 * I1;
  double dpsi_dI2 = 0.25 * lambda * (I2 - 3) + 0.25 * mu * I2 + 0.25 * mu * I1 * I1 - 0.5 * mu;
  double dpsi_dI3 = mu * I1;

  ES::V9d dI1_dF, dI2_dF, dI3_dF;
  for (int i = 0; i < 3; i++) {
    dI1_dF.segment<3>(3 * i) = R.col(i);
    dI2_dF.segment<3>(3 * i) = 2 * F.col(i);
  }
  dI3_dF.segment<3>(0) = F.col(1).cross(F.col(2));
  dI3_dF.segment<3>(3) = F.col(2).cross(F.col(0));
  dI3_dF.segment<3>(6) = F.col(0).cross(F.col(1));

  ES::V9d g = dpsi_dI1 * dI1_dF + dpsi_dI2 * dI2_dF + dpsi_dI3 * dI3_dF;

  return Eigen::Map<const ES::M3d>(g.data());
}

ES::M9d ElasticModel3DSTVKMaterial::compute_dPdF(std::span<const double>, const SpectralState &state) const
{
  ES::M3d F = state.F;
  const LowerInvarianceResult invariance = computeLowerInvariance(F);
  const ES::M3d &R = invariance.rotation;
  const ES::M3d &U = invariance.U;
  const ES::M3d &V = invariance.V;
  const ES::V3d &s = invariance.signedStretches;
  const ES::V3d &i = invariance.invariants;

  double I1 = i[0];
  double I2 = i[1];
  double I3 = i[2];

  double dpsi_dI1 = mu * I3 + 0.5 * mu * I1 * I2 - 0.5 * mu * I1 * I1 * I1;
  double dpsi_dI2 = 0.25 * lambda * (I2 - 3) + 0.25 * mu * I2 + 0.25 * mu * I1 * I1 - 0.5 * mu;
  double dpsi_dI3 = mu * I1;

  ES::V9d dI1_dF, dI2_dF, dI3_dF;
  for (int i = 0; i < 3; i++) {
    dI1_dF.segment<3>(3 * i) = R.col(i);
    dI2_dF.segment<3>(3 * i) = 2 * F.col(i);
  }
  dI3_dF.segment<3>(0) = F.col(1).cross(F.col(2));
  dI3_dF.segment<3>(3) = F.col(2).cross(F.col(0));
  dI3_dF.segment<3>(6) = F.col(0).cross(F.col(1));

  ES::V3d eigv;
  eigv[0] = 2.0 / (s(0) + s(1));
  eigv[1] = 2.0 / (s(1) + s(2));
  eigv[2] = 2.0 / (s(2) + s(0));

  std::array<ES::M3d, 3> Q;
  Q[0] = 1.0 / sqrt(2) * U * C[0] * V.transpose();
  Q[1] = 1.0 / sqrt(2) * U * C[1] * V.transpose();
  Q[2] = 1.0 / sqrt(2) * U * C[2] * V.transpose();

  std::array<ES::V9d, 3> q;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      q[i].segment<3>(3 * j) = Q[i].col(j);
    }
  }

  ES::M9d d2I1_dF2;
  d2I1_dF2.setZero();

  for (int i = 0; i < 3; i++) {
    d2I1_dF2 += eigv[i] * q[i] * q[i].transpose();
  }

  ES::M9d d2I2_dF2 = 2.0 * ES::M9d::Identity();

  ES::M9d d2I3_dF2;
  d2I3_dF2.setZero();

  ES::M3d f0_hat, f1_hat, f2_hat;
  crossProductMatrix(F.col(0), f0_hat);
  crossProductMatrix(F.col(1), f1_hat);
  crossProductMatrix(F.col(2), f2_hat);
  d2I3_dF2.block<3, 3>(0, 3) = -f2_hat;
  d2I3_dF2.block<3, 3>(0, 6) = f1_hat;
  d2I3_dF2.block<3, 3>(3, 0) = f2_hat;
  d2I3_dF2.block<3, 3>(3, 6) = -f0_hat;
  d2I3_dF2.block<3, 3>(6, 0) = -f1_hat;
  d2I3_dF2.block<3, 3>(6, 3) = f0_hat;

  ES::M9d H;
  H.setZero();
  H += dpsi_dI1 * d2I1_dF2 + dpsi_dI2 * d2I2_dF2 + dpsi_dI3 * d2I3_dF2;

  double d2psi_dI1dI1 = 0.5 * mu * I2 - 1.5 * mu * I1 * I1;
  double d2psi_dI1dI2 = 0.5 * mu * I1;
  double d2psi_dI1dI3 = mu;
  double d2psi_dI2dI2 = 0.25 * lambda + 0.25 * mu;

  H += d2psi_dI1dI1 * dI1_dF * dI1_dF.transpose();
  H += d2psi_dI1dI2 * (dI1_dF * dI2_dF.transpose() + dI2_dF * dI1_dF.transpose());
  H += d2psi_dI1dI3 * (dI1_dF * dI3_dF.transpose() + dI3_dF * dI1_dF.transpose());
  H += d2psi_dI2dI2 * dI2_dF * dI2_dF.transpose();

  return H;
}


#include <stdexcept>
#include <string>

namespace pgo::SolidDeformationModel {
namespace {
}
MaterialChannelSchema StVKDefinition::optimizableChannelSchema() const { return {}; }
MaterialChannelSchema StVKDefinition::fixedChannelSchema() const { static constexpr std::array<std::string_view, 2> names{"E", "nu"}; return MaterialChannelSchema(names); }
std::unique_ptr<ElasticModel> StVKDefinition::createModelFromFixed(std::span<const double> values, const MaterialFrame &) const
{
  if (values.size() != 2) throw std::invalid_argument("stvk requires fixed channels E, nu");
  const double E = values[0], nu = values[1];
  return std::make_unique<ElasticModel3DSTVKMaterial>(E / (2 * (1 + nu)), (nu * E) / ((1 + nu) * (1 - 2 * nu)));
}
}  // namespace pgo::SolidDeformationModel
