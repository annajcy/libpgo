#include "material/elastic/deformationGradient/spectral/elasticModel3DValanisLandel.h"

#include <array>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace pgo::SolidDeformationModel
{
namespace
{
namespace ES = EigenSupport;

constexpr std::array<std::array<int, 2>, 3> stretchPairs = {
  std::array<int, 2>{ 0, 1 },
  std::array<int, 2>{ 1, 2 },
  std::array<int, 2>{ 2, 0 }
};

int checkedParameterCount(
  const ElasticModel1D &model,
  const char *modelName)
{
  const int count = model.getNumParameters();
  if (count < 0)
    throw std::invalid_argument(
      std::string("ElasticModel3DValanisLandel ") +
      modelName + " model returned a negative parameter count");
  return count;
}

}  // namespace

ElasticModel3DValanisLandel::ElasticModel3DValanisLandel(
  std::shared_ptr<const ElasticModel1D> f,
  std::shared_ptr<const ElasticModel1D> g,
  std::shared_ptr<const ElasticModel1D> h):
  f_(std::move(f)),
  g_(std::move(g)),
  h_(std::move(h))
{
  if (!f_ || !g_ || !h_)
    throw std::invalid_argument(
      "ElasticModel3DValanisLandel requires non-null f, g, and h models");

  fParameterCount_ = checkedParameterCount(*f_, "f");
  gParameterCount_ = checkedParameterCount(*g_, "g");
  hParameterCount_ = checkedParameterCount(*h_, "h");

  const long long total =
    static_cast<long long>(fParameterCount_) +
    static_cast<long long>(gParameterCount_) +
    static_cast<long long>(hParameterCount_);
  if (total > std::numeric_limits<int>::max())
    throw std::overflow_error(
      "ElasticModel3DValanisLandel parameter count exceeds int range");
}

ElasticModel3DValanisLandel::ParameterSpans
ElasticModel3DValanisLandel::splitParameters(
  std::span<const double> param) const
{
  if (param.size() !=
    static_cast<std::size_t>(getNumParameters()))
    throw std::invalid_argument(
      "ElasticModel3DValanisLandel parameter count does not "
      "match its f, g, and h models");

  const std::size_t fCount =
    static_cast<std::size_t>(fParameterCount_);
  const std::size_t gCount =
    static_cast<std::size_t>(gParameterCount_);
  return {
    param.first(fCount),
    param.subspan(fCount, gCount),
    param.subspan(fCount + gCount)
  };
}

void ElasticModel3DValanisLandel::compute_dpsi_dparams(
  std::span<const double> param,
  const SpectralState &state,
  ES::RefVecXd dpsiDparam) const
{
  const ParameterSpans parameters = splitParameters(param);
  const int parameterCount = getNumParameters();
  if (dpsiDparam.size() != parameterCount)
    throw std::invalid_argument(
      "ElasticModel3DValanisLandel parameter gradient has the wrong size");

  const ES::V3d &s = state.stretches;
  validatePositiveStretches(s);
  dpsiDparam.setZero();

  for (int a = 0; a < fParameterCount_; ++a)
    for (int i = 0; i < 3; ++i)
      dpsiDparam(a) +=
        f_->compute_dpsi_dparam(parameters.f, a, s(i));

  const int gOffset = fParameterCount_;
  for (int a = 0; a < gParameterCount_; ++a)
    for (const auto &pair : stretchPairs)
      dpsiDparam(gOffset + a) +=
        g_->compute_dpsi_dparam(
          parameters.g, a, s(pair[0]) * s(pair[1]));

  const int hOffset = gOffset + gParameterCount_;
  const double J = s.prod();
  for (int a = 0; a < hParameterCount_; ++a)
    dpsiDparam(hOffset + a) =
      h_->compute_dpsi_dparam(parameters.h, a, J);
}

void ElasticModel3DValanisLandel::compute_d2psi_dparams2(
  std::span<const double> param,
  const SpectralState &state,
  ES::RefMatXd d2psiDparam2) const
{
  const ParameterSpans parameters = splitParameters(param);
  const int parameterCount = getNumParameters();
  if (d2psiDparam2.rows() != parameterCount ||
    d2psiDparam2.cols() != parameterCount)
    throw std::invalid_argument(
      "ElasticModel3DValanisLandel parameter Hessian has the wrong shape");

  const ES::V3d &s = state.stretches;
  validatePositiveStretches(s);
  d2psiDparam2.setZero();

  for (int a = 0; a < fParameterCount_; ++a)
    for (int b = 0; b < fParameterCount_; ++b)
      for (int i = 0; i < 3; ++i)
        d2psiDparam2(a, b) +=
          f_->compute_d2psi_dparam2(parameters.f, a, b, s(i));

  const int gOffset = fParameterCount_;
  for (int a = 0; a < gParameterCount_; ++a)
    for (int b = 0; b < gParameterCount_; ++b)
      for (const auto &pair : stretchPairs)
        d2psiDparam2(gOffset + a, gOffset + b) +=
          g_->compute_d2psi_dparam2(
            parameters.g, a, b, s(pair[0]) * s(pair[1]));

  const int hOffset = gOffset + gParameterCount_;
  const double J = s.prod();
  for (int a = 0; a < hParameterCount_; ++a)
    for (int b = 0; b < hParameterCount_; ++b)
      d2psiDparam2(hOffset + a, hOffset + b) =
        h_->compute_d2psi_dparam2(parameters.h, a, b, J);
}

void ElasticModel3DValanisLandel::compute_dP_dparams(
  std::span<const double> param,
  const SpectralState &state,
  ES::RefMatXd dP_dparam) const
{
  const ParameterSpans parameters = splitParameters(param);
  const int parameterCount = getNumParameters();
  if (dP_dparam.rows() != 9 ||
    dP_dparam.cols() != parameterCount)
    throw std::invalid_argument(
      "ElasticModel3DValanisLandel P Jacobian has the wrong shape");

  const ES::V3d &s = state.stretches;
  validatePositiveStretches(s);
  dP_dparam.setZero();

  auto writePColumn = [&](int column, const ES::V3d &dp) {
    const ES::M3d dP =
      state.U * dp.asDiagonal() * state.V.transpose();
    dP_dparam.col(column) =
      Eigen::Map<const ES::V9d>(dP.data());
  };

  for (int a = 0; a < fParameterCount_; ++a) {
    ES::V3d dp;
    for (int i = 0; i < 3; ++i)
      dp(i) = f_->compute_d2psi_dx_dparam(
        parameters.f, a, s(i));
    writePColumn(a, dp);
  }

  const int gOffset = fParameterCount_;
  for (int a = 0; a < gParameterCount_; ++a) {
    ES::V3d dp = ES::V3d::Zero();
    for (const auto &pair : stretchPairs) {
      const int i = pair[0];
      const int j = pair[1];
      const double mixed =
        g_->compute_d2psi_dx_dparam(
          parameters.g, a, s(i) * s(j));
      dp(i) += mixed * s(j);
      dp(j) += mixed * s(i);
    }
    writePColumn(gOffset + a, dp);
  }

  const int hOffset = gOffset + gParameterCount_;
  const double J = s.prod();
  const ES::V3d dJds(
    s(1) * s(2),
    s(0) * s(2),
    s(0) * s(1));
  for (int a = 0; a < hParameterCount_; ++a) {
    const double mixed =
      h_->compute_d2psi_dx_dparam(parameters.h, a, J);
    writePColumn(hOffset + a, mixed * dJds);
  }
}

double ElasticModel3DValanisLandel::compute_psi_s(
  std::span<const double> param,
  const ES::V3d &s) const
{
  const ParameterSpans parameters = splitParameters(param);
  double energy = h_->compute_psi(parameters.h, s.prod());

  for (int i = 0; i < 3; ++i)
    energy += f_->compute_psi(parameters.f, s(i));

  for (const auto &pair : stretchPairs)
    energy += g_->compute_psi(
      parameters.g, s(pair[0]) * s(pair[1]));

  return energy;
}

ES::V3d ElasticModel3DValanisLandel::compute_dpsi_ds(
  std::span<const double> param,
  const ES::V3d &s) const
{
  const ParameterSpans parameters = splitParameters(param);
  ES::V3d gradient;
  for (int i = 0; i < 3; ++i)
    gradient(i) = f_->compute_dpsi_dx(parameters.f, s(i));

  for (const auto &pair : stretchPairs) {
    const int i = pair[0];
    const int j = pair[1];
    const double gFirst = g_->compute_dpsi_dx(
      parameters.g, s(i) * s(j));
    gradient(i) += gFirst * s(j);
    gradient(j) += gFirst * s(i);
  }

  const double J = s.prod();
  const double hFirst =
    h_->compute_dpsi_dx(parameters.h, J);
  gradient(0) += hFirst * s(1) * s(2);
  gradient(1) += hFirst * s(0) * s(2);
  gradient(2) += hFirst * s(0) * s(1);
  return gradient;
}

ES::M3d ElasticModel3DValanisLandel::compute_d2psi_ds2(
  std::span<const double> param,
  const ES::V3d &s) const
{
  const ParameterSpans parameters = splitParameters(param);
  ES::M3d hessian = ES::M3d::Zero();
  for (int i = 0; i < 3; ++i)
    hessian(i, i) =
      f_->compute_d2psi_dx2(parameters.f, s(i));

  for (const auto &pair : stretchPairs) {
    const int i = pair[0];
    const int j = pair[1];
    const double area = s(i) * s(j);
    const double gFirst =
      g_->compute_dpsi_dx(parameters.g, area);
    const double gSecond =
      g_->compute_d2psi_dx2(parameters.g, area);

    hessian(i, i) += gSecond * s(j) * s(j);
    hessian(j, j) += gSecond * s(i) * s(i);
    const double mixed = gSecond * area + gFirst;
    hessian(i, j) += mixed;
    hessian(j, i) += mixed;
  }

  const double J = s.prod();
  const ES::V3d dJds(
    s(1) * s(2),
    s(0) * s(2),
    s(0) * s(1));
  const double hFirst =
    h_->compute_dpsi_dx(parameters.h, J);
  const double hSecond =
    h_->compute_d2psi_dx2(parameters.h, J);
  hessian.noalias() +=
    hSecond * dJds * dJds.transpose();
  for (const auto &pair : stretchPairs) {
    const int i = pair[0];
    const int j = pair[1];
    const int remaining = 3 - i - j;
    const double mixed = hFirst * s(remaining);
    hessian(i, j) += mixed;
    hessian(j, i) += mixed;
  }
  return hessian;
}

}  // namespace pgo::SolidDeformationModel
