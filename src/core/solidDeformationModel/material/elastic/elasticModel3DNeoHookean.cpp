#include "material/elastic/elasticModel3DNeoHookean.h"

#include <array>
#include <cmath>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace
{
namespace ES = EigenSupport;

double innerProduct(
  const ES::M3d &a,
  const ES::M3d &b)
{
  return a.cwiseProduct(b).sum();
}

}  // namespace

ElasticModel3DNeoHookean::ElasticModel3DNeoHookean(
  double mu,
  double lambda):
  mu_(mu),
  lambda_(lambda)
{
  if (!std::isfinite(mu_) || !std::isfinite(lambda_))
    throw std::invalid_argument(
      "Neo-Hookean Lame parameters must be finite");
  if (!(mu_ > 0.0))
    throw std::invalid_argument(
      "Neo-Hookean mu must be positive");
  if (!(lambda_ + 2.0 * mu_ / 3.0 > 0.0))
    throw std::invalid_argument(
      "Neo-Hookean bulk modulus must be positive");
}

double ElasticModel3DNeoHookean::validateAndComputeLogJ(
  std::span<const double> param,
  const SpectralState &state) const
{
  if (!param.empty())
    throw std::invalid_argument(
      "Neo-Hookean material has no optimizable parameters");
  if (!state.F.allFinite())
    throw std::invalid_argument(
      "Neo-Hookean deformation gradient must be finite");
  const double J = state.F.determinant();
  if (!std::isfinite(J) || !(J > 0.0))
    throw std::domain_error(
      "Neo-Hookean requires det(F) > 0");
  return std::log(J);
}

double ElasticModel3DNeoHookean::compute_psi(
  std::span<const double> param,
  const SpectralState &state) const
{
  const double logJ =
    validateAndComputeLogJ(param, state);
  return 0.5 * mu_ *
      (state.F.squaredNorm() - 3.0) -
    mu_ * logJ +
    0.5 * lambda_ * logJ * logJ;
}

ES::M3d ElasticModel3DNeoHookean::compute_P(
  std::span<const double> param,
  const SpectralState &state) const
{
  const double logJ =
    validateAndComputeLogJ(param, state);
  const ES::M3d inverseTranspose =
    state.F.inverse().transpose();
  return mu_ * state.F +
    (lambda_ * logJ - mu_) * inverseTranspose;
}

ES::M9d ElasticModel3DNeoHookean::compute_dPdF(
  std::span<const double> param,
  const SpectralState &state) const
{
  const double logJ =
    validateAndComputeLogJ(param, state);
  const ES::M3d inverseTranspose =
    state.F.inverse().transpose();
  const double pressure = lambda_ * logJ - mu_;

  ES::M9d tangent;
  for (int column = 0; column < 9; ++column) {
    ES::M3d dF = ES::M3d::Zero();
    dF.data()[column] = 1.0;
    const double dLogJ =
      innerProduct(inverseTranspose, dF);
    const ES::M3d dInverseTranspose =
      -inverseTranspose * dF.transpose() *
      inverseTranspose;
    const ES::M3d dP =
      mu_ * dF +
      lambda_ * dLogJ * inverseTranspose +
      pressure * dInverseTranspose;
    tangent.col(column) =
      Eigen::Map<const ES::V9d>(dP.data());
  }
  return tangent;
}

MaterialChannelSchema
NeoHookeanDefinition::fixedChannelSchema() const
{
  static constexpr std::array<std::string_view, 2>
    names{ "E", "nu" };
  return MaterialChannelSchema(names);
}

MaterialChannelSchema
NeoHookeanDefinition::optimizableChannelSchema() const
{
  return {};
}

std::unique_ptr<ElasticModel>
NeoHookeanDefinition::createModel(
  std::span<const double> fixedChannels,
  const MaterialFrame &) const
{
  if (fixedChannels.size() != 2)
    throw std::invalid_argument(
      "neo_hookean requires fixed channels E, nu");
  const double youngsModulus = fixedChannels[0];
  const double poissonRatio = fixedChannels[1];
  if (!std::isfinite(youngsModulus) ||
    !(youngsModulus > 0.0))
    throw std::invalid_argument(
      "neo_hookean E must be finite and positive");
  if (!std::isfinite(poissonRatio) ||
    !(poissonRatio > -1.0 && poissonRatio < 0.5))
    throw std::invalid_argument(
      "neo_hookean nu must lie in (-1, 0.5)");

  const double mu =
    youngsModulus / (2.0 * (1.0 + poissonRatio));
  const double lambda =
    youngsModulus * poissonRatio /
    ((1.0 + poissonRatio) *
      (1.0 - 2.0 * poissonRatio));
  return std::make_unique<ElasticModel3DNeoHookean>(
    mu, lambda);
}

}  // namespace pgo::SolidDeformationModel
