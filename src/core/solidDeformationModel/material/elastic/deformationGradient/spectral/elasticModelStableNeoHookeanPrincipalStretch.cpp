#include "material/elastic/deformationGradient/spectral/elasticModelStableNeoHookeanPrincipalStretch.h"

#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace
{
namespace ES = EigenSupport;
}

ElasticModelStableNeoHookeanPrincipalStretch::
  ElasticModelStableNeoHookeanPrincipalStretch(double mu, double lambda)
{
  setMaterial(mu, lambda);
}

void ElasticModelStableNeoHookeanPrincipalStretch::setMaterial(
  double mu,
  double lambda)
{
  mu_ = mu;
  lambda_ = lambda + mu;
  ratio_ = mu_ / lambda_;
}

double ElasticModelStableNeoHookeanPrincipalStretch::compute_psi_s(
  std::span<const double>,
  const ES::V3d &s) const
{
  const double Jminus1 = s.prod() - 1.0 - ratio_;
  return 0.5 * (mu_ * (s.squaredNorm() - 3.0) +
    lambda_ * Jminus1 * Jminus1) - 0.5 * lambda_ * ratio_ * ratio_;
}

ES::V3d ElasticModelStableNeoHookeanPrincipalStretch::compute_dpsi_ds(
  std::span<const double>,
  const ES::V3d &s) const
{
  const double Jminus1 = s.prod() - 1.0 - ratio_;
  const ES::V3d dJds(
    s(1) * s(2),
    s(0) * s(2),
    s(0) * s(1));
  return mu_ * s + lambda_ * Jminus1 * dJds;
}

ES::M3d ElasticModelStableNeoHookeanPrincipalStretch::compute_d2psi_ds2(
  std::span<const double>,
  const ES::V3d &s) const
{
  const double Jminus1 = s.prod() - 1.0 - ratio_;
  const ES::V3d dJds(
    s(1) * s(2),
    s(0) * s(2),
    s(0) * s(1));

  ES::M3d hessian = mu_ * ES::M3d::Identity();
  hessian.noalias() += lambda_ * dJds * dJds.transpose();
  hessian(0, 1) += lambda_ * Jminus1 * s(2);
  hessian(1, 0) = hessian(0, 1);
  hessian(0, 2) += lambda_ * Jminus1 * s(1);
  hessian(2, 0) = hessian(0, 2);
  hessian(1, 2) += lambda_ * Jminus1 * s(0);
  hessian(2, 1) = hessian(1, 2);
  return hessian;
}

MaterialChannelSchema
StableNeoPrincipalStretchDefinition::optimizableChannelSchema() const
{
  return {};
}

MaterialChannelSchema
StableNeoPrincipalStretchDefinition::fixedChannelSchema() const
{
  static constexpr std::array<std::string_view, 2> names{"E", "nu"};
  return MaterialChannelSchema(names);
}

std::unique_ptr<ElasticModel>
StableNeoPrincipalStretchDefinition::createModel(
  std::span<const double> values, const MaterialFrame &) const
{
  if (values.size() != 2) throw std::invalid_argument("stable_neo_principal_stretch requires fixed channels E, nu");
  const double E = values[0], nu = values[1];
  return std::make_unique<ElasticModelStableNeoHookeanPrincipalStretch>(
    E / (2 * (1 + nu)), (nu * E) / ((1 + nu) * (1 - 2 * nu)));
}


}  // namespace pgo::SolidDeformationModel
