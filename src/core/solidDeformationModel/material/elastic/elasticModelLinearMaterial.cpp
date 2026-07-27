/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "material/elastic/elasticModelLinearMaterial.h"

#include "EigenSupport.h"

using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

double ElasticModelLinearMaterial::compute_psi(std::span<const double>, const SpectralState &state) const
{
  const ES::M3d &F = state.F;
  ES::M3d strain = (F.transpose() + F) * 0.5 - ES::M3d::Identity();

  // E = mu eps : eps + lambda/2 trace^2(eps)
  double t = strain.trace();
  return strain.squaredNorm() * mu + t * t * lambda * 0.5;
}

ES::M3d ElasticModelLinearMaterial::compute_P(std::span<const double>, const SpectralState &state) const
{
  // P = 2 mu eps + lambda trace(eps) I
  const ES::M3d &F = state.F;
  ES::M3d strain = (F.transpose() + F) * 0.5 - ES::M3d::Identity();

  return strain * 2 * mu + ES::M3d::Identity() * lambda * strain.trace();
}

ES::M9d ElasticModelLinearMaterial::compute_dPdF(std::span<const double>, const SpectralState &) const
{
  // P = 2 mu eps + lambda trace(eps) I
  // P = mu (F + F^T) - 2mu I + lambda tr(F - I) I

  ES::M9d dPdF;
  dPdF = ES::M9d::Identity() * mu;

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      dPdF(i + 3 * j, j + 3 * i) += mu;
      dPdF(4 * i, 4 * j) += lambda;
    }
  }
  return dPdF;
}


#include <stdexcept>
#include <string>

namespace pgo::SolidDeformationModel {
namespace {
}
MaterialChannelSchema LinearElasticDefinition::optimizableChannelSchema() const { return {}; }
MaterialChannelSchema LinearElasticDefinition::fixedChannelSchema() const { static constexpr std::array<std::string_view, 2> names{"E", "nu"}; return MaterialChannelSchema(names); }
std::unique_ptr<ElasticModel> LinearElasticDefinition::createModel(std::span<const double> values, const MaterialFrame &) const
{
  if (values.size() != 2) throw std::invalid_argument("linear requires fixed channels E, nu");
  const double E = values[0], nu = values[1];
  return std::make_unique<ElasticModelLinearMaterial>(E / (2 * (1 + nu)), (nu * E) / ((1 + nu) * (1 - 2 * nu)));
}
}  // namespace pgo::SolidDeformationModel
