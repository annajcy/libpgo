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


#include "simulation/simulationMesh.h"
#include <stdexcept>
#include <string>

namespace pgo::SolidDeformationModel {
namespace {
const SimulationMeshENuMaterial &enuMaterial(const SimulationMesh &mesh, int element) {
  return mesh.requireElementField<SimulationMeshENuMaterial>().at(element);
}
void expectSize(std::span<double> output, std::size_t expected) {
  if (output.size() != expected) throw std::invalid_argument("elastic config default parameter buffer has the wrong size");
}
}
std::span<const std::string_view> LinearElasticConfig::parameterChannelNames() const { return {}; }
void LinearElasticConfig::initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double> output) const { expectSize(output, 0); }
std::unique_ptr<ElasticModel> LinearElasticConfig::createModel(const SimulationMesh &mesh, int element, const MaterialFrame &) const
{
  const auto &mat = enuMaterial(mesh, element);
  return std::make_unique<ElasticModelLinearMaterial>(mat.getMuLame(), mat.getLambdaLame());
}
}  // namespace pgo::SolidDeformationModel
