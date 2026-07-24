/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "material/elastic/elasticModelLinearMaterial.h"

#include "EigenSupport.h"

using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

double ElasticModelLinearMaterial::compute_psi(const double *, const double F[9], const double[9], const double[9], const double[3]) const
{
  ES::Mp<const ES::M3d> FMap(F);
  ES::M3d strain = (FMap.transpose() + FMap) * 0.5 - ES::M3d::Identity();

  // E = mu eps : eps + lambda/2 trace^2(eps)
  double t = strain.trace();
  return strain.squaredNorm() * mu + t * t * lambda * 0.5;
}

void ElasticModelLinearMaterial::compute_P(const double *, const double F[9], const double[9], const double[9], const double[3], double P[9]) const
{
  // P = 2 mu eps + lambda trace(eps) I
  ES::Mp<const ES::M3d> FMap(F);
  ES::M3d strain = (FMap.transpose() + FMap) * 0.5 - ES::M3d::Identity();

  (ES::Mp<ES::M3d>(P)) = strain * 2 * mu + ES::M3d::Identity() * lambda * strain.trace();
}

void ElasticModelLinearMaterial::compute_dPdF(const double *, const double F[9], const double[9], const double[9], const double[3], double dPdFOut[81]) const
{
  // P = 2 mu eps + lambda trace(eps) I
  // P = mu (F + F^T) - 2mu I + lambda tr(F - I) I

  ES::Mp<ES::M9d> dPdF(dPdFOut);
  dPdF = ES::M9d::Identity() * mu;

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      dPdF(i + 3 * j, j + 3 * i) += mu;
      dPdF(4 * i, 4 * j) += lambda;
    }
  }
}


#include "simulation/simulationMesh.h"
#include <stdexcept>
#include <string>

namespace pgo::SolidDeformationModel {
namespace {
const SimulationMeshENuMaterial &enuMaterial(const SimulationMesh &mesh, int element) {
  const auto *mat = dynamic_cast<const SimulationMeshENuMaterial *>(mesh.getElementMaterial(element, 0));
  if (!mat) throw std::invalid_argument("elastic config requires SimulationMeshENuMaterial");
  return *mat;
}
void expectSize(std::span<double> output, std::size_t expected) {
  if (output.size() != expected) throw std::invalid_argument("elastic config default parameter buffer has the wrong size");
}
MaterialParameterSpec numberedChannels(int count) {
  MaterialParameterSpec spec;
  for (int i = 0; i < count; ++i) spec.channelNames.push_back("parameter_" + std::to_string(i));
  return spec;
}
}
MaterialParameterSpec LinearElasticConfig::parameterSpec() const { return numberedChannels(0); }
void LinearElasticConfig::initializeDefaultParameters(const SimulationMesh &, int, std::span<double> output) const { expectSize(output, 0); }
std::unique_ptr<ElasticModel> LinearElasticConfig::createModel(const SimulationMesh &mesh, int element, const MaterialFrame &) const
{
  const auto &mat = enuMaterial(mesh, element);
  return std::make_unique<ElasticModelLinearMaterial>(mat.getMuLame(), mat.getLambdaLame());
}
}  // namespace pgo::SolidDeformationModel
