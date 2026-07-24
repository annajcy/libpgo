/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "material/elastic/elasticModelVolumeMaterial.h"

#include "determinantDerivatives.h"

using namespace pgo::SolidDeformationModel;
using namespace pgo::NonlinearOptimization;

double ElasticModelVolumeMaterial::compute_psi(const double *, const double F[9], const double[9], const double[9], const double[3]) const
{
  // energy = 0.5 * ( detF - 1)^2
  double detF = Determinant::Dim3::det(F);
  return (detF - 1) * (detF - 1) * 0.5 * scale;
}

void ElasticModelVolumeMaterial::compute_P(const double *, const double F[9], const double[9], const double[9], const double[3], double P[9]) const
{
  // d psi / d F =  (detF - 1) d detF / dF
  double detF = Determinant::Dim3::det(F);
  Determinant::Dim3::ddetA_dA(F, P);

  for (int i = 0; i < 9; i++) {
    P[i] *= (detF - 1) * scale;
  }
}

void ElasticModelVolumeMaterial::compute_dPdF(const double *, const double F[9], const double[9], const double[9], const double[3], double dPdFOut[81]) const
{
  // d^2 psi / dF dF
  // = d ((detF - 1) d detF / dF) /dF
  // = d detF / * dF d detF / dF + (detF - 1) * dP/dF

  double detF = Determinant::Dim3::det(F);
  double P[9];
  Determinant::Dim3::ddetA_dA(F, P);

  Determinant::Dim3::d2detA_dA2(F, dPdFOut);
  for (int i = 0; i < 81; i++)
    dPdFOut[i] *= (detF - 1);

  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 9; j++) {
      dPdFOut[i * 9 + j] += P[i] * P[j];
    }
  }

  for (int i = 0; i < 81; i++)
    dPdFOut[i] *= scale;
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
MaterialParameterSpec numberedChannels(int count) {
  MaterialParameterSpec spec;
  for (int i = 0; i < count; ++i) spec.channelNames.push_back("parameter_" + std::to_string(i));
  return spec;
}
}
MaterialParameterSpec VolumePenaltyConfig::parameterSpec() const { return numberedChannels(0); }
void VolumePenaltyConfig::initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double> output) const { expectSize(output, 0); }
std::unique_ptr<ElasticModel> VolumePenaltyConfig::createModel(const SimulationMesh &mesh, int element, const MaterialFrame &) const
{
  return std::make_unique<ElasticModelVolumeMaterial>(enuMaterial(mesh, element).getCompressionRatio());
}
}  // namespace pgo::SolidDeformationModel
