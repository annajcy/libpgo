/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "material/elastic/elasticModelVolumeMaterial.h"

#include "determinantDerivatives.h"

using namespace pgo::SolidDeformationModel;
using namespace pgo::NonlinearOptimization;
namespace ES = pgo::EigenSupport;

double ElasticModelVolumeMaterial::compute_psi(std::span<const double>, const SpectralState &state) const
{
  // energy = 0.5 * ( detF - 1)^2
  double detF = Determinant::Dim3::det(state.F.data());
  return (detF - 1) * (detF - 1) * 0.5 * scale;
}

ES::M3d ElasticModelVolumeMaterial::compute_P(std::span<const double>, const SpectralState &state) const
{
  // d psi / d F =  (detF - 1) d detF / dF
  ES::V9d P;
  double detF = Determinant::Dim3::det(state.F.data());
  Determinant::Dim3::ddetA_dA(state.F.data(), P.data());

  for (int i = 0; i < 9; i++) {
    P[i] *= (detF - 1) * scale;
  }
  return Eigen::Map<const ES::M3d>(P.data());
}

ES::M9d ElasticModelVolumeMaterial::compute_dPdF(std::span<const double>, const SpectralState &state) const
{
  // d^2 psi / dF dF
  // = d ((detF - 1) d detF / dF) /dF
  // = d detF / * dF d detF / dF + (detF - 1) * dP/dF

  double detF = Determinant::Dim3::det(state.F.data());
  ES::V9d P;
  Determinant::Dim3::ddetA_dA(state.F.data(), P.data());

  ES::M9d dPdFOut;
  Determinant::Dim3::d2detA_dA2(state.F.data(), dPdFOut.data());
  for (int i = 0; i < 81; i++)
    dPdFOut.data()[i] *= (detF - 1);

  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 9; j++) {
      dPdFOut.data()[i * 9 + j] += P[i] * P[j];
    }
  }

  for (int i = 0; i < 81; i++)
    dPdFOut.data()[i] *= scale;
  return dPdFOut;
}


#include <stdexcept>
#include <string>

namespace pgo::SolidDeformationModel {
namespace {
}
std::unique_ptr<ElasticModel> VolumePenaltyDefinition::createModel(std::span<const double> values, const MaterialFrame &) const
{
  requireFixedChannels(values, 1, id(), "J");
  return std::make_unique<ElasticModelVolumeMaterial>(values[0]);
}
}  // namespace pgo::SolidDeformationModel
