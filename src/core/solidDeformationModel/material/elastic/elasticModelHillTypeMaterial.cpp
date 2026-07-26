/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "material/elastic/elasticModelHillTypeMaterial.h"
#include <array>
#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/elasticModelInvariantBasedMaterial.h"
#include "material/elastic/invariantBasedMaterialStVK.h"
#include "material/elastic/elasticModelVolumeMaterial.h"

#include "EigenSupport.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace ES = pgo::EigenSupport;

using namespace pgo::SolidDeformationModel;

ElasticModelHillTypeMaterial::ElasticModelHillTypeMaterial(double shapeParam, double maximalContractionForce,
  double optimalLengthRatio, const ES::V3d &fd):
  gamma(shapeParam),
  maxf(maximalContractionForce), lo(optimalLengthRatio)
{
  if (!fd.allFinite() || std::abs(fd.norm() - 1.0) > 1e-8)
    throw std::invalid_argument(
      "ElasticModelHillTypeMaterial: fiber direction must be finite and normalized.");
  fiberDirection = fd;

  sqrt_gamma = sqrt(gamma);
  sqrt_pi = sqrt(M_PI);
  erf_sqrt_gamma = erf(-1 / sqrt_gamma);

  ES::M9d dFddT_dFMat;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      ES::M3d dFddT_dFij = ES::M3d::Zero();
      dFddT_dFij(i, 0) = fiberDirection[j] * fiberDirection[0];
      dFddT_dFij(i, 1) = fiberDirection[j] * fiberDirection[1];
      dFddT_dFij(i, 2) = fiberDirection[j] * fiberDirection[2];

      ES::V9d dFddT_dFijVec = Eigen::Map<const ES::V9d>(dFddT_dFij.data());

      dFddT_dFMat.row(j * 3 + i) = dFddT_dFijVec;
    }
  }

  // std::cout << "dFddT_dFMat:\n" << dFddT_dFMat << std::endl;

  dFddT_dF = dFddT_dFMat;
}

double ElasticModelHillTypeMaterial::compute_psi(std::span<const double> param,
  const SpectralState &state) const
{
  // std::cout << "F: ";
  // for (int i = 0; i < 9; i++) {
  //   std::cout << F[i] << ' ';
  // }
  // std::cout << std::endl;

  double l = compute_length(state.F).value;
  // std::cout << "l: " << l << std::endl;

  double psi = 0.5 * param[0] * maxf * sqrt_gamma * sqrt_pi * lo * (erf((l / lo - 1) / sqrt_gamma) - erf_sqrt_gamma);
  // std::cout << l << ',' << param[0] << ',' << maxf * sqrt_gamma * sqrt_pi * lo << ','
  //   << erf((l / lo - 1) / sqrt_gamma) << std::endl;
  // std::cout << "psi: " << psi << std::endl;

  return psi;
}

ES::M3d ElasticModelHillTypeMaterial::compute_P(std::span<const double> param,
  const SpectralState &state) const
{
  const LengthResult length = compute_length(state.F);
  double l = length.value;
  const ES::V3d &Fd = length.deformedFiber;

  const ES::M3d dldF = compute_dldF(Fd);

  double fh = param[0] * maxf * exp(-(l / lo - 1) * (l / lo - 1) / gamma);

  return fh * dldF;
}

ES::M9d ElasticModelHillTypeMaterial::compute_dPdF(std::span<const double> param,
  const SpectralState &state) const
{
  const LengthResult length = compute_length(state.F);
  double l = length.value;
  const ES::V3d &Fd = length.deformedFiber;

  const ES::M3d dldF = compute_dldF(Fd);
  const ES::M9d d2ldF2 = compute_d2ldF2(Fd);

  double coeff1 = -2.0 / (gamma * lo) * param[0] * maxf * exp(-(l / lo - 1) * (l / lo - 1) / gamma) * (l / lo - 1);
  ES::V9d dldFVec = Eigen::Map<const ES::V9d>(dldF.data());

  ES::M9d dPdF;
  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 9; j++) {
      dPdF(i, j) = dldFVec[i] * dldFVec[j] * coeff1;
    }
  }

  double coeff2 = param[0] * maxf * exp(-(l / lo - 1) * (l / lo - 1) / gamma);

  for (int j = 0; j < 9; j++) {
    for (int i = 0; i < 9; i++) {
      dPdF(i, j) += d2ldF2(i, j) * coeff2;
    }
  }

  return dPdF;
}

ElasticModelHillTypeMaterial::LengthResult
ElasticModelHillTypeMaterial::compute_length(const ES::M3d &F) const
{
  LengthResult result;
  result.deformedFiber = F * fiberDirection;
  result.value = result.deformedFiber.norm();
  return result;
}

ES::M3d ElasticModelHillTypeMaterial::compute_dldF(const ES::V3d &Fd) const
{
  const ES::V3d &d = fiberDirection;
  ES::M3d FddT = ES::tensorProduct(Fd, d);

  double coeff = 0.5 / sqrt(Fd.squaredNorm());

  return coeff * 2.0 * FddT;
}

ES::M9d ElasticModelHillTypeMaterial::compute_d2ldF2(const ES::V3d &Fd) const
{
  const ES::V3d &d = fiberDirection;
  ES::M3d FddT = ES::tensorProduct(Fd, d);

  double coeff1 = -0.25 * pow(Fd.squaredNorm(), -1.5);

  ES::V9d FddTVec = Eigen::Map<ES::V9d>(FddT.data());

  ES::M9d d2ldF2;
  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 9; j++) {
      d2ldF2(i, j) = 4.0 * FddTVec[i] * FddTVec[j] * coeff1;
    }
  }

  double coeff2 = 0.5 / sqrt(Fd.squaredNorm());

  for (int j = 0; j < 9; j++) {
    for (int i = 0; i < 9; i++) {
      d2ldF2(i, j) += 2.0 * dFddT_dF(i, j) * coeff2;
    }
  }

  return d2ldF2;
}

double ElasticModelHillTypeMaterial::compute_dpsi_dparam(std::span<const double> /*param*/, int /*i*/,
  const SpectralState &state) const
{
  double l = compute_length(state.F).value;
  return 0.5 * maxf * sqrt_gamma * sqrt_pi * lo * (erf((l / lo - 1) / sqrt_gamma) - erf_sqrt_gamma);
}

double ElasticModelHillTypeMaterial::compute_d2psi_dparam2(std::span<const double> /*param*/, int /*i*/, int /*j*/,
  const SpectralState &) const
{
  return 0;
}

ES::M3d ElasticModelHillTypeMaterial::compute_dP_dparam(std::span<const double> /*param*/, int /*i*/,
  const SpectralState &state) const
{
  const LengthResult length = compute_length(state.F);
  double l = length.value;
  const ES::V3d &Fd = length.deformedFiber;

  const ES::M3d dldF = compute_dldF(Fd);

  double fh = maxf * exp(-(l / lo - 1) * (l / lo - 1) / gamma);

  return fh * dldF;
}


#include "simulation/simulationMesh.h"
#include <initializer_list>
#include <stdexcept>

namespace pgo::SolidDeformationModel {
namespace {
const SimulationMeshENuMaterial &enuMaterial(const SimulationMesh &mesh, int element) {
  return mesh.requireElementField<SimulationMeshENuMaterial>().at(element);
}
const SimulationMeshHillMaterial &hillMaterial(const SimulationMesh &mesh, int element) {
  return mesh.requireElementField<SimulationMeshHillMaterial>().at(element);
}
void expectSize(std::span<double> output, std::size_t expected) {
  if (output.size() != expected) throw std::invalid_argument("elastic config default parameter buffer has the wrong size");
}
}
std::span<const std::string_view> HillStableNeoConfig::parameterChannelNames() const { static constexpr std::array<std::string_view, 1> names{"activation"}; return names; }
void HillStableNeoConfig::initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double> output) const { expectSize(output, 1); output[0] = 1.0; }
std::unique_ptr<ElasticModel> HillStableNeoConfig::createModel(const SimulationMesh &mesh, int element, const MaterialFrame &frame) const
{
  const auto &mat = enuMaterial(mesh, element); const auto &hill = hillMaterial(mesh, element);
  return std::make_unique<ElasticModelCombinedMaterial<2>>(
    std::make_unique<ElasticModelStableNeoHookeanMaterial>(mat.getMuLame(), mat.getLambdaLame()),
    std::make_unique<ElasticModelHillTypeMaterial>(hill.getGamma(), hill.getEact(), hill.getLo(), frame.col(0)));
}

std::span<const std::string_view> HillStVKConfig::parameterChannelNames() const { static constexpr std::array<std::string_view, 1> names{"activation"}; return names; }
void HillStVKConfig::initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double> output) const { expectSize(output, 1); output[0] = 1.0; }
std::unique_ptr<ElasticModel> HillStVKConfig::createModel(const SimulationMesh &mesh, int element, const MaterialFrame &frame) const
{
  const auto &mat = enuMaterial(mesh, element); const auto &hill = hillMaterial(mesh, element);
  return std::make_unique<ElasticModelCombinedMaterial<2>>(
    std::make_unique<ElasticModelInvariantBasedMaterial>(
      std::make_unique<InvariantBasedMaterialStVK>(mat.getE(), mat.getNu(), mat.getCompressionRatio())),
    std::make_unique<ElasticModelHillTypeMaterial>(hill.getGamma(), hill.getEact(), hill.getLo(), frame.col(0)));
}

std::span<const std::string_view> HillStVKVolumeConfig::parameterChannelNames() const { static constexpr std::array<std::string_view, 1> names{"activation"}; return names; }
void HillStVKVolumeConfig::initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double> output) const { expectSize(output, 1); output[0] = 1.0; }
std::unique_ptr<ElasticModel> HillStVKVolumeConfig::createModel(const SimulationMesh &mesh, int element, const MaterialFrame &frame) const
{
  const auto &mat = enuMaterial(mesh, element); const auto &hill = hillMaterial(mesh, element);
  return std::make_unique<ElasticModelCombinedMaterial<3>>(
    std::make_unique<ElasticModelInvariantBasedMaterial>(
      std::make_unique<InvariantBasedMaterialStVK>(mat.getE(), mat.getNu(), mat.getCompressionRatio())),
    std::make_unique<ElasticModelHillTypeMaterial>(hill.getGamma(), hill.getEact(), hill.getLo(), frame.col(0)),
    std::make_unique<ElasticModelVolumeMaterial>(mat.getCompressionRatio()));
}
}  // namespace pgo::SolidDeformationModel
