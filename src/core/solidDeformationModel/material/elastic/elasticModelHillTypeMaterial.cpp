/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "material/elastic/elasticModelHillTypeMaterial.h"
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
  (Eigen::Map<ES::V3d>(fiberDirection)) = fd;

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

  (Eigen::Map<ES::M9d>(dFddT_dF)) = dFddT_dFMat;
}

double ElasticModelHillTypeMaterial::compute_psi(const double *param,
  const double F[9], const double * /*U[9]*/, const double * /*V[9]*/, const double * /*S[3]*/) const
{
  // std::cout << "F: ";
  // for (int i = 0; i < 9; i++) {
  //   std::cout << F[i] << ' ';
  // }
  // std::cout << std::endl;

  double l = compute_length(F);
  // std::cout << "l: " << l << std::endl;

  double psi = 0.5 * param[0] * maxf * sqrt_gamma * sqrt_pi * lo * (erf((l / lo - 1) / sqrt_gamma) - erf_sqrt_gamma);
  // std::cout << l << ',' << param[0] << ',' << maxf * sqrt_gamma * sqrt_pi * lo << ','
  //   << erf((l / lo - 1) / sqrt_gamma) << std::endl;
  // std::cout << "psi: " << psi << std::endl;

  return psi;
}

void ElasticModelHillTypeMaterial::compute_P(const double *param,
  const double F[9], const double * /*U[9]*/, const double * /*V[9]*/, const double * /*S[3]*/, double P[9]) const
{
  ES::V3d Fd;
  double l = compute_length(F, Fd.data());

  ES::M3d dldF;
  compute_dldF(F, Fd.data(), dldF.data());

  double fh = param[0] * maxf * exp(-(l / lo - 1) * (l / lo - 1) / gamma);

  (Eigen::Map<ES::M3d>(P)) = fh * dldF;
}

void ElasticModelHillTypeMaterial::compute_dPdF(const double *param,
  const double F[9], const double * /*U[9]*/, const double * /*V[9]*/, const double * /*S[3]*/, double dPdFOut[81]) const
{
  ES::V3d Fd;
  double l = compute_length(F, Fd.data());

  ES::M3d dldF;
  compute_dldF(F, Fd.data(), dldF.data());

  ES::M9d d2ldF2;
  compute_d2ldF2(F, Fd.data(), d2ldF2.data());

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

  (Eigen::Map<ES::M9d>(dPdFOut)) = dPdF;
}

double ElasticModelHillTypeMaterial::compute_length(const double F[9], double FdOut[]) const
{
  ES::V3d Fd = Eigen::Map<const ES::M3d>(F) * Eigen::Map<const ES::V3d>(fiberDirection);
  if (FdOut)
    (Eigen::Map<ES::V3d>(FdOut)) = Fd;

  return Fd.norm();
}

void ElasticModelHillTypeMaterial::compute_dldF(const double * /*F[9]*/, const double FdIn[], double dldF[9]) const
{
  ES::V3d d = Eigen::Map<const ES::V3d>(fiberDirection);
  ES::V3d Fd = Eigen::Map<const ES::V3d>(FdIn);
  ES::M3d FddT = ES::tensorProduct(Fd, d);

  double coeff = 0.5 / sqrt(Fd.squaredNorm());

  (Eigen::Map<ES::M3d>(dldF)) = coeff * 2.0 * FddT;
}

void ElasticModelHillTypeMaterial::compute_d2ldF2(const double * /*F[9]*/, const double FdIn[], double d2ldF2Out[81]) const
{
  ES::V3d d = Eigen::Map<const ES::V3d>(fiberDirection);
  ES::V3d Fd = Eigen::Map<const ES::V3d>(FdIn);
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
      d2ldF2(i, j) += 2.0 * dFddT_dF[j * 9 + i] * coeff2;
    }
  }

  (Eigen::Map<ES::M9d>(d2ldF2Out)) = d2ldF2;
}

double ElasticModelHillTypeMaterial::compute_dpsi_dparam(const double * /*param*/, int /*i*/,
  const double F[9], const double * /*U[9]*/, const double * /*V[9]*/, const double * /*S[3]*/) const
{
  double l = compute_length(F);
  return 0.5 * maxf * sqrt_gamma * sqrt_pi * lo * (erf((l / lo - 1) / sqrt_gamma) - erf_sqrt_gamma);
}

double ElasticModelHillTypeMaterial::compute_d2psi_dparam2(const double * /*param*/, int /*i*/, int /*j*/,
  const double * /*F[9]*/, const double * /*U[9]*/, const double * /*V[9]*/, const double * /*S[3]*/) const
{
  return 0;
}

void ElasticModelHillTypeMaterial::compute_dP_dparam(const double * /*param*/, int /*i*/,
  const double F[9], const double * /*U[9]*/, const double * /*V[9]*/, const double * /*S[3]*/, double *ret) const
{
  ES::V3d Fd;
  double l = compute_length(F, Fd.data());

  ES::M3d dldF;
  compute_dldF(F, Fd.data(), dldF.data());

  double fh = maxf * exp(-(l / lo - 1) * (l / lo - 1) / gamma);

  (Eigen::Map<ES::M3d>(ret)) = fh * dldF;
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
MaterialParameterSpec channels(std::initializer_list<const char *> names) {
  MaterialParameterSpec spec;
  for (const char *name : names) spec.channelNames.emplace_back(name);
  return spec;
}
}
MaterialParameterSpec HillStableNeoConfig::parameterSpec() const { return channels({"activation"}); }
void HillStableNeoConfig::initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double> output) const { expectSize(output, 1); output[0] = 1.0; }
std::unique_ptr<ElasticModel> HillStableNeoConfig::createModel(const SimulationMesh &mesh, int element, const MaterialFrame &frame) const
{
  const auto &mat = enuMaterial(mesh, element); const auto &hill = hillMaterial(mesh, element);
  return std::make_unique<ElasticModelCombinedMaterial<2>>(
    std::make_unique<ElasticModelStableNeoHookeanMaterial>(mat.getMuLame(), mat.getLambdaLame()),
    std::make_unique<ElasticModelHillTypeMaterial>(hill.getGamma(), hill.getEact(), hill.getLo(), frame.col(0)));
}

MaterialParameterSpec HillStVKConfig::parameterSpec() const { return channels({"activation"}); }
void HillStVKConfig::initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double> output) const { expectSize(output, 1); output[0] = 1.0; }
std::unique_ptr<ElasticModel> HillStVKConfig::createModel(const SimulationMesh &mesh, int element, const MaterialFrame &frame) const
{
  const auto &mat = enuMaterial(mesh, element); const auto &hill = hillMaterial(mesh, element);
  return std::make_unique<ElasticModelCombinedMaterial<2>>(
    std::make_unique<ElasticModelInvariantBasedMaterial>(
      std::make_unique<InvariantBasedMaterialStVK>(mat.getE(), mat.getNu(), mat.getCompressionRatio())),
    std::make_unique<ElasticModelHillTypeMaterial>(hill.getGamma(), hill.getEact(), hill.getLo(), frame.col(0)));
}

MaterialParameterSpec HillStVKVolumeConfig::parameterSpec() const { return channels({"activation"}); }
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
