/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "material/plastic/plasticModel3DConstant.h"

#include "EigenSupport.h"

#include <cstring>

namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

PlasticModel3DConstant::PlasticModel3DConstant(const double Fp_[9]):
  PlasticModel3DDeformationGradient()
{
  std::memcpy(Fp, Fp_, sizeof(double) * 9);
  (Eigen::Map<ES::M3d>(FpInv)) = (Eigen::Map<ES::M3d>(Fp)).fullPivLu().inverse();
  detFp = (Eigen::Map<ES::M3d>(Fp)).determinant();
}

void PlasticModel3DConstant::computeA(const double *, double A[9]) const
{
  std::memcpy(A, Fp, sizeof(double) * 9);
}

void PlasticModel3DConstant::computeAInv(const double *, double AInv[9]) const
{
  std::memcpy(AInv, FpInv, sizeof(double) * 9);
}

void PlasticModel3DConstant::defaultFp(double FpOut[9]) const
{
  std::memcpy(FpOut, Fp, sizeof(double) * 9);
}

void PlasticModel3DConstant::computeR(const double *, double R[9]) const
{
  static constexpr double kIdentity[9] = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
  };
  std::memcpy(R, kIdentity, sizeof(kIdentity));
}


#include "simulation/simulationMesh.h"
#include <algorithm>
#include <initializer_list>
#include <stdexcept>

namespace pgo::SolidDeformationModel {
namespace {
void expectSize(std::span<double> output, std::size_t expected) {
  if (output.size() != expected) throw std::invalid_argument("plastic config default parameter buffer has the wrong size");
}
MaterialParameterSpec channels(std::initializer_list<const char *> names) {
  MaterialParameterSpec spec;
  for (const char *name : names) spec.channelNames.emplace_back(name);
  return spec;
}
}
MaterialParameterSpec VolumetricPlasticity0Config::parameterSpec() const { return {}; }
void VolumetricPlasticity0Config::initializeDefaultParameters(const SimulationMesh &, int, std::span<double> output) const { expectSize(output, 0); }
std::unique_ptr<PlasticModel> VolumetricPlasticity0Config::createModel(const SimulationMesh &, int, const MaterialFrame &) const
{
  static constexpr double identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  return std::make_unique<PlasticModel3DConstant>(identity);
}
}  // namespace pgo::SolidDeformationModel
