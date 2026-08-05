/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "material/plastic/plasticModel3D3DOF.h"
#include <array>

#include "EigenSupport.h"

namespace ES = pgo::EigenSupport; 

using namespace pgo::SolidDeformationModel;

PlasticModel3D3DOF::PlasticModel3D3DOF(
  const ES::M3d &referenceToMaterial):
  PlasticModel3DDeformationGradient()
{
  R = referenceToMaterial;
  RT = R.transpose();
}

ES::M3d PlasticModel3D3DOF::computeA(std::span<const double> param) const
{
  ES::M3d S = ES::V3d(param[0], param[1], param[2]).asDiagonal();
  return RT * S * R;
}

ES::M3d PlasticModel3D3DOF::computeAInv(std::span<const double> param) const
{
  ES::M3d SInv = ES::M3d::Identity();
  for (int i = 0; i < 3; i++) {
    if (param[i] < zeroThreshold)
      SInv(i, i) = 1.0 / zeroThreshold;
    else {
      SInv(i, i) = 1.0 / param[i];
    }
  }

  return RT * SInv * R;
}

double PlasticModel3D3DOF::compute_detA(std::span<const double> param) const
{
  return param[0] * param[1] * param[2];
}

void PlasticModel3D3DOF::compute_ddetA_da(
  std::span<const double> param, ES::RefVecXd ddetA_da) const
{
  ddetA_da[0] = param[1] * param[2];
  ddetA_da[1] = param[0] * param[2];
  ddetA_da[2] = param[0] * param[1];
}

 ES::M3d PlasticModel3D3DOF::compute_dAInv_da(std::span<const double> param, int pi) const
{
  ES::M3d dSInv_dai = ES::M3d::Zero();

  double val = param[pi];
  if (val < zeroThreshold)
    val = zeroThreshold;

  dSInv_dai(pi, pi) = -1.0 / (val * val);
  return RT * dSInv_dai * R;
}

 void PlasticModel3D3DOF::projectParam(std::span<double> param, double zeroThreshold) const
{
  if (param[0] < zeroThreshold)
    param[0] = zeroThreshold;

  if (param[1] < zeroThreshold)
    param[1] = zeroThreshold;

  if (param[2] < zeroThreshold)
    param[2] = zeroThreshold;
}


#include <stdexcept>

namespace pgo::SolidDeformationModel {
int VolumetricPlasticity3Definition::numOptimizableChannels() const { return 3; }
std::unique_ptr<PlasticModel> VolumetricPlasticity3Definition::createModel(std::span<const double> values, const MaterialFrame &frame) const
{
  if (!values.empty()) throw std::invalid_argument("volumetric_dof3 has no fixed channels");
  return std::make_unique<PlasticModel3D3DOF>(frame.transpose());
}
}  // namespace pgo::SolidDeformationModel
