/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "material/plastic/plasticModelDefinition.h"

#include "material/plastic/plasticModel3DDeformationGradient.h"
#include "EigenSupport.h"


namespace pgo
{
namespace SolidDeformationModel
{
class PlasticModel3D3DOF : public PlasticModel3DDeformationGradient
{
public:
  explicit PlasticModel3D3DOF(const EigenSupport::M3d &referenceToMaterial);
  ~PlasticModel3D3DOF() {}

  virtual int getNumParameters() const override { return 3; }
  virtual EigenSupport::M3d computeA(std::span<const double> param) const override;
  virtual EigenSupport::M3d computeAInv(std::span<const double> param) const override;
  virtual double compute_detA(std::span<const double> param) const override;

  void compute_ddetA_da(
    std::span<const double> param, EigenSupport::RefVecXd ddetA_da) const override;
  virtual EigenSupport::M3d compute_dAInv_da(std::span<const double> param, int pi) const override;

  virtual EigenSupport::M3d defaultFp() const override
  {
    return EigenSupport::M3d::Identity();
  }
  void defaultParams(std::span<double> param) const override
  {
    param[0] = param[1] = param[2] = 1.0;
  }
  virtual void projectParam(std::span<double> param, double zeroThreshold) const override;
  virtual void toParam(const EigenSupport::M3d &Fp, std::span<double> param) const override;

  virtual EigenSupport::M3d computeR(std::span<const double> param) const override;

protected:
  EigenSupport::M3d R, RT;
  const double zeroThreshold = 1e-4;
};

inline void PlasticModel3D3DOF::toParam(const EigenSupport::M3d &Fp, std::span<double> param) const
{
  const EigenSupport::M3d local = R * Fp * RT;
  param[0] = local(0, 0);
  param[1] = local(1, 1);
  param[2] = local(2, 2);
}

inline EigenSupport::M3d PlasticModel3D3DOF::computeR(std::span<const double>) const
{
  return RT;
}

class VolumetricPlasticity3Definition final : public PlasticModelDefinition
{
public:
  std::string_view id() const override { return "volumetric_dof3"; }
  int numFixedChannels() const override { return 0; }
  int numOptimizableChannels() const override;
  std::unique_ptr<PlasticModel> createModel(std::span<const double>, const MaterialFrame &) const override;
private:
};
}  // namespace SolidDeformationModel
}  // namespace pgo
