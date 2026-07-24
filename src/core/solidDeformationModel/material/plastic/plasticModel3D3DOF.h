/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

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
  virtual void computeA(const double *param, double A[9]) const override;
  virtual void computeAInv(const double *param, double AInv[9]) const override;
  virtual double compute_detA(const double *param) const override;

  virtual void compute_ddetA_da(const double *param, double *ddetaA_da, int leadingDim) const override;
  virtual void compute_d2detA_da2(const double *param, double *d2detA_da2, int leadingDim) const override;
  virtual void compute_dAInv_da(const double *param, int pi, double ret[9]) const override;
  virtual void compute_d2AInv_da2(const double *param, int pi, int pj, double ret[9]) const override;

  virtual void defaultFp(double *Fp) const override { Fp[0] = Fp[1] = Fp[2] = 1.0; }
  void defaultParams(double *param) const override
  {
    param[0] = param[1] = param[2] = 1.0;
  }
  virtual void projectParam(double *param, double zeroThreshold) const override;
  virtual void toParam(const double *Fp, double *param) const override;

  virtual void computeR(const double *param, double R[9]) const override;
  virtual void compute_dparamfull_dparamsub(const double *param, const double *basis, int numHandles, double *dpf_dps) const override;
  virtual void compute_d2paramfull_dparamsub2(const double *param, const double *basis, int numHandles, int pi, double *d2a_dz2) const override;

protected:
  double R[9], RT[9];
  const double zeroThreshold = 1e-4;
};

inline void PlasticModel3D3DOF::toParam(const double *Fp, double *param) const
{
  const EigenSupport::M3d local =
    Eigen::Map<const EigenSupport::M3d>(R) *
    Eigen::Map<const EigenSupport::M3d>(Fp) *
    Eigen::Map<const EigenSupport::M3d>(RT);
  param[0] = local(0, 0);
  param[1] = local(1, 1);
  param[2] = local(2, 2);
}

inline void PlasticModel3D3DOF::computeR(const double *param, double R[9]) const
{
  for (int i = 0; i < 9; i++)
    R[i] = this->RT[i];
}

class VolumetricPlasticity3Config final : public PlasticModelConfig
{
public:
  std::string_view id() const override { return "volumetric_dof3"; }
  MaterialParameterSpec parameterSpec() const override;
  MaterialFrameRequirement frameRequirement() const override { return MaterialFrameRequirement::FullFrame; }
  void initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double>) const override;
private:
  std::unique_ptr<PlasticModel> createModel(const SimulationMesh &, int, const MaterialFrame &) const override;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
