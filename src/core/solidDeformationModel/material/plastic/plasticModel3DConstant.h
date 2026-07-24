/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "material/plastic/plasticModel3DDeformationGradient.h"

namespace pgo
{
namespace SolidDeformationModel
{
class PlasticModel3DConstant : public PlasticModel3DDeformationGradient
{
public:
  PlasticModel3DConstant(const double Fp_[9]);
  ~PlasticModel3DConstant() {}

  virtual int getNumParameters() const override { return 0; }
  virtual void computeA(const double *param, double A[9]) const override;
  virtual void computeAInv(const double *param, double AInv[9]) const override;
  virtual double compute_detA(const double * /*param*/) const override { return detFp; }

  // A constant plastic state has no parameter coordinates.  Keep its
  // parameter-space operations explicit here instead of inheriting the
  // identity/no-op fallbacks from PlasticModel3DDeformationGradient.
  void defaultFp(double *Fp) const override;
  void projectParam(double *param, double zeroThreshold) const override
  {
    (void)param;
    (void)zeroThreshold;
  }
  void toParam(const double *Fp, double *param) const override
  {
    (void)Fp;
    (void)param;
  }
  void computeR(const double *param, double R[9]) const override;
  void compute_dparamfull_dparamsub(
    const double *param, const double *basis, int numHandles,
    double *dpf_dps) const override
  {
    (void)param;
    (void)basis;
    (void)numHandles;
    (void)dpf_dps;
  }
  void compute_d2paramfull_dparamsub2(
    const double *param, const double *basis, int numHandles, int pi,
    double *d2a_dz2) const override
  {
    (void)param;
    (void)basis;
    (void)numHandles;
    (void)pi;
    (void)d2a_dz2;
  }
  void compute_paramfull(double *param) const override { (void)param; }
  void defaultParams(double *param) const override { (void)param; }

protected:
  double Fp[9], FpInv[9], detFp;
};

class VolumetricPlasticity0Config final : public PlasticModelConfig
{
public:
  std::string_view id() const override { return "volumetric_dof0"; }
  MaterialParameterSpec parameterSpec() const override;
  MaterialFrameRequirement frameRequirement() const override { return MaterialFrameRequirement::None; }
  void initializeDefaultParameters(const SimulationMesh &, int, std::span<double>) const override;
private:
  std::unique_ptr<PlasticModel> createModel(const SimulationMesh &, int, const MaterialFrame &) const override;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
