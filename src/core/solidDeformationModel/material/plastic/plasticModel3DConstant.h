/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "material/plastic/plasticModelDefinition.h"

#include "material/plastic/plasticModel3DDeformationGradient.h"

namespace pgo
{
namespace SolidDeformationModel
{
class PlasticModel3DConstant : public PlasticModel3DDeformationGradient
{
public:
  explicit PlasticModel3DConstant(const EigenSupport::M3d &Fp);
  ~PlasticModel3DConstant() {}

  virtual int getNumParameters() const override { return 0; }
  virtual EigenSupport::M3d computeA(std::span<const double> param) const override;
  virtual EigenSupport::M3d computeAInv(std::span<const double> param) const override;
  virtual double compute_detA(std::span<const double> /*param*/) const override { return detFp; }

  // A constant plastic state has no parameter coordinates.  Keep its
  // parameter-space operations explicit here instead of inheriting the
  // identity/no-op fallbacks from PlasticModel3DDeformationGradient.
  EigenSupport::M3d defaultFp() const override;
  void projectParam(std::span<double> param, double zeroThreshold) const override
  {
    (void)param;
    (void)zeroThreshold;
  }
  void toParam(const EigenSupport::M3d &Fp, std::span<double> param) const override
  {
    (void)Fp;
    (void)param;
  }
  EigenSupport::M3d computeR(std::span<const double> param) const override;

protected:
  EigenSupport::M3d Fp, FpInv;
  double detFp = 0.0;
};

class VolumetricPlasticity0Definition final : public PlasticModelDefinition
{
public:
  std::string_view id() const override { return "volumetric_dof0"; }
  int numFixedChannels() const override { return 0; }
  int numOptimizableChannels() const override { return 0; }
  std::unique_ptr<PlasticModel> createModel(std::span<const double>, const MaterialFrame &) const override;
private:
};
}  // namespace SolidDeformationModel
}  // namespace pgo
