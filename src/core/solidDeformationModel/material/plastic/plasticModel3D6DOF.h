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
class PlasticModel3D6DOF : public PlasticModel3DDeformationGradient
{
public:
  PlasticModel3D6DOF();
  explicit PlasticModel3D6DOF(const EigenSupport::M3d &R);
  ~PlasticModel3D6DOF() {}

  virtual int getNumParameters() const override { return 6; }
  virtual EigenSupport::M3d computeA(std::span<const double> param) const override;
  virtual EigenSupport::M3d computeAInv(std::span<const double> param) const override;
  virtual double compute_detA(std::span<const double> param) const override;

  void compute_ddetA_da(
    std::span<const double> param, EigenSupport::RefVecXd ddetA_da) const override;
  void compute_d2detA_da2(
    std::span<const double> param, EigenSupport::RefMatXd d2detA_da2) const override;
  virtual EigenSupport::M3d compute_dAInv_da(std::span<const double> param, int pi) const override;
  virtual EigenSupport::M3d compute_d2AInv_da2(std::span<const double> param, int pi, int pj) const override;

  virtual EigenSupport::M3d defaultFp() const override;
  void defaultParams(std::span<double> param) const override
  {
    param[0] = 1.0;
    param[1] = 0.0;
    param[2] = 0.0;
    param[3] = 1.0;
    param[4] = 0.0;
    param[5] = 1.0;
  }
  virtual void projectParam(std::span<double> param, double zeroThreshold) const override;
  virtual void toParam(const EigenSupport::M3d &Fp, std::span<double> param) const override;

  virtual EigenSupport::M3d computeR(std::span<const double> param) const override;

protected:
  EigenSupport::M3d R, RT;
};

inline EigenSupport::M3d PlasticModel3D6DOF::defaultFp() const
{
  return EigenSupport::M3d::Identity();
}

inline void PlasticModel3D6DOF::toParam(const EigenSupport::M3d &Fp, std::span<double> param) const
{
  param[0] = Fp(0, 0);
  param[1] = Fp(0, 1);
  param[2] = Fp(0, 2);

  param[3] = Fp(1, 1);
  param[4] = Fp(1, 2);

  param[5] = Fp(2, 2);
}

class VolumetricPlasticity6Config final : public PlasticModelConfig
{
public:
  std::string_view id() const override { return "volumetric_dof6"; }
  std::span<const std::string_view> parameterChannelNames() const override;
  MaterialFrameRequirement frameRequirement() const override { return MaterialFrameRequirement::None; }
  void initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double>) const override;
private:
  std::unique_ptr<PlasticModel> createModel(const SimulationMesh &, int, const MaterialFrame &) const override;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
