#pragma once

#include "deformation/deformationModelEvaluator.h"
#include "volumetricDeformationModelCacheData.h"

#include "EigenSupport.h"

#include <span>

namespace pgo
{
namespace SolidDeformationModel
{

class VolumetricDeformationModel;

class VolumetricDeformationModelEvaluator final : public DeformationModelEvaluator
{
public:
  explicit VolumetricDeformationModelEvaluator(
    const VolumetricDeformationModel &model);

  void prepare(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams) override;
  double compute_E() const override;
  void compute_dE_dx(EigenSupport::RefVecXd grad) const override;
  void compute_d2E_dx2(EigenSupport::RefMatXd hess) const override;
  void compute_d2E_dudp(EigenSupport::RefMatXd hess,
    int materialLocation = -1) const override;
  void compute_d2E_dude(EigenSupport::RefMatXd hess,
    int materialLocation = -1) const override;
  void compute_dE_dp(EigenSupport::RefVecXd grad,
    int materialLocation = -1) const override;
  void compute_d2E_dp2(EigenSupport::RefMatXd hess,
    int materialLocation = -1) const override;
  void compute_dE_de(EigenSupport::RefVecXd grad,
    int materialLocation = -1) const override;
  void compute_d2E_de2(EigenSupport::RefMatXd hess,
    int materialLocation = -1) const override;
  void compute_d2E_dpde(EigenSupport::RefMatXd hess,
    int materialLocation = -1) const override;
  int computeVonMisesStress(std::span<double> stresses,
    int capacity) const override;
  int computeMaxStrain(std::span<double> strains,
    int capacity) const override;

  EigenSupport::M3d compute_Fe(int materialLocationID) const;
  EigenSupport::M3d compute_P(int materialLocationID) const;
  EigenSupport::M9d compute_dP_dF(int materialLocationID) const;
  void compute_dF_dx(int materialLocationID,
    EigenSupport::RefMatXd dFdxOut) const;
  void computeForceFromP(int materialLocationID,
    const EigenSupport::M3d &P, EigenSupport::RefVecXd f) const;

private:
  const VolumetricDeformationModel &model_;
  VolumetricDeformationModelCacheData state_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
