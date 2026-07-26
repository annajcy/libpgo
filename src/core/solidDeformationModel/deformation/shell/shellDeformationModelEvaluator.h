#pragma once

#include "deformation/deformationModelEvaluator.h"
#include "shellDeformationModelCacheData.h"

#include <span>

namespace pgo
{
namespace SolidDeformationModel
{

class ShellDeformationModel;

class ShellDeformationModelEvaluator final : public DeformationModelEvaluator
{
public:
  explicit ShellDeformationModelEvaluator(const ShellDeformationModel &model);

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

private:
  const ShellDeformationModel &model_;
  ShellDeformationModelCacheData state_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
