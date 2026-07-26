/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "material/elastic/invariantBasedMaterial.h"

namespace pgo
{
namespace SolidDeformationModel
{
class InvariantBasedMaterialStVK : public InvariantBasedMaterial
{
public:
  InvariantBasedMaterialStVK(double E, double nu, double compressionRatio);
  virtual ~InvariantBasedMaterialStVK() {}

  virtual double compute_psi(const EigenSupport::V3d &invariants) const override;
  virtual EigenSupport::V3d compute_dpsi_dI(
    const EigenSupport::V3d &invariants) const override;
  virtual EigenSupport::V6d compute_d2psi_dI2(
    const EigenSupport::V3d &invariants) const override;

  void setMaterial(double mu_, double lambda_) { this->mu = mu_, this->lambda = lambda_; }

protected:
  double mu, lambda, coeffJ;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
