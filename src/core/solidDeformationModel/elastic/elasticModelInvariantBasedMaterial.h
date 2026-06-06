/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "elastic/elasticModel3DDeformationGradient.h"

#include <memory>

namespace pgo
{
namespace SolidDeformationModel
{
class InvariantBasedMaterial;

class ElasticModelInvariantBasedMaterial : public ElasticModel3DDeformationGradient
{
public:
  explicit ElasticModelInvariantBasedMaterial(std::unique_ptr<InvariantBasedMaterial> invMat);
  ~ElasticModelInvariantBasedMaterial() override = default;

  void enableSPD(int enable) override { enforceSPD_ = enable ? 1 : 0; }

  virtual double compute_psi(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3]) const override;
  virtual void compute_P(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3], double P[9]) const override;
  virtual void compute_dPdF(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3], double dPdFOut[81]) const override;

  const InvariantBasedMaterial *getInvariantBasedMaterial() const { return invariantBasedMaterial_.get(); }

protected:
  std::unique_ptr<InvariantBasedMaterial> invariantBasedMaterial_;
  int enforceSPD_ = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo