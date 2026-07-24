/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "material/elastic/elasticModel3DDeformationGradient.h"

namespace pgo
{
namespace SolidDeformationModel
{
class ElasticModelLinearMaterial : public ElasticModel3DDeformationGradient
{
public:
  ElasticModelLinearMaterial(double mu_, double lambda_):
    mu(mu_), lambda(lambda_) {}
  virtual ~ElasticModelLinearMaterial() {}

  int getNumParameters() const override { return 0; }

  virtual double compute_psi(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3]) const override;

  virtual void compute_P(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3], double P[9]) const override;

  virtual void compute_dPdF(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3], double dPdFOut[81]) const override;

protected:
  double mu, lambda;
};
class LinearElasticConfig final : public ElasticModelConfig
{
public:
  std::string_view id() const override { return "linear"; }
  MaterialParameterSpec parameterSpec() const override;
  MaterialFrameRequirement frameRequirement() const override { return MaterialFrameRequirement::None; }
  void initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double>) const override;
private:
  std::unique_ptr<ElasticModel> createModel(const SimulationMesh &, int, const MaterialFrame &) const override;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
