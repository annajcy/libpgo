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
class ElasticModelStableNeoHookeanMaterial : public ElasticModel3DDeformationGradient
{
public:
  ElasticModelStableNeoHookeanMaterial(double mu, double lambda);
  virtual ~ElasticModelStableNeoHookeanMaterial();

  int getNumParameters() const override { return 0; }

  virtual double compute_psi(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3]) const override;
  virtual void compute_P(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3], double P[9]) const override;
  virtual void compute_dPdF(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3], double dPdFOut[81]) const override;
  void compute_dPdF_psd(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3], double dPdFOut[81]) const override;

  void setMaterial(double mu_, double lambda_);

protected:
  double _mu, _lambda, _ratio;

private:
  void compute_dPdF_impl(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3],
    double dPdFOut[81], bool project) const;
};
class StableNeoConfig final : public ElasticModelConfig
{
public:
  std::string_view id() const override { return "stable_neo"; }
  MaterialParameterSpec parameterSpec() const override;
  MaterialFrameRequirement frameRequirement() const override { return MaterialFrameRequirement::None; }
  void initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double>) const override;
private:
  std::unique_ptr<ElasticModel> createModel(const SimulationMesh &, int, const MaterialFrame &) const override;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
