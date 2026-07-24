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
class ElasticModelVolumeMaterial : public ElasticModel3DDeformationGradient
{
public:
  ElasticModelVolumeMaterial(double coeff = 1.0):
    scale(coeff) {}
  virtual ~ElasticModelVolumeMaterial() {}

  int getNumParameters() const override { return 0; }

  virtual double compute_psi(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3]) const override;

  virtual void compute_P(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3], double P[9]) const override;

  virtual void compute_dPdF(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3], double dPdFOut[81]) const override;

  void setCoeff(double v) { scale = v; }

protected:
  double scale;
};
class VolumePenaltyConfig final : public ElasticModelConfig
{
public:
  std::string_view id() const override { return "volume"; }
  MaterialParameterSpec parameterSpec() const override;
  MaterialFrameRequirement frameRequirement() const override { return MaterialFrameRequirement::None; }
  void initializeDefaultParameters(const SimulationMesh &, int, std::span<double>) const override;
private:
  std::unique_ptr<ElasticModel> createModel(const SimulationMesh &, int, const MaterialFrame &) const override;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
