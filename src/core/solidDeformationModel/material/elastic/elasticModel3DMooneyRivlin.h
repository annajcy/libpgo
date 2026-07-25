#pragma once

#include "material/elastic/elasticModel3DDeformationGradient.h"
#include "EigenDef.h"

namespace pgo
{
namespace SolidDeformationModel
{
class ElasticModel3DMooneyRivlin : public ElasticModel3DDeformationGradient
{
public:
  ElasticModel3DMooneyRivlin(double mu01, double mu10, double v1);
  virtual ~ElasticModel3DMooneyRivlin() {}

  int getNumParameters() const override { return 0; }

  virtual double compute_psi(const double *param, const double _F[9], const double _U[], const double _V[], const double _S[]) const override;
  virtual void compute_P(const double *param, const double _F[9], const double _U[], const double _V[], const double _S[], double P[9]) const override;
  virtual void compute_dPdF(const double *param, const double _F[9], const double _U[], const double _V[], const double _S[], double dPdF[81]) const override;  

protected:
  double mu01_;
  double mu10_;
  double v1_;
};

class MooneyRivlinConfig final : public ElasticModelConfig
{
public:
  std::string_view id() const override { return "mooney_rivlin"; }
  MaterialParameterSpec parameterSpec() const override;
  MaterialFrameRequirement frameRequirement() const override { return MaterialFrameRequirement::None; }
  void initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double>) const override;
private:
  std::unique_ptr<ElasticModel> createModel(const SimulationMesh &, int, const MaterialFrame &) const override;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
