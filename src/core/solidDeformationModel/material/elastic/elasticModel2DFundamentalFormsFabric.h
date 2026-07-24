#pragma once

#include "material/elastic/elasticModel2DFundamentalForms.h"
#include "EigenDef.h"

namespace pgo
{
namespace SolidDeformationModel
{
class ElasticModel2DFundamentalFormsFabric : public ElasticModel2DFundamentalForms
{
public:
  ElasticModel2DFundamentalFormsFabric(const EigenSupport::V2d &warpDir_, const EigenSupport::V2d &weftDir_):
    warpDir(warpDir_), weftDir(weftDir_) {}
  virtual ~ElasticModel2DFundamentalFormsFabric() {}

  double compute_psi_a(const double *param, const double a[4], const double abar[4]) const override;
  double compute_psi_b(const double *param, const double b[4], const double abar[4], const double bbar[4]) const override;

  void compute_dpsi_da(const double *param, const double a[4], const double abar[4], double da[4]) const override;
  void compute_dpsi_db(const double *param, const double b[4], const double abar[4], const double bbar[4], double db[4]) const override;

  void compute_d2psi_da2(const double *param, const double a[4], const double abar[4], double da2[16]) const override;
  void compute_d2psi_db2(const double *param, const double b[4], const double abar[4], const double bbar[4], double db2[16]) const override;

  void compute_d2psi_da_dparam(const double *param, const double a[4], const double abar[4], double d2psi_dadparam[/*4 x numParams*/]) const override;
  void compute_d2psi_db_dparam(const double *param, const double b[4], const double abar[4], const double bbar[4], double d2psi_dbdparam[/*4 x numParams*/]) const override;

  int getNumParameters() const override { return 12; };

protected:
  // Fiber directions in reference configuration
  EigenSupport::V2d warpDir;  // warp
  EigenSupport::V2d weftDir;  // weft

  // C1 smoothing radii for warp/weft fibers (Macauley smoothing)
  double eps4 = 1e-6;
  double eps6 = 1e-6;
};

class KoiterFabricConfig final : public ElasticModelConfig
{
public:
  std::string_view id() const override { return "koiter_fabric"; }
  MaterialParameterSpec parameterSpec() const override;
  MaterialFrameRequirement frameRequirement() const override { return MaterialFrameRequirement::None; }
  void initializeDefaultParameters(const SimulationMesh &, int, std::span<double>) const override;
private:
  std::unique_ptr<ElasticModel> createModel(const SimulationMesh &, int, const MaterialFrame &) const override;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
