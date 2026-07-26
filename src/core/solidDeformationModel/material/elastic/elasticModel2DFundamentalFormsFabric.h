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

  double compute_psi_a(std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &abar) const override;
  double compute_psi_b(std::span<const double> param, const EigenSupport::M2d &b, const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const override;

  EigenSupport::M2d compute_dpsi_da(std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &abar) const override;
  EigenSupport::M2d compute_dpsi_db(std::span<const double> param, const EigenSupport::M2d &b, const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const override;

  EigenSupport::M4d compute_d2psi_da2(std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &abar) const override;
  EigenSupport::M4d compute_d2psi_db2(std::span<const double> param, const EigenSupport::M2d &b, const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const override;

  void compute_d2psi_da_dparam(std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &abar, EigenSupport::RefMatXd d2psi_dadparam) const override;
  void compute_d2psi_db_dparam(std::span<const double> param, const EigenSupport::M2d &b, const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar, EigenSupport::RefMatXd d2psi_dbdparam) const override;

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
  std::span<const std::string_view> parameterChannelNames() const override;
  MaterialFrameRequirement frameRequirement() const override { return MaterialFrameRequirement::None; }
  void initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double>) const override;
private:
  std::unique_ptr<ElasticModel> createModel(const SimulationMesh &, int, const MaterialFrame &) const override;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
