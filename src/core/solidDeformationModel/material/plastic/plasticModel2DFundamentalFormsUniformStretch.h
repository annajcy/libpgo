/*
author: Bohan Wang
copyright to USC,MIT, NUS
*/

#pragma once

#include "material/plastic/plasticModel2DFundamentalForms.h"

namespace pgo
{
namespace SolidDeformationModel
{
class PlasticModel2DFundamentalFormsUniformStretch : public PlasticModel2DFundamentalForms
{
public:
  PlasticModel2DFundamentalFormsUniformStretch();
  ~PlasticModel2DFundamentalFormsUniformStretch() {}

  EigenSupport::M2d compute_abar(std::span<const double> params) const override;
  EigenSupport::M2d compute_bbar(std::span<const double> params) const override;

  EigenSupport::M3d compute_tbar(std::span<const double> params) const override;
  double computeArea(std::span<const double> params) const override;

  EigenSupport::M2d compute_Fp(std::span<const double> params) const override;

  EigenSupport::M3d compute_dtbar_inv_dparam(std::span<const double> params, int j) const override;
  EigenSupport::M3d compute_dqbar_dparam(std::span<const double> params, int j) const override;
  void compute_dK_dparam(std::span<const double> params, EigenSupport::RefVecXd dK_da) const override;
  void compute_dH_dparam(std::span<const double> params, EigenSupport::RefVecXd dH_da) const override;
  void compute_darea_dparam(std::span<const double> params, EigenSupport::RefVecXd darea_da) const override;
  void compute_dabar_dparam(std::span<const double> params, EigenSupport::RefMatXd dabar_dparam) const override;
  void compute_dbbar_dparam(std::span<const double> params, EigenSupport::RefMatXd dbbar_dparam) const override;
  EigenSupport::M2d compute_d2abar_dparam2(std::span<const double> params, int pi, int pj) const override;
  EigenSupport::M2d compute_d2dbbar_dparam2(std::span<const double> params, int pi, int pj) const override;
  double compute_d2area_dparam2(std::span<const double> params, int pi, int pj) const override;

  int getNumParameters() const override { return 1; }
  void defaultParams(std::span<double> param) const override { param[0] = 1.0; }

protected:
};

class ShellPlasticity1Config final : public PlasticModelConfig
{
public:
  std::string_view id() const override { return "shell_ff_dof1"; }
  std::span<const std::string_view> parameterChannelNames() const override;
  MaterialFrameRequirement frameRequirement() const override { return MaterialFrameRequirement::None; }
  void initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double>) const override;
private:
  std::unique_ptr<PlasticModel> createModel(const SimulationMesh &, int, const MaterialFrame &) const override;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
