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

  void compute_abar(const double *params, double *a) const override;
  void compute_bbar(const double *params, double *b) const override;

  void compute_tbar(const double *params, double *t) const override;
  double computeArea(const double *params) const override;

  void compute_Fp(const double *params, double *Fp) const override;

  void compute_dtbar_inv_dparam(const double *params, int j, double *dtbar_da) const override;
  void compute_dqbar_dparam(const double *params, int j, double *dqbar_da) const override;
  void compute_dK_dparam(const double *params, double *dK_da) const override;
  void compute_dH_dparam(const double *params, double *dH_da) const override;
  void compute_darea_dparam(const double *params, double *darea_da) const override;
  void compute_dabar_dparam(const double *params, double *dabar_dparam) const override;
  void compute_dbbar_dparam(const double *params, double *dbbar_dparam) const override;
  void compute_d2abar_dparam2(const double *params, int pi, int pj, double *d2abar_dparam2) const override;
  void compute_d2dbbar_dparam2(const double *params, int pi, int pj, double *d2dbbar_dparam2) const override;
  double compute_d2area_dparam2(const double *params, int pi, int pj) const override;

  int getNumParameters() const override { return 1; }
  void defaultParams(double *param) const override { param[0] = 1.0; }

protected:
};

class ShellPlasticity1Config final : public PlasticModelConfig
{
public:
  std::string_view id() const override { return "shell_ff_dof1"; }
  MaterialParameterSpec parameterSpec() const override;
  MaterialFrameRequirement frameRequirement() const override { return MaterialFrameRequirement::None; }
  void initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double>) const override;
private:
  std::unique_ptr<PlasticModel> createModel(const SimulationMesh &, int, const MaterialFrame &) const override;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
