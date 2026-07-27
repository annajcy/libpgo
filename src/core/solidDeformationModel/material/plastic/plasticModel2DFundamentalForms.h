#pragma once

#include "material/model/plasticModelDefinition.h"

#include "material/plastic/plasticModel.h"
#include "EigenDef.h"

#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{
class PlasticModel2DFundamentalForms : public PlasticModel
{
public:
  PlasticModel2DFundamentalForms() {}
  virtual ~PlasticModel2DFundamentalForms() {}

  void set_abar(const EigenSupport::M2d &abar_) { abar = abar_; }
  void set_bbar(const EigenSupport::M2d &bbar_) { bbar = bbar_; }
  void set_tbar(const EigenSupport::M3d &tbar_) { tbar = tbar_; }
  void set_qbar(const EigenSupport::M3d &qbar_) { qbar = qbar_; }
  void setArea(double a) { areaRest = a; }

  int getNumParameters() const override { return 0; }

  // ShellPlasticity0 has no parameter coordinates, so its identity state is
  // intentionally a no-op.  Keeping this override here makes the contract
  // explicit while leaving PlasticModel's defaultParams pure virtual.
  void defaultParams(std::span<double> param) const override { (void)param; }

  virtual EigenSupport::M2d compute_abar(std::span<const double> params) const { (void)params; return abar; }
  virtual EigenSupport::M2d compute_bbar(std::span<const double> params) const { (void)params; return bbar; }

  virtual EigenSupport::M3d compute_tbar(std::span<const double> params) const { (void)params; return tbar; }
  virtual EigenSupport::M3d compute_qbar(std::span<const double> params) const { (void)params; return qbar; }

  virtual EigenSupport::M2d compute_Fp(std::span<const double> params) const
  { (void)params; return EigenSupport::M2d::Identity(); }

  virtual double computeArea(std::span<const double> params) const { (void)params; return areaRest; }

  virtual EigenSupport::M3d compute_dtbar_inv_dparam(std::span<const double> params, int j) const
  {
    (void)params;
    (void)j;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_dtbar_inv_dparam is not implemented");
  }
  virtual EigenSupport::M3d compute_dqbar_dparam(std::span<const double> params, int j) const
  {
    (void)params;
    (void)j;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_dqbar_dparam is not implemented");
  }
  virtual void compute_dK_dparam(std::span<const double> params, EigenSupport::RefVecXd dK_da) const
  {
    (void)params;
    (void)dK_da;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_dK_dparam is not implemented");
  }
  virtual void compute_dH_dparam(std::span<const double> params, EigenSupport::RefVecXd dH_da) const
  {
    (void)params;
    (void)dH_da;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_dH_dparam is not implemented");
  }
  virtual void compute_darea_dparam(std::span<const double> params, EigenSupport::RefVecXd darea_da) const
  {
    (void)params;
    (void)darea_da;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_darea_dparam is not implemented");
  }

  virtual void compute_dabar_dparam(std::span<const double> params, EigenSupport::RefMatXd dabar_dparam) const
  {
    (void)params;
    (void)dabar_dparam;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_dabar_dparam is not implemented");
  }
  virtual void compute_dbbar_dparam(std::span<const double> params, EigenSupport::RefMatXd dbbar_dparam) const
  {
    (void)params;
    (void)dbbar_dparam;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_dbbar_dparam is not implemented");
  }
  virtual EigenSupport::M2d compute_d2abar_dparam2(std::span<const double> params, int pi, int pj) const
  {
    (void)params;
    (void)pi;
    (void)pj;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_d2abar_dparam2 is not implemented");
  }
  virtual EigenSupport::M2d compute_d2dbbar_dparam2(std::span<const double> params, int pi, int pj) const
  {
    (void)params;
    (void)pi;
    (void)pj;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_d2dbbar_dparam2 is not implemented");
  }
  virtual double compute_d2area_dparam2(std::span<const double> params, int pi, int pj) const
  {
    (void)params;
    (void)pi;
    (void)pj;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_d2area_dparam2 is not implemented");
  }

protected:
  EigenSupport::M2d abar = EigenSupport::M2d::Identity(), bbar = EigenSupport::M2d::Identity();
  EigenSupport::M3d tbar = EigenSupport::M3d::Identity(), qbar = EigenSupport::M3d::Identity();
  double areaRest = 1.0;
};
class ShellPlasticity0Definition final : public PlasticModelDefinition
{
public:
  std::string_view id() const override { return "shell_ff_dof0"; }
  MaterialChannelSchema fixedChannelSchema() const override { return {}; }
  MaterialChannelSchema optimizableChannelSchema() const override;
  std::unique_ptr<PlasticModel> createModel(std::span<const double>, const MaterialFrame &) const override;
private:
};
}  // namespace SolidDeformationModel
}  // namespace pgo
