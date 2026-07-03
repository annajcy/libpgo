#pragma once

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

  virtual int getNumParameters() const { return 0; }

  virtual void compute_abar(const double *params, double *a) const { (Eigen::Map<EigenSupport::M2d>(a)) = abar; }
  virtual void compute_bbar(const double *params, double *b) const { (Eigen::Map<EigenSupport::M2d>(b)) = bbar; }

  virtual void compute_tbar(const double *params, double *t) const { (Eigen::Map<EigenSupport::M3d>(t)) = tbar; }
  virtual void compute_qbar(const double *params, double *q) const { (Eigen::Map<EigenSupport::M3d>(q)) = qbar; }

  virtual void compute_Fp(const double *params, double *Fp) const
  {
    Fp[0] = 1.0;
    Fp[1] = 0.0;
    Fp[2] = 0.0;
    Fp[3] = 1.0;
  }

  virtual double computeArea(const double *params) const { return areaRest; }

  virtual void compute_dtbar_inv_dparam(const double *params, int j, double *dtbar_da) const
  {
    (void)params;
    (void)j;
    (void)dtbar_da;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_dtbar_inv_dparam is not implemented");
  }
  virtual void compute_dqbar_dparam(const double *params, int j, double *dqbar_da) const
  {
    (void)params;
    (void)j;
    (void)dqbar_da;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_dqbar_dparam is not implemented");
  }
  virtual void compute_dK_dparam(const double *params, double *dK_da) const
  {
    (void)params;
    (void)dK_da;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_dK_dparam is not implemented");
  }
  virtual void compute_dH_dparam(const double *params, double *dH_da) const
  {
    (void)params;
    (void)dH_da;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_dH_dparam is not implemented");
  }
  virtual void compute_darea_dparam(const double *params, double *darea_da) const
  {
    (void)params;
    (void)darea_da;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_darea_dparam is not implemented");
  }

  virtual void compute_dabar_dparam(const double *params, double *dabar_dparam) const
  {
    (void)params;
    (void)dabar_dparam;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_dabar_dparam is not implemented");
  }
  virtual void compute_dbbar_dparam(const double *params, double *dbbar_dparam) const
  {
    (void)params;
    (void)dbbar_dparam;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_dbbar_dparam is not implemented");
  }
  virtual void compute_d2abar_dparam2(const double *params, int pi, int pj, double *d2abar_dparam2) const
  {
    (void)params;
    (void)pi;
    (void)pj;
    (void)d2abar_dparam2;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_d2abar_dparam2 is not implemented");
  }
  virtual void compute_d2dbbar_dparam2(const double *params, int pi, int pj, double *d2dbbar_dparam2) const
  {
    (void)params;
    (void)pi;
    (void)pj;
    (void)d2dbbar_dparam2;
    throw std::logic_error("PlasticModel2DFundamentalForms::compute_d2dbbar_dparam2 is not implemented");
  }
  virtual double compute_d2area_dparam2(const double *params, int pi, int pj) const
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
}  // namespace SolidDeformationModel
}  // namespace pgo
