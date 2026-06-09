/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "material/elastic/elasticModel.h"

#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{
class ElasticModel2DFundamentalForms : public ElasticModel
{
public:
  ElasticModel2DFundamentalForms() {}
  virtual ~ElasticModel2DFundamentalForms() {}

  virtual double compute_psi_a(const double *param, const double a[4], const double abar[4]) const = 0;
  virtual double compute_psi_b(const double *param, const double b[4], const double abar[4], const double bbar[4]) const = 0;

  virtual void compute_dpsi_da(const double *param, const double a[4], const double abar[4], double da[4]) const = 0;
  virtual void compute_dpsi_db(const double *param, const double b[4], const double abar[4], const double bbar[4], double db[4]) const = 0;

  virtual void compute_d2psi_da2(const double *param, const double a[4], const double abar[4], double da2[16]) const = 0;
  virtual void compute_d2psi_db2(const double *param, const double b[4], const double abar[4], const double bbar[4], double db2[16]) const = 0;

  virtual void compute_d2psi_dadabar(const double *param, const double a[4], const double abar[4], double dadabar[16]) const
  {
    (void)param;
    (void)a;
    (void)abar;
    (void)dadabar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_dadabar is not implemented");
  }
  virtual void compute_d2psi_db_dabar(const double *param, const double b[4], const double abar[4], const double bbar[4], double dbdabar[16]) const
  {
    (void)param;
    (void)b;
    (void)abar;
    (void)bbar;
    (void)dbdabar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_db_dabar is not implemented");
  }
  virtual void compute_d2psi_db_dbbar(const double *param, const double b[4], const double abar[4], const double bbar[4], double dbdbbar[16]) const
  {
    (void)param;
    (void)b;
    (void)abar;
    (void)bbar;
    (void)dbdbbar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_db_dbbar is not implemented");
  }

  virtual void compute_d2psi_da_dparam(const double *param, const double a[4], const double abar[4], double d2psi_dadparam[/*4 x numParams*/]) const
  {
    (void)param;
    (void)a;
    (void)abar;
    (void)d2psi_dadparam;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_da_dparam is not implemented");
  }
  virtual void compute_d2psi_db_dparam(const double *param, const double b[4], const double abar[4], const double bbar[4], double d2psi_dbdparam[/*4 x numParams*/]) const
  {
    (void)param;
    (void)b;
    (void)abar;
    (void)bbar;
    (void)d2psi_dbdparam;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_db_dparam is not implemented");
  }

  virtual void compute_dpsi_dabar(
    const double *param, const double a[4], const double b[4],
    const double abar[4], const double bbar[4], double dpsi_dabar[4]) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    (void)dpsi_dabar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_dpsi_dabar is not implemented");
  }
  virtual void compute_dpsi_dbbar(
    const double *param, const double a[4], const double b[4],
    const double abar[4], const double bbar[4], double dpsi_dbbar[4]) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    (void)dpsi_dbbar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_dpsi_dbbar is not implemented");
  }
  virtual void compute_dpsi_dparam(
    const double *param, const double a[4], const double b[4],
    const double abar[4], const double bbar[4], double dpsi_dparam[/*numParams*/]) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    (void)dpsi_dparam;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_dpsi_dparam is not implemented");
  }
  virtual void compute_d2psi_dparam2(
    const double *param, const double a[4], const double b[4],
    const double abar[4], const double bbar[4], double d2psi_dparam2[/*numParams x numParams*/]) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    (void)d2psi_dparam2;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_dparam2 is not implemented");
  }
  virtual void compute_d2psi_dabar_dparam(
    const double *param, const double a[4], const double b[4],
    const double abar[4], const double bbar[4], double d2psi_dabar_dparam[/*4 x numParams*/]) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    (void)d2psi_dabar_dparam;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_dabar_dparam is not implemented");
  }
  virtual void compute_d2psi_dbbar_dparam(
    const double *param, const double a[4], const double b[4],
    const double abar[4], const double bbar[4], double d2psi_dbbar_dparam[/*4 x numParams*/]) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    (void)d2psi_dbbar_dparam;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_dbbar_dparam is not implemented");
  }
  virtual void compute_d2psi_dabar2(
    const double *param, const double a[4], const double b[4],
    const double abar[4], const double bbar[4], double d2psi_dabar2[16]) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    (void)d2psi_dabar2;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_dabar2 is not implemented");
  }
  virtual void compute_d2psi_dabar_dbbar(
    const double *param, const double a[4], const double b[4],
    const double abar[4], const double bbar[4], double d2psi_dabar_dbbar[16]) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    (void)d2psi_dabar_dbbar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_dabar_dbbar is not implemented");
  }
  virtual void compute_d2psi_dbbar2(
    const double *param, const double a[4], const double b[4],
    const double abar[4], const double bbar[4], double d2psi_dbbar2[16]) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    (void)d2psi_dbbar2;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_dbbar2 is not implemented");
  }

  int getNumParameters() const override { return 0; };
};

}  // namespace SolidDeformationModel
}  // namespace pgo
