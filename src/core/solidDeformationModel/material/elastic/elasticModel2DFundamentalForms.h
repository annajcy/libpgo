/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "EigenSupport.h"
#include "material/elastic/elasticModel.h"

#include <span>
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

  virtual bool computeVonMisesStress(std::span<const double> param,
    const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar, double &stress) const
  {
    (void)param; (void)a; (void)b; (void)abar; (void)bbar; (void)stress;
    return false;
  }

  virtual double compute_psi_a(std::span<const double> param,
    const EigenSupport::M2d &a, const EigenSupport::M2d &abar) const = 0;
  virtual double compute_psi_b(std::span<const double> param,
    const EigenSupport::M2d &b, const EigenSupport::M2d &abar,
    const EigenSupport::M2d &bbar) const = 0;

  virtual EigenSupport::M2d compute_dpsi_da(std::span<const double> param,
    const EigenSupport::M2d &a, const EigenSupport::M2d &abar) const = 0;
  virtual EigenSupport::M2d compute_dpsi_db(std::span<const double> param,
    const EigenSupport::M2d &b, const EigenSupport::M2d &abar,
    const EigenSupport::M2d &bbar) const = 0;

  virtual EigenSupport::M4d compute_d2psi_da2(std::span<const double> param,
    const EigenSupport::M2d &a, const EigenSupport::M2d &abar) const = 0;
  virtual EigenSupport::M4d compute_d2psi_db2(std::span<const double> param,
    const EigenSupport::M2d &b, const EigenSupport::M2d &abar,
    const EigenSupport::M2d &bbar) const = 0;

  virtual EigenSupport::M4d compute_d2psi_dadabar(std::span<const double> param,
    const EigenSupport::M2d &a, const EigenSupport::M2d &abar) const
  {
    (void)param;
    (void)a;
    (void)abar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_dadabar is not implemented");
  }
  virtual EigenSupport::M4d compute_d2psi_db_dabar(std::span<const double> param,
    const EigenSupport::M2d &b, const EigenSupport::M2d &abar,
    const EigenSupport::M2d &bbar) const
  {
    (void)param;
    (void)b;
    (void)abar;
    (void)bbar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_db_dabar is not implemented");
  }
  virtual EigenSupport::M4d compute_d2psi_db_dbbar(std::span<const double> param,
    const EigenSupport::M2d &b, const EigenSupport::M2d &abar,
    const EigenSupport::M2d &bbar) const
  {
    (void)param;
    (void)b;
    (void)abar;
    (void)bbar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_db_dbbar is not implemented");
  }

  virtual void compute_d2psi_da_dparam(std::span<const double> param,
    const EigenSupport::M2d &a, const EigenSupport::M2d &abar,
    EigenSupport::RefMatXd d2psi_dadparam) const
  {
    (void)param;
    (void)a;
    (void)abar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_da_dparam is not implemented");
  }
  virtual void compute_d2psi_db_dparam(std::span<const double> param,
    const EigenSupport::M2d &b, const EigenSupport::M2d &abar,
    const EigenSupport::M2d &bbar, EigenSupport::RefMatXd d2psi_dbdparam) const
  {
    (void)param;
    (void)b;
    (void)abar;
    (void)bbar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_db_dparam is not implemented");
  }

  virtual EigenSupport::M2d compute_dpsi_dabar(std::span<const double> param,
    const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_dpsi_dabar is not implemented");
  }
  virtual EigenSupport::M2d compute_dpsi_dbbar(std::span<const double> param,
    const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_dpsi_dbbar is not implemented");
  }
  virtual void compute_dpsi_dparam(std::span<const double> param,
    const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar,
    EigenSupport::RefVecXd dpsi_dparam) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_dpsi_dparam is not implemented");
  }
  virtual void compute_d2psi_dparam2(std::span<const double> param,
    const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar,
    EigenSupport::RefMatXd d2psi_dparam2) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_dparam2 is not implemented");
  }
  virtual void compute_d2psi_dabar_dparam(std::span<const double> param,
    const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar,
    EigenSupport::RefMatXd d2psi_dabar_dparam) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_dabar_dparam is not implemented");
  }
  virtual void compute_d2psi_dbbar_dparam(std::span<const double> param,
    const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar,
    EigenSupport::RefMatXd d2psi_dbbar_dparam) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_dbbar_dparam is not implemented");
  }
  virtual EigenSupport::M4d compute_d2psi_dabar2(std::span<const double> param,
    const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_dabar2 is not implemented");
  }
  virtual EigenSupport::M4d compute_d2psi_dabar_dbbar(std::span<const double> param,
    const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_dabar_dbbar is not implemented");
  }
  virtual EigenSupport::M4d compute_d2psi_dbbar2(std::span<const double> param,
    const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const
  {
    (void)param;
    (void)a;
    (void)b;
    (void)abar;
    (void)bbar;
    throw std::logic_error("ElasticModel2DFundamentalForms::compute_d2psi_dbbar2 is not implemented");
  }

  int getNumParameters() const override = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
