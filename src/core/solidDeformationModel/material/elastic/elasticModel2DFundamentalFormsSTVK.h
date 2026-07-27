/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "material/model/elasticModelDefinition.h"

#include "material/elastic/elasticModel2DFundamentalForms.h"

namespace pgo
{
namespace SolidDeformationModel
{
class ElasticModel2DFundamentalFormsSTVK : public ElasticModel2DFundamentalForms
{
public:
  ElasticModel2DFundamentalFormsSTVK() {}
  virtual ~ElasticModel2DFundamentalFormsSTVK() {}

  bool computeVonMisesStress(std::span<const double> param,
    const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar, double &stress) const override;

  double compute_psi_a(std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &abar) const override;
  double compute_psi_b(std::span<const double> param, const EigenSupport::M2d &b, const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const override;

  EigenSupport::M2d compute_dpsi_da(std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &abar) const override;
  EigenSupport::M2d compute_dpsi_db(std::span<const double> param, const EigenSupport::M2d &b, const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const override;

  EigenSupport::M4d compute_d2psi_da2(std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &abar) const override;
  EigenSupport::M4d compute_d2psi_db2(std::span<const double> param, const EigenSupport::M2d &b, const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const override;

  EigenSupport::M4d compute_d2psi_dadabar(std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &abar) const override;
  EigenSupport::M4d compute_d2psi_db_dabar(std::span<const double> param, const EigenSupport::M2d &b, const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const override;
  EigenSupport::M4d compute_d2psi_db_dbbar(std::span<const double> param, const EigenSupport::M2d &b, const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const override;

  void compute_d2psi_da_dparam(std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &abar, EigenSupport::RefMatXd d2psi_dadparam) const override;
  void compute_d2psi_db_dparam(std::span<const double> param, const EigenSupport::M2d &b, const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar, EigenSupport::RefMatXd d2psi_dbdparam) const override;

  EigenSupport::M2d compute_dpsi_dabar(
    std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const override;
  EigenSupport::M2d compute_dpsi_dbbar(
    std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const override;
  void compute_dpsi_dparam(
    std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar,
    EigenSupport::RefVecXd dpsi_dparam) const override;
  void compute_d2psi_dparam2(
    std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar,
    EigenSupport::RefMatXd d2psi_dparam2) const override;
  void compute_d2psi_dabar_dparam(
    std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar,
    EigenSupport::RefMatXd d2psi_dabar_dparam) const override;
  void compute_d2psi_dbbar_dparam(
    std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar,
    EigenSupport::RefMatXd d2psi_dbbar_dparam) const override;
  EigenSupport::M4d compute_d2psi_dabar2(
    std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const override;
  EigenSupport::M4d compute_d2psi_dabar_dbbar(
    std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const override;
  EigenSupport::M4d compute_d2psi_dbbar2(
    std::span<const double> param, const EigenSupport::M2d &a, const EigenSupport::M2d &b,
    const EigenSupport::M2d &abar, const EigenSupport::M2d &bbar) const override;

  int getNumParameters() const override { return 5; };
};

class KoiterStVKDefinition final : public ElasticModelDefinition
{
public:
  std::string_view id() const override { return "koiter_stvk"; }
  MaterialChannelSchema fixedChannelSchema() const override { return {}; }
  MaterialChannelSchema optimizableChannelSchema() const override;
  std::unique_ptr<ElasticModel> createModel(std::span<const double>, const MaterialFrame &) const override;
private:
};
}  // namespace SolidDeformationModel
}  // namespace pgo
