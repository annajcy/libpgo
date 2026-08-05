#pragma once

#include "deformation/deformationElement.h"
#include "deformation/shell/shellElementMapping.h"
#include "material/elastic/elasticModel2DFundamentalForms.h"
#include "material/plastic/plasticModel2DFundamentalForms.h"
#include "shellDeformationElementCache.h"
#include "EigenSupport.h"

#include <memory>
#include <span>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

// ShellDeformationElement — generic shell element facade.
// Works with any ShellElementMapping implementation.

class ShellDeformationElement : public DeformationElement
{
public:
  ShellDeformationElement(std::unique_ptr<ShellElementMapping> mapping,
    std::unique_ptr<ElasticModel2DFundamentalForms> elasticModel,
    std::unique_ptr<PlasticModel2DFundamentalForms> plasticModel,
    DeformationElementConstructionOptions options = {});

  double computeEnergy(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams) const override;
  void computeDisplacementGradient(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    EigenSupport::RefVecXd output) const override;
  void computeDisplacementHessian(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    EigenSupport::RefMatXd output) const override;
  void computeElasticGradient(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    EigenSupport::RefVecXd output) const override;
  void computePlasticGradient(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    EigenSupport::RefVecXd output) const override;
  void computeElasticVJP(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    std::span<const double> displacementAdjoint,
    EigenSupport::RefVecXd output) const override;
  void computePlasticVJP(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    std::span<const double> displacementAdjoint,
    EigenSupport::RefVecXd output) const override;
  int computeVonMisesStress(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    std::span<double> stresses) const override;

private:
  void prepareData(std::span<const double> x, std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    ShellDeformationElementCache &cacheDataBase) const;

  double compute_E(const ShellDeformationElementCache &cacheDataBase) const;

  void compute_dE_dx(const ShellDeformationElementCache &cacheDataBase,
    EigenSupport::RefVecXd grad) const;

  void compute_d2E_dx2(const ShellDeformationElementCache &cacheDataBase,
    EigenSupport::RefMatXd hess) const;

  void compute_d2E_dudp(const ShellDeformationElementCache &cacheDataBase,
    EigenSupport::RefMatXd hess, int materialLocation = -1) const;

  void compute_d2E_dude(const ShellDeformationElementCache &cacheDataBase,
    EigenSupport::RefMatXd hess, int materialLocation = -1) const;

  void compute_dE_dp(const ShellDeformationElementCache &cacheDataBase,
    EigenSupport::RefVecXd grad, int materialLocation = -1) const;

  void compute_dE_de(const ShellDeformationElementCache &cacheDataBase,
    EigenSupport::RefVecXd grad, int materialLocation = -1) const;

  int computeVonMisesStress(
    const ShellDeformationElementCache &cacheData,
    std::span<double> stresses, int capacity) const;

public:
  int getNumElasticParameters() const override { return numElasticParams_; }
  int getNumPlasticParameters() const override { return numPlasticParams_; }

  int getNumVertices() const override;
  int getNumDOFs() const override;

  LocalMaxStepResult computeLocalMaxStepSize(
    std::span<const double> x_local, std::span<const double> dx_local) const override;

private:
  std::unique_ptr<ShellElementMapping> elementMapping_;
  std::unique_ptr<ElasticModel2DFundamentalForms> elastic2D_;
  std::unique_ptr<PlasticModel2DFundamentalForms> plastic2D_;
  bool projectHessianPSD_ = false;

  int numPlasticParams_ = 0;
  int numElasticParams_ = 0;
  mutable ShellDeformationElementCache cache_;

  const ShellDeformationElementCache &cacheData(const ShellDeformationElementCache &cacheDataBase) const;
  ShellDeformationElementCache &cacheData(ShellDeformationElementCache &cacheDataBase) const;
  double compute_E(const ShellDeformationElementCache &cacheData,
    std::span<const double> plasticParams, std::span<const double> elasticParams) const;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
