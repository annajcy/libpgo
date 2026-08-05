#pragma once

#include "deformation/deformationElement.h"
#include "formulations/shapeFunction/shapeFunction.h"
#include "formulations/quadrature/quadrature.h"
#include "material/elastic/elasticModel3DDeformationGradient.h"
#include "material/plastic/plasticModel3DDeformationGradient.h"
#include "volumetricDeformationElementCache.h"

#include "EigenSupport.h"

#include <memory>
#include <span>

namespace pgo
{
namespace SolidDeformationModel
{

class VolumetricDeformationElement : public DeformationElement
{
public:
  using M3xN = Eigen::Matrix<double, 3, Eigen::Dynamic>;
  using M9xNDOF = Eigen::Matrix<double, 9, Eigen::Dynamic>;

  VolumetricDeformationElement(std::span<const double> restPositions,
    const ShapeFunction &shapeFunction, const Quadrature &quadrature,
    std::unique_ptr<ElasticModel3DDeformationGradient> elasticModel,
    std::unique_ptr<PlasticModel3DDeformationGradient> plasticModel,
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
  int computeMaxStrain(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    std::span<double> strains) const override;

private:
  void prepareData(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    VolumetricDeformationElementCache &cacheDataBase) const;

  // DeformationModels.

  double compute_E(const VolumetricDeformationElementCache &cacheData) const;
  void compute_dE_dx(const VolumetricDeformationElementCache &cacheData, EigenSupport::RefVecXd grad) const;
  void compute_d2E_dx2(const VolumetricDeformationElementCache &cacheData, EigenSupport::RefMatXd hess) const;
  void compute_d2E_dudp(
    const VolumetricDeformationElementCache &cacheData, EigenSupport::RefMatXd hess,
    int materialLocation = -1) const;
  void compute_d2E_dude(
    const VolumetricDeformationElementCache &cacheData, EigenSupport::RefMatXd hess,
    int materialLocation = -1) const;

  void compute_dE_dp(
    const VolumetricDeformationElementCache &cacheData, EigenSupport::RefVecXd grad,
    int materialLocation = -1) const;
  void compute_dE_de(
    const VolumetricDeformationElementCache &cacheData, EigenSupport::RefVecXd grad,
    int materialLocation = -1) const;

public:
  int getNumElasticParameters() const override { return numElasticParams_; }
  int getNumPlasticParameters() const override { return numPlasticParams_; }
  int getNumVertices() const override { return numNodes_; }
  int getNumDOFs() const override { return localDofs_; }
  int getNumMaterialLocations() const override { return numQuadPts_; }

  LocalMaxStepResult computeLocalMaxStepSize(std::span<const double> x_local,
    std::span<const double> dx_local) const override;

  ES::M3d compute_F(std::span<const double> x, int materialLocationID) const;

private:
  ES::M3d compute_Fe(const VolumetricDeformationElementCache &cacheData, int materialLocationID) const;
  ES::M3d compute_P(const VolumetricDeformationElementCache &cacheData, int materialLocationID) const;
  ES::M9d compute_dP_dF(const VolumetricDeformationElementCache &cacheData, int materialLocationID) const;
  void compute_dF_dx(const VolumetricDeformationElementCache &cacheData, int materialLocationID,
    EigenSupport::RefMatXd dFdxOut) const;
  void computeForceFromP(const VolumetricDeformationElementCache &cacheData, int materialLocationID,
    const ES::M3d &P, EigenSupport::RefVecXd f) const;

  int computeVonMisesStress(
    const VolumetricDeformationElementCache &cacheData,
    std::span<double> stresses, int capacity) const;
  int computeMaxStrain(
    const VolumetricDeformationElementCache &cacheData,
    std::span<double> strains, int capacity) const;

public:
  ES::M3d computeFirstPiola(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    int materialLocationID) const;
  ES::M9d computeFirstPiolaDerivative(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    int materialLocationID) const;
  void computeDeformationGradientDerivative(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    int materialLocationID, EigenSupport::RefMatXd output) const;
  void computeForceFromFirstPiola(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    int materialLocationID, const ES::M3d &P,
    EigenSupport::RefVecXd output) const;

  static SpectralState computeSpectralState(const ES::M3d &Fe);

private:
  struct RestGeometry
  {
    RestGeometry(std::span<const double> restPositions,
      const ShapeFunction &shapeFunction, const Quadrature &quadrature);

    ES::M3d computeDeformationGradient(
      std::span<const double> localPositions, int q) const;

    int numNodes = 0;
    int numQuadraturePoints = 0;
    int localDofs = 0;
    std::vector<M3xN> dN_dxi;
    std::vector<ES::M3d> restDmInv;
    std::vector<M9xNDOF> rest_dF_dx;
    std::vector<double> weightDetJ;
    std::vector<M3xN> restBm;
  };

  RestGeometry geometry_;
  int numNodes_ = 0;
  int numQuadPts_ = 0;
  int localDofs_ = 0;
  std::unique_ptr<ElasticModel3DDeformationGradient> elasticModel_;
  std::unique_ptr<PlasticModel3DDeformationGradient> plasticModel_;

  int numPlasticParams_ = 0;
  int numElasticParams_ = 0;
  bool projectHessianPSD_ = false;
  mutable VolumetricDeformationElementCache cache_;

  std::span<const double> elasticParams(const VolumetricDeformationElementCache &cacheData) const;

  double compute_dV_dai(double weightDetJ, double ddetA_dai) const;
  ES::M3d compute_dFe_dai(
    const ES::M3d &Fref, const ES::M3d &dAInvdai) const;
  ES::M3d compute_dP_dai(
    const ES::M9d &dPdF, const ES::M3d &dFdai) const;
  double compute_dpsi_dai(const ES::M3d &Fref, const ES::M3d &dAInv_dai, const ES::M3d &P) const;
  void compute_d2Fe_dx_dai(const ES::M3d &dAInvdai, const M9xNDOF &rest_dF_dx, M9xNDOF &d2Fdudai) const;
  void computeCurrent_dF_dx(const M9xNDOF &rest_dF_dx, const ES::M3d &FpInv, M9xNDOF &dFdx) const;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
