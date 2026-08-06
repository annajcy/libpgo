#pragma once

#include "deformation/deformationElement.h"
#include "material/elastic/elasticModel2DFundamentalForms.h"
#include "material/plastic/plasticModel2DFundamentalForms.h"
#include "shellDeformationElementCache.h"
#include "EigenSupport.h"

#include <memory>
#include <array>
#include <span>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

class KoiterShellDeformationElement final : public DeformationElement
{
public:
  using APositions = std::array<ES::V3d, 3>;
  using BPositions = std::array<ES::V3d, 6>;

  static constexpr int numNodes = 6;
  static constexpr int localDofs = 18;
  static constexpr std::array<int, 3> oppVtx = { 4, 5, 3 };

  KoiterShellDeformationElement(
    const ES::V18d &restPositions, const std::array<bool, 6> &hasVertex,
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
  double computeEnergyGradient(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    EigenSupport::RefVecXd displacementGradient) const override;
  double computeEnergyGradientHessian(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    EigenSupport::RefVecXd displacementGradient,
    EigenSupport::RefMatXd displacementHessian) const override;
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

  ES::M2d computeFirstFundamentalForm(const APositions &x) const;
  ES::M4x9d computeFirstFundamentalFormDerivative(
    const APositions &x) const;
  ES::M9x36d computeFirstFundamentalFormHessian(
    const APositions &x) const;
  ES::M2d computeSecondFundamentalForm(const BPositions &x) const;
  ES::M4x18d computeSecondFundamentalFormDerivative(
    const BPositions &x) const;
  ES::M18x72d computeSecondFundamentalFormHessian(
    const BPositions &x) const;

private:
  void prepareData(std::span<const double> x, std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    ShellDeformationElementCache &cacheDataBase) const;

  double computeEnergy(const ShellDeformationElementCache &cacheDataBase) const;

  void computeDisplacementGradient(const ShellDeformationElementCache &cacheDataBase,
    EigenSupport::RefVecXd grad) const;

  void computeDisplacementHessian(const ShellDeformationElementCache &cacheDataBase,
    EigenSupport::RefMatXd hess) const;

  void computeDisplacementPlasticHessian(const ShellDeformationElementCache &cacheDataBase,
    EigenSupport::RefMatXd hess, int materialLocation = -1) const;

  void computeDisplacementElasticHessian(const ShellDeformationElementCache &cacheDataBase,
    EigenSupport::RefMatXd hess, int materialLocation = -1) const;

  void computePlasticGradient(const ShellDeformationElementCache &cacheDataBase,
    EigenSupport::RefVecXd grad, int materialLocation = -1) const;

  void computeElasticGradient(const ShellDeformationElementCache &cacheDataBase,
    EigenSupport::RefVecXd grad, int materialLocation = -1) const;

  int computeVonMisesStress(
    const ShellDeformationElementCache &cacheData,
    std::span<double> stresses, int capacity) const;

public:
  int getNumElasticParameters() const override { return numElasticParams_; }
  int getNumPlasticParameters() const override { return numPlasticParams_; }

  int getNumVertices() const override { return numNodes; }
  int getNumDOFs() const override { return localDofs; }

private:
  struct FirstFundamentalFormResult
  {
    ES::M2d value = ES::M2d::Zero();
    ES::M4x9d derivative = ES::M4x9d::Zero();
    std::array<ES::M9d, 4> hessian{};
  };

  struct SecondFundamentalFormMatrixResult
  {
    ES::M2d value = ES::M2d::Zero();
    ES::M4x18d derivative = ES::M4x18d::Zero();
    std::array<ES::M18d, 4> hessian{};
  };

  struct SecondFundamentalFormEntriesResult
  {
    ES::V3d value = ES::V3d::Zero();
    ES::M3x18d derivative = ES::M3x18d::Zero();
    std::array<ES::M18d, 3> hessian{};
  };

  struct FaceNormalResult
  {
    ES::V3d value = ES::V3d::Zero();
    ES::M3x9d derivative = ES::M3x9d::Zero();
    std::array<ES::M9d, 3> hessian{};
  };

  FirstFundamentalFormResult computeFirstFundamentalFormImpl(
    const APositions &x, bool computeDerivative, bool computeHessian) const;
  SecondFundamentalFormMatrixResult computeSecondFundamentalFormImpl(
    const BPositions &x, bool computeDerivative, bool computeHessian) const;
  SecondFundamentalFormEntriesResult secondFundamentalFormEntries(
    const BPositions &x, bool computeDerivative, bool computeHessian) const;
  FaceNormalResult faceNormal(
    const ES::V3d &x0, const ES::V3d &x1, const ES::V3d &x2,
    bool computeDerivative, bool computeHessian) const;
  static ES::M3d crossMatrix(const Eigen::Vector3d &v);

  std::array<ES::V3d, 6> restPositions_;
  std::array<bool, 6> hasVertex_;
  ES::M2d restFirstFundamentalForm_;
  ES::M2d restSecondFundamentalForm_;
  double restArea_ = 0.0;

  std::unique_ptr<ElasticModel2DFundamentalForms> elastic2D_;
  std::unique_ptr<PlasticModel2DFundamentalForms> plastic2D_;
  bool projectHessianPSD_ = false;

  int numPlasticParams_ = 0;
  int numElasticParams_ = 0;
  mutable ShellDeformationElementCache cache_;

  const ShellDeformationElementCache &cacheData(const ShellDeformationElementCache &cacheDataBase) const;
  ShellDeformationElementCache &cacheData(ShellDeformationElementCache &cacheDataBase) const;
  double computeEnergy(const ShellDeformationElementCache &cacheData,
    std::span<const double> plasticParams, std::span<const double> elasticParams) const;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
