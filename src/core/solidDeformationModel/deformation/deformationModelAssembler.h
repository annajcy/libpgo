/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "deformation/deformationElement.h"
#include "formulations/dof/dofLayout.h"
#include "material/frame/materialFrames.h"
#include "material/runtime/materialState.h"
#include "EigenDef.h"

#include <memory>
#include <span>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

class ElasticModelDefinition;
class Formulation;
class MaterialBinding;
class PlasticModelDefinition;
class SimulationMesh;

class DeformationModelAssembler
{
public:
  struct MaterialMaxStepObservation
  {
    double alpha = 1.0;
    bool hasIllegalInitialState = false;
    int limitingElementId = -1;
    int limitingLocationId = -1;
  };

  DeformationModelAssembler(
    std::shared_ptr<const SimulationMesh> mesh,
    std::shared_ptr<const MaterialBinding> materialBinding,
    const Formulation &formulation,
    bool projectHessianPSD = true,
    std::span<const double> elementWeights = {});
  ~DeformationModelAssembler();

  MaterialMaxStepObservation computeMaxStepObservation(
    std::span<const double> x, std::span<const double> dx) const;
  double computeMaxStepSize(
    std::span<const double> x, std::span<const double> dx) const;

  double compute_E(std::span<const double> x, MaterialStateView state) const;
  void compute_dE_dx(std::span<const double> x, MaterialStateView state,
    EigenSupport::RefVecXd grad) const;
  void compute_d2E_dx2(std::span<const double> x, MaterialStateView state,
    EigenSupport::SpMatD &hess) const;

  void computePlasticMaterialVJP(
    std::span<const double> absolutePositions,
    std::span<const double> adjoint,
    MaterialStateView state, std::span<double> output) const;
  void computeElasticMaterialVJP(
    std::span<const double> absolutePositions,
    std::span<const double> adjoint,
    MaterialStateView state, std::span<double> output) const;
  void compute_dE_dp(std::span<const double> x, MaterialStateView state,
    EigenSupport::RefVecXd grad) const;
  void compute_dE_de(std::span<const double> x, MaterialStateView state,
    EigenSupport::RefVecXd grad) const;

  void computeVonMisesStresses(std::span<const double> x,
    MaterialStateView state, std::span<double> elementStresses) const;
  void computeMaxStrains(std::span<const double> x,
    MaterialStateView state, std::span<double> elementStrain) const;

  int getNumDOFs() const { return numDOFs; }
  int getNumElasticGlobalParams() const;
  int getNumPlasticGlobalParams() const;
  int getNumElasticParams() const { return numElasticParams_; }
  int getNumPlasticParams() const { return numPlasticParams_; }

  const SimulationMesh &mesh() const { return *mesh_; }
  const MaterialBinding &materialBinding() const { return *binding_; }
  std::shared_ptr<const ElasticModelDefinition> elasticModelDefinition() const;
  std::shared_ptr<const PlasticModelDefinition> plasticModelDefinition() const;
  const MaterialFrame &materialFrame(int elementId) const;
  const DeformationElement &element(int elementId) const;

  const DofLayout &getDofLayout() const { return *dofLayout; }
  const EigenSupport::VXd &getRestDofs() const { return restDofs_; }
  const EigenSupport::SpMatD &getHessianTemplate() const { return KTemplate; }

private:
  struct Element
  {
    std::unique_ptr<DeformationElement> deformation;
    std::vector<DofGroup> dofGroups;
    EigenSupport::VXd localPosition;
    EigenSupport::VXd localDirection;
    EigenSupport::VXd localGradient;
    EigenSupport::VXd localParameterGradient;
    std::vector<double> localMatrixData;
    std::vector<double> materialLocationValues;
    double energy = 0.0;

    Element(std::unique_ptr<DeformationElement> deformation,
      int localDofs, int maxMaterialLocations, int maxMaterialParams);
  };

  void initializeElements(
    const Formulation &formulation,
    DeformationElementConstructionOptions options);
  void gatherPosition(
    std::span<const double> x, Element &element) const;
  std::span<const double> elasticValues(
    int elementId, const MaterialStateView &state) const;
  std::span<const double> plasticValues(
    int elementId, const MaterialStateView &state) const;
  void assembleMaterialVJP(
    std::span<const double> absolutePositions,
    std::span<const double> adjoint,
    MaterialStateView state, bool elastic,
    std::span<double> output, const char *label) const;
  void validateMaterialState(const MaterialStateView &state) const;
  void validatePositionSpan(
    std::span<const double> x, const char *label) const;

  std::shared_ptr<const SimulationMesh> mesh_;
  std::shared_ptr<const MaterialBinding> binding_;
  std::shared_ptr<const DofLayout> dofLayout;
  EigenSupport::VXd restDofs_;
  mutable std::vector<Element> elements_;

  int numDOFs = 0;
  int nele = 0;
  int localDOFs = 0;
  int numElasticParams_ = 0;
  int numPlasticParams_ = 0;

  EigenSupport::SpMatD KTemplate;
  std::vector<std::vector<HessianBlockOffset>> elementKBlockOffsets;
  std::vector<double> elementWeights;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
