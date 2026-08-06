#pragma once

#include "peer.h"

#include "deformation/deformationModelAssembler.h"
#include "energy/deformationEnergyOperator.h"
#include "energy/deformationPotentialEnergy.h"
#include "material/runtime/materialBinding.h"
#include "material/runtime/materialState.h"
#include "material/frame/materialFrames.h"
#include "../fem/elastic/core.h"
#include "../fem/plastic/core.h"
#include "energy/energySet.h"
#include "../simulation/core.h"
#include "constraints/core.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

// Energy peers that carry type-specific behaviour or metadata beyond the
// shared PyPotentialEnergy protocol.  Generic energies use PyOwnedPotentialEnergy
// (declared in peer.h); these are the typed peers.

namespace pgo
{
class PyFormulation;
}

class PyMaterialState
{
public:
  explicit PyMaterialState(
    pgo::SolidDeformationModel::MaterialState state):
    state_(std::move(state)) {}
  nb::ndarray<nb::numpy, double> elasticValues() const;
  nb::ndarray<nb::numpy, double> plasticValues() const;
  std::shared_ptr<PyMaterialState> withElasticValues(
    nb::ndarray<nb::numpy, const double> values) const;
  std::shared_ptr<PyMaterialState> withPlasticValues(
    nb::ndarray<nb::numpy, const double> values) const;
  const pgo::SolidDeformationModel::MaterialState &state() const { return state_; }

private:
  pgo::SolidDeformationModel::MaterialState state_;
};

class PyMaterialFrames
{
public:
  explicit PyMaterialFrames(
    pgo::SolidDeformationModel::MaterialFrames frames):
    frames_(std::move(frames))
  {
  }

  int numElements() const { return frames_.numElements(); }
  const pgo::SolidDeformationModel::MaterialFrames &frames() const
  {
    return frames_;
  }

private:
  pgo::SolidDeformationModel::MaterialFrames frames_;
};

class PyMaterialBinding
{
public:
  explicit PyMaterialBinding(
    std::shared_ptr<const pgo::SolidDeformationModel::MaterialBinding> binding):
    binding_(std::move(binding))
  {
  }

  const auto &binding() const { return binding_; }

private:
  std::shared_ptr<const pgo::SolidDeformationModel::MaterialBinding> binding_;
};

// Weighted sum of energy terms.  Inherits PyPotentialEnergy directly so
// _handle is the concrete PyEnergySet peer (evaluation inherited from base).
class PyEnergySet final : public PyPotentialEnergy
{
public:
  explicit PyEnergySet(std::shared_ptr<pgo::NonlinearOptimization::EnergySet> set): set_(std::move(set))
  {
  }

  std::shared_ptr<const NO::PotentialEnergy> potentialEnergyHandle() const override { return set_; }

  int numTerms() const { return set_->numTerms(); }

  nb::object term(int i) const
  {
    nb::dict info;
    info["weight"] = set_->term(i).weight;
    info["num_dofs"] = set_->term(i).energy->getNumDOFs();
    return info;
  }

  void setWeight(int i, double w) { set_->setWeight(i, w); }

  std::string repr() const
  {
    return "EnergySet(" + std::to_string(set_->numTerms()) + " terms, " +
      std::to_string(set_->getNumDOFs()) + " DOFs, state_kind='" +
      PyPotentialEnergy::stateKind() + "')";
  }

  std::shared_ptr<pgo::NonlinearOptimization::EnergySet> set_;
};

class PyDeformationEnergyOperator
{
public:
  explicit PyDeformationEnergyOperator(std::shared_ptr<pgo::SolidDeformationModel::DeformationEnergyOperator> energy): energy_(std::move(energy))
  {
  }

  std::shared_ptr<pgo::SolidDeformationModel::DeformationEnergyOperator> energy() const { return energy_; }

  nb::ndarray<nb::numpy, double> restState() const;
  nb::ndarray<nb::numpy, double> vertexRestPositions() const;
  int numVertices() const { return energy_->getNumVertices(); }
  int numElements() const { return energy_->getNumElements(); }
  int numElasticParams() const { return energy_->assembler().getNumElasticParams(); }
  int numPlasticParams() const { return energy_->assembler().getNumPlasticParams(); }
  int numElasticValues() const { return energy_->assembler().getNumElasticGlobalParams(); }
  int numPlasticValues() const { return energy_->assembler().getNumPlasticGlobalParams(); }
  int numDofs() const { return energy_->getNumDOFs(); }
  double value(nb::ndarray<nb::numpy, const double> displacement, const PyMaterialState &state) const;
  nb::ndarray<nb::numpy, double> gradient(nb::ndarray<nb::numpy, const double> displacement, const PyMaterialState &state) const;
  PySparseMatrix hessian(nb::ndarray<nb::numpy, const double> displacement, const PyMaterialState &state) const;
  nb::ndarray<nb::numpy, double> zeroState() const;
  nb::ndarray<nb::numpy, double> dE_de(nb::ndarray<nb::numpy, const double> displacement, const PyMaterialState &state) const;
  nb::ndarray<nb::numpy, double> elementVonMisesStresses(nb::ndarray<nb::numpy, const double> displacement, const PyMaterialState &state) const;
  nb::ndarray<nb::numpy, double> dE_dp(nb::ndarray<nb::numpy, const double> displacement, const PyMaterialState &state) const;
  nb::ndarray<nb::numpy, double> elasticMaterialVJP(
    nb::ndarray<nb::numpy, const double> displacement,
    const PyMaterialState &state,
    nb::ndarray<nb::numpy, const double> adjoint) const;
  nb::ndarray<nb::numpy, double> plasticMaterialVJP(
    nb::ndarray<nb::numpy, const double> displacement,
    const PyMaterialState &state,
    nb::ndarray<nb::numpy, const double> adjoint) const;

private:
  std::shared_ptr<pgo::SolidDeformationModel::DeformationEnergyOperator> energy_;
};

class PyDeformationPotentialEnergy final : public PyPotentialEnergy
{
public:
  PyDeformationPotentialEnergy(
    std::shared_ptr<PyDeformationEnergyOperator> energyOperator,
    std::shared_ptr<PyMaterialState> materialState);

  std::shared_ptr<const NO::PotentialEnergy> potentialEnergyHandle() const override
  {
    return energy_;
  }
  std::shared_ptr<PyDeformationEnergyOperator> energyOperator() const
  {
    return energyOperator_;
  }
  std::shared_ptr<PyMaterialState> materialState() const { return materialState_; }

private:
  std::shared_ptr<PyDeformationEnergyOperator> energyOperator_;
  std::shared_ptr<PyMaterialState> materialState_;
  std::shared_ptr<pgo::SolidDeformationModel::DeformationPotentialEnergy> energy_;
};

// ── Factories (implemented in energy/core.cpp) ────────────────────────────

std::shared_ptr<PyOwnedPotentialEnergy> createQuadraticEnergyForTest(
  int rows, int cols,
  const std::vector<int> &rowIndices,
  const std::vector<int> &colIndices,
  const std::vector<double> &values);

std::shared_ptr<PyOwnedPotentialEnergy> createLinearEnergy(nb::ndarray<nb::numpy, const double> b);

std::shared_ptr<PyOwnedPotentialEnergy> createConstraintPenalty(
  std::shared_ptr<PyConstraintFunctions> constraints, double weight);

std::shared_ptr<PyOwnedPotentialEnergy> createConstraintViolationPenalty(
  std::shared_ptr<PyConstraintFunctions> constraints,
  nb::ndarray<nb::numpy, const double> lower,
  nb::ndarray<nb::numpy, const double> upper,
  double weight);

std::shared_ptr<PyOwnedPotentialEnergy> createQuadraticEnergyFromSparse(const PySparseMatrix &A);

std::shared_ptr<PyOwnedPotentialEnergy> createQuadraticEnergyFromSparseWithB(
  const PySparseMatrix &A, nb::ndarray<nb::numpy, const double> b);

std::shared_ptr<PyOwnedPotentialEnergy> createQuadraticEnergyFromCOO(
  int rows, int cols,
  const std::vector<int> &rowIndices,
  const std::vector<int> &colIndices,
  nb::ndarray<nb::numpy, const double> values);

std::shared_ptr<PyOwnedPotentialEnergy> createQuadraticEnergyFromCOOWithB(
  int rows, int cols,
  const std::vector<int> &rowIndices,
  const std::vector<int> &colIndices,
  nb::ndarray<nb::numpy, const double> values,
  nb::ndarray<nb::numpy, const double> b);

std::shared_ptr<PyVertexAttachmentEnergy> createVertexAttachment(
  int numDofs, int rows, int cols,
  const std::vector<int> &kRowIndices,
  const std::vector<int> &kColIndices,
  const std::vector<double> &kValues,
  nb::ndarray<nb::numpy, const double> restPositions,
  nb::ndarray<nb::numpy, const std::int64_t> vertexIndicesArr,
  nb::ndarray<nb::numpy, const double> targetPositions,
  double coeff, bool isDisplacement);

std::shared_ptr<PyEnergySet> createEnergySet(nb::list terms);

std::shared_ptr<PyMaterialBinding> createMaterialBinding(
  const pgo::PyElasticModelDefinition &elasticDefinition,
  int numElements,
  nb::ndarray<nb::numpy, const double> elasticFixedValues,
  const pgo::PyPlasticModelDefinition &plasticDefinition,
  nb::ndarray<nb::numpy, const double> plasticFixedValues,
  std::shared_ptr<PyMaterialFrames> materialFrames);

std::shared_ptr<PyMaterialState> createMaterialState(
  nb::ndarray<nb::numpy, const double> elasticValues,
  nb::ndarray<nb::numpy, const double> plasticValues);

std::shared_ptr<PyDeformationEnergyOperator> createDeformationEnergyOperator(
  const pgo::PySimulationMesh &mesh,
  const PyMaterialBinding &materialBinding,
  const pgo::PyFormulation &formulation,
  nb::object elementWeights,
  bool projectHessianPSD);

std::shared_ptr<PyDeformationPotentialEnergy> createDeformationPotentialEnergy(
  std::shared_ptr<PyDeformationEnergyOperator> energyOperator,
  std::shared_ptr<PyMaterialState> materialState);

std::shared_ptr<PyMaterialFrames> makeMaterialFrames(
  const std::vector<double> &frameValues);
std::shared_ptr<PyMaterialFrames> makeMaterialFramesFromPrimaryAxes(
  const std::vector<double> &axisValues);
