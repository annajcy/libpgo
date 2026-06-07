#pragma once

#include "peer.h"

#include "deformation/deformationModelState.h"
#include "deformation/deformationModelAssembler.h"
#include "energy/deformationModelEnergy.h"
#include "formulations/parameters/parameterField.h"
#include "energySet.h"
#include "../simulation/core.h"
#include "constraints/core.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// Energy peers that carry type-specific behaviour or metadata beyond the
// shared PyPotentialEnergy protocol.  Generic energies use PyOwnedPotentialEnergy
// (declared in peer.h); these are the typed peers.

class PyParameterField
{
public:
  explicit PyParameterField(std::shared_ptr<pgo::SolidDeformationModel::OptimizableField> field)
    : field_(std::move(field))
  {
    if (!field_) {
      throw std::invalid_argument("PyParameterField requires a non-null field.");
    }
  }

  std::shared_ptr<pgo::SolidDeformationModel::OptimizableField> field() const { return field_; }

  std::string domain() const
  {
    return field_->spec().domain == pgo::SolidDeformationModel::ParameterDomain::ELASTIC ? "elastic" : "plastic";
  }

  std::string model() const { return field_->spec().modelId; }
  int numChannels() const { return field_->numChannels(); }
  int numValueRows() const { return field_->numValueRows(); }

  // Compatibility alias for the number of stored parameter rows, not the mesh
  // element count. Elementwise fields store one row per element; constant fields
  // store one shared row.
  int numElements() const
  {
    return numValueRows();
  }

  nb::ndarray<nb::numpy, double> values() const
  {
    const auto *layout = field_->dofLayout();
    const int nc = field_->numChannels();
    const int n = layout ? layout->numGlobalDofs() : 0;
    const int rows = field_->numValueRows();
    const double *src = field_->globalData();
    auto data = (n == 0 || !src)
      ? new std::vector<double>()
      : new std::vector<double>(src, src + n);
    nb::capsule owner(data, [](void *p) noexcept {
      delete static_cast<std::vector<double> *>(p);
    });
    return nb::ndarray<nb::numpy, double>(
      data->data(),
      {static_cast<size_t>(rows), static_cast<size_t>(nc)},
      owner);
  }

  void setValues(nb::ndarray<nb::numpy, const double> values)
  {
    const auto *layout = field_->dofLayout();
    const int expected = layout ? layout->numGlobalDofs() : 0;
    auto vec = pgo::python::ndarrayToVectorXd(values);
    if (vec.size() != expected) {
      throw nb::value_error("ParameterField.set_values: values size does not match the field's global dof count.");
    }
    field_->setGlobalData(vec.data());
  }

private:
  std::shared_ptr<pgo::SolidDeformationModel::OptimizableField> field_;
};

class PyDeformationModelState
{
public:
  explicit PyDeformationModelState(
    std::shared_ptr<pgo::SolidDeformationModel::DeformationModelState> state)
    : state_(std::move(state))
  {
    if (!state_) {
      throw std::invalid_argument("PyDeformationModelState requires non-null state.");
    }
  }

  std::shared_ptr<pgo::SolidDeformationModel::DeformationModelState> state() const { return state_; }

  std::string elasticModel() const { return state_->elasticField().spec().modelId; }
  std::string plasticModel() const { return state_->plasticField().spec().modelId; }
  int numElements() const { return state_->mesh()->getNumElements(); }

  std::shared_ptr<PyParameterField> elasticField() const
  {
    return std::make_shared<PyParameterField>(state_->elasticFieldPtr());
  }

  std::shared_ptr<PyParameterField> plasticField() const
  {
    return std::make_shared<PyParameterField>(state_->plasticFieldPtr());
  }

  void setElasticValues(nb::ndarray<nb::numpy, const double> values)
  {
    state_->setElasticValues(pgo::python::ndarrayToVectorMapXd(values));
  }

  void setPlasticValues(nb::ndarray<nb::numpy, const double> values)
  {
    state_->setPlasticValues(pgo::python::ndarrayToVectorMapXd(values));
  }

private:
  std::shared_ptr<pgo::SolidDeformationModel::DeformationModelState> state_;
};

// Weighted sum of energy terms.  Inherits PyPotentialEnergy directly so
// _handle is the concrete PyEnergySet peer (evaluation inherited from base).
class PyEnergySet final : public PyPotentialEnergy
{
public:
  explicit PyEnergySet(std::shared_ptr<pgo::NonlinearOptimization::EnergySet> set)
    : set_(std::move(set))
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

// Deformation energy peer.  Inherits PyPotentialEnergy directly so _handle is
// the concrete peer; adds deformation-specific metadata (rest_position,
// plastic_gradient, etc.).  Owns the model state so the C++ mesh and parameter
// fields outlive the energy chain.
class PyDeformationEnergy : public PyPotentialEnergy
{
public:
  PyDeformationEnergy(std::shared_ptr<pgo::SolidDeformationModel::DeformationModelEnergy> energy,
    std::shared_ptr<PyDeformationModelState> stateOwner)
    : energy_(std::move(energy)), stateOwner_(std::move(stateOwner))
  {
  }

  std::shared_ptr<const NO::PotentialEnergy> potentialEnergyHandle() const override { return energy_; }
  std::shared_ptr<pgo::SolidDeformationModel::DeformationModelEnergy> energy() const { return energy_; }

  nb::ndarray<nb::numpy, double> restPosition() const;
  int numVertices() const { return static_cast<int>(energy_->getRestPosition().size() / 3); }
  int numPlasticDofs() const { return energy_->assembler().getNumPlasticGlobalParams(); }
  nb::ndarray<nb::numpy, double> plasticGradient(nb::ndarray<nb::numpy, const double> displacement) const;
  PySparseMatrix plasticHessian(nb::ndarray<nb::numpy, const double> displacement) const;
  PySparseMatrix plasticJacobian(nb::ndarray<nb::numpy, const double> displacement) const;

private:
  std::shared_ptr<pgo::SolidDeformationModel::DeformationModelEnergy> energy_;
  std::shared_ptr<PyDeformationModelState> stateOwner_;
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

std::shared_ptr<PyDeformationModelState> createDeformationModelState(
  std::shared_ptr<pgo::PySimulationMesh> meshCore,
  const std::string &elasticModel,
  nb::object elasticValues,
  const std::string &plasticModel,
  nb::object plasticValues,
  const std::string &elasticFieldType,
  const std::string &plasticFieldType);

int elasticNumChannels(
  const std::shared_ptr<pgo::PySimulationMesh> &meshCore,
  const std::string &elasticModel);

std::shared_ptr<PyDeformationEnergy> createDeformationEnergy(
  std::shared_ptr<PyDeformationModelState> stateCore,
  const std::string &formulationName,
  bool enforceSPD,
  bool enableMaterialMaxStep);

std::shared_ptr<PyPotentialEnergy> createPlasticMaterialEnergy(
  std::shared_ptr<PyDeformationModelState> stateCore,
  std::shared_ptr<PyDeformationEnergy> deformationEnergyCore,
  nb::ndarray<nb::numpy, const double> fixedDisplacement);
