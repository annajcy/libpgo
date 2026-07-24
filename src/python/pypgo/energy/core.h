#pragma once

#include "peer.h"

#include "deformation/deformationModelAssembler.h"
#include "energy/deformationModelEnergy.h"
#include "material/fields/materialParameterBuilder.h"
#include "../fem/elastic/core.h"
#include "../fem/plastic/core.h"
#include "energy/energySet.h"
#include "../simulation/core.h"
#include "constraints/core.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// Energy peers that carry type-specific behaviour or metadata beyond the
// shared PyPotentialEnergy protocol.  Generic energies use PyOwnedPotentialEnergy
// (declared in peer.h); these are the typed peers.

namespace pgo { class PyFormulation; }

class PyParameterDofLayout
{
public:
  using Creator = std::function<std::shared_ptr<const pgo::SolidDeformationModel::ParameterDofLayout>(int, int)>;
  explicit PyParameterDofLayout(Creator creator): creator_(std::move(creator)) {}
  std::shared_ptr<const pgo::SolidDeformationModel::ParameterDofLayout> create(int elements, int localDofs) const
  { return creator_(elements, localDofs); }

private:
  Creator creator_;
};

class PyMaterialChannelMapping
{
public:
  using Creator = std::function<std::shared_ptr<const pgo::SolidDeformationModel::MaterialChannelMapping>(int)>;
  explicit PyMaterialChannelMapping(Creator creator): creator_(std::move(creator)) {}
  std::shared_ptr<const pgo::SolidDeformationModel::MaterialChannelMapping> create(int channels) const
  { return creator_(channels); }

private:
  Creator creator_;
};

class PyMaterialParameterRef
{
public:
  PyMaterialParameterRef(
    std::shared_ptr<const pgo::SolidDeformationModel::MaterialParameterSpace> space,
    pgo::SolidDeformationModel::MaterialParameterRef ref):
    space_(std::move(space)), ref_(ref) {}

  std::string name() const { return std::string(ref_.name()); }
  int channel() const { return ref_.channel(); }
  const pgo::SolidDeformationModel::MaterialParameterRef &ref() const { return ref_; }
  std::shared_ptr<const pgo::SolidDeformationModel::MaterialParameterSpace> space() const
  {
    return space_;
  }

private:
  std::shared_ptr<const pgo::SolidDeformationModel::MaterialParameterSpace> space_;
  pgo::SolidDeformationModel::MaterialParameterRef ref_;
};

class PyMaterialParameterBlock
{
public:
  PyMaterialParameterBlock(
    std::shared_ptr<const pgo::SolidDeformationModel::MaterialParameterSpace> space,
    const pgo::SolidDeformationModel::MaterialParameterBlock *block):
    space_(std::move(space)), block_(block) {}

  const pgo::SolidDeformationModel::MaterialParameterBlock &block() const
  {
    return *block_;
  }
  int numChannels() const { return block().channelMapping().numChannels(); }
  int numLocalDofs() const { return block().dofLayout().numLocalDofs(); }
  int numGlobalDofs() const { return block().dofLayout().numGlobalDofs(); }
  int numValueRows() const { return block().dofLayout().numValueRows(); }
  std::vector<std::string> channelNames() const
  {
    const auto names = block().channelNames();
    return std::vector<std::string>(names.begin(), names.end());
  }
  std::shared_ptr<PyMaterialParameterRef> parameter(const std::string &name) const
  {
    return std::make_shared<PyMaterialParameterRef>(
      space_, block().parameter(name));
  }

private:
  std::shared_ptr<const pgo::SolidDeformationModel::MaterialParameterSpace> space_;
  const pgo::SolidDeformationModel::MaterialParameterBlock *block_ = nullptr;
};

class PyMaterialParameterSpace
{
public:
  explicit PyMaterialParameterSpace(
    std::shared_ptr<const pgo::SolidDeformationModel::MaterialParameterSpace> space):
    space_(std::move(space)) {}

  std::shared_ptr<PyMaterialParameterBlock> elastic() const
  {
    return std::make_shared<PyMaterialParameterBlock>(
      space_, &space_->elastic());
  }
  std::shared_ptr<PyMaterialParameterBlock> plastic() const
  {
    return std::make_shared<PyMaterialParameterBlock>(
      space_, &space_->plastic());
  }
  std::shared_ptr<const pgo::SolidDeformationModel::MaterialParameterSpace> space() const
  {
    return space_;
  }

private:
  std::shared_ptr<const pgo::SolidDeformationModel::MaterialParameterSpace> space_;
};

class PyMaterialParameters
{
public:
  explicit PyMaterialParameters(
    std::shared_ptr<pgo::SolidDeformationModel::MaterialParameters> parameters):
    parameters_(std::move(parameters)) {}

  std::shared_ptr<PyMaterialParameterSpace> space() const
  {
    return std::make_shared<PyMaterialParameterSpace>(parameters_->space());
  }
  nb::ndarray<nb::numpy, double> elasticValues() const;
  nb::ndarray<nb::numpy, double> plasticValues() const;
  void setElasticValues(nb::ndarray<nb::numpy, const double> values);
  void setPlasticValues(nb::ndarray<nb::numpy, const double> values);
  bool sameSpace(const PyMaterialParameters &other) const
  {
    return parameters_->space().get() == other.parameters_->space().get();
  }
  std::shared_ptr<pgo::SolidDeformationModel::MaterialParameters> parameters() const
  {
    return parameters_;
  }

private:
  std::shared_ptr<pgo::SolidDeformationModel::MaterialParameters> parameters_;
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
// the concrete peer; adds deformation-specific metadata
// (rest_state, vertex_rest_positions, dE_dp, parameter fields, etc.).
class PyDeformationEnergy : public PyPotentialEnergy
{
public:
  explicit PyDeformationEnergy(std::shared_ptr<pgo::SolidDeformationModel::DeformationModelEnergy> energy)
    : energy_(std::move(energy))
  {
  }

  std::shared_ptr<const NO::PotentialEnergy> potentialEnergyHandle() const override { return energy_; }
  std::shared_ptr<pgo::SolidDeformationModel::DeformationModelEnergy> energy() const { return energy_; }

  nb::ndarray<nb::numpy, double> restState() const;
  nb::ndarray<nb::numpy, double> vertexRestPositions() const;
  int numVertices() const { return energy_->assembler().getDeformationModelManager().getMesh()->getNumVertices(); }
  int numElasticParams() const { return energy_->assembler().getNumElasticParams(); }
  int numPlasticParams() const { return energy_->assembler().getNumPlasticParams(); }
  int numElasticDofs() const { return energy_->assembler().getNumElasticGlobalParams(); }
  int numPlasticDofs() const { return energy_->assembler().getNumPlasticGlobalParams(); }
  std::shared_ptr<pgo::PyElasticModelConfig> elasticModel() const
  {
    return std::make_shared<pgo::PyElasticModelConfig>(
      energy_->assembler().getDeformationModelManager().elasticModelConfig());
  }
  std::shared_ptr<pgo::PyPlasticModelConfig> plasticModel() const
  {
    return std::make_shared<pgo::PyPlasticModelConfig>(
      energy_->assembler().getDeformationModelManager().plasticModelConfig(),
      energy_->assembler().getNumPlasticParams());
  }
  std::shared_ptr<PyMaterialParameters> parameters() const
  {
    return std::make_shared<PyMaterialParameters>(energy_->materialParameters());
  }
  nb::ndarray<nb::numpy, double> dE_de(nb::ndarray<nb::numpy, const double> displacement) const;
  nb::ndarray<nb::numpy, double> elementVonMisesStresses(nb::ndarray<nb::numpy, const double> displacement) const;
  PySparseMatrix d2E_de2(nb::ndarray<nb::numpy, const double> displacement) const;
  PySparseMatrix d2E_dpde(nb::ndarray<nb::numpy, const double> displacement) const;
  nb::ndarray<nb::numpy, double> dE_dp(nb::ndarray<nb::numpy, const double> displacement) const;
  PySparseMatrix d2E_dp2(nb::ndarray<nb::numpy, const double> displacement) const;
  PySparseMatrix d2E_dude(nb::ndarray<nb::numpy, const double> displacement) const;
  PySparseMatrix d2E_dudp(nb::ndarray<nb::numpy, const double> displacement) const;

private:
  std::shared_ptr<pgo::SolidDeformationModel::DeformationModelEnergy> energy_;
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

std::shared_ptr<PyMaterialParameterSpace> createMaterialParameterSpace(
  std::shared_ptr<pgo::PySimulationMesh> meshCore,
  const pgo::PyElasticModelConfig &elasticModel,
  const PyParameterDofLayout &elasticLayout,
  const PyMaterialChannelMapping &elasticMapping,
  const pgo::PyPlasticModelConfig &plasticModel,
  const PyParameterDofLayout &plasticLayout,
  const PyMaterialChannelMapping &plasticMapping);

std::shared_ptr<PyMaterialParameters> createDefaultMaterialParameters(
  std::shared_ptr<pgo::PySimulationMesh> meshCore,
  const pgo::PyElasticModelConfig &elasticModel,
  const pgo::PyPlasticModelConfig &plasticModel);

std::shared_ptr<PyMaterialParameters> createMaterialParameters(
  std::shared_ptr<PyMaterialParameterSpace> space,
  nb::ndarray<nb::numpy, const double> elasticValues,
  nb::ndarray<nb::numpy, const double> plasticValues);

std::shared_ptr<PyDeformationEnergy> createDeformationEnergyWithParameters(
  std::shared_ptr<pgo::PySimulationMesh> meshCore,
  const pgo::PyElasticModelConfig &elasticModel,
  const pgo::PyPlasticModelConfig &plasticModel,
  const PyMaterialParameters &materialParameters,
  const pgo::PyFormulation &formulation,
  nb::object elementWeights,
  bool enforceSPD,
  bool enableMaterialMaxStep);

std::shared_ptr<PyParameterDofLayout> makeElementwiseParameterDofLayout();
std::shared_ptr<PyParameterDofLayout> makeConstantParameterDofLayout();
std::shared_ptr<PyMaterialChannelMapping> makeIdentityMaterialChannelMapping();

std::shared_ptr<PyPotentialEnergy> createPlasticMaterialEnergy(
  std::shared_ptr<PyDeformationEnergy> deformationEnergyCore,
  nb::ndarray<nb::numpy, const double> fixedDisplacement);

std::shared_ptr<PyPotentialEnergy> createElasticMaterialEnergy(
  std::shared_ptr<PyDeformationEnergy> deformationEnergyCore,
  nb::ndarray<nb::numpy, const double> fixedDisplacement);
