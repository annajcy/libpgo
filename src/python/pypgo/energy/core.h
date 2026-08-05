#pragma once

#include "peer.h"

#include "deformation/deformationModelAssembler.h"
#include "energy/deformationEnergyOperator.h"
#include "energy/deformationPotentialEnergy.h"
#include "material/runtime/materialAssignment.h"
#include "material/runtime/optimizableParameterRef.h"
#include "material/frame/materialFrameField.h"
#include "material/projection/materialInputProjection.h"
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

inline int parameterLayoutValueRows(
  const pgo::SolidDeformationModel::ParameterLayout &layout)
{
  if (layout.numLocalParameters() == 0)
    return 0;
  if (dynamic_cast<
        const pgo::SolidDeformationModel::ConstantParameterLayout *>(
        &layout))
    return 1;
  if (dynamic_cast<
        const pgo::SolidDeformationModel::ElementwiseParameterLayout *>(
        &layout))
    return layout.numElements();
  throw std::logic_error(
    "The parameter layout has no rectangular Python value shape.");
}

class PyParameterLayout
{
public:
  explicit PyParameterLayout(
    std::shared_ptr<const pgo::SolidDeformationModel::ParameterLayout> layout):
    layout_(std::move(layout))
  {
  }
  const std::shared_ptr<const pgo::SolidDeformationModel::ParameterLayout> &
  layout() const { return layout_; }
  int numElements() const { return layout_->numElements(); }
  int numLocalParameters() const { return layout_->numLocalParameters(); }
  int numGlobalParameters() const { return layout_->numGlobalParameters(); }
  int numValueRows() const { return parameterLayoutValueRows(*layout_); }
  std::string kind() const
  {
    if (dynamic_cast<
          const pgo::SolidDeformationModel::ConstantParameterLayout *>(
          layout_.get()))
      return "constant";
    if (dynamic_cast<
          const pgo::SolidDeformationModel::ElementwiseParameterLayout *>(
          layout_.get()))
      return "elementwise";
    return "custom";
  }

private:
  std::shared_ptr<const pgo::SolidDeformationModel::ParameterLayout> layout_;
};

class PyMaterialChannelMapping
{
public:
  explicit PyMaterialChannelMapping(
    std::shared_ptr<const pgo::SolidDeformationModel::MaterialChannelMapping> mapping):
    mapping_(std::move(mapping))
  {
  }
  const std::shared_ptr<const pgo::SolidDeformationModel::MaterialChannelMapping> &
  mapping() const { return mapping_; }
  int numParameters() const { return mapping_->numInputs(); }
  int numChannels() const { return mapping_->numChannels(); }
  std::string kind() const
  {
    if (dynamic_cast<
          const pgo::SolidDeformationModel::IdentityMaterialChannelMapping *>(
          mapping_.get()))
      return "identity";
    return "custom";
  }

private:
  std::shared_ptr<const pgo::SolidDeformationModel::MaterialChannelMapping> mapping_;
};

class PyDifferentiableMaterialChannelMapping : public PyMaterialChannelMapping
{
public:
  explicit PyDifferentiableMaterialChannelMapping(
    std::shared_ptr<
      const pgo::SolidDeformationModel::DifferentiableMaterialChannelMapping> mapping):
    PyMaterialChannelMapping(mapping),
    mapping_(std::move(mapping))
  {
  }

  const std::shared_ptr<
    const pgo::SolidDeformationModel::DifferentiableMaterialChannelMapping> &
  differentiableMapping() const { return mapping_; }

private:
  std::shared_ptr<
    const pgo::SolidDeformationModel::DifferentiableMaterialChannelMapping> mapping_;
};

class PyOptimizableParameterRef
{
public:
  explicit PyOptimizableParameterRef(pgo::SolidDeformationModel::OptimizableParameterRef ref):
    field_(ref.fieldHandle()), ref_(std::move(ref)) {}

  std::string name() const { return std::string(ref_.name()); }
  int parameterIndex() const { return ref_.parameterIndex(); }
  const pgo::SolidDeformationModel::OptimizableParameterRef &ref() const { return ref_; }
  std::shared_ptr<const pgo::SolidDeformationModel::OptimizableParameterField> fieldHandle() const { return field_; }

private:
  std::shared_ptr<const pgo::SolidDeformationModel::OptimizableParameterField> field_;
  pgo::SolidDeformationModel::OptimizableParameterRef ref_;
};

class PyOptimizableMaterialChannelRef
{
public:
  explicit PyOptimizableMaterialChannelRef(
    pgo::SolidDeformationModel::OptimizableMaterialChannelRef ref):
    field_(ref.fieldHandle()), ref_(std::move(ref)) {}

  std::string name() const { return std::string(ref_.name()); }
  int channelIndex() const { return ref_.channelIndex(); }
  std::shared_ptr<const pgo::SolidDeformationModel::OptimizableParameterField>
  fieldHandle() const { return field_; }

private:
  std::shared_ptr<const pgo::SolidDeformationModel::OptimizableParameterField>
    field_;
  pgo::SolidDeformationModel::OptimizableMaterialChannelRef ref_;
};

class PyOptimizableParameterField
{
public:
  explicit PyOptimizableParameterField(
    std::shared_ptr<const pgo::SolidDeformationModel::OptimizableParameterField> field):
    field_(std::move(field)) {}

  const pgo::SolidDeformationModel::OptimizableParameterField &field() const
  {
    return *field_;
  }
  int numMaterialChannels() const { return field().numMaterialChannels(); }
  int numLocalParameters() const { return field().layout().numLocalParameters(); }
  int numGlobalParameters() const { return field().layout().numGlobalParameters(); }
  int numValueRows() const
  {
    return parameterLayoutValueRows(field().layout());
  }
  std::vector<std::string> parameterNames() const
  {
    const auto names = field().inputSchema().parameterNames();
    return std::vector<std::string>(names.begin(), names.end());
  }
  std::shared_ptr<PyOptimizableParameterRef> parameter(const std::string &name) const
  {
    return std::make_shared<PyOptimizableParameterRef>(
      pgo::SolidDeformationModel::OptimizableParameterRef(field_, name));
  }
  std::shared_ptr<const pgo::SolidDeformationModel::OptimizableParameterField> fieldHandle() const
  {
    return field_;
  }
  std::shared_ptr<PyParameterLayout> layout() const
  {
    if (!layout_)
      layout_ = std::make_shared<PyParameterLayout>(field().layoutHandle());
    return layout_;
  }
  std::shared_ptr<PyDifferentiableMaterialChannelMapping> mapping() const
  {
    if (!mapping_)
      mapping_ = std::make_shared<PyDifferentiableMaterialChannelMapping>(
        field().mappingHandle());
    return mapping_;
  }

private:
  std::shared_ptr<const pgo::SolidDeformationModel::OptimizableParameterField> field_;
  mutable std::shared_ptr<PyParameterLayout> layout_;
  mutable std::shared_ptr<PyDifferentiableMaterialChannelMapping> mapping_;
};

class PyMaterialState
{
public:
  explicit PyMaterialState(
    pgo::SolidDeformationModel::MaterialState state):
    state_(std::move(state)) {}

  std::shared_ptr<PyOptimizableParameterField> elasticField() const
  {
    if (!elasticFieldWrapper_)
      elasticFieldWrapper_ = std::make_shared<PyOptimizableParameterField>(
        state_.elasticFieldHandle());
    return elasticFieldWrapper_;
  }
  std::shared_ptr<PyOptimizableParameterField> plasticField() const
  {
    if (!plasticFieldWrapper_)
      plasticFieldWrapper_ = std::make_shared<PyOptimizableParameterField>(
        state_.plasticFieldHandle());
    return plasticFieldWrapper_;
  }
  nb::ndarray<nb::numpy, double> elasticValues() const;
  nb::ndarray<nb::numpy, double> plasticValues() const;
  std::shared_ptr<PyMaterialState> withElasticValues(
    nb::ndarray<nb::numpy, const double> values) const;
  std::shared_ptr<PyMaterialState> withPlasticValues(
    nb::ndarray<nb::numpy, const double> values) const;
  bool sameParameterFields(const PyMaterialState &other) const
  {
    return state_.elasticField().sharesStateWith(other.state_.elasticField()) &&
      state_.plasticField().sharesStateWith(other.state_.plasticField());
  }
  const pgo::SolidDeformationModel::MaterialState &state() const { return state_; }

private:
  pgo::SolidDeformationModel::MaterialState state_;
  mutable std::shared_ptr<PyOptimizableParameterField> elasticFieldWrapper_;
  mutable std::shared_ptr<PyOptimizableParameterField> plasticFieldWrapper_;
};

class PyFixedParameterField
{
public:
  explicit PyFixedParameterField(
    std::shared_ptr<const pgo::SolidDeformationModel::FixedParameterField> field):
    field_(std::move(field)) {}
  std::vector<std::string> parameterNames() const
  {
    const auto names = field_->inputSchema().parameterNames();
    return std::vector<std::string>(names.begin(), names.end());
  }
  int numElements() const { return field_->numElements(); }
  int numLocalParameters() const { return field_->layout().numLocalParameters(); }
  int numGlobalParameters() const { return field_->layout().numGlobalParameters(); }
  int numValueRows() const
  {
    return parameterLayoutValueRows(field_->layout());
  }
  int numMaterialChannels() const { return field_->numMaterialChannels(); }
  std::shared_ptr<const pgo::SolidDeformationModel::FixedParameterField> field() const { return field_; }
  std::shared_ptr<PyParameterLayout> layout() const
  {
    if (!layout_)
      layout_ = std::make_shared<PyParameterLayout>(field_->layoutHandle());
    return layout_;
  }
  std::shared_ptr<PyMaterialChannelMapping> mapping() const
  {
    if (!mapping_)
      mapping_ = std::make_shared<PyMaterialChannelMapping>(
        field_->mappingHandle());
    return mapping_;
  }

private:
  std::shared_ptr<const pgo::SolidDeformationModel::FixedParameterField> field_;
  mutable std::shared_ptr<PyParameterLayout> layout_;
  mutable std::shared_ptr<PyMaterialChannelMapping> mapping_;
};

class PyMaterialAssignment
{
public:
  explicit PyMaterialAssignment(
    std::shared_ptr<const pgo::SolidDeformationModel::MaterialAssignment> assignment):
    assignment_(std::move(assignment)) {}
  std::shared_ptr<const pgo::SolidDeformationModel::MaterialAssignment> assignment() const { return assignment_; }
  std::shared_ptr<PyMaterialState> initialMaterialState() const
  {
    return std::make_shared<PyMaterialState>(assignment_->initialMaterialState());
  }

private:
  std::shared_ptr<const pgo::SolidDeformationModel::MaterialAssignment> assignment_;
};

class PyMaterialFrameField
{
public:
  explicit PyMaterialFrameField(
    std::shared_ptr<const pgo::SolidDeformationModel::MaterialFrameField> field):
    field_(std::move(field))
  {
  }

  int numElements() const { return field_->numElements(); }
  std::shared_ptr<const pgo::SolidDeformationModel::MaterialFrameField> field() const
  {
    return field_;
  }

private:
  std::shared_ptr<const pgo::SolidDeformationModel::MaterialFrameField> field_;
};

class PyElasticParameterization
{
public:
  explicit PyElasticParameterization(
    std::shared_ptr<const pgo::SolidDeformationModel::ElasticParameterization>
      parameterization):
    parameterization_(std::move(parameterization)) {}

  std::shared_ptr<pgo::PyElasticModelDefinition> definition() const
  {
    return std::make_shared<pgo::PyElasticModelDefinition>(
      parameterization_->definition());
  }
  std::shared_ptr<PyFixedParameterField> fixedField() const
  {
    return std::make_shared<PyFixedParameterField>(
      parameterization_->fixedField());
  }
  std::shared_ptr<PyOptimizableParameterField> optimizableField() const
  {
    return std::make_shared<PyOptimizableParameterField>(
      parameterization_->optimizableField());
  }
  std::vector<std::string> fixedChannelNames() const
  {
    const auto schema = parameterization_->fixedChannelSchema();
    const auto names = schema.channelNames();
    return std::vector<std::string>(names.begin(), names.end());
  }
  std::vector<std::string> optimizableChannelNames() const
  {
    const auto schema = parameterization_->optimizableChannelSchema();
    const auto names = schema.channelNames();
    return std::vector<std::string>(names.begin(), names.end());
  }
  std::shared_ptr<PyOptimizableMaterialChannelRef> optimizableChannel(
    const std::string &name) const
  {
    return std::make_shared<PyOptimizableMaterialChannelRef>(
      pgo::SolidDeformationModel::OptimizableMaterialChannelRef(
        *parameterization_, name));
  }
  int numElements() const { return parameterization_->fixedField()->numElements(); }
  std::shared_ptr<const pgo::SolidDeformationModel::ElasticParameterization>
  parameterization() const { return parameterization_; }

private:
  std::shared_ptr<const pgo::SolidDeformationModel::ElasticParameterization>
    parameterization_;
};

class PyPlasticParameterization
{
public:
  explicit PyPlasticParameterization(
    std::shared_ptr<const pgo::SolidDeformationModel::PlasticParameterization>
      parameterization,
    int dofs):
    parameterization_(std::move(parameterization)), dofs_(dofs) {}

  std::shared_ptr<pgo::PyPlasticModelDefinition> definition() const
  {
    return std::make_shared<pgo::PyPlasticModelDefinition>(
      parameterization_->definition(), dofs_);
  }
  std::shared_ptr<PyFixedParameterField> fixedField() const
  {
    return std::make_shared<PyFixedParameterField>(
      parameterization_->fixedField());
  }
  std::shared_ptr<PyOptimizableParameterField> optimizableField() const
  {
    return std::make_shared<PyOptimizableParameterField>(
      parameterization_->optimizableField());
  }
  std::vector<std::string> fixedChannelNames() const
  {
    const auto schema = parameterization_->fixedChannelSchema();
    const auto names = schema.channelNames();
    return std::vector<std::string>(names.begin(), names.end());
  }
  std::vector<std::string> optimizableChannelNames() const
  {
    const auto schema = parameterization_->optimizableChannelSchema();
    const auto names = schema.channelNames();
    return std::vector<std::string>(names.begin(), names.end());
  }
  std::shared_ptr<PyOptimizableMaterialChannelRef> optimizableChannel(
    const std::string &name) const
  {
    return std::make_shared<PyOptimizableMaterialChannelRef>(
      pgo::SolidDeformationModel::OptimizableMaterialChannelRef(
        *parameterization_, name));
  }
  int numElements() const { return parameterization_->fixedField()->numElements(); }
  std::shared_ptr<const pgo::SolidDeformationModel::PlasticParameterization>
  parameterization() const { return parameterization_; }
  int dofs() const { return dofs_; }

private:
  std::shared_ptr<const pgo::SolidDeformationModel::PlasticParameterization>
    parameterization_;
  int dofs_;
};

class PyMaterialParameterization
{
public:
  PyMaterialParameterization(
    std::shared_ptr<const pgo::SolidDeformationModel::MaterialParameterization>
      parameterization,
    std::shared_ptr<PyElasticParameterization> elastic,
    std::shared_ptr<PyPlasticParameterization> plastic):
    parameterization_(std::move(parameterization)),
    elastic_(std::move(elastic)), plastic_(std::move(plastic)) {}
  std::shared_ptr<const pgo::SolidDeformationModel::MaterialParameterization> parameterization() const
  {
    return parameterization_;
  }
  std::shared_ptr<PyElasticParameterization> elastic() const { return elastic_; }
  std::shared_ptr<PyPlasticParameterization> plastic() const { return plastic_; }
  int numElements() const { return parameterization_->numElements(); }

private:
  std::shared_ptr<const pgo::SolidDeformationModel::MaterialParameterization> parameterization_;
  std::shared_ptr<PyElasticParameterization> elastic_;
  std::shared_ptr<PyPlasticParameterization> plastic_;
};

class PyMaterialParameterData
{
public:
  PyMaterialParameterData() = default;
  explicit PyMaterialParameterData(
    std::shared_ptr<const pgo::SolidDeformationModel::MaterialParameterData> data):
    data_(std::move(data)) {}
  const std::shared_ptr<const pgo::SolidDeformationModel::MaterialParameterData> &data() const
  {
    return data_;
  }
  nb::ndarray<nb::numpy, double> elasticFixedValues() const;
  nb::ndarray<nb::numpy, double> elasticInitialOptimizableValues() const;
  nb::ndarray<nb::numpy, double> plasticFixedValues() const;
  nb::ndarray<nb::numpy, double> plasticInitialOptimizableValues() const;

private:
  std::shared_ptr<const pgo::SolidDeformationModel::MaterialParameterData> data_;
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
  int numVertices() const { return energy_->assembler().getDeformationModelManager().getMesh().getNumVertices(); }
  int numElasticParams() const { return energy_->assembler().getNumElasticParams(); }
  int numPlasticParams() const { return energy_->assembler().getNumPlasticParams(); }
  int numElasticDofs() const { return energy_->assembler().getNumElasticGlobalParams(); }
  int numPlasticDofs() const { return energy_->assembler().getNumPlasticGlobalParams(); }
  std::shared_ptr<pgo::PyElasticModelDefinition> elasticDefinition() const
  {
    return std::make_shared<pgo::PyElasticModelDefinition>(
      energy_->assembler().getDeformationModelManager().elasticModelDefinition());
  }
  std::shared_ptr<pgo::PyPlasticModelDefinition> plasticDefinition() const
  {
    return std::make_shared<pgo::PyPlasticModelDefinition>(
      energy_->assembler().getDeformationModelManager().plasticModelDefinition(),
      energy_->assembler().getNumPlasticParams());
  }
  int numDofs() const { return energy_->getNumDOFs(); }
  double value(nb::ndarray<nb::numpy, const double> displacement, const PyMaterialState &state) const;
  nb::ndarray<nb::numpy, double> gradient(nb::ndarray<nb::numpy, const double> displacement, const PyMaterialState &state) const;
  PySparseMatrix hessian(nb::ndarray<nb::numpy, const double> displacement, const PyMaterialState &state) const;
  nb::ndarray<nb::numpy, double> zeroState() const;
  nb::ndarray<nb::numpy, double> dE_de(nb::ndarray<nb::numpy, const double> displacement, const PyMaterialState &state) const;
  nb::ndarray<nb::numpy, double> elementVonMisesStresses(nb::ndarray<nb::numpy, const double> displacement, const PyMaterialState &state) const;
  PySparseMatrix d2E_de2(nb::ndarray<nb::numpy, const double> displacement, const PyMaterialState &state) const;
  PySparseMatrix d2E_dpde(nb::ndarray<nb::numpy, const double> displacement, const PyMaterialState &state) const;
  nb::ndarray<nb::numpy, double> dE_dp(nb::ndarray<nb::numpy, const double> displacement, const PyMaterialState &state) const;
  PySparseMatrix d2E_dp2(nb::ndarray<nb::numpy, const double> displacement, const PyMaterialState &state) const;
  PySparseMatrix d2E_dude(nb::ndarray<nb::numpy, const double> displacement, const PyMaterialState &state) const;
  PySparseMatrix d2E_dudp(nb::ndarray<nb::numpy, const double> displacement, const PyMaterialState &state) const;
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

std::shared_ptr<PyOptimizableParameterField> createOptimizableParameterField(
  const std::vector<std::string> &parameterNames,
  const PyParameterLayout &layout,
  const PyDifferentiableMaterialChannelMapping &mapping);

std::shared_ptr<PyFixedParameterField> createFixedParameterField(
  const std::vector<std::string> &parameterNames,
  const PyParameterLayout &layout,
  const PyMaterialChannelMapping &mapping);

std::shared_ptr<PyElasticParameterization> createElasticParameterization(
  const pgo::PyElasticModelDefinition &elasticDefinition,
  const PyFixedParameterField &elasticFixed,
  const PyOptimizableParameterField &elasticOptimizable);

std::shared_ptr<PyPlasticParameterization> createPlasticParameterization(
  const pgo::PyPlasticModelDefinition &plasticDefinition,
  const PyFixedParameterField &plasticFixed,
  const PyOptimizableParameterField &plasticOptimizable);

std::shared_ptr<PyMaterialParameterization> createMaterialParameterization(
  const PyElasticParameterization &elastic,
  const PyPlasticParameterization &plastic);

std::shared_ptr<PyMaterialParameterData> createMaterialParameterData(
  nb::ndarray<nb::numpy, const double> elasticFixedValues,
  nb::ndarray<nb::numpy, const double> elasticInitialOptimizableValues,
  nb::ndarray<nb::numpy, const double> plasticFixedValues,
  nb::ndarray<nb::numpy, const double> plasticInitialOptimizableValues);
nb::ndarray<nb::numpy, double> projectImportedMaterialInputs(
  const pgo::PyImportedMaterialCatalog &source,
  const std::vector<std::string> &parameterNames,
  const PyParameterLayout &layout);
nb::ndarray<nb::numpy, double> projectNamedMaterialInputs(
  const pgo::PyNamedMaterialInputData &source,
  const std::vector<std::string> &parameterNames,
  const PyParameterLayout &layout);
void validateMaterialParameterData(
  const PyMaterialParameterization &parameterization,
  const PyMaterialParameterData &data);

std::shared_ptr<PyMaterialAssignment> createMaterialAssignmentFromParameterization(
  const pgo::PySimulationMesh &mesh,
  const PyMaterialParameterization &parameterization,
  const PyMaterialParameterData &data,
  const PyMaterialFrameField &materialFrames);

std::shared_ptr<PyMaterialState> createMaterialState(
  const PyMaterialAssignment &assignment,
  nb::ndarray<nb::numpy, const double> elasticValues,
  nb::ndarray<nb::numpy, const double> plasticValues);

std::shared_ptr<PyDeformationEnergyOperator> createDeformationEnergyOperator(
  const PyMaterialAssignment &assignment,
  const pgo::PyFormulation &formulation,
  nb::object elementWeights,
  bool projectHessianPSD,
  bool enableMaterialMaxStep);

std::shared_ptr<PyDeformationPotentialEnergy> createDeformationPotentialEnergy(
  std::shared_ptr<PyDeformationEnergyOperator> energyOperator,
  std::shared_ptr<PyMaterialState> materialState);

std::shared_ptr<PyParameterLayout> makeElementwiseParameterLayout(
  int numElements, int numLocalParameters);
std::shared_ptr<PyParameterLayout> makeConstantParameterLayout(
  int numElements, int numLocalParameters);
std::shared_ptr<PyDifferentiableMaterialChannelMapping>
makeIdentityMaterialChannelMapping(int numParameters);

std::shared_ptr<PyMaterialFrameField> makeGlobalAxesMaterialFrameField(
  int numElements);
std::shared_ptr<PyMaterialFrameField> makeConstantMaterialFrameField(
  int numElements,
  const std::vector<double> &frameValues);
std::shared_ptr<PyMaterialFrameField> makeElementwiseMaterialFrameField(
  const std::vector<double> &frameValues);
std::shared_ptr<PyMaterialFrameField> makeMaterialFramesFromPrimaryAxes(
  const std::vector<double> &axisValues);
std::shared_ptr<PyMaterialFrameField> projectImportedMaterialFrameField(
  const pgo::PyImportedMaterialCatalog &source,
  const std::string &property);
