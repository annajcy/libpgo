#include "core.h"

#include "energy/deformationEnergyBuilder.h"
#include "deformation/deformationModel.h"
#include "deformation/deformationModelManager.h"
#include "../fem/formulation/core.h"
#include "energy/elasticMaterialEnergy.h"
#include "energy/plasticMaterialEnergy.h"
#include "constraints/core.h"
#include "constraints/potentialEnergyFromConstraintFunctions.h"
#include "linearPotentialEnergy.h"
#include "quadraticPotentialEnergy.h"
#include "multiVertexPullingSoftConstraints.h"
#include "eigen_numpy.h"
#include "EigenDef.h"
#include "energy/potentialEnergy.h"
#include "simulation/simulationMesh.h"
#include "../sparse/core.h"

#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <algorithm>
#include <stdexcept>
#include <tuple>
#include <utility>

using namespace pgo;

namespace {

std::optional<EigenSupport::VXd> optionalVectorFromObject(const nb::object &values)
{
  if (values.is_none()) {
    return std::nullopt;
  }
  auto arr = nb::cast<nb::ndarray<nb::numpy, const double>>(values);
  return python::ndarrayToVectorXd(arr);
}

std::shared_ptr<const SolidDeformationModel::ParameterDofLayout> makeLayout(
  const PyParameterDofLayout &layout,
  int numElements,
  int numLocalDofs)
{
  using namespace SolidDeformationModel;
  return layout.create(numElements, numLocalDofs);
}

std::shared_ptr<const SolidDeformationModel::MaterialChannelMapping> makeMapping(
  const PyMaterialChannelMapping &mapping,
  int numChannels)
{
  return mapping.create(numChannels);
}

}  // namespace

namespace
{
nb::ndarray<nb::numpy, double> materialValuesArray(
  EigenSupport::VXd values,
  const SolidDeformationModel::ParameterDofLayout &layout)
{
  auto data = new std::vector<double>(values.size());
  if (values.size() > 0)
    std::copy(values.data(), values.data() + values.size(), data->data());
  nb::capsule owner(data, [](void *p) noexcept {
    delete static_cast<std::vector<double> *>(p);
  });
  return nb::ndarray<nb::numpy, double>(
    data->data(),
    {static_cast<std::size_t>(layout.numValueRows()),
      static_cast<std::size_t>(layout.numLocalDofs())},
    owner);
}
}  // namespace

nb::ndarray<nb::numpy, double> PyMaterialParameters::elasticValues() const
{
  return materialValuesArray(
    parameters_->elasticSnapshot(),
    parameters_->space()->elastic().dofLayout());
}

nb::ndarray<nb::numpy, double> PyMaterialParameters::plasticValues() const
{
  return materialValuesArray(
    parameters_->plasticSnapshot(),
    parameters_->space()->plastic().dofLayout());
}

void PyMaterialParameters::setElasticValues(
  nb::ndarray<nb::numpy, const double> values)
{
  parameters_->setElasticValues(python::ndarrayToVectorXd(values));
}

void PyMaterialParameters::setPlasticValues(
  nb::ndarray<nb::numpy, const double> values)
{
  parameters_->setPlasticValues(python::ndarrayToVectorXd(values));
}

// ── PyDeformationEnergy out-of-line methods ───────────────────────────────

nb::ndarray<nb::numpy, double> PyDeformationEnergy::restState() const
{
  return python::vectorXdToNdarray(EigenSupport::VXd(energy_->getRestDofs()));
}

nb::ndarray<nb::numpy, double> PyDeformationEnergy::vertexRestPositions() const
{
  const auto &positions = energy_->getVertexRestPositions();
  const int nv = numVertices();
  EigenSupport::MXd result(nv, 3);
  for (int vi = 0; vi < nv; vi++)
    result.row(vi) = positions.segment<3>(vi * 3).transpose();
  return python::matrixXdToNdarray(std::move(result));
}

nb::ndarray<nb::numpy, double> PyDeformationEnergy::dE_dp(
  nb::ndarray<nb::numpy, const double> displacement) const
{
  auto u = python::ndarrayToVectorMapXd(displacement);
  if (u.size() != energy_->getRestDofs().size()) {
    throw nb::value_error("displacement size must match deformation energy num_dofs.");
  }

  EigenSupport::VXd grad = EigenSupport::VXd::Zero(numPlasticDofs());
  {
    nb::gil_scoped_release release;
    energy_->compute_dE_dp(u, grad);
  }
  return python::vectorXdToNdarray(std::move(grad));
}

nb::ndarray<nb::numpy, double> PyDeformationEnergy::dE_de(
  nb::ndarray<nb::numpy, const double> displacement) const
{
  auto u = python::ndarrayToVectorMapXd(displacement);
  if (u.size() != energy_->getRestDofs().size()) {
    throw nb::value_error("displacement size must match deformation energy num_dofs.");
  }

  EigenSupport::VXd grad = EigenSupport::VXd::Zero(numElasticDofs());
  {
    nb::gil_scoped_release release;
    energy_->compute_dE_de(u, grad);
  }
  return python::vectorXdToNdarray(std::move(grad));
}

nb::ndarray<nb::numpy, double> PyDeformationEnergy::elementVonMisesStresses(
  nb::ndarray<nb::numpy, const double> displacement) const
{
  auto u = python::ndarrayToVectorMapXd(displacement);
  if (u.size() != energy_->getRestDofs().size()) {
    throw nb::value_error("displacement size must match deformation energy num_dofs.");
  }

  const int nele = energy_->assembler().getDeformationModelManager().getMesh()->getNumElements();
  EigenSupport::VXd out = EigenSupport::VXd::Zero(nele);
  try {
    nb::gil_scoped_release release;
    energy_->computeVonMisesStresses(u, out);
  }
  catch (const SolidDeformationModel::UnsupportedDeformationDiagnosticError &e) {
    PyErr_SetString(PyExc_NotImplementedError, e.what());
    throw nb::python_error();
  }
  return python::vectorXdToNdarray(std::move(out));
}

PySparseMatrix PyDeformationEnergy::d2E_de2(
  nb::ndarray<nb::numpy, const double> displacement) const
{
  auto u = python::ndarrayToVectorMapXd(displacement);
  if (u.size() != energy_->getRestDofs().size()) {
    throw nb::value_error("displacement size must match deformation energy num_dofs.");
  }

  EigenSupport::SpMatD hess =
    energy_->assembler().d2E_de2_template();
  {
    nb::gil_scoped_release release;
    energy_->compute_d2E_de2(u, hess);
  }
  return PySparseMatrix(std::move(hess));
}

PySparseMatrix PyDeformationEnergy::d2E_dpde(
  nb::ndarray<nb::numpy, const double> displacement) const
{
  auto u = python::ndarrayToVectorMapXd(displacement);
  if (u.size() != energy_->getRestDofs().size()) {
    throw nb::value_error("displacement size must match deformation energy num_dofs.");
  }

  EigenSupport::SpMatD hess =
    energy_->assembler().d2E_dpde_template();
  {
    nb::gil_scoped_release release;
    energy_->compute_d2E_dpde(u, hess);
  }
  return PySparseMatrix(std::move(hess));
}

PySparseMatrix PyDeformationEnergy::d2E_dp2(
  nb::ndarray<nb::numpy, const double> displacement) const
{
  auto u = python::ndarrayToVectorMapXd(displacement);
  if (u.size() != energy_->getRestDofs().size()) {
    throw nb::value_error("displacement size must match deformation energy num_dofs.");
  }

  EigenSupport::SpMatD hess =
    energy_->assembler().d2E_dp2_template();
  {
    nb::gil_scoped_release release;
    energy_->compute_d2E_dp2(u, hess);
  }
  return PySparseMatrix(std::move(hess));
}

PySparseMatrix PyDeformationEnergy::d2E_dude(
  nb::ndarray<nb::numpy, const double> displacement) const
{
  auto u = python::ndarrayToVectorMapXd(displacement);
  if (u.size() != energy_->getRestDofs().size()) {
    throw nb::value_error("displacement size must match deformation energy num_dofs.");
  }

  EigenSupport::SpMatD mixedHessian =
    energy_->assembler().d2E_dude_template();
  {
    nb::gil_scoped_release release;
    energy_->compute_d2E_dude(u, mixedHessian);
  }
  return PySparseMatrix(std::move(mixedHessian));
}

PySparseMatrix PyDeformationEnergy::d2E_dudp(
  nb::ndarray<nb::numpy, const double> displacement) const
{
  auto u = python::ndarrayToVectorMapXd(displacement);
  if (u.size() != energy_->getRestDofs().size()) {
    throw nb::value_error("displacement size must match deformation energy num_dofs.");
  }

  EigenSupport::SpMatD mixedHessian =
    energy_->assembler().d2E_dudp_template();
  {
    nb::gil_scoped_release release;
    energy_->compute_d2E_dudp(u, mixedHessian);
  }
  return PySparseMatrix(std::move(mixedHessian));
}

// ── Factories ─────────────────────────────────────────────────────────────

std::shared_ptr<PyOwnedPotentialEnergy> createQuadraticEnergyForTest(
  int rows, int cols,
  const std::vector<int> &rowIndices,
  const std::vector<int> &colIndices,
  const std::vector<double> &values)
{
  std::vector<EigenSupport::TripletD> triplets;
  triplets.reserve(values.size());
  for (size_t i = 0; i < values.size(); ++i) {
    triplets.emplace_back(rowIndices[i], colIndices[i], values[i]);
  }
  EigenSupport::SpMatD A(rows, cols);
  A.setFromTriplets(triplets.begin(), triplets.end());

  auto energy = std::make_shared<PredefinedPotentialEnergies::QuadraticPotentialEnergy>(std::move(A));
  return std::make_shared<PyOwnedPotentialEnergy>(std::move(energy));
}

std::shared_ptr<PyOwnedPotentialEnergy> createLinearEnergy(nb::ndarray<nb::numpy, const double> b)
{
  auto bVec = python::ndarrayToVectorXd(b);
  auto energy = std::make_shared<PredefinedPotentialEnergies::LinearPotentialEnergy>(std::move(bVec));
  return std::make_shared<PyOwnedPotentialEnergy>(std::move(energy));
}

std::shared_ptr<PyOwnedPotentialEnergy> createConstraintPenalty(
  std::shared_ptr<PyConstraintFunctions> constraints,
  double weight)
{
  if (!constraints) {
    throw nb::value_error("constraints must be non-null");
  }
  auto penalty = std::make_shared<NonlinearOptimization::PotentialEnergyConstraintFunctions>(
    constraints->numDofs(), constraints->handle_);
  std::vector<NonlinearOptimization::EnergySet::Term> terms;
  terms.push_back({ std::move(penalty), weight });
  auto weighted = std::make_shared<NonlinearOptimization::EnergySet>(constraints->numDofs(), std::move(terms));
  return std::make_shared<PyOwnedPotentialEnergy>(std::move(weighted));
}

std::shared_ptr<PyOwnedPotentialEnergy> createConstraintViolationPenalty(
  std::shared_ptr<PyConstraintFunctions> constraints,
  nb::ndarray<nb::numpy, const double> lower,
  nb::ndarray<nb::numpy, const double> upper,
  double weight)
{
  if (!constraints) {
    throw nb::value_error("constraints must be non-null");
  }
  auto lowerVec = python::ndarrayToVectorXd(lower);
  auto upperVec = python::ndarrayToVectorXd(upper);
  auto penalty = std::make_shared<NonlinearOptimization::PotentialEnergyBoundedConstraintFunctions>(
    constraints->numDofs(), constraints->handle_, std::move(lowerVec), std::move(upperVec));
  std::vector<NonlinearOptimization::EnergySet::Term> terms;
  terms.push_back({ std::move(penalty), weight });
  auto weighted = std::make_shared<NonlinearOptimization::EnergySet>(constraints->numDofs(), std::move(terms));
  return std::make_shared<PyOwnedPotentialEnergy>(std::move(weighted));
}

std::shared_ptr<PyOwnedPotentialEnergy> createQuadraticEnergyFromSparse(const PySparseMatrix &A)
{
  auto coo = A.toCOO();
  const auto &rowIndices = std::get<0>(coo);
  const auto &colIndices = std::get<1>(coo);
  const auto &values = std::get<2>(coo);
  int rows = A.rows();
  int cols = A.cols();

  std::vector<EigenSupport::TripletD> triplets;
  triplets.reserve(values.size());
  for (size_t i = 0; i < values.size(); ++i) {
    triplets.emplace_back(rowIndices[i], colIndices[i], values[i]);
  }
  EigenSupport::SpMatD A_eigen(rows, cols);
  A_eigen.setFromTriplets(triplets.begin(), triplets.end());

  auto energy = std::make_shared<PredefinedPotentialEnergies::QuadraticPotentialEnergy>(std::move(A_eigen));
  return std::make_shared<PyOwnedPotentialEnergy>(std::move(energy));
}

std::shared_ptr<PyOwnedPotentialEnergy> createQuadraticEnergyFromSparseWithB(
  const PySparseMatrix &A,
  nb::ndarray<nb::numpy, const double> b)
{
  auto coo = A.toCOO();
  const auto &rowIndices = std::get<0>(coo);
  const auto &colIndices = std::get<1>(coo);
  const auto &values = std::get<2>(coo);
  int rows = A.rows();
  int cols = A.cols();

  std::vector<EigenSupport::TripletD> triplets;
  triplets.reserve(values.size());
  for (size_t i = 0; i < values.size(); ++i) {
    triplets.emplace_back(rowIndices[i], colIndices[i], values[i]);
  }
  EigenSupport::SpMatD A_eigen(rows, cols);
  A_eigen.setFromTriplets(triplets.begin(), triplets.end());

  auto bVec = python::ndarrayToVectorXd(b);
  auto energy = std::make_shared<PredefinedPotentialEnergies::QuadraticPotentialEnergy>(
    std::move(A_eigen), std::move(bVec));
  return std::make_shared<PyOwnedPotentialEnergy>(std::move(energy));
}

std::shared_ptr<PyOwnedPotentialEnergy> createQuadraticEnergyFromCOO(
  int rows, int cols,
  const std::vector<int> &rowIndices,
  const std::vector<int> &colIndices,
  nb::ndarray<nb::numpy, const double> values)
{
  python::requireFloat64(values);
  if (values.ndim() != 1) {
    throw nb::value_error("values must be 1-D");
  }
  if (rowIndices.size() != colIndices.size() ||
      rowIndices.size() != values.shape(0)) {
    throw nb::value_error("row_indices, col_indices, values must have same length");
  }

  std::vector<EigenSupport::TripletD> triplets;
  triplets.reserve(rowIndices.size());
  for (size_t i = 0; i < rowIndices.size(); ++i) {
    double v = values.data()[static_cast<int64_t>(i)];
    triplets.emplace_back(rowIndices[i], colIndices[i], v);
  }
  EigenSupport::SpMatD A(rows, cols);
  A.setFromTriplets(triplets.begin(), triplets.end());

  auto energy = std::make_shared<PredefinedPotentialEnergies::QuadraticPotentialEnergy>(std::move(A));
  return std::make_shared<PyOwnedPotentialEnergy>(std::move(energy));
}

std::shared_ptr<PyOwnedPotentialEnergy> createQuadraticEnergyFromCOOWithB(
  int rows, int cols,
  const std::vector<int> &rowIndices,
  const std::vector<int> &colIndices,
  nb::ndarray<nb::numpy, const double> values,
  nb::ndarray<nb::numpy, const double> b)
{
  python::requireFloat64(values);
  if (values.ndim() != 1) {
    throw nb::value_error("values must be 1-D");
  }
  if (rowIndices.size() != colIndices.size() ||
      rowIndices.size() != values.shape(0)) {
    throw nb::value_error("row_indices, col_indices, values must have same length");
  }

  std::vector<EigenSupport::TripletD> triplets;
  triplets.reserve(rowIndices.size());
  for (size_t i = 0; i < rowIndices.size(); ++i) {
    double v = values.data()[static_cast<int64_t>(i)];
    triplets.emplace_back(rowIndices[i], colIndices[i], v);
  }
  EigenSupport::SpMatD A(rows, cols);
  A.setFromTriplets(triplets.begin(), triplets.end());

  auto bVec = python::ndarrayToVectorXd(b);
  auto energy = std::make_shared<PredefinedPotentialEnergies::QuadraticPotentialEnergy>(
    std::move(A), std::move(bVec));
  return std::make_shared<PyOwnedPotentialEnergy>(std::move(energy));
}

std::shared_ptr<PyVertexAttachmentEnergy> createVertexAttachment(
  int numDofs, int rows, int cols,
  const std::vector<int> &kRowIndices,
  const std::vector<int> &kColIndices,
  const std::vector<double> &kValues,
  nb::ndarray<nb::numpy, const double> restPositions,
  nb::ndarray<nb::numpy, const std::int64_t> vertexIndicesArr,
  nb::ndarray<nb::numpy, const double> targetPositions,
  double coeff, bool isDisplacement)
{
  std::vector<EigenSupport::TripletD> kTriplets;
  kTriplets.reserve(kValues.size());
  for (size_t i = 0; i < kValues.size(); ++i) {
    kTriplets.emplace_back(kRowIndices[i], kColIndices[i], kValues[i]);
  }
  EigenSupport::SpMatD Koff(rows, cols);
  Koff.setFromTriplets(kTriplets.begin(), kTriplets.end());

  auto restp = python::ndarrayToVectorXd(restPositions);

  std::vector<int> vtxIndices;
  vtxIndices.reserve(vertexIndicesArr.shape(0));
  for (size_t i = 0; i < vertexIndicesArr.shape(0); ++i) {
    vtxIndices.push_back(static_cast<int>(vertexIndicesArr.data()[static_cast<int64_t>(i)]));
  }

  auto tgt = python::ndarrayToVectorXd(targetPositions);

  auto energy = std::make_shared<ConstraintPotentialEnergies::MultipleVertexPulling>(
    std::move(Koff),
    std::move(restp),
    std::move(vtxIndices),
    std::move(tgt),
    coeff,
    isDisplacement);

  return std::make_shared<PyVertexAttachmentEnergy>(std::move(energy));
}

std::shared_ptr<PyEnergySet> createEnergySet(nb::list terms)
{
  std::vector<NonlinearOptimization::EnergySet::Term> cppTerms;
  cppTerms.reserve(nb::len(terms));

  int numDofs = -1;

  for (size_t i = 0; i < nb::len(terms); ++i) {
    auto item = terms[i];
    if (!nb::isinstance<nb::tuple>(item)) {
      throw nb::value_error("Each EnergySet term must be a tuple (energy, weight)");
    }
    auto tup = nb::cast<nb::tuple>(item);
    if (nb::len(tup) != 2) {
      throw nb::value_error("Each EnergySet term must be a tuple (energy, weight)");
    }

    auto energyHandle = nb::cast<std::shared_ptr<PyPotentialEnergy>>(tup[0]);
    double weight = nb::cast<double>(tup[1]);

    if (numDofs < 0) {
      numDofs = energyHandle->numDofs();
    } else if (energyHandle->numDofs() != numDofs) {
      throw nb::value_error("All EnergySet terms must have the same num_dofs");
    }

    cppTerms.push_back({energyHandle->potentialEnergyHandle(), weight});
  }

  if (cppTerms.empty()) {
    throw nb::value_error("EnergySet requires at least one term");
  }

  auto set = std::make_shared<NonlinearOptimization::EnergySet>(numDofs, std::move(cppTerms));
  return std::make_shared<PyEnergySet>(std::move(set));
}

std::shared_ptr<PyMaterialParameterSpace> createMaterialParameterSpace(
  std::shared_ptr<pgo::PySimulationMesh> meshCore,
  const pgo::PyElasticModelConfig &elasticModel,
  const PyParameterDofLayout &elasticLayout,
  const PyMaterialChannelMapping &elasticMapping,
  const pgo::PyPlasticModelConfig &plasticModel,
  const PyParameterDofLayout &plasticLayout,
  const PyMaterialChannelMapping &plasticMapping)
{
  if (!meshCore)
    throw nb::value_error("mesh_core must be non-null");
  const auto elasticConfig = elasticModel.config();
  const auto plasticConfig = plasticModel.config();
  const int ne = meshCore->mesh().getNumElements();
  const int elasticChannels = static_cast<int>(elasticConfig->parameterSpec().channelNames.size());
  const int plasticChannels = static_cast<int>(plasticConfig->parameterSpec().channelNames.size());
  auto space = SolidDeformationModel::makeMaterialParameterSpace(
    *elasticConfig, makeLayout(elasticLayout, ne, elasticChannels),
    makeMapping(elasticMapping, elasticChannels), *plasticConfig,
    makeLayout(plasticLayout, ne, plasticChannels), makeMapping(plasticMapping, plasticChannels));
  return std::make_shared<PyMaterialParameterSpace>(std::move(space));
}

std::shared_ptr<PyMaterialParameters> createDefaultMaterialParameters(
  std::shared_ptr<pgo::PySimulationMesh> meshCore,
  const pgo::PyElasticModelConfig &elasticModel,
  const pgo::PyPlasticModelConfig &plasticModel)
{
  if (!meshCore)
    throw nb::value_error("mesh_core must be non-null");
  return std::make_shared<PyMaterialParameters>(
    SolidDeformationModel::makeDefaultMaterialParameters(
      meshCore->mesh(), *elasticModel.config(), *plasticModel.config()));
}

std::shared_ptr<PyMaterialParameters> createMaterialParameters(
  std::shared_ptr<PyMaterialParameterSpace> space,
  nb::ndarray<nb::numpy, const double> elasticValues,
  nb::ndarray<nb::numpy, const double> plasticValues)
{
  if (!space)
    throw nb::value_error("space must be non-null");
  return std::make_shared<PyMaterialParameters>(
    SolidDeformationModel::makeMaterialParameters(
      space->space(), python::ndarrayToVectorXd(elasticValues),
      python::ndarrayToVectorXd(plasticValues)));
}

std::shared_ptr<PyDeformationEnergy> createDeformationEnergyWithParameters(
  std::shared_ptr<pgo::PySimulationMesh> meshCore,
  const pgo::PyElasticModelConfig &elasticModel,
  const pgo::PyPlasticModelConfig &plasticModel,
  const PyMaterialParameters &materialParameters,
  const pgo::PyFormulation &formulation,
  nb::object elementWeights,
  bool enforceSPD,
  bool enableMaterialMaxStep)
{
  if (!meshCore)
    throw nb::value_error("mesh_core must be non-null");
  SolidDeformationModel::DeformationModelOptions opts;
  opts.enforceSPD = enforceSPD;
  opts.enableMaterialMaxStep = enableMaterialMaxStep;
  if (auto weights = optionalVectorFromObject(elementWeights))
    opts.elementWeights = std::move(*weights);
  auto frames = SolidDeformationModel::makeGlobalAxesMaterialFrameField(
    meshCore->mesh().getNumElements());
  std::shared_ptr<SolidDeformationModel::DeformationModelEnergy> energy;
  {
    nb::gil_scoped_release release;
    energy = SolidDeformationModel::makeDeformationEnergy(
      meshCore->meshPtr(), elasticModel.config(), plasticModel.config(),
      materialParameters.parameters(), frames, formulation.get(), opts);
  }
  return std::make_shared<PyDeformationEnergy>(std::move(energy));
}

std::shared_ptr<PyParameterDofLayout> makeElementwiseParameterDofLayout()
{
  return std::make_shared<PyParameterDofLayout>(
    [](int elements, int localDofs) {
      return std::make_shared<SolidDeformationModel::ElementwiseParameterDofLayout>(elements, localDofs);
    });
}

std::shared_ptr<PyParameterDofLayout> makeConstantParameterDofLayout()
{
  return std::make_shared<PyParameterDofLayout>(
    [](int elements, int localDofs) {
      return std::make_shared<SolidDeformationModel::ConstantParameterDofLayout>(elements, localDofs);
    });
}

std::shared_ptr<PyMaterialChannelMapping> makeIdentityMaterialChannelMapping()
{
  return std::make_shared<PyMaterialChannelMapping>(
    [](int channels) {
      return std::make_shared<SolidDeformationModel::IdentityMaterialChannelMapping>(channels);
    });
}

std::shared_ptr<PyPotentialEnergy> createPlasticMaterialEnergy(
  std::shared_ptr<PyDeformationEnergy> deformationEnergyCore,
  nb::ndarray<nb::numpy, const double> fixedDisplacement)
{
  if (!deformationEnergyCore) {
    throw nb::value_error("deformation_energy must be non-null");
  }

  auto fixed = python::ndarrayToVectorXd(fixedDisplacement);
  auto energy = std::make_shared<SolidDeformationModel::PlasticMaterialEnergy>(
    deformationEnergyCore->energy(), fixed);
  return std::make_shared<PyOwnedPotentialEnergy>(std::move(energy));
}

std::shared_ptr<PyPotentialEnergy> createElasticMaterialEnergy(
  std::shared_ptr<PyDeformationEnergy> deformationEnergyCore,
  nb::ndarray<nb::numpy, const double> fixedDisplacement)
{
  if (!deformationEnergyCore) {
    throw nb::value_error("deformation_energy must be non-null");
  }

  auto fixed = python::ndarrayToVectorXd(fixedDisplacement);
  auto energy = std::make_shared<SolidDeformationModel::ElasticMaterialEnergy>(
    deformationEnergyCore->energy(), fixed);
  return std::make_shared<PyOwnedPotentialEnergy>(std::move(energy));
}
