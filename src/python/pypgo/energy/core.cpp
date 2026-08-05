#include "core.h"

#include "energy/deformationEnergyOperator.h"
#include "deformation/deformationElement.h"
#include "../fem/formulation/core.h"
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

namespace
{

std::optional<EigenSupport::VXd> optionalVectorFromObject(const nb::object &values)
{
  if (values.is_none()) {
    return std::nullopt;
  }
  auto arr = nb::cast<nb::ndarray<nb::numpy, const double>>(values);
  return python::ndarrayToVectorXd(arr);
}

}  // namespace

nb::ndarray<nb::numpy, double> PyMaterialState::elasticValues() const
{
  return python::vectorXdToNdarray(EigenSupport::VXd(state_.elasticValues()));
}

nb::ndarray<nb::numpy, double> PyMaterialState::plasticValues() const
{
  return python::vectorXdToNdarray(EigenSupport::VXd(state_.plasticValues()));
}

std::shared_ptr<PyMaterialState> PyMaterialState::withElasticValues(
  nb::ndarray<nb::numpy, const double> values) const
{
  return std::make_shared<PyMaterialState>(SolidDeformationModel::MaterialState(
    python::ndarrayToVectorXd(values), state_.plasticValues()));
}

std::shared_ptr<PyMaterialState> PyMaterialState::withPlasticValues(
  nb::ndarray<nb::numpy, const double> values) const
{
  return std::make_shared<PyMaterialState>(SolidDeformationModel::MaterialState(
    state_.elasticValues(), python::ndarrayToVectorXd(values)));
}

// ── Explicit deformation operator and fixed-state potential ───────────────

nb::ndarray<nb::numpy, double> PyDeformationEnergyOperator::restState() const
{
  return python::vectorXdToNdarray(EigenSupport::VXd(energy_->getRestDofs()));
}

nb::ndarray<nb::numpy, double> PyDeformationEnergyOperator::vertexRestPositions() const
{
  const auto &positions = energy_->getVertexRestPositions();
  const int nv = numVertices();
  EigenSupport::MXd result(nv, 3);
  for (int vi = 0; vi < nv; vi++)
    result.row(vi) = positions.segment<3>(vi * 3).transpose();
  return python::matrixXdToNdarray(std::move(result));
}

double PyDeformationEnergyOperator::value(
  nb::ndarray<nb::numpy, const double> displacement,
  const PyMaterialState &state) const
{
  auto u = python::ndarrayToVectorMapXd(displacement);
  if (u.size() != energy_->getNumDOFs())
    throw nb::value_error("displacement size must match deformation operator num_dofs.");
  nb::gil_scoped_release release;
  return energy_->func(u, state.state().view());
}

nb::ndarray<nb::numpy, double> PyDeformationEnergyOperator::gradient(
  nb::ndarray<nb::numpy, const double> displacement,
  const PyMaterialState &state) const
{
  auto u = python::ndarrayToVectorMapXd(displacement);
  if (u.size() != energy_->getNumDOFs())
    throw nb::value_error("displacement size must match deformation operator num_dofs.");
  EigenSupport::VXd grad(energy_->getNumDOFs());
  {
    nb::gil_scoped_release release;
    energy_->gradient(u, state.state().view(), grad);
  }
  return python::vectorXdToNdarray(std::move(grad));
}

PySparseMatrix PyDeformationEnergyOperator::hessian(
  nb::ndarray<nb::numpy, const double> displacement,
  const PyMaterialState &state) const
{
  auto u = python::ndarrayToVectorMapXd(displacement);
  if (u.size() != energy_->getNumDOFs())
    throw nb::value_error("displacement size must match deformation operator num_dofs.");
  EigenSupport::SpMatD hess;
  energy_->hessianAlloc(hess);
  {
    nb::gil_scoped_release release;
    energy_->hessianInPlace(u, state.state().view(), hess);
  }
  return PySparseMatrix(std::move(hess));
}

nb::ndarray<nb::numpy, double> PyDeformationEnergyOperator::zeroState() const
{
  return python::vectorXdToNdarray(
    EigenSupport::VXd::Zero(energy_->getNumDOFs()));
}

nb::ndarray<nb::numpy, double> PyDeformationEnergyOperator::dE_dp(
  nb::ndarray<nb::numpy, const double> displacement,
  const PyMaterialState &state) const
{
  auto u = python::ndarrayToVectorMapXd(displacement);
  if (u.size() != energy_->getRestDofs().size()) {
    throw nb::value_error("displacement size must match deformation energy num_dofs.");
  }

  EigenSupport::VXd grad = EigenSupport::VXd::Zero(numPlasticValues());
  {
    nb::gil_scoped_release release;
    energy_->compute_dE_dp(u, state.state().view(), grad);
  }
  return python::vectorXdToNdarray(std::move(grad));
}

nb::ndarray<nb::numpy, double> PyDeformationEnergyOperator::dE_de(
  nb::ndarray<nb::numpy, const double> displacement,
  const PyMaterialState &state) const
{
  auto u = python::ndarrayToVectorMapXd(displacement);
  if (u.size() != energy_->getRestDofs().size()) {
    throw nb::value_error("displacement size must match deformation energy num_dofs.");
  }

  EigenSupport::VXd grad = EigenSupport::VXd::Zero(numElasticValues());
  {
    nb::gil_scoped_release release;
    energy_->compute_dE_de(u, state.state().view(), grad);
  }
  return python::vectorXdToNdarray(std::move(grad));
}

nb::ndarray<nb::numpy, double> PyDeformationEnergyOperator::elementVonMisesStresses(
  nb::ndarray<nb::numpy, const double> displacement,
  const PyMaterialState &state) const
{
  auto u = python::ndarrayToVectorMapXd(displacement);
  if (u.size() != energy_->getRestDofs().size()) {
    throw nb::value_error("displacement size must match deformation energy num_dofs.");
  }

  const int nele = energy_->getNumElements();
  EigenSupport::VXd out = EigenSupport::VXd::Zero(nele);
  try {
    nb::gil_scoped_release release;
    energy_->computeVonMisesStresses(u, state.state().view(), out);
  }
  catch (const SolidDeformationModel::UnsupportedDeformationDiagnosticError &e) {
    PyErr_SetString(PyExc_NotImplementedError, e.what());
    throw nb::python_error();
  }
  return python::vectorXdToNdarray(std::move(out));
}

nb::ndarray<nb::numpy, double>
PyDeformationEnergyOperator::elasticMaterialVJP(
  nb::ndarray<nb::numpy, const double> displacement,
  const PyMaterialState &state,
  nb::ndarray<nb::numpy, const double> adjoint) const
{
  auto u = python::ndarrayToVectorMapXd(displacement);
  auto lambda = python::ndarrayToVectorMapXd(adjoint);
  if (u.size() != energy_->getNumDOFs() ||
      lambda.size() != energy_->getNumDOFs())
    throw nb::value_error(
      "displacement and adjoint sizes must match deformation energy num_dofs.");
  EigenSupport::VXd output = EigenSupport::VXd::Zero(numElasticValues());
  {
    nb::gil_scoped_release release;
    energy_->computeElasticMaterialVJP(
      u, state.state().view(), lambda, output);
  }
  return python::vectorXdToNdarray(std::move(output));
}

nb::ndarray<nb::numpy, double>
PyDeformationEnergyOperator::plasticMaterialVJP(
  nb::ndarray<nb::numpy, const double> displacement,
  const PyMaterialState &state,
  nb::ndarray<nb::numpy, const double> adjoint) const
{
  auto u = python::ndarrayToVectorMapXd(displacement);
  auto lambda = python::ndarrayToVectorMapXd(adjoint);
  if (u.size() != energy_->getNumDOFs() ||
      lambda.size() != energy_->getNumDOFs())
    throw nb::value_error(
      "displacement and adjoint sizes must match deformation energy num_dofs.");
  EigenSupport::VXd output = EigenSupport::VXd::Zero(numPlasticValues());
  {
    nb::gil_scoped_release release;
    energy_->computePlasticMaterialVJP(
      u, state.state().view(), lambda, output);
  }
  return python::vectorXdToNdarray(std::move(output));
}

PyDeformationPotentialEnergy::PyDeformationPotentialEnergy(
  std::shared_ptr<PyDeformationEnergyOperator> energyOperator,
  std::shared_ptr<PyMaterialState> materialState):
  energyOperator_(std::move(energyOperator)),
  materialState_(std::move(materialState))
{
  if (!energyOperator_ || !materialState_)
    throw nb::value_error(
      "DeformationPotentialEnergy requires an operator and material state.");
  try {
    energy_ = std::make_shared<SolidDeformationModel::DeformationPotentialEnergy>(
      energyOperator_->energy(), materialState_->state());
  }
  catch (const std::invalid_argument &error) {
    throw nb::value_error(error.what());
  }
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
    }
    else if (energyHandle->numDofs() != numDofs) {
      throw nb::value_error("All EnergySet terms must have the same num_dofs");
    }

    cppTerms.push_back({ energyHandle->potentialEnergyHandle(), weight });
  }

  if (cppTerms.empty()) {
    throw nb::value_error("EnergySet requires at least one term");
  }

  auto set = std::make_shared<NonlinearOptimization::EnergySet>(numDofs, std::move(cppTerms));
  return std::make_shared<PyEnergySet>(std::move(set));
}

std::shared_ptr<PyMaterialBinding> createMaterialBinding(
  const pgo::PyElasticModelDefinition &elasticDefinition,
  int numElements,
  nb::ndarray<nb::numpy, const double> elasticFixedValues,
  const pgo::PyPlasticModelDefinition &plasticDefinition,
  nb::ndarray<nb::numpy, const double> plasticFixedValues,
  std::shared_ptr<PyMaterialFrames> materialFrames)
{
  try {
    std::optional<SolidDeformationModel::MaterialFrames> frames;
    if (materialFrames)
      frames = materialFrames->frames();
    auto binding = std::make_shared<const SolidDeformationModel::MaterialBinding>(
      SolidDeformationModel::ElasticMaterialBinding(
        elasticDefinition.definition(),
        numElements,
        python::ndarrayToVectorXd(elasticFixedValues)),
      SolidDeformationModel::PlasticMaterialBinding(
        plasticDefinition.definition(),
        numElements,
        python::ndarrayToVectorXd(plasticFixedValues)),
      std::move(frames));
    return std::make_shared<PyMaterialBinding>(std::move(binding));
  }
  catch (const std::invalid_argument &error) {
    throw nb::value_error(error.what());
  }
}

std::shared_ptr<PyMaterialState> createMaterialState(
  nb::ndarray<nb::numpy, const double> elasticValues,
  nb::ndarray<nb::numpy, const double> plasticValues)
{
  try {
    return std::make_shared<PyMaterialState>(SolidDeformationModel::MaterialState(
      python::ndarrayToVectorXd(elasticValues),
      python::ndarrayToVectorXd(plasticValues)));
  }
  catch (const std::invalid_argument &error) {
    throw nb::value_error(error.what());
  }
}

std::shared_ptr<PyDeformationEnergyOperator> createDeformationEnergyOperator(
  const pgo::PySimulationMesh &mesh,
  const PyMaterialBinding &materialBinding,
  const pgo::PyFormulation &formulation,
  nb::object elementWeights,
  bool projectHessianPSD,
  bool enableMaterialMaxStep)
{
  SolidDeformationModel::DeformationModelOptions opts;
  opts.projectHessianPSD = projectHessianPSD;
  opts.enableMaterialMaxStep = enableMaterialMaxStep;
  if (auto weights = optionalVectorFromObject(elementWeights))
    opts.elementWeights = std::move(*weights);
  std::shared_ptr<SolidDeformationModel::DeformationEnergyOperator> energy;
  {
    nb::gil_scoped_release release;
    energy = std::make_shared<SolidDeformationModel::DeformationEnergyOperator>(
      mesh.mesh(), *materialBinding.binding(), formulation.get(), opts);
  }
  return std::make_shared<PyDeformationEnergyOperator>(std::move(energy));
}

std::shared_ptr<PyDeformationPotentialEnergy> createDeformationPotentialEnergy(
  std::shared_ptr<PyDeformationEnergyOperator> energyOperator,
  std::shared_ptr<PyMaterialState> materialState)
{
  return std::make_shared<PyDeformationPotentialEnergy>(
    std::move(energyOperator), std::move(materialState));
}

namespace
{
SolidDeformationModel::MaterialFrame frameFromFlatValues(
  const std::vector<double> &values,
  std::size_t offset)
{
  SolidDeformationModel::MaterialFrame frame;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      frame(row, col) = values[offset + static_cast<std::size_t>(row * 3 + col)];
  return frame;
}
}

std::shared_ptr<PyMaterialFrames> makeMaterialFrames(
  const std::vector<double> &frameValues)
{
  if (frameValues.empty() || frameValues.size() % 9 != 0)
    throw nb::value_error("elementwise frames must contain 9 values per element");
  std::vector<SolidDeformationModel::MaterialFrame> frames;
  frames.reserve(frameValues.size() / 9);
  for (std::size_t offset = 0; offset < frameValues.size(); offset += 9)
    frames.push_back(frameFromFlatValues(frameValues, offset));
  return std::make_shared<PyMaterialFrames>(
    SolidDeformationModel::MaterialFrames(std::move(frames)));
}

std::shared_ptr<PyMaterialFrames> makeMaterialFramesFromPrimaryAxes(
  const std::vector<double> &axisValues)
{
  if (axisValues.empty() || axisValues.size() % 3 != 0)
    throw nb::value_error(
      "primary axes must contain three values per element");
  const Eigen::Index numElements =
    static_cast<Eigen::Index>(axisValues.size() / 3);
  EigenSupport::M3Xd axes(3, numElements);
  for (Eigen::Index element = 0; element < numElements; ++element)
    for (int component = 0; component < 3; ++component)
      axes(component, element) =
        axisValues[
          static_cast<std::size_t>(element) * 3 + component];
  try {
    return std::make_shared<PyMaterialFrames>(
      SolidDeformationModel::materialFramesFromPrimaryAxes(axes));
  }
  catch (const std::invalid_argument &error) {
    throw nb::value_error(error.what());
  }
}
