#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/shared_ptr.h>

#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "energy/deformationEnergyBuilder.h"
#include "deformation/deformationModelState.h"
#include "deformation/deformationModelAssembler.h"
#include "energy/deformationModelEnergy.h"
#include "deformation/deformationModelManager.h"
#include "formulations/parameters/parameterField.h"
#include "elastic/elasticModelFactory.h"
#include "energy/plasticMaterialEnergy.h"
#include "constraints/constraint_core.h"
#include "EigenDef.h"
#include "core.h"
#include "eigen_numpy.h"
#include "energySet.h"
#include "evaluation.h"
#include "linearPotentialEnergy.h"
#include "multiVertexPullingSoftConstraints.h"
#include "potentialEnergy.h"
#include "constraints/potentialEnergyFromConstraintFunctions.h"
#include "quadraticPotentialEnergy.h"
#include "../simulation/core.h"
#include "simulation/simulationMesh.h"
#include "solver/common/solveDiagnostics.h"
#include "../sparse/core.h"

namespace nb = nanobind;
using namespace pgo;

namespace {

SolidDeformationModel::DeformationModelElasticMaterial parseElasticMaterial(const std::string &s)
{
  return SolidDeformationModel::ElasticModelFactory::materialFromModelId(s);
}

SolidDeformationModel::DeformationModelPlasticMaterial parsePlasticMaterial(const std::string &s)
{
  if (s == "volumetric_dof6") return SolidDeformationModel::DeformationModelPlasticMaterial::VOLUMETRIC_DOF6;
  if (s == "volumetric_dof3") return SolidDeformationModel::DeformationModelPlasticMaterial::VOLUMETRIC_DOF3;
  if (s == "volumetric_dof0") return SolidDeformationModel::DeformationModelPlasticMaterial::VOLUMETRIC_DOF0;
  if (s == "shell_ff_dof1") return SolidDeformationModel::DeformationModelPlasticMaterial::SHELL_FF_DOF1;
  if (s == "shell_ff_dof0") return SolidDeformationModel::DeformationModelPlasticMaterial::SHELL_FF_DOF0;
  throw std::invalid_argument("Unknown plastic material: " + s);
}

// ── Quadratic test factory ─────────────────────────────────────────

std::shared_ptr<PyOwnedPotentialEnergy> createQuadraticEnergyForTest(
  int rows,
  int cols,
  const std::vector<int> &rowIndices,
  const std::vector<int> &colIndices,
  const std::vector<double> &values)
{
  // Build coo sparse A
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

// ── LinearEnergy factory ────────────────────────────────────────────

// b^T x.  b is (n,) float64 NumPy array; copied into owned VXd.
std::shared_ptr<PyOwnedPotentialEnergy> createLinearEnergy(
  nb::ndarray<nb::numpy, const double> b)
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

class PyParameterField
{
public:
  explicit PyParameterField(std::shared_ptr<SolidDeformationModel::OptimizableField> field)
    : field_(std::move(field))
  {
    if (!field_) {
      throw std::invalid_argument("PyParameterField requires a non-null field.");
    }
  }

  std::shared_ptr<SolidDeformationModel::OptimizableField> field() const { return field_; }

  std::string domain() const
  {
    return field_->spec().domain == SolidDeformationModel::ParameterDomain::ELASTIC ? "elastic" : "plastic";
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
    auto vec = python::ndarrayToVectorXd(values);
    if (vec.size() != expected) {
      throw nb::value_error("ParameterField.set_values: values size does not match the field's global dof count.");
    }
    field_->setGlobalData(vec.data());
  }

private:
  std::shared_ptr<SolidDeformationModel::OptimizableField> field_;
};

std::optional<EigenSupport::VXd> optionalVectorFromObject(const nb::object &values)
{
  if (values.is_none()) {
    return std::nullopt;
  }
  auto arr = nb::cast<nb::ndarray<nb::numpy, const double>>(values);
  return python::ndarrayToVectorXd(arr);
}

class PyDeformationModelState
{
public:
  explicit PyDeformationModelState(
    std::shared_ptr<SolidDeformationModel::DeformationModelState> state)
    : state_(std::move(state))
  {
    if (!state_) {
      throw std::invalid_argument("PyDeformationModelState requires non-null state.");
    }
  }

  std::shared_ptr<SolidDeformationModel::DeformationModelState> state() const { return state_; }

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
    state_->setElasticValues(python::ndarrayToVectorMapXd(values));
  }

  void setPlasticValues(nb::ndarray<nb::numpy, const double> values)
  {
    state_->setPlasticValues(python::ndarrayToVectorMapXd(values));
  }

private:
  std::shared_ptr<SolidDeformationModel::DeformationModelState> state_;
};

SolidDeformationModel::ElasticMaterialFieldType parseElasticFieldType(const std::string &type)
{
  if (type == "elementwise") return SolidDeformationModel::ElasticMaterialFieldType::ELEMENTWISE;
  if (type == "constant") return SolidDeformationModel::ElasticMaterialFieldType::CONSTANT;
  throw nb::value_error("unknown elastic field type (expected 'elementwise' or 'constant')");
}

SolidDeformationModel::PlasticMaterialFieldType parsePlasticFieldType(const std::string &type)
{
  if (type == "elementwise") return SolidDeformationModel::PlasticMaterialFieldType::ELEMENTWISE;
  if (type == "constant") return SolidDeformationModel::PlasticMaterialFieldType::CONSTANT;
  throw nb::value_error("unknown plastic field type (expected 'elementwise' or 'constant')");
}

std::shared_ptr<PyDeformationModelState> createDeformationModelState(
  std::shared_ptr<PySimulationMesh> meshCore,
  const std::string &elasticModel,
  nb::object elasticValues,
  const std::string &plasticModel,
  nb::object plasticValues,
  const std::string &elasticFieldType,
  const std::string &plasticFieldType)
{
  if (!meshCore) {
    throw nb::value_error("mesh_core must be non-null");
  }

  SolidDeformationModel::ElasticFieldInit elasticField;
  elasticField.type = parseElasticFieldType(elasticFieldType);
  elasticField.values = optionalVectorFromObject(elasticValues);

  SolidDeformationModel::PlasticFieldInit plasticField;
  plasticField.type = parsePlasticFieldType(plasticFieldType);
  plasticField.values = optionalVectorFromObject(plasticValues);

  auto state = SolidDeformationModel::DeformationModelState::create(
    meshCore->meshPtr(),
    parseElasticMaterial(elasticModel),
    std::move(elasticField),
    parsePlasticMaterial(plasticModel),
    std::move(plasticField));

  return std::make_shared<PyDeformationModelState>(std::move(state));
}

int elasticNumChannels(
  const std::shared_ptr<PySimulationMesh> &meshCore,
  const std::string &elasticModel)
{
  if (!meshCore) {
    throw nb::value_error("mesh_core must be non-null");
  }
  const auto type = parseElasticMaterial(elasticModel);
  return SolidDeformationModel::ElasticModelFactory::parameterSpec(meshCore->mesh(), type).numChannels;
}

// ── QuadraticEnergy factories ────────────────────────────────────────

// 1/2 x^T A x  from a PySparseMatrix.
std::shared_ptr<PyOwnedPotentialEnergy> createQuadraticEnergyFromSparse(
  const PySparseMatrix &A)
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

// 1/2 x^T A x + b^T x  from SparseMatrix + optional (n,) float64 b.
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

// 1/2 x^T A x  from (rows, cols, row_indices, col_indices, values) COO tuple.
std::shared_ptr<PyOwnedPotentialEnergy> createQuadraticEnergyFromCOO(
  int rows,
  int cols,
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

// 1/2 x^T A x + b^T x  from COO tuple + optional b.
std::shared_ptr<PyOwnedPotentialEnergy> createQuadraticEnergyFromCOOWithB(
  int rows,
  int cols,
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

// ── VertexAttachment factory ─────────────────────────────────────────

// Creates a MultipleVertexPulling energy.  Koff is built from the COO
// inputs; all other inputs are copied/moved into owned storage.
std::shared_ptr<PyVertexAttachmentEnergy> createVertexAttachment(
  int numDofs,
  int rows, int cols,
  const std::vector<int> &kRowIndices,
  const std::vector<int> &kColIndices,
  const std::vector<double> &kValues,
  nb::ndarray<nb::numpy, const double> restPositions,
  nb::ndarray<nb::numpy, const std::int64_t> vertexIndicesArr,
  nb::ndarray<nb::numpy, const double> targetPositions,
  double coeff,
  bool isDisplacement)
{
  // Build Koff sparse matrix
  std::vector<EigenSupport::TripletD> kTriplets;
  kTriplets.reserve(kValues.size());
  for (size_t i = 0; i < kValues.size(); ++i) {
    kTriplets.emplace_back(kRowIndices[i], kColIndices[i], kValues[i]);
  }
  EigenSupport::SpMatD Koff(rows, cols);
  Koff.setFromTriplets(kTriplets.begin(), kTriplets.end());

  // Convert inputs
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


// ── EnergySet binding ───────────────────────────────────────────────

// EnergySet Python wrapper.  Inherits PyPotentialEnergy directly so
// _handle is the concrete PyEnergySet peer (no separate handle_ wrapper).
class PyEnergySet final : public PyPotentialEnergy
{
public:
  explicit PyEnergySet(std::shared_ptr<NonlinearOptimization::EnergySet> set)
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

  std::shared_ptr<NonlinearOptimization::EnergySet> set_;
};

// Construct an EnergySet from a list of (PyPotentialEnergy, weight) pairs.
std::shared_ptr<PyEnergySet> createEnergySet(
  nb::list terms)
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

// Deformation energy wrapper — inherits PyPotentialEnergy directly so
// _handle is the concrete peer.  Provides deformation-specific metadata
// (rest_position, plastic_gradient, etc.).
//
// Owns the model state so the C++ mesh and parameter fields outlive the energy chain.
class PyDeformationEnergy : public PyPotentialEnergy
{
public:
  PyDeformationEnergy(std::shared_ptr<SolidDeformationModel::DeformationModelEnergy> energy,
    std::shared_ptr<PyDeformationModelState> stateOwner)
    : energy_(std::move(energy)), stateOwner_(std::move(stateOwner))
  {
  }

  std::shared_ptr<const NO::PotentialEnergy> potentialEnergyHandle() const override { return energy_; }
  std::shared_ptr<SolidDeformationModel::DeformationModelEnergy> energy() const { return energy_; }

  // Rest position as (num_vertices, 3) ndarray.
  nb::ndarray<nb::numpy, double> restPosition() const
  {
    const auto &rp = energy_->getRestPosition();
    int n3 = static_cast<int>(rp.size());
    int nv = n3 / 3;
    auto data = new std::vector<double>(rp.data(), rp.data() + n3);
    nb::capsule owner(data, [](void *p) noexcept {
      delete static_cast<std::vector<double> *>(p);
    });
    return nb::ndarray<nb::numpy, double>(
      data->data(), {static_cast<size_t>(nv), static_cast<size_t>(3)}, owner);
  }

  // Number of vertices (rest_position rows).
  int numVertices() const { return static_cast<int>(energy_->getRestPosition().size() / 3); }

  int numPlasticDofs() const { return energy_->assembler().getNumPlasticGlobalParams(); }

  nb::ndarray<nb::numpy, double> plasticGradient(nb::ndarray<nb::numpy, const double> displacement) const
  {
    auto u = python::ndarrayToVectorMapXd(displacement);
    if (u.size() != energy_->getRestPosition().size()) {
      throw nb::value_error("displacement size must match deformation energy num_dofs.");
    }

    EigenSupport::VXd grad = EigenSupport::VXd::Zero(numPlasticDofs());
    {
      nb::gil_scoped_release release;
      const EigenSupport::VXd p = energy_->getRestPosition() + u;
      energy_->assembler().computePlasticGradient(p.data(), grad.data());
    }
    return python::vectorXdToNdarray(std::move(grad));
  }

  PySparseMatrix plasticHessian(nb::ndarray<nb::numpy, const double> displacement) const
  {
    auto u = python::ndarrayToVectorMapXd(displacement);
    if (u.size() != energy_->getRestPosition().size()) {
      throw nb::value_error("displacement size must match deformation energy num_dofs.");
    }

    EigenSupport::SpMatD hess = energy_->assembler().getPlasticHessianTemplate();
    {
      nb::gil_scoped_release release;
      const EigenSupport::VXd p = energy_->getRestPosition() + u;
      energy_->assembler().computePlasticHessian(p.data(), hess);
    }
    return PySparseMatrix(std::move(hess));
  }

  PySparseMatrix plasticJacobian(nb::ndarray<nb::numpy, const double> displacement) const
  {
    auto u = python::ndarrayToVectorMapXd(displacement);
    if (u.size() != energy_->getRestPosition().size()) {
      throw nb::value_error("displacement size must match deformation energy num_dofs.");
    }

    EigenSupport::SpMatD jac = energy_->assembler().get_dfda_Template();
    {
      nb::gil_scoped_release release;
      const EigenSupport::VXd p = energy_->getRestPosition() + u;
      energy_->assembler().compute_df_da(p.data(), jac);
    }
    return PySparseMatrix(std::move(jac));
  }

private:
  std::shared_ptr<SolidDeformationModel::DeformationModelEnergy> energy_;
  std::shared_ptr<PyDeformationModelState> stateOwner_;
};

std::shared_ptr<PyDeformationEnergy> createDeformationEnergy(
  std::shared_ptr<PyDeformationModelState> stateCore,
  const std::string &formulationName,
  bool enforceSPD,
  bool enableMaterialMaxStep)
{
  if (!stateCore) {
    throw nb::value_error("state must be non-null");
  }

  SolidDeformationModel::DeformationModelOptions opts;
  opts.enforceSPD = enforceSPD;
  opts.enableMaterialMaxStep = enableMaterialMaxStep;

  std::shared_ptr<SolidDeformationModel::DeformationModelEnergy> energy;
  {
    nb::gil_scoped_release release;
    if (formulationName == "tet_p1") {
      energy = SolidDeformationModel::makeDeformationEnergy(
        stateCore->state(), SolidDeformationModel::P1TetFormulation{}, opts);
    } else if (formulationName == "hex_trilinear") {
      energy = SolidDeformationModel::makeDeformationEnergy(
        stateCore->state(), SolidDeformationModel::LinearCubicFormulation{}, opts);
    } else if (formulationName == "hex_tricubic_hermite") {
      energy = SolidDeformationModel::makeDeformationEnergy(
        stateCore->state(), SolidDeformationModel::TricubicHermiteFormulation{}, opts);
    } else if (formulationName == "shell_koiter") {
      energy = SolidDeformationModel::makeDeformationEnergy(
        stateCore->state(), SolidDeformationModel::KoiterShellFormulation{}, opts);
    } else {
      throw std::invalid_argument(
        "Unknown formulation: '" + formulationName +
        "'.  Expected 'tet_p1', 'hex_trilinear', 'hex_tricubic_hermite', or 'shell_koiter'.");
    }
  }
  return std::make_shared<PyDeformationEnergy>(std::move(energy), std::move(stateCore));
}

std::shared_ptr<PyPotentialEnergy> createPlasticMaterialEnergy(
  std::shared_ptr<PyDeformationModelState> stateCore,
  std::shared_ptr<PyDeformationEnergy> deformationEnergyCore,
  nb::ndarray<nb::numpy, const double> fixedDisplacement)
{
  if (!stateCore) {
    throw nb::value_error("state must be non-null");
  }
  if (!deformationEnergyCore) {
    throw nb::value_error("deformation_energy must be non-null");
  }

  auto fixed = python::ndarrayToVectorXd(fixedDisplacement);
  auto energy = std::make_shared<SolidDeformationModel::PlasticMaterialEnergy>(
    stateCore->state(), deformationEnergyCore->energy(), fixed);
  return std::make_shared<PyOwnedPotentialEnergy>(std::move(energy));
}

}  // namespace

void init_energy_bindings(nb::module_ &m)
{
  // ── StepConstraint ─────────────────────────────────────────────

  nb::class_<NonlinearOptimization::StepConstraint>(m, "StepConstraint")
    .def_ro("alpha", &NonlinearOptimization::StepConstraint::alpha)
    .def_prop_ro("clamped", [](const NonlinearOptimization::StepConstraint &r) { return r.clamped(); })
    .def_prop_ro("source", [](const NonlinearOptimization::StepConstraint &r) {
      return r.source == NonlinearOptimization::StepSource::Contact ? "contact" : "material";
    });

  // ── PotentialEnergy handle ─────────────────────────────────────
  //
  // Polymorphic base for all energy peers.  Each concrete Python energy
  // type stores a derived peer in _handle.  Evaluation dispatches through
  // the virtual potentialEnergyHandle() method.

  nb::class_<PyPotentialEnergy>(m, "PyPotentialEnergy")
    .def("__repr__", &PyPotentialEnergy::repr)
    .def_prop_ro("num_dofs", &PyPotentialEnergy::numDofs)
    .def("dofs", &PyPotentialEnergy::dofs)
    .def_prop_ro("state_kind", &PyPotentialEnergy::stateKind)
    .def("value", &PyPotentialEnergy::value, nb::arg("x"))
    .def("gradient", &PyPotentialEnergy::gradient, nb::arg("x"))
    .def("hessian", &PyPotentialEnergy::hessian, nb::arg("x"))
    .def("max_step", &PyPotentialEnergy::maxStep, nb::arg("x"), nb::arg("dx"))
    .def("zero_state", &PyPotentialEnergy::zeroState);

  nb::class_<PyOwnedPotentialEnergy, PyPotentialEnergy>(m, "PyOwnedPotentialEnergy");

  nb::class_<PyVertexAttachmentEnergy, PyPotentialEnergy>(m, "PyVertexAttachmentEnergy")
    .def("set_target_positions", &PyVertexAttachmentEnergy::setTargetPositions);

  // ── PyDeformationEnergy ────────────────────────────────────────
  //
  // Bridges the shared PyPotentialEnergy handle protocol with
  // deformation-specific metadata.  Python DeformationEnergy wraps
  // this: evaluation goes through handle(), rest_position is separate.

  nb::class_<PyDeformationEnergy, PyPotentialEnergy>(m, "PyDeformationEnergy")
    .def("rest_position", &PyDeformationEnergy::restPosition)
    .def_prop_ro("num_vertices", &PyDeformationEnergy::numVertices)
    .def_prop_ro("num_plastic_dofs", &PyDeformationEnergy::numPlasticDofs)
    .def("plastic_gradient", &PyDeformationEnergy::plasticGradient, nb::arg("displacement"))
    .def("plastic_hessian", &PyDeformationEnergy::plasticHessian, nb::arg("displacement"))
    .def("plastic_jacobian", &PyDeformationEnergy::plasticJacobian, nb::arg("displacement"));

  nb::class_<PyParameterField>(m, "PyParameterField")
    .def_prop_ro("domain", &PyParameterField::domain)
    .def_prop_ro("model", &PyParameterField::model)
    .def_prop_ro("num_elements", &PyParameterField::numElements)
    .def_prop_ro("num_value_rows", &PyParameterField::numValueRows)
    .def_prop_ro("num_channels", &PyParameterField::numChannels)
    .def("values", &PyParameterField::values)
    .def("set_values", &PyParameterField::setValues, nb::arg("values"));

  nb::class_<PyDeformationModelState>(m, "PyDeformationModelState")
    .def_prop_ro("elastic_model", &PyDeformationModelState::elasticModel)
    .def_prop_ro("plastic_model", &PyDeformationModelState::plasticModel)
    .def_prop_ro("num_elements", &PyDeformationModelState::numElements)
    .def_prop_ro("elastic_field", &PyDeformationModelState::elasticField)
    .def_prop_ro("plastic_field", &PyDeformationModelState::plasticField)
    .def("set_elastic_values", &PyDeformationModelState::setElasticValues, nb::arg("values"))
    .def("set_plastic_values", &PyDeformationModelState::setPlasticValues, nb::arg("values"));

  m.def("_create_deformation_model_state", &createDeformationModelState,
    nb::arg("mesh_core"),
    nb::arg("elastic_model"),
    nb::arg("elastic_values").none(),
    nb::arg("plastic_model"),
    nb::arg("plastic_values").none(),
    nb::arg("elastic_field_type") = "elementwise",
    nb::arg("plastic_field_type") = "elementwise");

  m.def("_elastic_num_channels", &elasticNumChannels,
    nb::arg("mesh_core"),
    nb::arg("elastic_model"));

  // Unified deformation energy factory (public API entry point).
  m.def("_create_deformation_energy", &createDeformationEnergy,
    nb::arg("state_core"),
    nb::arg("formulation"),
    nb::arg("enforce_spd") = true,
    nb::arg("enable_material_max_step") = true);

  m.def("_create_plastic_material_energy", &createPlasticMaterialEnergy,
    nb::arg("state_core"),
    nb::arg("deformation_energy_core"),
    nb::arg("fixed_displacement"));

  // Private/experimental — minimal QuadraticPotentialEnergy factory for
  // PotentialEnergy-handle tests.  Signature will change when
  // pypgo.energy.QuadraticEnergy is finalised (Task E5).
  m.def("_create_quadratic_energy_for_test", &createQuadraticEnergyForTest,
    nb::arg("rows"),
    nb::arg("cols"),
    nb::arg("row_indices"),
    nb::arg("col_indices"),
    nb::arg("values"));

  // ── LinearEnergy / QuadraticEnergy factories (Task E5) ────────────

  m.def("_create_linear_energy", &createLinearEnergy,
    nb::arg("b"));

  m.def("_create_constraint_penalty", &createConstraintPenalty,
    nb::arg("constraints"),
    nb::arg("weight") = 1.0);

  m.def("_create_constraint_violation_penalty", &createConstraintViolationPenalty,
    nb::arg("constraints"),
    nb::arg("lower"),
    nb::arg("upper"),
    nb::arg("weight") = 1.0);

  m.def("_create_quadratic_energy_from_sparse", &createQuadraticEnergyFromSparse,
    nb::arg("A"));

  m.def("_create_quadratic_energy_from_sparse_with_b", &createQuadraticEnergyFromSparseWithB,
    nb::arg("A"), nb::arg("b"));

  m.def("_create_quadratic_energy_from_coo", &createQuadraticEnergyFromCOO,
    nb::arg("rows"),
    nb::arg("cols"),
    nb::arg("row_indices"),
    nb::arg("col_indices"),
    nb::arg("values"));

  m.def("_create_quadratic_energy_from_coo_with_b", &createQuadraticEnergyFromCOOWithB,
    nb::arg("rows"),
    nb::arg("cols"),
    nb::arg("row_indices"),
    nb::arg("col_indices"),
    nb::arg("values"),
    nb::arg("b"));

  // ── VertexAttachment factory (Task E9b) ───────────────────────────

  m.def("_create_vertex_attachment", &createVertexAttachment,
    nb::arg("num_dofs"),
    nb::arg("rows"),
    nb::arg("cols"),
    nb::arg("k_row_indices"),
    nb::arg("k_col_indices"),
    nb::arg("k_values"),
    nb::arg("rest_positions"),
    nb::arg("vertex_indices"),
    nb::arg("target_positions"),
    nb::arg("coeff") = 1.0,
    nb::arg("is_displacement") = true);

  // ── EnergySet (Task E6) ────────────────────────────────────────────
  //
  // Inherits PyPotentialEnergy so _handle is the concrete peer and
  // num_dofs / value / gradient / etc. are inherited from the base.

  nb::class_<PyEnergySet, PyPotentialEnergy>(m, "PyEnergySet")
    .def("__repr__", &PyEnergySet::repr)
    .def_prop_ro("num_terms", &PyEnergySet::numTerms)
    .def("term", &PyEnergySet::term, nb::arg("i"))
    .def("set_weight", &PyEnergySet::setWeight, nb::arg("i"), nb::arg("w"));

  m.def("_create_energy_set", &createEnergySet, nb::arg("terms"));
}
