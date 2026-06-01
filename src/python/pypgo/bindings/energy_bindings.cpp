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
#include <vector>

#include "deformationModelFactory.h"
#include "deformationModelEnergy.h"
#include "EigenDef.h"
#include "eigen_numpy.h"
#include "energySet.h"
#include "evaluation.h"
#include "linearPotentialEnergy.h"
#include "multiVertexPullingSoftConstraints.h"
#include "potentialEnergy.h"
#include "quadraticPotentialEnergy.h"
#include "simulation_mesh_core.h"
#include "solveDiagnostics.h"
#include "sparse_matrix_core.h"

namespace nb = nanobind;
using namespace pgo;

namespace {

// ── int64 ndarray helper ───────────────────────────────────────────

nb::ndarray<nb::numpy, std::int64_t> intVectorToNdarray(std::vector<int> values)
{
  auto storage = new std::vector<std::int64_t>(values.begin(), values.end());
  nb::capsule owner(storage, [](void *p) noexcept {
    delete static_cast<std::vector<std::int64_t> *>(p);
  });
  return nb::ndarray<nb::numpy, std::int64_t>(
    storage->data(),
    { storage->size() },
    owner);
}

SolidDeformationModel::DeformationModelElasticMaterial parseElasticMaterial(const std::string &s)
{
  if (s == "stable_neo") return SolidDeformationModel::DeformationModelElasticMaterial::STABLE_NEO;
  if (s == "stvk") return SolidDeformationModel::DeformationModelElasticMaterial::STVK;
  if (s == "stvk_vol") return SolidDeformationModel::DeformationModelElasticMaterial::STVK_VOL;
  if (s == "linear") return SolidDeformationModel::DeformationModelElasticMaterial::LINEAR;
  if (s == "mooney_rivlin") return SolidDeformationModel::DeformationModelElasticMaterial::MOONEY_RIVLIN;
  if (s == "koiter_stvk") return SolidDeformationModel::DeformationModelElasticMaterial::KOITER_STVK;
  if (s == "hill_stable_neo") return SolidDeformationModel::DeformationModelElasticMaterial::HILL_STABLE_NEO;
  if (s == "hill_stvk") return SolidDeformationModel::DeformationModelElasticMaterial::HILL_STVK;
  if (s == "hill_stvk_vol") return SolidDeformationModel::DeformationModelElasticMaterial::HILL_STVK_VOL;
  throw std::invalid_argument("Unknown elastic material: " + s);
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

// ── PotentialEnergy handle — base for all Python energy types ────────
//
// Non-subclassable from Python. Every public pypgo.energy class holds
// one of these internally; the handle_ is the single source of truth
// for all evaluation paths (value / gradient / hessian / max_step).

class PyPotentialEnergy
{
public:
  explicit PyPotentialEnergy(std::shared_ptr<const NonlinearOptimization::PotentialEnergy> energy)
    : handle_(std::move(energy))
  {
  }

  int numDofs() const { return handle_->getNumDOFs(); }

  nb::ndarray<nb::numpy, std::int64_t> dofs() const
  {
    auto d = NonlinearOptimization::dofsOf(*handle_);
    return intVectorToNdarray(std::move(d));
  }

  std::string stateKind() const
  {
    switch (handle_->stateKind()) {
      case NonlinearOptimization::EnergyStateKind::Displacement:
        return "displacement";
      default:
        return "generic";
    }
  }

  double value(nb::ndarray<nb::numpy, const double> x) const
  {
    auto xMap = python::ndarrayToVectorMapXd(x);
    double result;
    {
      nb::gil_scoped_release release;
      result = NonlinearOptimization::evaluateValue(*handle_, xMap);
    }
    return result;
  }

  nb::ndarray<nb::numpy, double> gradient(nb::ndarray<nb::numpy, const double> x) const
  {
    auto xMap = python::ndarrayToVectorMapXd(x);
    EigenSupport::VXd grad;
    {
      nb::gil_scoped_release release;
      grad = NonlinearOptimization::evaluateGradient(*handle_, xMap);
    }
    return python::vectorXdToNdarray(std::move(grad));
  }

  PySparseMatrix hessian(nb::ndarray<nb::numpy, const double> x) const
  {
    auto xMap = python::ndarrayToVectorMapXd(x);
    EigenSupport::SpMatD H;
    {
      nb::gil_scoped_release release;
      H = NonlinearOptimization::evaluateHessian(*handle_, xMap);
    }
    return PySparseMatrix(std::move(H));
  }

  NonlinearOptimization::MaxStepResult maxStep(
    nb::ndarray<nb::numpy, const double> x,
    nb::ndarray<nb::numpy, const double> dx) const
  {
    auto xMap = python::ndarrayToVectorMapXd(x);
    auto dxMap = python::ndarrayToVectorMapXd(dx);
    NonlinearOptimization::MaxStepResult result;
    {
      nb::gil_scoped_release release;
      result = NonlinearOptimization::evaluateMaxStep(*handle_, xMap, dxMap);
    }
    return result;
  }

  nb::ndarray<nb::numpy, double> zeroState() const
  {
    int n = handle_->getNumDOFs();
    auto data = new std::vector<double>(static_cast<size_t>(n), 0.0);
    nb::capsule owner(data, [](void *p) noexcept {
      delete static_cast<std::vector<double> *>(p);
    });
    return nb::ndarray<nb::numpy, double>(
      data->data(),
      { data->size() },
      owner);
  }

  std::string repr() const
  {
    return "PotentialEnergy(" + std::to_string(handle_->getNumDOFs()) + " DOFs)";
  }

  std::shared_ptr<const NonlinearOptimization::PotentialEnergy> handle_;
};

// ── Quadratic test factory ─────────────────────────────────────────

std::shared_ptr<PyPotentialEnergy> createQuadraticEnergyForTest(
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
  return std::make_shared<PyPotentialEnergy>(std::move(energy));
}

// ── LinearEnergy factory ────────────────────────────────────────────

// b^T x.  b is (n,) float64 NumPy array; copied into owned VXd.
std::shared_ptr<PyPotentialEnergy> createLinearEnergy(
  nb::ndarray<nb::numpy, const double> b)
{
  auto bVec = python::ndarrayToVectorXd(b);
  auto energy = std::make_shared<PredefinedPotentialEnergies::LinearPotentialEnergy>(std::move(bVec));
  return std::make_shared<PyPotentialEnergy>(std::move(energy));
}

// ── QuadraticEnergy factories ────────────────────────────────────────

// 1/2 x^T A x  from a PySparseMatrix.
std::shared_ptr<PyPotentialEnergy> createQuadraticEnergyFromSparse(
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
  return std::make_shared<PyPotentialEnergy>(std::move(energy));
}

// 1/2 x^T A x + b^T x  from SparseMatrix + optional (n,) float64 b.
std::shared_ptr<PyPotentialEnergy> createQuadraticEnergyFromSparseWithB(
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
  return std::make_shared<PyPotentialEnergy>(std::move(energy));
}

// 1/2 x^T A x  from (rows, cols, row_indices, col_indices, values) COO tuple.
std::shared_ptr<PyPotentialEnergy> createQuadraticEnergyFromCOO(
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
  return std::make_shared<PyPotentialEnergy>(std::move(energy));
}

// 1/2 x^T A x + b^T x  from COO tuple + optional b.
std::shared_ptr<PyPotentialEnergy> createQuadraticEnergyFromCOOWithB(
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
  return std::make_shared<PyPotentialEnergy>(std::move(energy));
}

// ── VertexAttachment factory ─────────────────────────────────────────

// Creates a MultipleVertexPulling energy.  Koff is built from the COO
// inputs; all other inputs are copied/moved into owned storage.
std::shared_ptr<PyPotentialEnergy> createVertexAttachment(
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

  return std::make_shared<PyPotentialEnergy>(std::move(energy));
}

// ── EnergySet binding ───────────────────────────────────────────────

// EnergySet Python wrapper.  Holds the set directly as its handle.
class PyEnergySet
{
public:
  explicit PyEnergySet(std::shared_ptr<NonlinearOptimization::EnergySet> set)
    : set_(std::move(set))
  {
    // Also expose as a generic PotentialEnergy handle for eval.
    handle_ = std::make_shared<PyPotentialEnergy>(set_);
  }

  int numDofs() const { return set_->getNumDOFs(); }
  int numTerms() const { return set_->numTerms(); }

  nb::object term(int i) const
  {
    // Returns (energy_handle, weight) — energy_handle is a new PyPotentialEnergy
    // wrapping the child, but we can't retrieve the original Python wrapper.
    // For now this returns a lightweight info tuple.
    nb::dict info;
    info["weight"] = set_->term(i).weight;
    info["num_dofs"] = set_->term(i).energy->getNumDOFs();
    return info;
  }

  void setWeight(int i, double w) { set_->setWeight(i, w); }

  std::shared_ptr<PyPotentialEnergy> handle() const { return handle_; }

  std::string repr() const
  {
    return "EnergySet(" + std::to_string(set_->numTerms()) + " terms, " +
      std::to_string(set_->getNumDOFs()) + " DOFs, state_kind='" +
      handle_->stateKind() + "')";
  }

  std::shared_ptr<NonlinearOptimization::EnergySet> set_;
  std::shared_ptr<PyPotentialEnergy> handle_;
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

    cppTerms.push_back({energyHandle->handle_, weight});
  }

  if (cppTerms.empty()) {
    throw nb::value_error("EnergySet requires at least one term");
  }

  auto set = std::make_shared<NonlinearOptimization::EnergySet>(numDofs, std::move(cppTerms));
  return std::make_shared<PyEnergySet>(std::move(set));
}

// Private/experimental deformation energy wrapper.
// Keeps the PySimulationMesh alive so the borrowed mesh outlives the energy chain.
class PyDeformationEnergy
{
public:
  PyDeformationEnergy(std::shared_ptr<SolidDeformationModel::DeformationModelEnergy> energy,
    std::shared_ptr<PySimulationMesh> meshOwner)
    : meshOwner_(std::move(meshOwner)),
      energy_(std::move(energy))
  {
  }

  int numDofs() const { return energy_->getNumDOFs(); }

  std::vector<double> restPositionFlat() const
  {
    const auto &rp = energy_->getRestPosition();
    return std::vector<double>(rp.data(), rp.data() + rp.size());
  }

  std::vector<double> zeroState() const
  {
    return std::vector<double>(static_cast<size_t>(numDofs()), 0.0);
  }

  double value(const std::vector<double> &u) const
  {
    validateInput(u);
    Eigen::Map<const Eigen::VectorXd> uMap(u.data(), static_cast<Eigen::Index>(u.size()));
    double result;
    {
      nb::gil_scoped_release release;
      result = energy_->func(uMap);
    }
    return result;
  }

  std::vector<double> gradient(const std::vector<double> &u) const
  {
    validateInput(u);
    Eigen::Map<const Eigen::VectorXd> uMap(u.data(), static_cast<Eigen::Index>(u.size()));
    std::vector<double> grad(static_cast<size_t>(numDofs()));
    Eigen::Map<Eigen::VectorXd> gradMap(grad.data(), static_cast<Eigen::Index>(grad.size()));
    {
      nb::gil_scoped_release release;
      energy_->gradient(uMap, gradMap);
    }
    return grad;
  }

  PySparseMatrix hessian(const std::vector<double> &u) const
  {
    validateInput(u);
    Eigen::Map<const Eigen::VectorXd> uMap(u.data(), static_cast<Eigen::Index>(u.size()));
    EigenSupport::SpMatD H;
    {
      nb::gil_scoped_release release;
      energy_->hessian(uMap, H);
    }
    return PySparseMatrix(std::move(H));
  }

private:
  void validateInput(const std::vector<double> &u) const
  {
    if (static_cast<int>(u.size()) != numDofs()) {
      throw std::invalid_argument(
        "u size " + std::to_string(u.size()) + " must equal num_dofs (" + std::to_string(numDofs()) + ")");
    }
  }

  std::shared_ptr<PySimulationMesh> meshOwner_;
  std::shared_ptr<SolidDeformationModel::DeformationModelEnergy> energy_;
};

std::shared_ptr<PyDeformationEnergy> createTetDeformationEnergyForTest(
  std::shared_ptr<PySimulationMesh> meshCore,
  const std::string &elasticMaterial,
  const std::string &plasticMaterial)
{
  auto elastic = parseElasticMaterial(elasticMaterial);
  auto plastic = parsePlasticMaterial(plasticMaterial);

  std::shared_ptr<SolidDeformationModel::DeformationModelEnergy> energy;
  {
    nb::gil_scoped_release release;
    energy = SolidDeformationModel::makeDeformationEnergy(
      meshCore->mesh(), SolidDeformationModel::P1TetFormulation{}, elastic, plastic);
  }
  return std::make_shared<PyDeformationEnergy>(std::move(energy), std::move(meshCore));
}

std::shared_ptr<PyDeformationEnergy> createCubicDeformationEnergyForTest(
  std::shared_ptr<PySimulationMesh> meshCore,
  const std::string &elasticMaterial,
  const std::string &plasticMaterial)
{
  auto elastic = parseElasticMaterial(elasticMaterial);
  auto plastic = parsePlasticMaterial(plasticMaterial);

  std::shared_ptr<SolidDeformationModel::DeformationModelEnergy> energy;
  {
    nb::gil_scoped_release release;
    energy = SolidDeformationModel::makeDeformationEnergy(
      meshCore->mesh(), SolidDeformationModel::LinearCubicFormulation{}, elastic, plastic);
  }
  return std::make_shared<PyDeformationEnergy>(std::move(energy), std::move(meshCore));
}

std::shared_ptr<PyDeformationEnergy> createShellDeformationEnergyForTest(
  std::shared_ptr<PySimulationMesh> meshCore,
  const std::string &elasticMaterial,
  const std::string &plasticMaterial)
{
  auto elastic = parseElasticMaterial(elasticMaterial);
  auto plastic = parsePlasticMaterial(plasticMaterial);

  std::shared_ptr<SolidDeformationModel::DeformationModelEnergy> energy;
  {
    nb::gil_scoped_release release;
    energy = SolidDeformationModel::makeDeformationEnergy(
      meshCore->mesh(), SolidDeformationModel::KoiterShellFormulation{}, elastic, plastic);
  }
  return std::make_shared<PyDeformationEnergy>(std::move(energy), std::move(meshCore));
}

}  // namespace

void init_energy_bindings(nb::module_ &m)
{
  // ── MaxStepResult ──────────────────────────────────────────────

  nb::class_<NonlinearOptimization::MaxStepResult>(m, "MaxStepResult")
    .def_ro("alpha", &NonlinearOptimization::MaxStepResult::alpha)
    .def_ro("material_alpha", &NonlinearOptimization::MaxStepResult::materialAlpha)
    .def_ro("contact_alpha", &NonlinearOptimization::MaxStepResult::contactAlpha)
    .def_ro("material_clamped", &NonlinearOptimization::MaxStepResult::materialClamped)
    .def_ro("contact_clamped", &NonlinearOptimization::MaxStepResult::contactClamped);

  // ── PotentialEnergy handle ─────────────────────────────────────
  //
  // Non-subclassable type.  Every public pypgo.energy class holds one
  // of these internally; evaluation always dispatches through
  // evaluation.h helpers so Python never sees hessianInPlace /
  // hessianAlloc / isHessianTopologyFixed.

  nb::class_<PyPotentialEnergy>(m, "PotentialEnergy")
    .def("__repr__", &PyPotentialEnergy::repr)
    .def_prop_ro("num_dofs", &PyPotentialEnergy::numDofs)
    .def("dofs", &PyPotentialEnergy::dofs)
    .def_prop_ro("state_kind", &PyPotentialEnergy::stateKind)
    .def("value", &PyPotentialEnergy::value, nb::arg("x"))
    .def("gradient", &PyPotentialEnergy::gradient, nb::arg("x"))
    .def("hessian", &PyPotentialEnergy::hessian, nb::arg("x"))
    .def("max_step", &PyPotentialEnergy::maxStep, nb::arg("x"), nb::arg("dx"))
    .def("zero_state", &PyPotentialEnergy::zeroState);

  // ── PyDeformationEnergy (private/experimental) ────────────────

  nb::class_<PyDeformationEnergy>(m, "PyDeformationEnergy")
    .def("num_dofs", &PyDeformationEnergy::numDofs)
    .def("rest_position_flat", &PyDeformationEnergy::restPositionFlat)
    .def("zero_state", &PyDeformationEnergy::zeroState)
    .def("value", &PyDeformationEnergy::value, nb::arg("u"))
    .def("gradient", &PyDeformationEnergy::gradient, nb::arg("u"))
    .def("hessian", &PyDeformationEnergy::hessian, nb::arg("u"));

  // Private/experimental factory hooks — only for regression/smoke validation.
  // These names and signatures are NOT public API and will change before Task 10.
  m.def("_create_tet_deformation_energy_for_test", &createTetDeformationEnergyForTest,
    nb::arg("mesh_core"),
    nb::arg("elastic_material") = "stable_neo",
    nb::arg("plastic_material") = "volumetric_dof6");

  m.def("_create_cubic_deformation_energy_for_test", &createCubicDeformationEnergyForTest,
    nb::arg("mesh_core"),
    nb::arg("elastic_material") = "stable_neo",
    nb::arg("plastic_material") = "volumetric_dof6");

  m.def("_create_shell_deformation_energy_for_test", &createShellDeformationEnergyForTest,
    nb::arg("mesh_core"),
    nb::arg("elastic_material") = "koiter_stvk",
    nb::arg("plastic_material") = "shell_ff_dof1");

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

  nb::class_<PyEnergySet>(m, "EnergySet")
    .def("__repr__", &PyEnergySet::repr)
    .def_prop_ro("num_dofs", &PyEnergySet::numDofs)
    .def_prop_ro("num_terms", &PyEnergySet::numTerms)
    .def("term", &PyEnergySet::term, nb::arg("i"))
    .def("set_weight", &PyEnergySet::setWeight, nb::arg("i"), nb::arg("w"))
    .def_prop_ro("handle", &PyEnergySet::handle);

  m.def("_create_energy_set", &createEnergySet, nb::arg("terms"));
}
