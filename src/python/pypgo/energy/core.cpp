#include "core.h"

#include "energy/deformationEnergyBuilder.h"
#include "deformation/deformationModelManager.h"
#include "material/elastic/elasticModelFactory.h"
#include "energy/plasticMaterialEnergy.h"
#include "constraints/core.h"
#include "constraints/potentialEnergyFromConstraintFunctions.h"
#include "linearPotentialEnergy.h"
#include "quadraticPotentialEnergy.h"
#include "multiVertexPullingSoftConstraints.h"
#include "eigen_numpy.h"
#include "EigenDef.h"
#include "potentialEnergy.h"
#include "simulation/simulationMesh.h"
#include "../sparse/core.h"

#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <stdexcept>
#include <tuple>
#include <utility>

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

std::optional<EigenSupport::VXd> optionalVectorFromObject(const nb::object &values)
{
  if (values.is_none()) {
    return std::nullopt;
  }
  auto arr = nb::cast<nb::ndarray<nb::numpy, const double>>(values);
  return python::ndarrayToVectorXd(arr);
}

struct DeformationEnergyInputs
{
  SolidDeformationModel::DeformationModelElasticMaterial elasticMaterial;
  SolidDeformationModel::DeformationModelPlasticMaterial plasticMaterial;
  std::shared_ptr<SolidDeformationModel::OptimizableField> elasticField;
  std::shared_ptr<SolidDeformationModel::OptimizableField> plasticField;
};

DeformationEnergyInputs makeDeformationEnergyInputs(
  const std::shared_ptr<PySimulationMesh> &meshCore,
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

  DeformationEnergyInputs inputs;
  inputs.elasticMaterial = parseElasticMaterial(elasticModel);
  inputs.plasticMaterial = parsePlasticMaterial(plasticModel);
  inputs.elasticField = SolidDeformationModel::createElasticParameterField(
    meshCore->mesh(), inputs.elasticMaterial, std::move(elasticField));
  inputs.plasticField = SolidDeformationModel::createPlasticParameterField(
    meshCore->mesh(), inputs.plasticMaterial, std::move(plasticField));
  return inputs;
}

}  // namespace

// ── PyDeformationEnergy out-of-line methods ───────────────────────────────

nb::ndarray<nb::numpy, double> PyDeformationEnergy::restPosition() const
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

nb::ndarray<nb::numpy, double> PyDeformationEnergy::plasticGradient(
  nb::ndarray<nb::numpy, const double> displacement) const
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

PySparseMatrix PyDeformationEnergy::plasticHessian(
  nb::ndarray<nb::numpy, const double> displacement) const
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

PySparseMatrix PyDeformationEnergy::plasticJacobian(
  nb::ndarray<nb::numpy, const double> displacement) const
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

std::shared_ptr<PyDeformationEnergy> createDeformationEnergy(
  std::shared_ptr<PySimulationMesh> meshCore,
  const std::string &elasticModel,
  nb::object elasticValues,
  const std::string &plasticModel,
  nb::object plasticValues,
  const std::string &elasticFieldType,
  const std::string &plasticFieldType,
  const std::string &formulationName,
  bool enforceSPD,
  bool enableMaterialMaxStep)
{
  auto inputs = makeDeformationEnergyInputs(
    meshCore,
    elasticModel,
    elasticValues,
    plasticModel,
    plasticValues,
    elasticFieldType,
    plasticFieldType);

  SolidDeformationModel::DeformationModelOptions opts;
  opts.enforceSPD = enforceSPD;
  opts.enableMaterialMaxStep = enableMaterialMaxStep;

  std::shared_ptr<SolidDeformationModel::DeformationModelEnergy> energy;
  {
    nb::gil_scoped_release release;
    if (formulationName == "tet_p1") {
      SolidDeformationModel::P1TetFormulation formulation;
      auto manager = std::make_shared<SolidDeformationModel::DeformationModelManager>(
        meshCore->meshPtr(), inputs.elasticMaterial, inputs.plasticMaterial,
        formulation, opts.enforceSPD ? 1 : 0, nullptr, nullptr);
      auto assembler = std::make_unique<SolidDeformationModel::DeformationModelAssembler>(
        std::move(manager), formulation, inputs.elasticField, inputs.plasticField, nullptr);
      energy = std::make_shared<SolidDeformationModel::DeformationModelEnergy>(
        std::move(assembler), 0, opts.enableMaterialMaxStep);
    } else if (formulationName == "hex_trilinear") {
      SolidDeformationModel::LinearCubicFormulation formulation;
      auto manager = std::make_shared<SolidDeformationModel::DeformationModelManager>(
        meshCore->meshPtr(), inputs.elasticMaterial, inputs.plasticMaterial,
        formulation, opts.enforceSPD ? 1 : 0, nullptr, nullptr);
      auto assembler = std::make_unique<SolidDeformationModel::DeformationModelAssembler>(
        std::move(manager), formulation, inputs.elasticField, inputs.plasticField, nullptr);
      energy = std::make_shared<SolidDeformationModel::DeformationModelEnergy>(
        std::move(assembler), 0, opts.enableMaterialMaxStep);
    } else if (formulationName == "hex_tricubic_hermite") {
      SolidDeformationModel::TricubicHermiteFormulation formulation;
      auto manager = std::make_shared<SolidDeformationModel::DeformationModelManager>(
        meshCore->meshPtr(), inputs.elasticMaterial, inputs.plasticMaterial,
        formulation, opts.enforceSPD ? 1 : 0, nullptr, nullptr);
      auto assembler = std::make_unique<SolidDeformationModel::DeformationModelAssembler>(
        std::move(manager), formulation, inputs.elasticField, inputs.plasticField, nullptr);
      energy = std::make_shared<SolidDeformationModel::DeformationModelEnergy>(
        std::move(assembler), 0, opts.enableMaterialMaxStep);
    } else if (formulationName == "shell_koiter") {
      SolidDeformationModel::KoiterShellFormulation formulation;
      auto manager = std::make_shared<SolidDeformationModel::DeformationModelManager>(
        meshCore->meshPtr(), inputs.elasticMaterial, inputs.plasticMaterial,
        formulation, opts.enforceSPD ? 1 : 0, nullptr, nullptr);
      auto assembler = std::make_unique<SolidDeformationModel::DeformationModelAssembler>(
        std::move(manager), formulation, inputs.elasticField, inputs.plasticField, nullptr);
      energy = std::make_shared<SolidDeformationModel::DeformationModelEnergy>(
        std::move(assembler), 0, opts.enableMaterialMaxStep);
    } else {
      throw std::invalid_argument(
        "Unknown formulation: '" + formulationName +
        "'.  Expected 'tet_p1', 'hex_trilinear', 'hex_tricubic_hermite', or 'shell_koiter'.");
    }
  }
  return std::make_shared<PyDeformationEnergy>(std::move(energy));
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
