#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/shared_ptr.h>

#include <Eigen/Core>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "deformationModelFactory.h"
#include "deformationModelEnergy.h"
#include "EigenDef.h"
#include "simulation_mesh_core.h"
#include "sparse_matrix_core.h"

namespace nb = nanobind;
using namespace pgo;

namespace {

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

// Private/experimental deformation energy wrapper.
// Keeps the SimulationMeshCore alive so the borrowed mesh outlives the energy chain.
class DeformationEnergyCore
{
public:
  DeformationEnergyCore(std::shared_ptr<SolidDeformationModel::DeformationModelEnergy> energy,
    std::shared_ptr<SimulationMeshCore> meshOwner)
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

  SparseMatrixCore hessian(const std::vector<double> &u) const
  {
    validateInput(u);
    Eigen::Map<const Eigen::VectorXd> uMap(u.data(), static_cast<Eigen::Index>(u.size()));
    EigenSupport::SpMatD H;
    {
      nb::gil_scoped_release release;
      energy_->hessian(uMap, H);
    }
    return SparseMatrixCore(std::move(H));
  }

private:
  void validateInput(const std::vector<double> &u) const
  {
    if (static_cast<int>(u.size()) != numDofs()) {
      throw std::invalid_argument(
        "u size " + std::to_string(u.size()) + " must equal num_dofs (" + std::to_string(numDofs()) + ")");
    }
  }

  std::shared_ptr<SimulationMeshCore> meshOwner_;
  std::shared_ptr<SolidDeformationModel::DeformationModelEnergy> energy_;
};

std::shared_ptr<DeformationEnergyCore> createTetDeformationEnergyForTest(
  std::shared_ptr<SimulationMeshCore> meshCore,
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
  return std::make_shared<DeformationEnergyCore>(std::move(energy), std::move(meshCore));
}

std::shared_ptr<DeformationEnergyCore> createCubicDeformationEnergyForTest(
  std::shared_ptr<SimulationMeshCore> meshCore,
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
  return std::make_shared<DeformationEnergyCore>(std::move(energy), std::move(meshCore));
}

std::shared_ptr<DeformationEnergyCore> createShellDeformationEnergyForTest(
  std::shared_ptr<SimulationMeshCore> meshCore,
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
  return std::make_shared<DeformationEnergyCore>(std::move(energy), std::move(meshCore));
}

}  // namespace

void init_energy_bindings(nb::module_ &m)
{
  nb::class_<DeformationEnergyCore>(m, "DeformationEnergyCore")
    .def("num_dofs", &DeformationEnergyCore::numDofs)
    .def("rest_position_flat", &DeformationEnergyCore::restPositionFlat)
    .def("zero_state", &DeformationEnergyCore::zeroState)
    .def("value", &DeformationEnergyCore::value, nb::arg("u"))
    .def("gradient", &DeformationEnergyCore::gradient, nb::arg("u"))
    .def("hessian", &DeformationEnergyCore::hessian, nb::arg("u"));

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
}
