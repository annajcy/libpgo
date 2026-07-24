#pragma once

#include "formulations/formulation/formulations.h"

#include "../../mesh/volume/core.h"
#include "../../sparse/core.h"
#include "../mass/core.h"
#include "../../simulation/core.h"

#include <memory>
#include <string>
#include <vector>

namespace pgo
{

// Python-facing Formulation hierarchy. Mirrors the C++ class hierarchy so Python
// can hold persistent formulation objects and dispatch through C++ virtual methods.
class PyVolumetricFormulation;

class PyFormulation
{
public:
  explicit PyFormulation(std::shared_ptr<SolidDeformationModel::Formulation> f)
    : formulation_(std::move(f)) {}

  const SolidDeformationModel::Formulation &get() const { return *formulation_; }

  std::string name() const { return std::string(formulation_->getName()); }
  int numBasisFunctionsPerElement() const
  {
    return formulation_->numBasisFunctionsPerElement();
  }
  int localDofs() const { return formulation_->getLocalDofs(); }

protected:
  std::shared_ptr<SolidDeformationModel::Formulation> formulation_;
};

class PyVolumetricFormulation : public PyFormulation
{
public:
  using PyFormulation::PyFormulation;

  const SolidDeformationModel::VolumetricFormulation &volumetric() const
  {
    return static_cast<const SolidDeformationModel::VolumetricFormulation &>(get());
  }
};

class PyShellFormulation : public PyFormulation
{
public:
  using PyFormulation::PyFormulation;

  const SolidDeformationModel::ShellFormulation &shell() const
  {
    return static_cast<const SolidDeformationModel::ShellFormulation &>(get());
  }
};

// --- factory functions ---

std::shared_ptr<PyVolumetricFormulation> make_tet_linear();
std::shared_ptr<PyVolumetricFormulation> make_cubic_linear();
std::shared_ptr<PyVolumetricFormulation> make_cubic_tricubic_hermite();
std::shared_ptr<PyShellFormulation> make_koiter_shell();

PySparseMatrix compute_formulation_mass_matrix(
  const PySimulationMesh &simMesh,
  const PyVolumetricFormulation &formulation,
  const PyVolumeMassField &massField);

std::vector<double> compute_formulation_body_force(
  const PySimulationMesh &simMesh,
  const PyVolumetricFormulation &formulation,
  const std::vector<double> &acceleration,
  const PyVolumeMassField &massField);

PySparseMatrix compute_formulation_surface_embedding_matrix(
  const PyVolumeMesh &volumeMesh,
  const PyVolumetricFormulation &formulation,
  const std::vector<double> &surfaceVerticesFlat);

PySparseMatrix compute_shell_formulation_mass_matrix(
  const PySimulationMesh &simMesh,
  const PyShellFormulation &formulation,
  const PyShellMassField &massField,
  std::shared_ptr<PyMaterialParameters> materialParameters);

std::vector<double> compute_shell_formulation_body_force(
  const PySimulationMesh &simMesh,
  const PyShellFormulation &formulation,
  const std::vector<double> &acceleration,
  const PyShellMassField &massField,
  std::shared_ptr<PyMaterialParameters> materialParameters);

PySparseMatrix compute_shell_formulation_body_force_parameter_jacobian(
  const PySimulationMesh &simMesh,
  const PyShellFormulation &formulation,
  const std::vector<double> &acceleration,
  const PyShellMassField &massField,
  std::shared_ptr<PyMaterialParameters> materialParameters);

}  // namespace pgo
