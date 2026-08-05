#pragma once

#include "formulations/formulation/formulations.h"

#include "../../mesh/volume/core.h"
#include "../../sparse/core.h"
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

class PyKoiterShellFormulation : public PyFormulation
{
public:
  using PyFormulation::PyFormulation;

  const SolidDeformationModel::KoiterShellFormulation &koiter() const
  {
    return static_cast<const SolidDeformationModel::KoiterShellFormulation &>(get());
  }
};

// --- factory functions ---

std::shared_ptr<PyVolumetricFormulation> make_tet_linear();
std::shared_ptr<PyVolumetricFormulation> make_cubic_linear();
std::shared_ptr<PyVolumetricFormulation> make_cubic_tricubic_hermite();
std::shared_ptr<PyKoiterShellFormulation> make_koiter_shell();

PySparseMatrix compute_formulation_mass_matrix(
  const PySimulationMesh &mesh,
  const PyVolumetricFormulation &formulation,
  const std::vector<double> &elementDensities);

std::vector<double> compute_formulation_body_force(
  const PySimulationMesh &mesh,
  const PyVolumetricFormulation &formulation,
  const std::vector<double> &acceleration,
  const std::vector<double> &elementDensities);

PySparseMatrix compute_formulation_surface_embedding_matrix(
  const PyVolumeMesh &volumeMesh,
  const PyVolumetricFormulation &formulation,
  const std::vector<double> &surfaceVerticesFlat);

PySparseMatrix compute_shell_formulation_mass_matrix(
  const PySimulationMesh &mesh,
  const PyKoiterShellFormulation &formulation,
  const std::vector<double> &elementArealDensities);

std::vector<double> compute_shell_formulation_body_force(
  const PySimulationMesh &mesh,
  const PyKoiterShellFormulation &formulation,
  const std::vector<double> &acceleration,
  const std::vector<double> &elementArealDensities);

}  // namespace pgo
