#pragma once

#include "formulations/formulation.h"

#include "../../mesh/volume/core.h"
#include "../../sparse/core.h"

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
  int nodesPerElement() const { return formulation_->getNodesPerElement(); }
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
};

// --- factory functions ---

std::shared_ptr<PyVolumetricFormulation> make_tet_p1();
std::shared_ptr<PyVolumetricFormulation> make_linear_cubic();
std::shared_ptr<PyVolumetricFormulation> make_tricubic_hermite();
std::shared_ptr<PyShellFormulation> make_koiter_shell();

PySparseMatrix compute_formulation_mass_matrix(
  const PyVolumeMesh &volumeMesh,
  const PyVolumetricFormulation &formulation);

std::vector<double> compute_formulation_body_force(
  const PyVolumeMesh &volumeMesh,
  const PyVolumetricFormulation &formulation,
  const std::vector<double> &acceleration);

PySparseMatrix compute_formulation_surface_embedding_matrix(
  const PyVolumeMesh &volumeMesh,
  const PyVolumetricFormulation &formulation,
  const std::vector<double> &surfaceVerticesFlat);

}  // namespace pgo
