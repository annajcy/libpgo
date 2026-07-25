#include "core.h"

#include <nanobind/nanobind.h>

#include <stdexcept>

namespace pgo
{

std::shared_ptr<PyVolumetricFormulation> make_tet_linear()
{
  return std::make_shared<PyVolumetricFormulation>(
    std::make_shared<SolidDeformationModel::TetLinearFormulation>());
}

std::shared_ptr<PyVolumetricFormulation> make_cubic_linear()
{
  return std::make_shared<PyVolumetricFormulation>(
    std::make_shared<SolidDeformationModel::CubicLinearFormulation>());
}

std::shared_ptr<PyVolumetricFormulation> make_cubic_tricubic_hermite()
{
  return std::make_shared<PyVolumetricFormulation>(
    std::make_shared<SolidDeformationModel::CubicTricubicHermiteFormulation>());
}

std::shared_ptr<PyShellFormulation> make_koiter_shell()
{
  return std::make_shared<PyShellFormulation>(
    std::make_shared<SolidDeformationModel::KoiterShellFormulation>());
}

PySparseMatrix compute_formulation_mass_matrix(
  const PySimulationMesh &simMesh,
  const PyVolumetricFormulation &formulation,
  const PyVolumeMassField &massField)
{
  pgo::EigenSupport::SpMatD M;
  {
    nanobind::gil_scoped_release release;
    M = formulation.volumetric().buildMassMatrix(simMesh.mesh(), massField.get());
  }
  return PySparseMatrix(std::move(M));
}

std::vector<double> compute_formulation_body_force(
  const PySimulationMesh &simMesh,
  const PyVolumetricFormulation &formulation,
  const std::vector<double> &acceleration,
  const PyVolumeMassField &massField)
{
  if (acceleration.size() != 3) {
    throw std::invalid_argument("acceleration must contain exactly 3 values");
  }

  pgo::EigenSupport::V3d a(acceleration[0], acceleration[1], acceleration[2]);
  pgo::EigenSupport::VXd f;
  {
    nanobind::gil_scoped_release release;
    f = formulation.volumetric().buildBodyForce(simMesh.mesh(), a, massField.get());
  }
  return std::vector<double>(f.data(), f.data() + f.size());
}

PySparseMatrix compute_formulation_surface_embedding_matrix(
  const PyVolumeMesh &volumeMesh,
  const PyVolumetricFormulation &formulation,
  const std::vector<double> &surfaceVerticesFlat)
{
  if (surfaceVerticesFlat.size() % 3 != 0) {
    throw std::invalid_argument("surface vertices must be a flat 3*m vector");
  }

  const int numVertices = static_cast<int>(surfaceVerticesFlat.size() / 3);
  pgo::EigenSupport::MXd surfaceVertices(numVertices, 3);
  for (int i = 0; i < numVertices; i++) {
    surfaceVertices(i, 0) = surfaceVerticesFlat[static_cast<size_t>(i) * 3 + 0];
    surfaceVertices(i, 1) = surfaceVerticesFlat[static_cast<size_t>(i) * 3 + 1];
    surfaceVertices(i, 2) = surfaceVerticesFlat[static_cast<size_t>(i) * 3 + 2];
  }

  pgo::EigenSupport::SpMatD W;
  {
    nanobind::gil_scoped_release release;
    W = formulation.volumetric().buildSurfaceEmbeddingMatrix(
      *volumeMesh.getVM(), surfaceVertices);
  }
  return PySparseMatrix(std::move(W));
}

PySparseMatrix compute_shell_formulation_mass_matrix(
  const PySimulationMesh &simMesh,
  const PyShellFormulation &formulation,
  const PyShellMassField &massField,
  std::shared_ptr<PyMaterialParameters> materialParameters)
{
  pgo::EigenSupport::SpMatD M;
  {
    nanobind::gil_scoped_release release;
    M = formulation.shell().buildMassMatrix(
      simMesh.mesh(), massField.get(),
      materialParameters ?
        materialParameters->parameters()->snapshot().view() :
        SolidDeformationModel::MaterialParameterEvaluationView{});
  }
  return PySparseMatrix(std::move(M));
}

std::vector<double> compute_shell_formulation_body_force(
  const PySimulationMesh &simMesh,
  const PyShellFormulation &formulation,
  const std::vector<double> &acceleration,
  const PyShellMassField &massField,
  std::shared_ptr<PyMaterialParameters> materialParameters)
{
  if (acceleration.size() != 3) {
    throw std::invalid_argument("acceleration must contain exactly 3 values");
  }

  pgo::EigenSupport::V3d a(acceleration[0], acceleration[1], acceleration[2]);
  pgo::EigenSupport::VXd f;
  {
    nanobind::gil_scoped_release release;
    f = formulation.shell().buildBodyForce(
      simMesh.mesh(), a, massField.get(),
      materialParameters ?
        materialParameters->parameters()->snapshot().view() :
        SolidDeformationModel::MaterialParameterEvaluationView{});
  }
  return std::vector<double>(f.data(), f.data() + f.size());
}

PySparseMatrix compute_shell_formulation_body_force_parameter_jacobian(
  const PySimulationMesh &simMesh,
  const PyShellFormulation &formulation,
  const std::vector<double> &acceleration,
  const PyShellMassField &massField,
  std::shared_ptr<PyMaterialParameters> materialParameters)
{
  if (acceleration.size() != 3) {
    throw std::invalid_argument("acceleration must contain exactly 3 values");
  }

  pgo::EigenSupport::V3d a(acceleration[0], acceleration[1], acceleration[2]);
  pgo::EigenSupport::SpMatD J;
  {
    nanobind::gil_scoped_release release;
    if (!materialParameters)
      throw std::invalid_argument(
        "body_force_parameter_jacobian requires material_parameters");
    J = formulation.shell().buildBodyForceParameterJacobian(
      simMesh.mesh(), a, massField.get(),
      materialParameters->parameters()->snapshot().view());
  }
  return PySparseMatrix(std::move(J));
}

}  // namespace pgo
