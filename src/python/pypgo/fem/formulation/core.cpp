#include "core.h"

#include <nanobind/nanobind.h>

#include <stdexcept>

namespace pgo
{
namespace
{
EigenSupport::VXd toEigenVector(const std::vector<double> &values)
{
  EigenSupport::VXd result(static_cast<Eigen::Index>(values.size()));
  for (std::size_t i = 0; i < values.size(); i++)
    result[static_cast<Eigen::Index>(i)] = values[i];
  return result;
}
}  // namespace

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

std::shared_ptr<PyKoiterShellFormulation> make_koiter_shell()
{
  return std::make_shared<PyKoiterShellFormulation>(
    std::make_shared<SolidDeformationModel::KoiterShellFormulation>());
}

PySparseMatrix compute_formulation_mass_matrix(
  const PySimulationMesh &mesh,
  const PyVolumetricFormulation &formulation,
  const std::vector<double> &elementDensities)
{
  const EigenSupport::VXd densities = toEigenVector(elementDensities);
  pgo::EigenSupport::SpMatD M;
  {
    nanobind::gil_scoped_release release;
    M = formulation.volumetric().buildMassMatrix(mesh.mesh(), densities);
  }
  return PySparseMatrix(std::move(M));
}

std::vector<double> compute_formulation_body_force(
  const PySimulationMesh &mesh,
  const PyVolumetricFormulation &formulation,
  const std::vector<double> &acceleration,
  const std::vector<double> &elementDensities)
{
  if (acceleration.size() != 3) {
    throw std::invalid_argument("acceleration must contain exactly 3 values");
  }

  pgo::EigenSupport::V3d a(acceleration[0], acceleration[1], acceleration[2]);
  const EigenSupport::VXd densities = toEigenVector(elementDensities);
  pgo::EigenSupport::VXd f;
  {
    nanobind::gil_scoped_release release;
    f = formulation.volumetric().buildBodyForce(mesh.mesh(), a, densities);
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
  const PySimulationMesh &mesh,
  const PyKoiterShellFormulation &formulation,
  const std::vector<double> &elementArealDensities)
{
  const EigenSupport::VXd densities = toEigenVector(elementArealDensities);
  pgo::EigenSupport::SpMatD M;
  {
    nanobind::gil_scoped_release release;
    M = formulation.koiter().buildMassMatrix(mesh.mesh(), densities);
  }
  return PySparseMatrix(std::move(M));
}

std::vector<double> compute_shell_formulation_body_force(
  const PySimulationMesh &mesh,
  const PyKoiterShellFormulation &formulation,
  const std::vector<double> &acceleration,
  const std::vector<double> &elementArealDensities)
{
  if (acceleration.size() != 3) {
    throw std::invalid_argument("acceleration must contain exactly 3 values");
  }

  pgo::EigenSupport::V3d a(acceleration[0], acceleration[1], acceleration[2]);
  const EigenSupport::VXd densities = toEigenVector(elementArealDensities);
  pgo::EigenSupport::VXd f;
  {
    nanobind::gil_scoped_release release;
    f = formulation.koiter().buildBodyForce(mesh.mesh(), a, densities);
  }
  return std::vector<double>(f.data(), f.data() + f.size());
}

}  // namespace pgo
