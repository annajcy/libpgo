#include "core.h"

#include <nanobind/nanobind.h>

#include <stdexcept>

namespace pgo
{

std::shared_ptr<PyVolumetricFormulation> make_tet_p1()
{
  return std::make_shared<PyVolumetricFormulation>(
    std::make_shared<SolidDeformationModel::P1TetFormulation>());
}

std::shared_ptr<PyVolumetricFormulation> make_linear_cubic()
{
  return std::make_shared<PyVolumetricFormulation>(
    std::make_shared<SolidDeformationModel::LinearCubicFormulation>());
}

std::shared_ptr<PyVolumetricFormulation> make_tricubic_hermite()
{
  return std::make_shared<PyVolumetricFormulation>(
    std::make_shared<SolidDeformationModel::TricubicHermiteFormulation>());
}

std::shared_ptr<PyShellFormulation> make_koiter_shell()
{
  return std::make_shared<PyShellFormulation>(
    std::make_shared<SolidDeformationModel::KoiterShellFormulation>());
}

PySparseMatrix compute_formulation_mass_matrix(
  const PyVolumeMesh &volumeMesh,
  const PyVolumetricFormulation &formulation)
{
  pgo::EigenSupport::SpMatD M;
  {
    nanobind::gil_scoped_release release;
    M = formulation.volumetric().buildMassMatrix(*volumeMesh.getVM());
  }
  return PySparseMatrix(std::move(M));
}

std::vector<double> compute_formulation_body_force(
  const PyVolumeMesh &volumeMesh,
  const PyVolumetricFormulation &formulation,
  const std::vector<double> &acceleration)
{
  if (acceleration.size() != 3) {
    throw std::invalid_argument("acceleration must contain exactly 3 values");
  }

  pgo::EigenSupport::V3d a(acceleration[0], acceleration[1], acceleration[2]);
  pgo::EigenSupport::VXd f;
  {
    nanobind::gil_scoped_release release;
    f = formulation.volumetric().buildBodyForce(*volumeMesh.getVM(), a);
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

}  // namespace pgo
