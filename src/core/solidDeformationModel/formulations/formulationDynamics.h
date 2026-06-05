#pragma once

#include "EigenDef.h"

#include <vector>

namespace pgo
{
namespace VolumetricMeshes
{
class VolumetricMesh;
}

namespace SolidDeformationModel
{

class Formulation;

enum class HermiteBoundaryPolicy
{
  Value,
  First,
  All,
};

EigenSupport::SpMatD buildFormulationMassMatrix(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const Formulation &formulation);

EigenSupport::VXd buildFormulationBodyForce(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const Formulation &formulation,
  const EigenSupport::V3d &acceleration);

EigenSupport::SpMatD buildFormulationSurfaceEmbeddingMatrix(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const Formulation &formulation,
  const EigenSupport::MXd &surfaceVertices);

std::vector<int> hermiteVertexDofs(
  const std::vector<int> &vertexIds,
  HermiteBoundaryPolicy policy);

std::vector<int> hermiteFaceDofs(
  const VolumetricMeshes::VolumetricMesh &mesh,
  int axis,
  bool maxSide,
  HermiteBoundaryPolicy policy);

}  // namespace SolidDeformationModel
}  // namespace pgo
