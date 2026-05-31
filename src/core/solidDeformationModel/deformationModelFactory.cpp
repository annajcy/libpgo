/*
author: Bohan Wang
copyright to USC
*/
#include "deformationModelFactory.h"

#include "simulationMesh.h"
#include "cubicMesh.h"
#include "tetMesh.h"
#include "volumetricMesh.h"

#include "formulations/dof/vertex3DofLayout.h"
#include "deformationModelAssembler.h"
#include "pgoLogging.h"

namespace pgo::SolidDeformationModel
{
namespace ES = pgo::EigenSupport;

std::unique_ptr<SimulationMesh> makeSimulationMesh(const VolumetricMeshes::VolumetricMesh &mesh)
{
  std::unique_ptr<SimulationMesh> result;
  switch (mesh.getElementType()) {
  case VolumetricMeshes::VolumetricMesh::TET: {
    const auto *tet = dynamic_cast<const VolumetricMeshes::TetMesh *>(&mesh);
    if (!tet)
      throw std::invalid_argument("makeSimulationMesh: element type is TET but object is not a TetMesh.");
    result = loadTetMesh(tet);
    break;
  }
  case VolumetricMeshes::VolumetricMesh::CUBIC: {
    const auto *cubic = dynamic_cast<const VolumetricMeshes::CubicMesh *>(&mesh);
    if (!cubic)
      throw std::invalid_argument("makeSimulationMesh: element type is CUBIC but object is not a CubicMesh.");
    result = loadCubicMesh(cubic);
    break;
  }
  default:
    throw std::invalid_argument("makeSimulationMesh: unsupported volumetric element type.");
  }

  if (!result)
    throw std::runtime_error("makeSimulationMesh: failed to create SimulationMesh from volumetric mesh.");

  return result;
}

std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  const SimulationMesh &mesh,
  const Formulation &formulation,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const DeformationModelOptions &opts)
{
  const int nele = mesh.getNumElements();
  const int n3 = mesh.getNumVertices() * 3;

  SPDLOG_LOGGER_INFO(pgo::Logging::lgr(), "Building deformation energy with formulation: {}",
    formulation.getName());

  auto restPosition = std::make_unique<ES::VXd>(n3);
  for (int vi = 0; vi < mesh.getNumVertices(); vi++) {
    double p[3];
    mesh.getVertex(vi, p);
    restPosition->segment<3>(vi * 3) = ES::V3d(p[0], p[1], p[2]);
  }

  // Formulation flows through to the manager (→ initImpl → ElementModelFactory).
  auto manager = std::make_unique<DeformationModelManager>(
    mesh, plastic, elastic, formulation,
    opts.enforceSPD ? 1 : 0,
    /*elementFiberDirections=*/nullptr,
    /*vertexFiberDirections=*/nullptr);

  ES::VXd elementWeights = opts.elementWeights;
  if (elementWeights.size() == 0)
    elementWeights = ES::VXd::Ones(nele);
  else if (static_cast<int>(elementWeights.size()) != nele)
    throw std::invalid_argument("makeDeformationEnergy: elementWeights size does not match the element count.");

  auto dofLayout = std::make_unique<Vertex3DofLayout>(&mesh);
  auto assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), std::move(dofLayout), elementWeights.data());

  auto energy = std::make_shared<DeformationModelEnergy>(
    std::move(assembler), std::move(restPosition), 0, opts.enableMaterialMaxStep);

  return energy;
}

}  // namespace pgo::SolidDeformationModel
