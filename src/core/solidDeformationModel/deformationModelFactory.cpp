/*
author: Bohan Wang
copyright to USC
*/
#include "deformationModelFactory.h"

#include "simulationMesh.h"
#include "deformationModel.h"
#include "deformationModelAssembler.h"
#include "deformationModelEnergy.h"
#include "plasticModel3DDeformationGradient.h"
#include "cubicMesh.h"
#include "tetMesh.h"
#include "volumetricMesh.h"

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

namespace detail
{
DeformationModelBundle makeDeformationModelBundle(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const DeformationModelOptions &opts)
{
  const int nele = mesh.getNumElements();
  const int n3 = mesh.getNumVertices() * 3;

  // Capture the rest pose from the mesh.
  ES::VXd restPosition(n3);
  for (int vi = 0; vi < mesh.getNumVertices(); vi++) {
    double p[3];
    mesh.getVertex(vi, p);
    restPosition.segment<3>(vi * 3) = ES::V3d(p[0], p[1], p[2]);
  }

  auto manager = std::make_unique<DeformationModelManager>(
    mesh, plastic, elastic,
    opts.enforceSPD ? 1 : 0,
    /*elementFiberDirections=*/nullptr,
    /*vertexFiberDirections=*/nullptr);

  ES::VXd elementWeights = opts.elementWeights;
  if (elementWeights.size() == 0)
    elementWeights = ES::VXd::Ones(nele);
  else if (static_cast<int>(elementWeights.size()) != nele)
    throw std::invalid_argument("makeDeformationModelBundle: elementWeights size does not match the element count.");

  const int numPlasticParams = manager->getNumPlasticParameters();
  ES::VXd plasticParams(static_cast<Eigen::Index>(nele) * numPlasticParams);
  if (numPlasticParams > 0) {
    plasticParams.setZero();
    const ES::M3d identity = ES::M3d::Identity();
    for (int ei = 0; ei < nele; ei++) {
      const auto *pm = dynamic_cast<const PlasticModel3DDeformationGradient *>(
        manager->getDeformationModel(ei)->getPlasticModel());
      if (pm)
        pm->toParam(identity.data(), plasticParams.data() + ei * numPlasticParams);
    }
  }

  const int numElasticParams = manager->getNumElasticParameters();
  ES::VXd elasticParams(static_cast<Eigen::Index>(nele) * numElasticParams);
  if (numElasticParams > 0) {
    elasticParams.setZero();
    for (int ei = 0; ei < nele; ei++) {
      if (elastic == DeformationModelElasticMaterial::KOITER_STVK) {
        const auto *mat = dynamic_cast<const SimulationMeshENuhMaterial *>(mesh.getElementMaterial(ei, 0));
        if (!mat)
          throw std::runtime_error("makeDeformationModelBundle: KOITER_STVK requires SimulationMeshENuhMaterial.");
        elasticParams.segment<5>(ei * 5) << mat->getE(), mat->getNu(), mat->getE(), mat->getNu(), mat->geth();
      }
      else {
        throw std::runtime_error("makeDeformationModelBundle: unsupported elastic parameterized material.");
      }
    }
  }

  auto assembler = std::make_unique<DeformationModelAssembler>(std::move(manager), elementWeights.data());

  DeformationModelBundle bundle;
  bundle.restPosition = std::move(restPosition);
  bundle.plasticParams = std::move(plasticParams);
  bundle.elasticParams = std::move(elasticParams);
  bundle.energy = std::make_shared<DeformationModelEnergy>(std::move(assembler), &bundle.restPosition, 0);
  bundle.energy->setEnableMaterialMaxStep(opts.enableMaterialMaxStep);
  bundle.energy->setPlasticParams(bundle.plasticParams);
  bundle.energy->setElasticParams(bundle.elasticParams);

  return bundle;
}
}  // namespace detail

DeformationModelBundle makeTetDeformationModel(
  const SimulationMesh &mesh,
  const TetFormulationVariant &formulation,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const DeformationModelOptions &opts)
{
  return std::visit([&](const auto &f) {
    return makeTetDeformationModel(mesh, f, elastic, plastic, opts);
  }, formulation);
}

DeformationModelBundle makeCubicDeformationModel(
  const SimulationMesh &mesh,
  const CubicFormulationVariant &formulation,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const DeformationModelOptions &opts)
{
  return std::visit([&](const auto &f) {
    return makeCubicDeformationModel(mesh, f, elastic, plastic, opts);
  }, formulation);
}

DeformationModelBundle makeShellDeformationModel(
  const SimulationMesh &mesh,
  const ShellFormulationVariant &formulation,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const DeformationModelOptions &opts)
{
  return std::visit([&](const auto &f) {
    return makeShellDeformationModel(mesh, f, elastic, plastic, opts);
  }, formulation);
}

}  // namespace pgo::SolidDeformationModel
