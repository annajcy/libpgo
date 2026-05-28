/*
author: Bohan Wang
copyright to USC
*/

#pragma once

#include "deformationModelManager.h"  // DeformationModelElasticMaterial / DeformationModelPlasticMaterial
#include "EigenSupport.h"

#include "formulations/deformationFormulations.h"
#include "formulations/formulationTraits.h"
#include "formulations/formulationConcepts.h"
#include "formulations/formulationVariants.h"

#include "factories/elementModelFactory.h"
#include "factories/elasticModelFactory.h"
#include "factories/plasticModelFactory.h"

#include "simulationMesh.h"
#include "deformationModelAssembler.h"
#include "deformationModelEnergy.h"

#include "pgoLogging.h"

#include <memory>
#include <stdexcept>
#include <string_view>

namespace pgo
{
namespace VolumetricMeshes
{
class VolumetricMesh;
}  // namespace VolumetricMeshes

namespace SolidDeformationModel
{
class SimulationMesh;
class DeformationModelEnergy;

struct DeformationModelOptions
{
  bool enforceSPD = true;
  bool enableMaterialMaxStep = true;
  // Per-element assembler weights; empty means all ones.
  EigenSupport::VXd elementWeights;
};

// One ready-to-use FEM deformation model. The SimulationMesh is borrowed by the
// energy chain (energy -> assembler -> manager), so callers must keep it alive.
struct DeformationModelBundle
{
  std::shared_ptr<DeformationModelEnergy> energy;
  // Snapshots for initializing caller-owned solver state. DeformationModelEnergy
  // stores its own copies, so mutating these fields does not update the energy.
  EigenSupport::VXd restPosition;
  EigenSupport::VXd plasticParams;
  EigenSupport::VXd elasticParams;
};

// Build a SimulationMesh from a volumetric mesh. The caller owns the returned mesh
// and must keep it alive while any deformation energy built from it is alive.
std::unique_ptr<SimulationMesh> makeSimulationMesh(const VolumetricMeshes::VolumetricMesh &mesh);

// Topology-specific factories from existing SimulationMesh objects.
template<TetFormulation F>
DeformationModelBundle makeTetDeformationModel(
  const SimulationMesh &mesh,
  const F &formulation,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic = DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
  const DeformationModelOptions &opts = {});

template<CubicFormulation F>
DeformationModelBundle makeCubicDeformationModel(
  const SimulationMesh &mesh,
  const F &formulation,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic = DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
  const DeformationModelOptions &opts = {});

template<ShellFormulation F>
DeformationModelBundle makeShellDeformationModel(
  const SimulationMesh &mesh,
  const F &formulation,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic = DeformationModelPlasticMaterial::SHELL_FF_DOF1,
  const DeformationModelOptions &opts = {});

// Runtime variant adapters for Python/config/CLI boundaries.
DeformationModelBundle makeTetDeformationModel(
  const SimulationMesh &mesh,
  const TetFormulationVariant &formulation,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic = DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
  const DeformationModelOptions &opts = {});

DeformationModelBundle makeCubicDeformationModel(
  const SimulationMesh &mesh,
  const CubicFormulationVariant &formulation,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic = DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
  const DeformationModelOptions &opts = {});

DeformationModelBundle makeShellDeformationModel(
  const SimulationMesh &mesh,
  const ShellFormulationVariant &formulation,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic = DeformationModelPlasticMaterial::SHELL_FF_DOF1,
  const DeformationModelOptions &opts = {});

namespace detail
{
// Legacy non-template path — creates element FEMs via DeformationModelManager::initImpl
// which uses legacy TetMeshDeformationModel / CubicMeshDeformationModel wrappers.
DeformationModelBundle makeDeformationModelBundle(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const DeformationModelOptions &opts);

// Formulation-aware template path — uses ElementModelFactory to create
// DeformationGradientElementModel<Kernel> instances directly and factory
// helpers for default param snapshots.
template<class Formulation>
DeformationModelBundle makeDeformationModelBundle(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const DeformationModelOptions &opts)
{
  using Traits = FormulationTraits<Formulation>;
  const int nele = mesh.getNumElements();
  const int n3 = mesh.getNumVertices() * 3;

  SPDLOG_LOGGER_INFO(pgo::Logging::lgr(), "Building deformation energy with formulation: {}",
    Traits::name);

  // Rest position snapshot.
  ES::VXd restPosition(n3);
  for (int vi = 0; vi < mesh.getNumVertices(); vi++) {
    double p[3];
    mesh.getVertex(vi, p);
    restPosition.segment<3>(vi * 3) = ES::V3d(p[0], p[1], p[2]);
  }

  // Build manager via legacy path (uses factories internally for elastic/plastic).
  auto manager = std::make_unique<DeformationModelManager>(
    mesh, plastic, elastic,
    opts.enforceSPD ? 1 : 0,
    /*elementFiberDirections=*/nullptr,
    /*vertexFiberDirections=*/nullptr);

  // Replace element FEMs with formulation-specific ones from ElementModelFactory.
  for (int ele = 0; ele < nele; ele++) {
    auto *em = const_cast<ElasticModel *>(manager->getDeformationModel(ele)->getElasticModel());
    auto *pm = const_cast<PlasticModel *>(manager->getDeformationModel(ele)->getPlasticModel());
    auto *newFEM = ElementModelFactory::create<Formulation>(mesh, ele, em, pm, elastic);
    manager->setDeformationModel(ele, newFEM);
  }

  // Element weights.
  ES::VXd elementWeights = opts.elementWeights;
  if (elementWeights.size() == 0)
    elementWeights = ES::VXd::Ones(nele);
  else if (static_cast<int>(elementWeights.size()) != nele)
    throw std::invalid_argument("makeDeformationModelBundle: elementWeights size does not match the element count.");

  // Default param snapshots using factory helpers.
  const int numPlasticParams = manager->getNumPlasticParameters();
  std::vector<PlasticModel *> plasticModels(nele);
  for (int ei = 0; ei < nele; ei++)
    plasticModels[ei] = const_cast<PlasticModel *>(manager->getDeformationModel(ei)->getPlasticModel());
  ES::VXd plasticParams = PlasticModelFactory::initializeDefaultPlasticParams(
    nele, numPlasticParams, plasticModels.data());

  const int numElasticParams = manager->getNumElasticParameters();
  ES::VXd elasticParams = ElasticModelFactory::initializeDefaultElasticParams(
    mesh, elastic, numElasticParams);

  // Assemble.
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

template<TetFormulation F>
DeformationModelBundle makeTetDeformationModel(
  const SimulationMesh &mesh,
  const F &,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const DeformationModelOptions &opts)
{
  if (mesh.getElementType() != SimulationMeshType::TET)
    throw std::invalid_argument("makeTetDeformationModel: SimulationMesh is not TET topology.");
  return detail::makeDeformationModelBundle<F>(mesh, elastic, plastic, opts);
}

template<CubicFormulation F>
DeformationModelBundle makeCubicDeformationModel(
  const SimulationMesh &mesh,
  const F &,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const DeformationModelOptions &opts)
{
  if (mesh.getElementType() != SimulationMeshType::CUBIC)
    throw std::invalid_argument("makeCubicDeformationModel: SimulationMesh is not CUBIC topology.");
  return detail::makeDeformationModelBundle<F>(mesh, elastic, plastic, opts);
}

template<ShellFormulation F>
DeformationModelBundle makeShellDeformationModel(
  const SimulationMesh &mesh,
  const F &,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const DeformationModelOptions &opts)
{
  if (mesh.getElementType() != SimulationMeshType::SHELL)
    throw std::invalid_argument("makeShellDeformationModel: SimulationMesh is not SHELL topology.");
  return detail::makeDeformationModelBundle<F>(mesh, elastic, plastic, opts);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
