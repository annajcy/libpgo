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

#include "simulationMesh.h"

#include <memory>
#include <stdexcept>

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
DeformationModelBundle makeDeformationModelBundle(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const DeformationModelOptions &opts);
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
  return detail::makeDeformationModelBundle(mesh, elastic, plastic, opts);
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
  return detail::makeDeformationModelBundle(mesh, elastic, plastic, opts);
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
  return detail::makeDeformationModelBundle(mesh, elastic, plastic, opts);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
