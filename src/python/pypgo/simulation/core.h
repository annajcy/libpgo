#pragma once

#include "simulation/simulationMesh.h"

#include <nanobind/nanobind.h>

#include <memory>
#include <string>

namespace pgo
{

namespace nb = nanobind;

// Python-facing SimulationMesh owner. Holds a shared SimulationMesh handle so
// deformation energies and parameter-field configs retain the exact same mesh instance.
class PySimulationMesh
{
public:
  explicit PySimulationMesh(std::shared_ptr<const SolidDeformationModel::SimulationMesh> mesh)
    : mesh_(std::move(mesh)) {}

  const SolidDeformationModel::SimulationMesh &mesh() const { return *mesh_; }
  std::shared_ptr<const SolidDeformationModel::SimulationMesh> meshPtr() const { return mesh_; }

  std::string meshType() const
  {
    switch (mesh_->getElementType()) {
    case SolidDeformationModel::SimulationMeshType::TET:
      return "tet";
    case SolidDeformationModel::SimulationMeshType::CUBIC:
      return "cubic";
    case SolidDeformationModel::SimulationMeshType::SHELL:
      return "shell";
    case SolidDeformationModel::SimulationMeshType::TRIANGLE:
      return "triangle";
    case SolidDeformationModel::SimulationMeshType::EDGE_QUAD:
      return "edge_quad";
    }
    return "unknown";
  }

  int numVertices() const { return mesh_->getNumVertices(); }
  int numElements() const { return mesh_->getNumElements(); }
  int numElementVertices() const { return mesh_->getNumElementVertices(); }

protected:
  std::shared_ptr<const SolidDeformationModel::SimulationMesh> mesh_;
};

}  // namespace pgo
