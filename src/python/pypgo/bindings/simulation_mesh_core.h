#pragma once

#include "simulationMesh.h"

#include <memory>
#include <string>

namespace pgo
{

// Python-facing SimulationMesh owner. Holds a unique_ptr<SimulationMesh> and
// exposes a const reference so energy_bindings can borrow the mesh without
// consuming it.
class PySimulationMesh
{
public:
  explicit PySimulationMesh(std::unique_ptr<SolidDeformationModel::SimulationMesh> mesh)
    : mesh_(std::move(mesh)) {}

  const SolidDeformationModel::SimulationMesh &mesh() const { return *mesh_; }

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

private:
  std::unique_ptr<SolidDeformationModel::SimulationMesh> mesh_;
};

}  // namespace pgo
