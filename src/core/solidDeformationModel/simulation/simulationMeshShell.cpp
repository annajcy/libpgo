#include "simulation/simulationMeshShell.h"

#include "triMeshGeo.h"
#include "triMeshNeighbor.h"

#include <memory>
#include <stdexcept>
#include <string_view>
#include <numeric>
#include <vector>

using namespace pgo::SolidDeformationModel;

namespace
{
std::unique_ptr<SimulationAsset> buildShellSimulationMesh(
  const pgo::Mesh::TriMeshGeo &triMeshGeo,
  ElementField<ImportedENuhMaterial> materials)
{
  const int numElements = triMeshGeo.numTriangles();
  if (materials.size() != numElements)
    throw std::invalid_argument(
      "shell material field size must match the triangle count");

  std::vector<double> vertices;
  vertices.reserve(static_cast<std::size_t>(triMeshGeo.numVertices()) * 3);
  for (int vi = 0; vi < triMeshGeo.numVertices(); vi++) {
    vertices.emplace_back(triMeshGeo.pos(vi)[0]);
    vertices.emplace_back(triMeshGeo.pos(vi)[1]);
    vertices.emplace_back(triMeshGeo.pos(vi)[2]);
  }

  std::vector<int> elementVertexIndices;
  elementVertexIndices.reserve(static_cast<std::size_t>(numElements) * 6);
  pgo::Mesh::TriMeshNeighbor triNeighbor(triMeshGeo);
  for (int element = 0; element < numElements; element++) {
    pgo::Vec3i triangle = triMeshGeo.tri(element);
    pgo::Vec3i neighborList = triNeighbor.getTriangleNeighbors(element);

    for (int j = 0; j < 3; j++)
      elementVertexIndices.emplace_back(triangle[j]);

    for (int j = 0; j < 3; j++) {
      int e0 = triangle[j];
      int e1 = triangle[(j + 1) % 3];
      if (neighborList[j] >= 0) {
        pgo::Vec3i neighbor = triMeshGeo.tri(neighborList[j]);
        elementVertexIndices.emplace_back(
          pgo::Mesh::getTriangleVertexOppositeEdge(neighbor, e0, e1));
      }
      else {
        elementVertexIndices.emplace_back(-1);
      }
    }
  }

  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(materials.size()) * 4);
  for (int element = 0; element < static_cast<int>(materials.size()); ++element) {
    values.push_back(materials.at(element).getE());
    values.push_back(materials.at(element).getNu());
    values.push_back(materials.at(element).geth());
    values.push_back(materials.at(element).getCompressionRatio());
  }
  auto mesh = std::make_shared<SimulationMesh>(
    triMeshGeo.numVertices(), vertices, numElements, 6,
    elementVertexIndices, SimulationMeshType::SHELL);
  pgo::EigenSupport::MXd rows(numElements, 4);
  for (int element = 0; element < numElements; ++element)
    for (int channel = 0; channel < 4; ++channel)
      rows(element, channel) = values[static_cast<std::size_t>(element) * 4 + channel];
  static constexpr std::string_view names[] = {"E", "nu", "h", "J"};
  std::vector<std::string> channelNames;
  for (const auto name : names)
    channelNames.emplace_back(name);
  std::vector<int> elementToRow(static_cast<std::size_t>(numElements));
  std::iota(elementToRow.begin(), elementToRow.end(), 0);
  auto materialData = ImportedMaterialData(
    numElements, {}, {}, {},
    {ImportedMaterialField(
      std::move(channelNames), std::move(rows), std::move(elementToRow), "shell")});
  return std::make_unique<SimulationAsset>(std::move(mesh), std::move(materialData));
}
}

std::unique_ptr<SimulationAsset> pgo::SolidDeformationModel::loadShellMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  const ImportedENuhMaterial &mat)
{
  return buildShellSimulationMesh(
    triMeshGeo, ElementField<ImportedENuhMaterial>::uniform(
      triMeshGeo.numTriangles(), mat));
}

std::unique_ptr<SimulationAsset> pgo::SolidDeformationModel::loadShellMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  ElementField<ImportedENuhMaterial> materials)
{
  return buildShellSimulationMesh(triMeshGeo, std::move(materials));
}
