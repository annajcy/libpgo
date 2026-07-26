#include "simulation/simulationMeshVolume.h"
#include "simulation/simulationMeshMaterial.h"

#include "cubicMesh.h"
#include "tetMesh.h"
#include "volumetricMeshENuMaterial.h"
#include "volumetricMeshMooneyRivlinMaterial.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

using namespace pgo::SolidDeformationModel;

namespace
{
template<class VolumeMesh>
ElementFieldStore makeVolumeMaterialField(
  const VolumeMesh &volumeMesh, const char *meshName)
{
  using Material = pgo::VolumetricMeshes::VolumetricMesh::Material;
  std::unordered_map<const Material *, int> paletteIndices;
  std::vector<int> elementToPalette;
  elementToPalette.reserve(volumeMesh.getNumElements());

  const Material *firstMaterial = volumeMesh.getElementMaterial(0);
  if (downcastENuMaterial(firstMaterial)) {
    std::vector<std::shared_ptr<const SimulationMeshENuMaterial>> palette;
    for (int element = 0; element < volumeMesh.getNumElements(); ++element) {
      const Material *source = volumeMesh.getElementMaterial(element);
      const auto *material = downcastENuMaterial(source);
      if (!material)
        throw std::invalid_argument(std::string(meshName) +
          " mesh mixes volume material families at element " + std::to_string(element));

      auto [it, inserted] = paletteIndices.emplace(
        source, static_cast<int>(palette.size()));
      if (inserted)
        palette.emplace_back(std::make_shared<const SimulationMeshENuMaterial>(
          material->getE(), material->getNu()));
      elementToPalette.push_back(it->second);
    }
    ElementFieldStore store;
    store.add(ElementField<SimulationMeshENuMaterial>::fromPalette(
      std::move(palette), std::move(elementToPalette)));
    return store;
  }

  if (!downcastMooneyRivlinMaterial(const_cast<Material *>(firstMaterial)))
    throw std::invalid_argument(std::string(meshName) +
      " mesh has an unsupported volume material at element 0");

  std::vector<std::shared_ptr<const SimulationMeshMooneyRivlinMaterial>> palette;
  for (int element = 0; element < volumeMesh.getNumElements(); ++element) {
    const Material *source = volumeMesh.getElementMaterial(element);
    auto *material = downcastMooneyRivlinMaterial(const_cast<Material *>(source));
    if (!material)
      throw std::invalid_argument(std::string(meshName) +
        " mesh mixes volume material families at element " + std::to_string(element));

    auto [it, inserted] = paletteIndices.emplace(
      source, static_cast<int>(palette.size()));
    if (inserted) {
      try {
        palette.emplace_back(std::make_shared<const SimulationMeshMooneyRivlinMaterial>(
          material->getmu01(), material->getmu10(), material->getv1()));
      }
      catch (const std::invalid_argument &error) {
        throw std::invalid_argument(std::string(meshName) +
          " mesh has invalid Mooney-Rivlin material at element " +
          std::to_string(element) + ": " + error.what());
      }
    }
    elementToPalette.push_back(it->second);
  }
  ElementFieldStore store;
  store.add(ElementField<SimulationMeshMooneyRivlinMaterial>::fromPalette(
    std::move(palette), std::move(elementToPalette)));
  return store;
}
}

std::unique_ptr<SimulationMesh> pgo::SolidDeformationModel::loadTetMesh(
  const VolumetricMeshes::TetMesh &tetMesh)
{
  std::vector<double> vertices;
  vertices.reserve(static_cast<std::size_t>(tetMesh.getNumVertices()) * 3);
  for (int vi = 0; vi < tetMesh.getNumVertices(); vi++) {
    Vec3d p = tetMesh.getVertex(vi);
    vertices.emplace_back(p[0]);
    vertices.emplace_back(p[1]);
    vertices.emplace_back(p[2]);
  }

  std::vector<int> elementVertices;
  elementVertices.reserve(static_cast<std::size_t>(tetMesh.getNumElements()) * 4);
  for (int ei = 0; ei < tetMesh.getNumElements(); ei++) {
    for (int j = 0; j < 4; j++)
      elementVertices.emplace_back(tetMesh.getVertexIndex(ei, j));
  }

  return std::make_unique<SimulationMesh>(
    tetMesh.getNumVertices(), vertices,
    tetMesh.getNumElements(), 4, elementVertices,
    makeVolumeMaterialField(tetMesh, "tet"), SimulationMeshType::TET);
}

std::unique_ptr<SimulationMesh> pgo::SolidDeformationModel::loadCubicMesh(
  const VolumetricMeshes::CubicMesh &cubicMesh)
{
  std::vector<double> vertices;
  vertices.reserve(static_cast<std::size_t>(cubicMesh.getNumVertices()) * 3);
  for (int vi = 0; vi < cubicMesh.getNumVertices(); vi++) {
    Vec3d p = cubicMesh.getVertex(vi);
    vertices.emplace_back(p[0]);
    vertices.emplace_back(p[1]);
    vertices.emplace_back(p[2]);
  }

  std::vector<int> elementVertices;
  elementVertices.reserve(static_cast<std::size_t>(cubicMesh.getNumElements()) * 8);
  for (int ei = 0; ei < cubicMesh.getNumElements(); ei++) {
    for (int j = 0; j < 8; j++)
      elementVertices.emplace_back(cubicMesh.getVertexIndex(ei, j));
  }

  return std::make_unique<SimulationMesh>(
    cubicMesh.getNumVertices(), vertices,
    cubicMesh.getNumElements(), 8, elementVertices,
    makeVolumeMaterialField(cubicMesh, "cubic"), SimulationMeshType::CUBIC);
}
