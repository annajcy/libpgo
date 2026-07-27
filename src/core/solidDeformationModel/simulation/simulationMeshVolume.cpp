#include "simulation/simulationMeshVolume.h"
#include "simulation/importedMaterial.h"

#include "cubicMesh.h"
#include "tetMesh.h"
#include "volumetricMeshENuMaterial.h"
#include "volumetricMeshMooneyRivlinMaterial.h"
#include "volumetricMeshOrthotropicMaterial.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace pgo::SolidDeformationModel;

namespace
{
template<class VolumeMesh>
ImportedMaterialData makeVolumeMaterialData(const VolumeMesh &volumeMesh)
{
  using VM = pgo::VolumetricMeshes::VolumetricMesh;
  using Record = ImportedMaterialRecord;
  std::vector<Record> materials;
  materials.reserve(static_cast<std::size_t>(volumeMesh.getNumMaterials()));

  for (int materialIndex = 0; materialIndex < volumeMesh.getNumMaterials(); ++materialIndex) {
    const VM::Material *source = volumeMesh.getMaterial(materialIndex);
    Record record;
    record.name = source->getName();
    record.properties.emplace("density", source->getDensity());

    if (const auto *enu = downcastENuMaterial(source)) {
      record.family = "enu";
      record.properties.emplace("E", enu->getE());
      record.properties.emplace("nu", enu->getNu());
    }
    else if (auto *mr = downcastMooneyRivlinMaterial(const_cast<VM::Material *>(source))) {
      record.family = "mooney_rivlin";
      record.properties.emplace("mu01", mr->getmu01());
      record.properties.emplace("mu10", mr->getmu10());
      record.properties.emplace("v1", mr->getv1());
    }
    else if (auto *orthotropic = downcastOrthotropicMaterial(const_cast<VM::Material *>(source))) {
      record.family = "orthotropic";
      record.properties.emplace("E1", orthotropic->getE1());
      record.properties.emplace("E2", orthotropic->getE2());
      record.properties.emplace("E3", orthotropic->getE3());
      record.properties.emplace("nu12", orthotropic->getNu12());
      record.properties.emplace("nu23", orthotropic->getNu23());
      record.properties.emplace("nu31", orthotropic->getNu31());
      record.properties.emplace("G12", orthotropic->getG12());
      record.properties.emplace("G23", orthotropic->getG23());
      record.properties.emplace("G31", orthotropic->getG31());
      std::vector<double> rotation(9);
      orthotropic->getR(rotation.data());
      record.properties.emplace("rotation", std::move(rotation));
    }
    else {
      throw std::invalid_argument(
        "Volume mesh contains an unsupported material type at index " +
        std::to_string(materialIndex));
    }
    materials.emplace_back(std::move(record));
  }

  std::vector<ImportedElementSet> sets;
  sets.reserve(static_cast<std::size_t>(volumeMesh.getNumSets()));
  for (int setIndex = 0; setIndex < volumeMesh.getNumSets(); ++setIndex) {
    const auto &source = volumeMesh.getSet(setIndex);
    sets.push_back(ImportedElementSet{
      source.getName(),
      std::vector<int>(source.getElements().begin(), source.getElements().end())});
  }

  std::vector<ImportedMaterialRegion> regions;
  regions.reserve(static_cast<std::size_t>(volumeMesh.getNumRegions()));
  for (int regionIndex = 0; regionIndex < volumeMesh.getNumRegions(); ++regionIndex) {
    const auto &source = volumeMesh.getRegion(regionIndex);
    regions.push_back(ImportedMaterialRegion{
      source.getMaterialIndex(), source.getSetIndex()});
  }

  // Programmatically-created volume meshes may expose direct element
  // material pointers without explicit regions. Preserve that assignment as
  // material-index element sets instead of dropping it during import.
  if (regions.empty()) {
    for (int materialIndex = 0; materialIndex < volumeMesh.getNumMaterials(); ++materialIndex) {
      const auto *material = volumeMesh.getMaterial(materialIndex);
      std::vector<int> elements;
      for (int element = 0; element < volumeMesh.getNumElements(); ++element)
        if (volumeMesh.getElementMaterial(element) == material)
          elements.push_back(element);
      if (elements.empty())
        continue;
      const int setIndex = static_cast<int>(sets.size());
      sets.push_back(ImportedElementSet{
        "material_" + std::to_string(materialIndex), std::move(elements)});
      regions.push_back(ImportedMaterialRegion{materialIndex, setIndex});
    }
  }

  return ImportedMaterialData(
    volumeMesh.getNumElements(), std::move(materials),
    std::move(sets), std::move(regions));
}

}

std::unique_ptr<SimulationAsset> pgo::SolidDeformationModel::loadTetMesh(
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

  auto mesh = std::make_shared<SimulationMesh>(
    tetMesh.getNumVertices(), vertices,
    tetMesh.getNumElements(), 4, elementVertices,
    SimulationMeshType::TET);
  return std::make_unique<SimulationAsset>(
    std::move(mesh), makeVolumeMaterialData(tetMesh));
}

std::unique_ptr<SimulationAsset> pgo::SolidDeformationModel::loadCubicMesh(
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

  auto mesh = std::make_shared<SimulationMesh>(
    cubicMesh.getNumVertices(), vertices,
    cubicMesh.getNumElements(), 8, elementVertices,
    SimulationMeshType::CUBIC);
  return std::make_unique<SimulationAsset>(
    std::move(mesh), makeVolumeMaterialData(cubicMesh));
}
