#pragma once

#include "simulation/simulationMesh.h"
#include "simulation/simulationAsset.h"

#include <nanobind/nanobind.h>

#include <memory>
#include <string>
#include <stdexcept>

namespace pgo
{

namespace nb = nanobind;

class PyImportedMaterialRecord
{
public:
  explicit PyImportedMaterialRecord(SolidDeformationModel::ImportedMaterialRecord record):
    record_(std::move(record)) {}

  const std::string &name() const { return record_.name; }
  const std::string &family() const { return record_.family; }
  nb::dict properties() const
  {
    nb::dict result;
    for (const auto &[name, value] : record_.properties) {
      if (const auto *scalar = std::get_if<double>(&value))
        result[nb::str(name.c_str())] = nb::float_(*scalar);
      else
        result[nb::str(name.c_str())] = nb::cast(std::get<std::vector<double>>(value));
    }
    return result;
  }

private:
  SolidDeformationModel::ImportedMaterialRecord record_;
};

class PyImportedElementSet
{
public:
  explicit PyImportedElementSet(SolidDeformationModel::ImportedElementSet set):
    set_(std::move(set)) {}

  const std::string &name() const { return set_.name; }
  const std::vector<int> &elements() const { return set_.elements; }

private:
  SolidDeformationModel::ImportedElementSet set_;
};

class PyImportedMaterialRegion
{
public:
  explicit PyImportedMaterialRegion(SolidDeformationModel::ImportedMaterialRegion region):
    region_(region) {}

  int materialIndex() const { return region_.materialIndex; }
  int setIndex() const { return region_.setIndex; }

private:
  SolidDeformationModel::ImportedMaterialRegion region_;
};

class PyImportedMaterialField
{
public:
  explicit PyImportedMaterialField(SolidDeformationModel::ImportedMaterialField field):
    field_(std::move(field)) {}

  const std::string &name() const { return field_.name(); }
  std::vector<std::string> channelNames() const
  {
    const auto names = field_.channelNames();
    return std::vector<std::string>(names.begin(), names.end());
  }
  std::vector<int> elementToRow() const
  {
    const auto rows = field_.elementToRow();
    return std::vector<int>(rows.begin(), rows.end());
  }
  std::vector<std::vector<double>> valueRows() const
  {
    std::vector<std::vector<double>> result;
    result.reserve(static_cast<std::size_t>(field_.numRows()));
    for (int row = 0; row < field_.numRows(); ++row) {
      const auto values = field_.valueRows().row(row);
      result.emplace_back(values.data(), values.data() + values.size());
    }
    return result;
  }

private:
  SolidDeformationModel::ImportedMaterialField field_;
};

class PyImportedMaterialData
{
public:
  explicit PyImportedMaterialData(SolidDeformationModel::ImportedMaterialData data):
    data_(std::move(data)) {}

  int numElements() const { return data_.numElements(); }
  std::vector<PyImportedMaterialRecord> materials() const
  {
    std::vector<PyImportedMaterialRecord> result;
    for (const auto &material : data_.materials())
      result.emplace_back(material);
    return result;
  }
  std::vector<PyImportedElementSet> sets() const
  {
    std::vector<PyImportedElementSet> result;
    for (const auto &set : data_.sets())
      result.emplace_back(set);
    return result;
  }
  std::vector<PyImportedMaterialRegion> regions() const
  {
    std::vector<PyImportedMaterialRegion> result;
    for (const auto &region : data_.regions())
      result.emplace_back(region);
    return result;
  }
  std::vector<PyImportedMaterialField> fields() const
  {
    std::vector<PyImportedMaterialField> result;
    for (const auto &field : data_.fields())
      result.emplace_back(field);
    return result;
  }
  std::vector<int> elementMaterialIndices() const
  {
    return data_.elementMaterialIndices();
  }
  const SolidDeformationModel::ImportedMaterialData &data() const { return data_; }

private:
  SolidDeformationModel::ImportedMaterialData data_;
};

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

class PySimulationAsset final : public PySimulationMesh
{
public:
  explicit PySimulationAsset(std::shared_ptr<const SolidDeformationModel::SimulationAsset> asset):
    PySimulationMesh(asset ? asset->mesh() : nullptr), asset_(std::move(asset))
  {
    if (!asset_ || !mesh_)
      throw std::invalid_argument("PySimulationAsset requires a non-null asset and mesh.");
  }

  std::shared_ptr<const SolidDeformationModel::SimulationAsset> asset() const { return asset_; }
  PyImportedMaterialData materialData() const
  {
    return PyImportedMaterialData(asset_->materialData());
  }

private:
  std::shared_ptr<const SolidDeformationModel::SimulationAsset> asset_;
};

}  // namespace pgo
