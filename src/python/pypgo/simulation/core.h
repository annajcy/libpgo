#pragma once

#include "simulation/simulationMesh.h"
#include "simulation/import/simulationImportResult.h"
#include "material/data/namedMaterialInputData.h"

#include <nanobind/nanobind.h>

#include <memory>
#include <string>
#include <stdexcept>
#include <vector>

namespace pgo
{

namespace nb = nanobind;

class PyImportedMaterialRecord
{
public:
  PyImportedMaterialRecord(std::string name, std::string family, nb::dict properties):
    record_{std::move(name), std::move(family), {}}
  {
    for (const auto &item : properties) {
      const std::string key = nb::cast<std::string>(item.first);
      const nb::handle value = item.second;
      if (nb::isinstance<nb::float_>(value) || nb::isinstance<nb::int_>(value))
        record_.properties[key] = nb::cast<double>(value);
      else
        record_.properties[key] = nb::cast<std::vector<double>>(value);
    }
  }
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
  const SolidDeformationModel::ImportedMaterialRecord &record() const { return record_; }

private:
  SolidDeformationModel::ImportedMaterialRecord record_;
};

class PyImportedElementSet
{
public:
  PyImportedElementSet(std::string name, std::vector<int> elements):
    set_{std::move(name), std::move(elements)} {}
  explicit PyImportedElementSet(SolidDeformationModel::ImportedElementSet set):
    set_(std::move(set)) {}

  const std::string &name() const { return set_.name; }
  const std::vector<int> &elements() const { return set_.elements; }
  const SolidDeformationModel::ImportedElementSet &set() const { return set_; }

private:
  SolidDeformationModel::ImportedElementSet set_;
};

class PyImportedMaterialRegion
{
public:
  PyImportedMaterialRegion(int materialIndex, int setIndex):
    region_{materialIndex, setIndex} {}
  explicit PyImportedMaterialRegion(SolidDeformationModel::ImportedMaterialRegion region):
    region_(region) {}

  int materialIndex() const { return region_.materialIndex; }
  int setIndex() const { return region_.setIndex; }
  const SolidDeformationModel::ImportedMaterialRegion &region() const { return region_; }

private:
  SolidDeformationModel::ImportedMaterialRegion region_;
};

class PyNamedMaterialInputField
{
public:
  PyNamedMaterialInputField(
    std::string name,
    std::vector<std::string> channelNames,
    std::vector<std::vector<double>> valueRows,
    std::vector<int> elementToRow):
    field_(std::move(channelNames), toMatrix(std::move(valueRows)),
      std::move(elementToRow), std::move(name)) {}
  explicit PyNamedMaterialInputField(SolidDeformationModel::NamedMaterialInputField field):
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
  const SolidDeformationModel::NamedMaterialInputField &field() const { return field_; }

private:
  static EigenSupport::MXd toMatrix(std::vector<std::vector<double>> rows)
  {
    const int numRows = static_cast<int>(rows.size());
    const int numChannels = numRows == 0 ? 0 : static_cast<int>(rows.front().size());
    EigenSupport::MXd result(numRows, numChannels);
    for (int row = 0; row < numRows; ++row) {
      if (static_cast<int>(rows[static_cast<std::size_t>(row)].size()) != numChannels)
        throw std::invalid_argument("NamedMaterialInputField rows must have equal lengths.");
      for (int col = 0; col < numChannels; ++col)
        result(row, col) = rows[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)];
    }
    return result;
  }

  SolidDeformationModel::NamedMaterialInputField field_;
};

class PyImportedMaterialCatalog
{
public:
  PyImportedMaterialCatalog(
    int numElements,
    const std::vector<PyImportedMaterialRecord> &materials,
    const std::vector<PyImportedElementSet> &sets,
    const std::vector<PyImportedMaterialRegion> &regions):
    data_(
      numElements, records(materials), elementSets(sets),
      materialRegions(regions)) {}
  explicit PyImportedMaterialCatalog(SolidDeformationModel::ImportedMaterialCatalog data):
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
  std::vector<int> elementMaterialIndices() const
  {
    return data_.elementMaterialIndices();
  }
  const SolidDeformationModel::ImportedMaterialCatalog &data() const { return data_; }

private:
  static std::vector<SolidDeformationModel::ImportedMaterialRecord> records(
    const std::vector<PyImportedMaterialRecord> &values)
  {
    std::vector<SolidDeformationModel::ImportedMaterialRecord> result;
    result.reserve(values.size());
    for (const auto &value : values) result.push_back(value.record());
    return result;
  }
  static std::vector<SolidDeformationModel::ImportedElementSet> elementSets(
    const std::vector<PyImportedElementSet> &values)
  {
    std::vector<SolidDeformationModel::ImportedElementSet> result;
    result.reserve(values.size());
    for (const auto &value : values) result.push_back(value.set());
    return result;
  }
  static std::vector<SolidDeformationModel::ImportedMaterialRegion> materialRegions(
    const std::vector<PyImportedMaterialRegion> &values)
  {
    std::vector<SolidDeformationModel::ImportedMaterialRegion> result;
    result.reserve(values.size());
    for (const auto &value : values) result.push_back(value.region());
    return result;
  }
  SolidDeformationModel::ImportedMaterialCatalog data_;
};

class PyNamedMaterialInputData
{
public:
  PyNamedMaterialInputData(
    int numElements,
    const std::vector<PyNamedMaterialInputField> &fields):
    data_(numElements, materialFields(fields))
  {
  }
  explicit PyNamedMaterialInputData(
    SolidDeformationModel::NamedMaterialInputData data):
    data_(std::move(data))
  {
  }

  int numElements() const { return data_.numElements(); }
  std::vector<PyNamedMaterialInputField> fields() const
  {
    std::vector<PyNamedMaterialInputField> result;
    for (const auto &field : data_.fields())
      result.emplace_back(field);
    return result;
  }
  const SolidDeformationModel::NamedMaterialInputData &data() const
  {
    return data_;
  }

private:
  static std::vector<SolidDeformationModel::NamedMaterialInputField>
  materialFields(const std::vector<PyNamedMaterialInputField> &values)
  {
    std::vector<SolidDeformationModel::NamedMaterialInputField> result;
    result.reserve(values.size());
    for (const auto &value : values)
      result.push_back(value.field());
    return result;
  }

  SolidDeformationModel::NamedMaterialInputData data_;
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

class PySimulationImportResult final
{
public:
  explicit PySimulationImportResult(
    std::shared_ptr<const SolidDeformationModel::SimulationImportResult> result):
    result_(std::move(result))
  {
    if (!result_ || !result_->mesh())
      throw std::invalid_argument(
        "PySimulationImportResult requires a non-null result and mesh.");
  }

  std::shared_ptr<const SolidDeformationModel::SimulationImportResult> result() const
  {
    return result_;
  }
  std::shared_ptr<PySimulationMesh> mesh() const
  {
    return std::make_shared<PySimulationMesh>(result_->mesh());
  }
  PyImportedMaterialCatalog materialCatalog() const
  {
    return PyImportedMaterialCatalog(result_->materialCatalog());
  }

private:
  std::shared_ptr<const SolidDeformationModel::SimulationImportResult> result_;
};

}  // namespace pgo
