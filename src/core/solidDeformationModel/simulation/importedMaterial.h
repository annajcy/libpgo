#pragma once

#include <cmath>
#include <algorithm>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <span>
#include <unordered_map>
#include <variant>
#include <vector>

#include "EigenSupport.h"

namespace pgo::SolidDeformationModel
{

/// A value copied from an import format without assigning it a constitutive
/// meaning.  Scalar properties cover the material constants used by the
/// built-in Vega materials; vector properties cover structured values such as
/// an orthotropic material frame.
using ImportedPropertyValue = std::variant<double, std::vector<double>>;

struct ImportedMaterialRecord
{
  std::string name;
  std::string family;
  std::unordered_map<std::string, ImportedPropertyValue> properties;
};

struct ImportedElementSet
{
  std::string name;
  std::vector<int> elements;
};

struct ImportedMaterialRegion
{
  int materialIndex = -1;
  int setIndex = -1;
};

/// An independent spatial field imported alongside material records.  Values
/// are row-major (row, channel); elementToRow may contain -1 for an element
/// for which the source has no value.
class ImportedMaterialField final
{
public:
  ImportedMaterialField() = default;
  ImportedMaterialField(
    std::vector<std::string> channelNames,
    EigenSupport::MXd valueRows,
    std::vector<int> elementToRow,
    std::string name = {}):
    name_(std::move(name)),
    channelNames_(std::move(channelNames)),
    valueRows_(std::move(valueRows)),
    elementToRow_(std::move(elementToRow))
  {
    validate();
  }

  const std::string &name() const { return name_; }
  std::span<const std::string> channelNames() const { return channelNames_; }
  const EigenSupport::MXd &valueRows() const { return valueRows_; }
  std::span<const int> elementToRow() const { return elementToRow_; }
  int numElements() const { return static_cast<int>(elementToRow_.size()); }
  int numRows() const { return static_cast<int>(valueRows_.rows()); }
  int numChannels() const { return static_cast<int>(channelNames_.size()); }

  int rowForElement(int element) const
  {
    if (element < 0 || element >= numElements())
      throw std::out_of_range("ImportedMaterialField element is out of range.");
    return elementToRow_[static_cast<std::size_t>(element)];
  }

  EigenSupport::VXd values(int element) const
  {
    const int row = rowForElement(element);
    if (row < 0)
      return {};
    return valueRows_.row(row).transpose();
  }

private:
  void validate() const;

  std::string name_;
  std::vector<std::string> channelNames_;
  EigenSupport::MXd valueRows_;
  std::vector<int> elementToRow_;
};

/// Lossless, model-neutral material payload imported from a source mesh.
class ImportedMaterialData final
{
public:
  ImportedMaterialData() = default;
  ImportedMaterialData(
    int numElements,
    std::vector<ImportedMaterialRecord> materials,
    std::vector<ImportedElementSet> sets,
    std::vector<ImportedMaterialRegion> regions,
    std::vector<ImportedMaterialField> fields = {}):
    numElements_(numElements),
    materials_(std::move(materials)),
    sets_(std::move(sets)),
    regions_(std::move(regions)),
    fields_(std::move(fields))
  {
    validate();
  }

  int numElements() const { return numElements_; }
  std::span<const ImportedMaterialRecord> materials() const { return materials_; }
  std::span<const ImportedElementSet> sets() const { return sets_; }
  std::span<const ImportedMaterialRegion> regions() const { return regions_; }
  std::span<const ImportedMaterialField> fields() const { return fields_; }

  /// Resolve material assignment using the source format's ordered-region
  /// semantics.  Later regions overwrite earlier regions.
  std::vector<int> elementMaterialIndices() const;

private:
  void validate() const;

  int numElements_ = 0;
  std::vector<ImportedMaterialRecord> materials_;
  std::vector<ImportedElementSet> sets_;
  std::vector<ImportedMaterialRegion> regions_;
  std::vector<ImportedMaterialField> fields_;
};

/// Source material payloads accepted by geometry loaders. They are not
/// stored by SimulationMesh; loaders translate them into ImportedMaterialData.
class ImportedENuMaterial
{
public:
  ImportedENuMaterial() = default;
  ImportedENuMaterial(double E_, double nu_, double J_ = 10000): E(E_), nu(nu_), J(J_) {}

  double getMuLame() const { return E / (2 * (1 + nu)); }
  double getLambdaLame() const { return (nu * E) / ((1 + nu) * (1 - 2 * nu)); }
  double getE() const { return E; }
  double getNu() const { return nu; }
  double getCompressionRatio() const { return J; }

private:
  double E = 6e3, nu = 0.4, J = 10000;
};

class ImportedENuhMaterial : public ImportedENuMaterial
{
public:
  ImportedENuhMaterial() = default;
  ImportedENuhMaterial(double E_, double nu_, double h_, double J_ = 10000):
    ImportedENuMaterial(E_, nu_, J_), h(h_) {}
  double geth() const { return h; }

private:
  double h = 1e-4;
};

class ImportedHillMaterial
{
public:
  ImportedHillMaterial() = default;
  ImportedHillMaterial(double E_act_, double gamma_, double lo_):
    E_act(E_act_), gamma(gamma_), lo(lo_) {}
  double getEact() const { return E_act; }
  double getGamma() const { return gamma; }
  double getLo() const { return lo; }

private:
  double E_act = 0.1e6, gamma = 1, lo = 0.6;
};

class ImportedMooneyRivlinMaterial
{
public:
  ImportedMooneyRivlinMaterial(double mu01_, double mu10_, double v1_):
    mu01Value(mu01_), mu10Value(mu10_), v1Value(v1_)
  {
    if (!std::isfinite(mu01Value) || !std::isfinite(mu10Value) || !std::isfinite(v1Value))
      throw std::invalid_argument("Mooney-Rivlin parameters must be finite");
    if (mu01Value + mu10Value <= 0.0)
      throw std::invalid_argument("Mooney-Rivlin requires mu01 + mu10 > 0");
    if (v1Value <= 0.0)
      throw std::invalid_argument("Mooney-Rivlin requires v1 > 0");
  }

  double mu01() const { return mu01Value; }
  double mu10() const { return mu10Value; }
  double v1() const { return v1Value; }

private:
  double mu01Value;
  double mu10Value;
  double v1Value;
};

}  // namespace pgo::SolidDeformationModel
