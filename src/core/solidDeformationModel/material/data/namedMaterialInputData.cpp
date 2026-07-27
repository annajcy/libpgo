#include "material/data/namedMaterialInputData.h"

#include <stdexcept>
#include <unordered_set>

namespace pgo::SolidDeformationModel
{

NamedMaterialInputField::NamedMaterialInputField(
  std::vector<std::string> channelNames,
  EigenSupport::MXd valueRows,
  std::vector<int> elementToRow,
  std::string name):
  name_(std::move(name)),
  channelNames_(std::move(channelNames)),
  valueRows_(std::move(valueRows)),
  elementToRow_(std::move(elementToRow))
{
  validate();
}

void NamedMaterialInputField::validate() const
{
  if (channelNames_.size() != static_cast<std::size_t>(valueRows_.cols()))
    throw std::invalid_argument(
      "NamedMaterialInputField channel count does not match value rows.");
  std::unordered_set<std::string> names;
  for (const auto &name : channelNames_) {
    if (name.empty() || !names.insert(name).second)
      throw std::invalid_argument(
        "NamedMaterialInputField channel names must be non-empty and unique.");
  }
  if (!valueRows_.allFinite())
    throw std::invalid_argument(
      "NamedMaterialInputField values must be finite.");
  for (const int row : elementToRow_) {
    if (row < -1 || row >= valueRows_.rows())
      throw std::invalid_argument(
        "NamedMaterialInputField element-to-row index is out of range.");
  }
}

int NamedMaterialInputField::rowForElement(int element) const
{
  if (element < 0 || element >= numElements())
    throw std::out_of_range(
      "NamedMaterialInputField element is out of range.");
  return elementToRow_[static_cast<std::size_t>(element)];
}

EigenSupport::VXd NamedMaterialInputField::values(int element) const
{
  const int row = rowForElement(element);
  if (row < 0)
    return {};
  return valueRows_.row(row).transpose();
}

NamedMaterialInputData::NamedMaterialInputData(
  int numElements,
  std::vector<NamedMaterialInputField> fields):
  numElements_(numElements), fields_(std::move(fields))
{
  if (numElements_ < 0)
    throw std::invalid_argument(
      "NamedMaterialInputData element count must be non-negative.");
  for (const auto &field : fields_) {
    if (field.numElements() != numElements_)
      throw std::invalid_argument(
        "NamedMaterialInputData field element count does not match the data domain.");
  }
}

}  // namespace pgo::SolidDeformationModel
