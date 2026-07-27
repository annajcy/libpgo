#pragma once

#include "EigenSupport.h"

#include <span>
#include <string>
#include <vector>

namespace pgo::SolidDeformationModel
{

/// A model-neutral named spatial input. Values are row-major (row, channel);
/// elementToRow may contain -1 when an element has no supplied value.
class NamedMaterialInputField final
{
public:
  NamedMaterialInputField() = default;
  NamedMaterialInputField(
    std::vector<std::string> channelNames,
    EigenSupport::MXd valueRows,
    std::vector<int> elementToRow,
    std::string name = {});

  const std::string &name() const { return name_; }
  std::span<const std::string> channelNames() const { return channelNames_; }
  const EigenSupport::MXd &valueRows() const { return valueRows_; }
  std::span<const int> elementToRow() const { return elementToRow_; }
  int numElements() const { return static_cast<int>(elementToRow_.size()); }
  int numRows() const { return static_cast<int>(valueRows_.rows()); }
  int numChannels() const { return static_cast<int>(channelNames_.size()); }

  int rowForElement(int element) const;
  EigenSupport::VXd values(int element) const;

private:
  void validate() const;

  std::string name_;
  std::vector<std::string> channelNames_;
  EigenSupport::MXd valueRows_;
  std::vector<int> elementToRow_;
};

class NamedMaterialInputData final
{
public:
  NamedMaterialInputData() = default;
  NamedMaterialInputData(
    int numElements,
    std::vector<NamedMaterialInputField> fields);

  int numElements() const { return numElements_; }
  std::span<const NamedMaterialInputField> fields() const { return fields_; }

private:
  int numElements_ = 0;
  std::vector<NamedMaterialInputField> fields_;
};

}  // namespace pgo::SolidDeformationModel
