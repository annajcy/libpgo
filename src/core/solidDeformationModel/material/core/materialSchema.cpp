#include "materialSchema.h"

#include <algorithm>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace
{
std::vector<std::string> ownNames(std::span<const std::string_view> names)
{
  std::vector<std::string> owned;
  owned.reserve(names.size());
  for (const std::string_view name : names)
    owned.emplace_back(name);
  return owned;
}

void validateNames(
  std::span<const std::string> names,
  std::string_view schemaName)
{
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (names[i].empty())
      throw std::invalid_argument(
        std::string(schemaName) + " names must be non-empty.");
    if (std::find(names.begin(), names.begin() + static_cast<std::ptrdiff_t>(i), names[i]) !=
      names.begin() + static_cast<std::ptrdiff_t>(i))
      throw std::invalid_argument(
        std::string(schemaName) + " names must be unique.");
  }
}

int findName(
  std::span<const std::string> names,
  std::string_view requested,
  std::string_view schemaName)
{
  const auto iter = std::find(names.begin(), names.end(), requested);
  if (iter == names.end())
    throw std::invalid_argument(
      std::string(schemaName) + " has no name '" +
      std::string(requested) + "'.");
  return static_cast<int>(std::distance(names.begin(), iter));
}
}  // namespace

ParameterSchema::ParameterSchema(
  std::initializer_list<std::string_view> names):
  ParameterSchema(std::span<const std::string_view>(
    names.begin(), names.size()))
{
}

ParameterSchema::ParameterSchema(
  std::span<const std::string_view> names):
  ParameterSchema(ownNames(names))
{
}

ParameterSchema::ParameterSchema(
  std::vector<std::string> names):
  parameterNames_(std::move(names))
{
  validateNames(parameterNames_, "ParameterSchema");
}

bool ParameterSchema::contains(std::string_view parameterName) const
{
  return std::find(
           parameterNames_.begin(), parameterNames_.end(), parameterName) !=
    parameterNames_.end();
}

int ParameterSchema::parameterIndex(std::string_view parameterName) const
{
  return findName(parameterNames_, parameterName, "ParameterSchema");
}

MaterialChannelSchema::MaterialChannelSchema(
  std::initializer_list<std::string_view> channelNames):
  MaterialChannelSchema(std::span<const std::string_view>(
    channelNames.begin(), channelNames.size()))
{
}

MaterialChannelSchema::MaterialChannelSchema(
  std::span<const std::string_view> channelNames):
  MaterialChannelSchema(ownNames(channelNames))
{
}

MaterialChannelSchema::MaterialChannelSchema(
  std::vector<std::string> channelNames):
  channelNames_(std::move(channelNames))
{
  validateNames(channelNames_, "MaterialChannelSchema");
}

bool MaterialChannelSchema::contains(std::string_view channelName) const
{
  return std::find(
           channelNames_.begin(), channelNames_.end(), channelName) !=
    channelNames_.end();
}

int MaterialChannelSchema::channelIndex(
  std::string_view channelName) const
{
  return findName(channelNames_, channelName, "MaterialChannelSchema");
}

}  // namespace pgo::SolidDeformationModel
