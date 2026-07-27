#pragma once

#include <initializer_list>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace pgo::SolidDeformationModel
{

/// Names the parameter values stored in MaterialParameterData. The schema says
/// nothing about the physical channels produced by an evaluator.
class ParameterSchema final
{
public:
  ParameterSchema() = default;
  ParameterSchema(std::initializer_list<std::string_view> names);
  explicit ParameterSchema(std::span<const std::string_view> names);
  explicit ParameterSchema(std::vector<std::string> names);

  std::span<const std::string> parameterNames() const
  {
    return parameterNames_;
  }
  int numParameters() const
  {
    return static_cast<int>(parameterNames_.size());
  }

  bool contains(std::string_view parameterName) const;
  int parameterIndex(std::string_view parameterName) const;

private:
  std::vector<std::string> parameterNames_;
};

/// Names physical material channels consumed by a constitutive model.
class MaterialChannelSchema final
{
public:
  MaterialChannelSchema() = default;
  MaterialChannelSchema(
    std::initializer_list<std::string_view> channelNames);
  explicit MaterialChannelSchema(std::span<const std::string_view> channelNames);
  explicit MaterialChannelSchema(std::vector<std::string> channelNames);

  std::span<const std::string> channelNames() const { return channelNames_; }
  int numChannels() const { return static_cast<int>(channelNames_.size()); }

  bool contains(std::string_view channelName) const;
  int channelIndex(std::string_view channelName) const;

private:
  std::vector<std::string> channelNames_;
};

}  // namespace pgo::SolidDeformationModel
