#pragma once

#include "parameterDofLayout.h"
#include "parameterFieldMapping.h"
#include "EigenSupport.h"

#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

struct MaterialParameterSpec
{
  std::vector<std::string> channelNames;
};

class MaterialParameterRef;
class MaterialParameterSpace;
class MaterialStateView;

class MaterialParameterBlock
{
public:
  MaterialParameterBlock(
    std::vector<std::string> channelNames,
    std::shared_ptr<const ParameterDofLayout> dofLayout,
    std::shared_ptr<const ParameterFieldMapping> mapping);

  std::span<const std::string> channelNames() const { return channelNames_; }

  const ParameterDofLayout &dofLayout() const { return *dofLayout_; }
  const ParameterFieldMapping &mapping() const { return *mapping_; }

  MaterialParameterRef parameter(std::string_view name) const;

private:
  friend class MaterialParameterSpace;
  friend class MaterialStateView;

  std::vector<std::string> channelNames_;
  std::shared_ptr<const ParameterDofLayout> dofLayout_;
  std::shared_ptr<const ParameterFieldMapping> mapping_;
  const MaterialParameterSpace *space_ = nullptr;
};

class MaterialParameterSpace :
  public std::enable_shared_from_this<MaterialParameterSpace>
{
public:
  MaterialParameterSpace(
    MaterialParameterBlock elastic,
    MaterialParameterBlock plastic);

  const MaterialParameterBlock &elastic() const { return elastic_; }
  const MaterialParameterBlock &plastic() const { return plastic_; }

  MaterialStateView makeStateView(
    std::span<const double> elasticValues,
    std::span<const double> plasticValues) const;

private:
  MaterialParameterBlock elastic_;
  MaterialParameterBlock plastic_;
};

class MaterialState
{
public:
  MaterialState(
    std::shared_ptr<const MaterialParameterSpace> space,
    EigenSupport::VXd elasticValues,
    EigenSupport::VXd plasticValues);

  std::shared_ptr<const MaterialParameterSpace> space() const { return space_; }
  const EigenSupport::VXd &elasticValues() const { return elasticValues_; }
  const EigenSupport::VXd &plasticValues() const { return plasticValues_; }
  MaterialStateView view() const;

private:
  std::shared_ptr<const MaterialParameterSpace> space_;
  EigenSupport::VXd elasticValues_;
  EigenSupport::VXd plasticValues_;
};

class MaterialStateView
{
public:
  MaterialStateView() = default;

  const MaterialParameterSpace &space() const;
  bool empty() const { return space_ == nullptr; }

  std::span<const double> elasticValues() const { return elasticValues_; }
  std::span<const double> plasticValues() const { return plasticValues_; }
  std::span<const double> values(const MaterialParameterBlock &block) const;

private:
  friend class MaterialParameterSpace;

  MaterialStateView(
    const MaterialParameterSpace *space,
    std::span<const double> elasticValues,
    std::span<const double> plasticValues):
    space_(space),
    elasticValues_(elasticValues),
    plasticValues_(plasticValues)
  {
  }

  const MaterialParameterSpace *space_ = nullptr;
  std::span<const double> elasticValues_;
  std::span<const double> plasticValues_;
};

class MaterialParameters
{
public:
  MaterialParameters(
    std::shared_ptr<const MaterialParameterSpace> space,
    EigenSupport::VXd elasticValues,
    EigenSupport::VXd plasticValues);

  std::shared_ptr<const MaterialParameterSpace> space() const { return space_; }

  MaterialState snapshot() const;
  MaterialStateView committedView() const;
  EigenSupport::VXd elasticSnapshot() const { return elasticValues_; }
  EigenSupport::VXd plasticSnapshot() const { return plasticValues_; }

  void setElasticValues(EigenSupport::ConstRefVecXd values);
  void setPlasticValues(EigenSupport::ConstRefVecXd values);

private:
  std::shared_ptr<const MaterialParameterSpace> space_;
  EigenSupport::VXd elasticValues_;
  EigenSupport::VXd plasticValues_;
};

class MaterialParameterRef
{
public:
  MaterialParameterRef() = default;
  MaterialParameterRef(const MaterialParameterBlock &block, int channel);

  const MaterialParameterBlock &block() const;
  int channel() const { return channel_; }
  std::string_view name() const;

  double value(
    int element,
    int quadrature,
    MaterialStateView state) const;

  void localDerivative(
    int element,
    int quadrature,
    MaterialStateView state,
    double *output) const;

private:
  const MaterialParameterBlock *block_ = nullptr;
  int channel_ = -1;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
