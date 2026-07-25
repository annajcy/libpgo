#pragma once

#include "parameterDofLayout.h"
#include "materialChannelMapping.h"
#include "EigenSupport.h"

#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

class MaterialParameterField;
class MaterialParameterRef;
class MaterialParameterSpace;
class MaterialParameterSnapshot;
class MaterialParameterEvaluationView;
struct MaterialParameterEvaluationScratch;

/// Immutable schema for one elastic or plastic material parameter field.
/// Instances are created through create() and shared by parameter references.
class MaterialParameterField final:
  public std::enable_shared_from_this<MaterialParameterField>
{
public:
  MaterialParameterField(const MaterialParameterField &) = delete;
  MaterialParameterField &operator=(const MaterialParameterField &) = delete;
  MaterialParameterField(MaterialParameterField &&) = delete;
  MaterialParameterField &operator=(MaterialParameterField &&) = delete;

  static std::shared_ptr<const MaterialParameterField> create(
    std::vector<std::string> channelNames,
    std::shared_ptr<const ParameterDofLayout> dofLayout,
    std::shared_ptr<const MaterialChannelMapping> mapping);

  std::span<const std::string> channelNames() const { return channelNames_; }
  const ParameterDofLayout &dofLayout() const { return *dofLayout_; }
  const MaterialChannelMapping &channelMapping() const { return *mapping_; }

  MaterialParameterRef parameter(std::string_view name) const;

private:
  MaterialParameterField(
    std::vector<std::string> channelNames,
    std::shared_ptr<const ParameterDofLayout> dofLayout,
    std::shared_ptr<const MaterialChannelMapping> mapping);

  std::vector<std::string> channelNames_;
  std::shared_ptr<const ParameterDofLayout> dofLayout_;
  std::shared_ptr<const MaterialChannelMapping> mapping_;
};

/// Reusable storage for sampling one material parameter field.  The storage
/// is intentionally owned by the caller so a batch of element evaluations can
/// reuse the same allocations.
struct MaterialParameterEvaluationScratch
{
  std::vector<double> local;
  std::vector<double> material;
  std::vector<double> jacobian;

  void prepare(const MaterialParameterField &field);
};

/// Immutable pair of elastic and plastic parameter field schemas.
class MaterialParameterSpace final
{
public:
  MaterialParameterSpace(
    std::shared_ptr<const MaterialParameterField> elastic,
    std::shared_ptr<const MaterialParameterField> plastic);

  MaterialParameterSpace(const MaterialParameterSpace &) = delete;
  MaterialParameterSpace &operator=(const MaterialParameterSpace &) = delete;
  MaterialParameterSpace(MaterialParameterSpace &&) = delete;
  MaterialParameterSpace &operator=(MaterialParameterSpace &&) = delete;

  const MaterialParameterField &elastic() const { return *elastic_; }
  const MaterialParameterField &plastic() const { return *plastic_; }
  const std::shared_ptr<const MaterialParameterField> &elasticHandle() const { return elastic_; }
  const std::shared_ptr<const MaterialParameterField> &plasticHandle() const { return plastic_; }

private:
  std::shared_ptr<const MaterialParameterField> elastic_;
  std::shared_ptr<const MaterialParameterField> plastic_;
};

/// Internal immutable committed state snapshot. It owns both value vectors,
/// so a view derived from it remains valid while evaluation is in progress.
class MaterialParameterSnapshot
{
public:
  MaterialParameterSnapshot() = default;

  std::shared_ptr<const MaterialParameterSpace> space() const { return space_; }
  bool empty() const { return !space_; }
  const EigenSupport::VXd &elasticValues() const { return *elasticValues_; }
  const EigenSupport::VXd &plasticValues() const { return *plasticValues_; }

  MaterialParameterEvaluationView view() const;
  MaterialParameterEvaluationView withElasticValues(
    std::span<const double> elasticValues) const;
  MaterialParameterEvaluationView withPlasticValues(
    std::span<const double> plasticValues) const;
  MaterialParameterEvaluationView withValues(
    std::span<const double> elasticValues,
    std::span<const double> plasticValues) const;

private:
  friend class MaterialParameters;

  MaterialParameterSnapshot(
    std::shared_ptr<const MaterialParameterSpace> space,
    std::shared_ptr<const EigenSupport::VXd> elasticValues,
    std::shared_ptr<const EigenSupport::VXd> plasticValues);

  std::shared_ptr<const MaterialParameterSpace> space_;
  std::shared_ptr<const EigenSupport::VXd> elasticValues_;
  std::shared_ptr<const EigenSupport::VXd> plasticValues_;
};

/// Internal non-owning trial view. It owns the space and any committed value
/// vectors it references, while trial spans are borrowed for one call only.
class MaterialParameterEvaluationView
{
public:
  MaterialParameterEvaluationView() = default;

  const MaterialParameterSpace &space() const;
  std::shared_ptr<const MaterialParameterSpace> spaceHandle() const { return space_; }
  bool empty() const { return !space_; }

  std::span<const double> elasticValues() const { return elasticValues_; }
  std::span<const double> plasticValues() const { return plasticValues_; }
  std::span<const double> values(const MaterialParameterField &field) const;

private:
  friend class MaterialParameterSpace;
  friend class MaterialParameterSnapshot;

  MaterialParameterEvaluationView(
    std::shared_ptr<const MaterialParameterSpace> space,
    std::span<const double> elasticValues,
    std::span<const double> plasticValues,
    std::shared_ptr<const EigenSupport::VXd> elasticOwner = {},
    std::shared_ptr<const EigenSupport::VXd> plasticOwner = {}):
    space_(std::move(space)),
    elasticValues_(elasticValues),
    plasticValues_(plasticValues),
    elasticOwner_(std::move(elasticOwner)),
    plasticOwner_(std::move(plasticOwner))
  {
  }

  std::shared_ptr<const MaterialParameterSpace> space_;
  std::span<const double> elasticValues_;
  std::span<const double> plasticValues_;
  std::shared_ptr<const EigenSupport::VXd> elasticOwner_;
  std::shared_ptr<const EigenSupport::VXd> plasticOwner_;
};

/// Thread-safe handle to the currently committed material parameter values.
class MaterialParameters final
{
public:
  MaterialParameters(
    std::shared_ptr<const MaterialParameterSpace> space,
    EigenSupport::VXd elasticValues,
    EigenSupport::VXd plasticValues);

  std::shared_ptr<const MaterialParameterSpace> space() const { return space_; }

  MaterialParameterSnapshot snapshot() const;
  EigenSupport::VXd elasticSnapshot() const;
  EigenSupport::VXd plasticSnapshot() const;

  // Each setter validates and atomically publishes a new immutable state
  // version. Existing evaluations continue using their acquired snapshot.
  void setElasticValues(EigenSupport::ConstRefVecXd values);
  void setPlasticValues(EigenSupport::ConstRefVecXd values);

private:
  struct CommittedValues
  {
    std::shared_ptr<const EigenSupport::VXd> elastic;
    std::shared_ptr<const EigenSupport::VXd> plastic;
  };

  std::shared_ptr<const MaterialParameterSpace> space_;
  mutable std::mutex mutex_;
  std::shared_ptr<const CommittedValues> committed_;
};

class MaterialParameterRef
{
public:
  MaterialParameterRef() = default;
  MaterialParameterRef(
    std::shared_ptr<const MaterialParameterField> field, int channel);

  const MaterialParameterField &field() const;
  std::shared_ptr<const MaterialParameterField> fieldHandle() const { return field_; }
  int channel() const { return channel_; }
  std::string_view name() const;

  double value(
    int element,
    int quadrature,
    const MaterialParameterEvaluationView &state) const;

  double value(
    int element,
    int quadrature,
    const MaterialParameterEvaluationView &state,
    MaterialParameterEvaluationScratch &scratch) const;

  void localDerivative(
    int element,
    int quadrature,
    const MaterialParameterEvaluationView &state,
    double *output) const;

  void localDerivative(
    int element,
    int quadrature,
    const MaterialParameterEvaluationView &state,
    MaterialParameterEvaluationScratch &scratch,
    double *output) const;

private:
  std::shared_ptr<const MaterialParameterField> field_;
  int channel_ = -1;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
