#pragma once

#include "EigenSupport.h"
#include "simulation/importedMaterial.h"

#include <memory>
#include <string_view>
#include <string>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

// Material axes are stored as columns of Q. Q maps material coordinates to
// reference coordinates: v_reference = Q * v_material.
using MaterialFrame = EigenSupport::M3d;

enum class MaterialFrameRequirement
{
  None,
  PrimaryAxis,
  FullFrame,
};

class MaterialFrameField
{
public:
  virtual ~MaterialFrameField() = default;

  virtual int numElements() const = 0;
  virtual MaterialFrameRequirement capability() const
  {
    return MaterialFrameRequirement::FullFrame;
  }
  virtual MaterialFrame materialToReferenceFrame(
    int elementId, int quadratureId) const = 0;

  EigenSupport::V3d primaryAxis(int elementId, int quadratureId) const;
  bool satisfies(MaterialFrameRequirement requirement) const;
};

/// One-shot conversion of imported frame payloads.  This is intentionally
/// independent from material parameter projection and model definitions.
class MaterialFrameFieldProjection
{
public:
  virtual ~MaterialFrameFieldProjection() = default;
  virtual std::shared_ptr<const MaterialFrameField> project(
    const ImportedMaterialData &source) const = 0;
};

class ImportedRotationMaterialFrameFieldProjection final :
  public MaterialFrameFieldProjection
{
public:
  explicit ImportedRotationMaterialFrameFieldProjection(
    std::string property = "rotation"):
    property_(std::move(property)) {}

  std::shared_ptr<const MaterialFrameField> project(
    const ImportedMaterialData &source) const override;

private:
  std::string property_;
};

class GlobalAxesMaterialFrameField final : public MaterialFrameField
{
public:
  explicit GlobalAxesMaterialFrameField(int numElements);

  int numElements() const override { return numElements_; }
  MaterialFrame materialToReferenceFrame(
    int elementId, int quadratureId) const override;

private:
  int numElements_ = 0;
};

class ConstantMaterialFrameField final : public MaterialFrameField
{
public:
  ConstantMaterialFrameField(int numElements, const MaterialFrame &frame);

  int numElements() const override { return numElements_; }
  MaterialFrame materialToReferenceFrame(
    int elementId, int quadratureId) const override;

private:
  int numElements_ = 0;
  MaterialFrame frame_ = MaterialFrame::Identity();
};

class ElementwiseMaterialFrameField final : public MaterialFrameField
{
public:
  explicit ElementwiseMaterialFrameField(
    std::vector<MaterialFrame> frames);

  static std::shared_ptr<const ElementwiseMaterialFrameField>
  fromPrimarySecondaryDirections(
    const EigenSupport::M3Xd &primaryDirections,
    const EigenSupport::M3Xd &secondaryDirections);

  int numElements() const override
  {
    return static_cast<int>(frames_.size());
  }
  MaterialFrame materialToReferenceFrame(
    int elementId, int quadratureId) const override;

private:
  std::vector<MaterialFrame> frames_;
};

// Return an immutable field with Q' = referenceRotation * Q for every element.
// The input field is not modified.
std::shared_ptr<const MaterialFrameField>
transformReferenceMaterialFrames(
  const MaterialFrameField &field,
  const MaterialFrame &referenceRotation);

void validateMaterialFrame(const MaterialFrame &frame);

}  // namespace SolidDeformationModel
}  // namespace pgo
