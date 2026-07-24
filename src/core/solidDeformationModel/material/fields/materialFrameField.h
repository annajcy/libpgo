#pragma once

#include "EigenSupport.h"

#include <memory>
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

enum class MaterialFrameFieldKind
{
  GlobalAxes,
  Constant,
  Elementwise,
};

class MaterialFrameField
{
public:
  virtual ~MaterialFrameField() = default;

  virtual MaterialFrameFieldKind kind() const = 0;
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

class GlobalAxesMaterialFrameField final : public MaterialFrameField
{
public:
  explicit GlobalAxesMaterialFrameField(int numElements);

  MaterialFrameFieldKind kind() const override
  {
    return MaterialFrameFieldKind::GlobalAxes;
  }
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

  MaterialFrameFieldKind kind() const override
  {
    return MaterialFrameFieldKind::Constant;
  }
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

  MaterialFrameFieldKind kind() const override
  {
    return MaterialFrameFieldKind::Elementwise;
  }
  int numElements() const override
  {
    return static_cast<int>(frames_.size());
  }
  MaterialFrame materialToReferenceFrame(
    int elementId, int quadratureId) const override;

private:
  std::vector<MaterialFrame> frames_;
};

std::shared_ptr<const MaterialFrameField>
makeGlobalAxesMaterialFrameField(int numElements);

// Return an immutable field with Q' = referenceRotation * Q for every element.
// The input field is not modified.
std::shared_ptr<const MaterialFrameField>
transformReferenceMaterialFrames(
  const MaterialFrameField &field,
  const MaterialFrame &referenceRotation);

void validateMaterialFrame(const MaterialFrame &frame);

}  // namespace SolidDeformationModel
}  // namespace pgo
