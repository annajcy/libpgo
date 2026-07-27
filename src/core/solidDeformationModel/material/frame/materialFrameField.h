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

class MaterialFrameField
{
public:
  virtual ~MaterialFrameField() = default;

  virtual int numElements() const = 0;
  virtual MaterialFrame materialToReferenceFrame(
    int elementId, int quadratureId) const = 0;

  EigenSupport::V3d primaryAxis(int elementId, int quadratureId) const;
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

/// Construct deterministic right-handed frames whose first columns are the
/// supplied primary directions. Secondary axes follow a stable global-axis
/// completion convention and carry no imported material semantics.
std::shared_ptr<const MaterialFrameField>
materialFramesFromPrimaryAxes(
  const EigenSupport::M3Xd &primaryAxes);

void validateMaterialFrame(const MaterialFrame &frame);

}  // namespace SolidDeformationModel
}  // namespace pgo
