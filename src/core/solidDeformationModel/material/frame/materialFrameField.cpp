#include "material/frame/materialFrameField.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <string_view>

namespace pgo::SolidDeformationModel
{
namespace ES = EigenSupport;

namespace
{
constexpr double kFrameTolerance = 1e-8;
constexpr double kDirectionTolerance = 1e-12;

void validateSampleIndices(
  int elementId, int quadratureId, int numElements)
{
  if (elementId < 0 || elementId >= numElements)
    throw std::out_of_range("material frame element index is out of range.");
  if (quadratureId < 0)
    throw std::out_of_range("material frame quadrature index must be non-negative.");
}

ES::V3d checkedDirection(const ES::V3d &direction, const char *name)
{
  if (!direction.allFinite())
    throw std::invalid_argument(std::string(name) + " must be finite.");
  const double norm = direction.norm();
  if (!(norm > kDirectionTolerance))
    throw std::invalid_argument(std::string(name) + " must be non-zero.");
  return direction / norm;
}
}  // namespace

void validateMaterialFrame(const MaterialFrame &frame)
{
  if (!frame.allFinite())
    throw std::invalid_argument("material frame must be finite.");

  const MaterialFrame gram = frame.transpose() * frame;
  if (!gram.isApprox(MaterialFrame::Identity(), kFrameTolerance))
    throw std::invalid_argument("material frame must be orthonormal.");

  const double determinant = frame.determinant();
  if (!std::isfinite(determinant) ||
    std::abs(determinant - 1.0) > kFrameTolerance)
    throw std::invalid_argument(
      "material frame must be right-handed with determinant +1.");
}

ES::V3d MaterialFrameField::primaryAxis(
  int elementId, int quadratureId) const
{
  return materialToReferenceFrame(elementId, quadratureId).col(0);
}

GlobalAxesMaterialFrameField::GlobalAxesMaterialFrameField(int numElements):
  numElements_(numElements)
{
  if (numElements < 0)
    throw std::invalid_argument(
      "GlobalAxesMaterialFrameField: numElements must be non-negative.");
}

MaterialFrame GlobalAxesMaterialFrameField::materialToReferenceFrame(
  int elementId, int quadratureId) const
{
  validateSampleIndices(elementId, quadratureId, numElements_);
  return MaterialFrame::Identity();
}

ConstantMaterialFrameField::ConstantMaterialFrameField(
  int numElements, const MaterialFrame &frame):
  numElements_(numElements),
  frame_(frame)
{
  if (numElements < 0)
    throw std::invalid_argument(
      "ConstantMaterialFrameField: numElements must be non-negative.");
  validateMaterialFrame(frame_);
}

MaterialFrame ConstantMaterialFrameField::materialToReferenceFrame(
  int elementId, int quadratureId) const
{
  validateSampleIndices(elementId, quadratureId, numElements_);
  return frame_;
}

ElementwiseMaterialFrameField::ElementwiseMaterialFrameField(
  std::vector<MaterialFrame> frames):
  frames_(std::move(frames))
{
  for (const MaterialFrame &frame : frames_)
    validateMaterialFrame(frame);
}

std::shared_ptr<const ElementwiseMaterialFrameField>
ElementwiseMaterialFrameField::fromPrimarySecondaryDirections(
  const ES::M3Xd &primaryDirections,
  const ES::M3Xd &secondaryDirections)
{
  if (primaryDirections.cols() != secondaryDirections.cols())
    throw std::invalid_argument(
      "primary and secondary direction counts must match.");

  std::vector<MaterialFrame> frames;
  frames.reserve(primaryDirections.cols());
  for (Eigen::Index i = 0; i < primaryDirections.cols(); i++) {
    const ES::V3d x =
      checkedDirection(primaryDirections.col(i), "primary direction");
    const ES::V3d secondary =
      checkedDirection(secondaryDirections.col(i), "secondary direction");
    ES::V3d y = secondary - x.dot(secondary) * x;
    if (!y.allFinite() || !(y.norm() > kDirectionTolerance))
      throw std::invalid_argument(
        "secondary direction must not be parallel to the primary direction.");
    y.normalize();
    const ES::V3d z = x.cross(y);

    MaterialFrame frame;
    frame.col(0) = x;
    frame.col(1) = y;
    frame.col(2) = z;
    validateMaterialFrame(frame);
    frames.push_back(frame);
  }
  return std::make_shared<const ElementwiseMaterialFrameField>(
    std::move(frames));
}

MaterialFrame ElementwiseMaterialFrameField::materialToReferenceFrame(
  int elementId, int quadratureId) const
{
  validateSampleIndices(
    elementId, quadratureId, static_cast<int>(frames_.size()));
  return frames_[elementId];
}

std::shared_ptr<const MaterialFrameField>
transformReferenceMaterialFrames(
  const MaterialFrameField &field,
  const MaterialFrame &referenceRotation)
{
  validateMaterialFrame(referenceRotation);
  std::vector<MaterialFrame> frames;
  frames.reserve(field.numElements());
  for (int elementId = 0; elementId < field.numElements(); elementId++)
    frames.push_back(
      referenceRotation *
      field.materialToReferenceFrame(elementId, 0));
  return std::make_shared<const ElementwiseMaterialFrameField>(
    std::move(frames));
}

std::shared_ptr<const MaterialFrameField>
materialFramesFromPrimaryAxes(const ES::M3Xd &primaryAxes)
{
  std::vector<MaterialFrame> frames;
  frames.reserve(primaryAxes.cols());
  for (Eigen::Index i = 0; i < primaryAxes.cols(); ++i) {
    const ES::V3d x =
      checkedDirection(primaryAxes.col(i), "primary direction");
    const std::array<ES::V3d, 3> globalAxes{
      ES::V3d::UnitX(), ES::V3d::UnitY(), ES::V3d::UnitZ()};
    const auto auxiliary = std::min_element(
      globalAxes.begin(), globalAxes.end(),
      [&](const ES::V3d &lhs, const ES::V3d &rhs) {
        return std::abs(x.dot(lhs)) < std::abs(x.dot(rhs));
      });
    ES::V3d y = *auxiliary - x.dot(*auxiliary) * x;
    y.normalize();
    const ES::V3d z = x.cross(y);

    MaterialFrame frame;
    frame.col(0) = x;
    frame.col(1) = y;
    frame.col(2) = z;
    validateMaterialFrame(frame);
    frames.push_back(frame);
  }
  return std::make_shared<const ElementwiseMaterialFrameField>(
    std::move(frames));
}

}  // namespace pgo::SolidDeformationModel
