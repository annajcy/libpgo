#include "material/frame/materialFrames.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace ES = EigenSupport;

namespace
{
constexpr double kFrameTolerance = 1e-8;
constexpr double kDirectionTolerance = 1e-12;

ES::V3d checkedDirection(const ES::V3d &direction)
{
  if (!direction.allFinite())
    throw std::invalid_argument("primary direction must be finite.");
  const double norm = direction.norm();
  if (!(norm > kDirectionTolerance))
    throw std::invalid_argument("primary direction must be non-zero.");
  return direction / norm;
}
}  // namespace

void validateMaterialFrame(const MaterialFrame &frame)
{
  if (!frame.allFinite())
    throw std::invalid_argument("material frame must be finite.");
  if (!(frame.transpose() * frame).isApprox(
        MaterialFrame::Identity(), kFrameTolerance))
    throw std::invalid_argument("material frame must be orthonormal.");
  const double determinant = frame.determinant();
  if (!std::isfinite(determinant) ||
    std::abs(determinant - 1.0) > kFrameTolerance)
    throw std::invalid_argument(
      "material frame must be right-handed with determinant +1.");
}

MaterialFrames::MaterialFrames(std::vector<MaterialFrame> frames):
  frames_(std::move(frames))
{
  for (const MaterialFrame &frame : frames_)
    validateMaterialFrame(frame);
}

MaterialFrames MaterialFrames::identity(int numElements)
{
  if (numElements < 0)
    throw std::invalid_argument(
      "MaterialFrames element count must be non-negative.");
  return MaterialFrames(std::vector<MaterialFrame>(
    static_cast<std::size_t>(numElements), MaterialFrame::Identity()));
}

const MaterialFrame &MaterialFrames::operator[](int elementId) const
{
  if (elementId < 0 || elementId >= numElements())
    throw std::out_of_range("material frame element index is out of range.");
  return frames_[static_cast<std::size_t>(elementId)];
}

MaterialFrames materialFramesFromPrimaryAxes(const ES::M3Xd &primaryAxes)
{
  std::vector<MaterialFrame> frames;
  frames.reserve(static_cast<std::size_t>(primaryAxes.cols()));
  for (Eigen::Index i = 0; i < primaryAxes.cols(); ++i) {
    const ES::V3d x = checkedDirection(primaryAxes.col(i));
    const std::array<ES::V3d, 3> globalAxes{
      ES::V3d::UnitX(), ES::V3d::UnitY(), ES::V3d::UnitZ()};
    const auto auxiliary = std::min_element(
      globalAxes.begin(), globalAxes.end(),
      [&](const ES::V3d &lhs, const ES::V3d &rhs) {
        return std::abs(x.dot(lhs)) < std::abs(x.dot(rhs));
      });
    ES::V3d y = *auxiliary - x.dot(*auxiliary) * x;
    y.normalize();

    MaterialFrame frame;
    frame.col(0) = x;
    frame.col(1) = y;
    frame.col(2) = x.cross(y);
    frames.push_back(frame);
  }
  return MaterialFrames(std::move(frames));
}

}  // namespace pgo::SolidDeformationModel
