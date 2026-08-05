#pragma once

#include "EigenSupport.h"

#include <vector>

namespace pgo::SolidDeformationModel
{

// Material axes are stored as columns of Q. Q maps material coordinates to
// reference coordinates: v_reference = Q * v_material.
using MaterialFrame = EigenSupport::M3d;

/// Complete material coordinate frames, stored explicitly per element.
class MaterialFrames final
{
public:
  explicit MaterialFrames(std::vector<MaterialFrame> frames);

  static MaterialFrames identity(int numElements);

  int numElements() const { return static_cast<int>(frames_.size()); }
  const MaterialFrame &operator[](int elementId) const;
  const std::vector<MaterialFrame> &values() const { return frames_; }

private:
  std::vector<MaterialFrame> frames_;
};

/// Construct deterministic right-handed frames whose first columns are the
/// supplied primary directions.
MaterialFrames materialFramesFromPrimaryAxes(
  const EigenSupport::M3Xd &primaryAxes);

void validateMaterialFrame(const MaterialFrame &frame);

}  // namespace pgo::SolidDeformationModel
