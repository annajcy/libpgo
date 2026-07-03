#pragma once

#include "core/ImplicitField.h"

#include <memory>

namespace pgo::ImplicitSurface {

enum class BooleanOp { Union, Intersection, Difference };

class BooleanField : public ImplicitField {
public:
  BooleanField(std::shared_ptr<ImplicitField> a,
    std::shared_ptr<ImplicitField> b, BooleanOp op);

  double eval(const V3d &p) const override;
  Mesh::LightBoundingBox bounds() const override;

private:
  std::shared_ptr<ImplicitField> a_;
  std::shared_ptr<ImplicitField> b_;
  BooleanOp op_;
};

std::shared_ptr<ImplicitField> makeUnion(std::shared_ptr<ImplicitField> a,
  std::shared_ptr<ImplicitField> b);
std::shared_ptr<ImplicitField> makeIntersection(std::shared_ptr<ImplicitField> a,
  std::shared_ptr<ImplicitField> b);
std::shared_ptr<ImplicitField> makeDifference(std::shared_ptr<ImplicitField> a,
  std::shared_ptr<ImplicitField> b);

}  // namespace pgo::ImplicitSurface
