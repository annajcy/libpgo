#pragma once

#include "core/ImplicitField.h"

#include <memory>
#include <stdexcept>

namespace pgo::ImplicitSurface {

class OffsetField : public ImplicitField {
public:
  OffsetField(std::shared_ptr<ImplicitField> inner, double offset)
    : inner_(std::move(inner)), offset_(offset)
  {
    if (!inner_)
      throw std::runtime_error("OffsetField input must not be null");
  }

  double eval(const V3d &p) const override
  {
    return inner_->eval(p) - offset_;
  }

  Mesh::LightBoundingBox bounds() const override
  {
    return inner_->bounds();
  }

private:
  std::shared_ptr<ImplicitField> inner_;
  double offset_ = 0.0;
};

}  // namespace pgo::ImplicitSurface
