#include "operations/booleanOps.h"

#include <algorithm>
#include <stdexcept>

namespace pgo::ImplicitSurface {

BooleanField::BooleanField(std::shared_ptr<ImplicitField> a,
  std::shared_ptr<ImplicitField> b, BooleanOp op)
  : a_(std::move(a)), b_(std::move(b)), op_(op)
{
  if (!a_ || !b_)
    throw std::runtime_error("BooleanField inputs must not be null");
}

double BooleanField::eval(const V3d &p) const
{
  const double av = a_->eval(p);
  const double bv = b_->eval(p);
  switch (op_) {
    case BooleanOp::Union:
      return std::min(av, bv);
    case BooleanOp::Intersection:
      return std::max(av, bv);
    case BooleanOp::Difference:
      return std::max(av, -bv);
  }
  return av;
}

Mesh::LightBoundingBox BooleanField::bounds() const
{
  const auto ba = a_->bounds();
  const auto bb = b_->bounds();

  switch (op_) {
    case BooleanOp::Union:
      if (isUnbounded(ba) || isUnbounded(bb))
        return Mesh::LightBoundingBox();
      {
        auto result = ba;
        result.expand(bb);
        return result;
      }
    case BooleanOp::Intersection:
      if (isUnbounded(ba))
        return bb;
      if (isUnbounded(bb))
        return ba;
      return ba.getIntersection(bb);
    case BooleanOp::Difference:
      return ba;
  }

  return Mesh::LightBoundingBox();
}

std::shared_ptr<ImplicitField> makeUnion(std::shared_ptr<ImplicitField> a,
  std::shared_ptr<ImplicitField> b)
{
  return std::make_shared<BooleanField>(std::move(a), std::move(b), BooleanOp::Union);
}

std::shared_ptr<ImplicitField> makeIntersection(std::shared_ptr<ImplicitField> a,
  std::shared_ptr<ImplicitField> b)
{
  return std::make_shared<BooleanField>(std::move(a), std::move(b), BooleanOp::Intersection);
}

std::shared_ptr<ImplicitField> makeDifference(std::shared_ptr<ImplicitField> a,
  std::shared_ptr<ImplicitField> b)
{
  return std::make_shared<BooleanField>(std::move(a), std::move(b), BooleanOp::Difference);
}

}  // namespace pgo::ImplicitSurface
