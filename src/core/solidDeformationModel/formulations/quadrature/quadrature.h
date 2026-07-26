#pragma once

#include "EigenSupport.h"

#include <memory>

namespace pgo
{
namespace SolidDeformationModel
{

class Quadrature
{
public:
  virtual ~Quadrature() = default;

  virtual int numPoints() const = 0;
  virtual std::unique_ptr<Quadrature> clone() const = 0;
  virtual EigenSupport::V3d point(int i) const = 0;
  virtual double weight(int i) const = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
