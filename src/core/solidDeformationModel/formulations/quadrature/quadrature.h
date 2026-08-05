#pragma once

#include "EigenSupport.h"

namespace pgo
{
namespace SolidDeformationModel
{

class Quadrature
{
public:
  virtual ~Quadrature() = default;

  virtual int numPoints() const = 0;
  virtual EigenSupport::V3d point(int i) const = 0;
  virtual double weight(int i) const = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
