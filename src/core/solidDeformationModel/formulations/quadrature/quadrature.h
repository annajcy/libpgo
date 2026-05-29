#pragma once

namespace pgo
{
namespace SolidDeformationModel
{

class Quadrature
{
public:
  virtual ~Quadrature() = default;

  virtual int numPoints() const = 0;
  virtual void point(int i, double xi[3]) const = 0;
  virtual double weight(int i) const = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
