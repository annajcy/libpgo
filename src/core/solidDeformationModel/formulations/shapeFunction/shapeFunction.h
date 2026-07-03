#pragma once

#include <memory>

namespace pgo
{
namespace SolidDeformationModel
{

class ShapeFunction
{
public:
  virtual ~ShapeFunction() = default;

  virtual int numNodes() const = 0;
  virtual int localDofs() const = 0;
  virtual std::unique_ptr<ShapeFunction> clone() const = 0;

  virtual void N(double xi, double eta, double zeta, double N_out[]) const = 0;
  virtual void dN_dxi(double xi, double eta, double zeta, double dN_out[]) const = 0;
  virtual void nodeCoords(int node, double xi[3]) const = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
