#pragma once

#include "EigenSupport.h"

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
  // Different concrete bases have different node counts. The polymorphic
  // boundary therefore uses dynamic Eigen references; concrete classes also
  // expose fixed-size value-returning overloads with the same names.
  virtual void compute_N(double xi, double eta, double zeta,
    EigenSupport::RefVecXd N_out) const = 0;
  virtual void compute_dN_dxi(double xi, double eta, double zeta,
    EigenSupport::RefMatXd dN_out) const = 0;
  virtual EigenSupport::V3d nodeCoords(int node) const = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
