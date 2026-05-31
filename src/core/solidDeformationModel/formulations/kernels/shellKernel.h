#pragma once

#include "EigenSupport.h"

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

// ShellKernel — abstract base for shell geometry / kinematics.
// Concrete implementations provide fundamental-form computation
// specific to a shell theory (Koiter, Belytschko, etc.).

class ShellKernel
{
public:
  virtual ~ShellKernel() = default;

  virtual int getNumNodes() const = 0;
  virtual int getLocalDofs() const = 0;

  // First fundamental form a (2x2) and its derivatives w.r.t. nodal positions x.
  virtual ES::M2d compute_a_and_derivatives(
    const ES::V3d x[6],
    Eigen::Matrix<double, 4, 9> *da_dx,
    ES::M9d ahess[4]) const = 0;

  // Second fundamental form b (2x2) and its derivatives w.r.t. nodal positions x.
  virtual ES::M2d compute_b_and_derivatives(
    const ES::V3d x[6],
    Eigen::Matrix<double, 4, 18> *db_dx,
    ES::M18d bhess[4]) const = 0;

  virtual const ES::M2d &restI() const = 0;
  virtual const ES::M2d &restII() const = 0;
  virtual double restArea() const = 0;
  virtual const bool *hasVtx() const = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
