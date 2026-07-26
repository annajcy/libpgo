#pragma once

#include "EigenSupport.h"

#include <array>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

// ShellElementMapping — abstract base for shell geometry / mapping.
// Concrete implementations provide fundamental-form computation
// specific to a shell theory (Koiter, Belytschko, etc.).

class ShellElementMapping
{
public:
  using APositions = std::array<ES::V3d, 3>;
  using BPositions = std::array<ES::V3d, 6>;

  virtual ~ShellElementMapping() = default;

  virtual int getNumNodes() const = 0;
  virtual int getLocalDofs() const = 0;

  // First fundamental form and its first derivative.  The four rows of
  // da_dx correspond to (a00, a01, a10, a11), and the columns are the nine
  // in-plane position DOFs.
  virtual ES::M2d compute_a(const APositions &x) const = 0;
  virtual ES::M4x9d compute_da_dx(const APositions &x) const = 0;

  // The Hessian is stored as four horizontal 9x9 blocks.  Block j is the
  // Hessian of the j-th entry in the order (a00, a01, a10, a11).
  virtual ES::M9x36d compute_d2a_dx2(const APositions &x) const = 0;

  // Second fundamental form and its derivatives.  The corresponding Hessian
  // has four horizontal 18x18 blocks in the same entry order.
  virtual ES::M2d compute_b(const BPositions &x) const = 0;
  virtual ES::M4x18d compute_db_dx(const BPositions &x) const = 0;
  virtual ES::M18x72d compute_d2b_dx2(const BPositions &x) const = 0;

  virtual const ES::M2d &restI() const = 0;
  virtual const ES::M2d &restII() const = 0;
  virtual double restArea() const = 0;
  virtual const std::array<bool, 6> &hasVtx() const = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
