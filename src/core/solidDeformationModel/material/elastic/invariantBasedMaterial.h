/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "EigenSupport.h"

namespace pgo
{
namespace SolidDeformationModel
{
class InvariantBasedMaterial
{
public:
  InvariantBasedMaterial() {}
  virtual ~InvariantBasedMaterial() {}

  virtual double compute_psi(const EigenSupport::V3d &invariants) const = 0;
  virtual EigenSupport::V3d compute_dpsi_dI(
    const EigenSupport::V3d &invariants) const = 0;
  // The symmetric Hessian is returned in the order (11, 12, 13, 22, 23, 33).
  virtual EigenSupport::V6d compute_d2psi_dI2(
    const EigenSupport::V3d &invariants) const = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
