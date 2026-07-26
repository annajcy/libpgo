#pragma once

#include "EigenSupport.h"

namespace pgo::SolidDeformationModel
{

// Spectral blocks of the Hessian of an isotropic principal-stretch energy.
// The pair entries use the fixed order (0, 1), (0, 2), (1, 2).
struct IsotropicSpectralTangentBlocks
{
  EigenSupport::M3d d2psi_ds2 = EigenSupport::M3d::Zero();
  EigenSupport::V3d beta = EigenSupport::V3d::Zero();
  EigenSupport::V3d alpha = EigenSupport::V3d::Zero();
};

class IsotropicSpectralTangent
{
public:
  // Build the exact spectral blocks for positive principal stretches.
  // Same-sign repeated stretches are evaluated with their continuous limits.
  static IsotropicSpectralTangentBlocks compute_dPdF_blocks(
    const EigenSupport::V3d &s,
    const EigenSupport::V3d &dpsi_ds,
    const EigenSupport::M3d &d2psi_ds2);

  // Project only the tangent blocks to the positive semidefinite cone.
  static IsotropicSpectralTangentBlocks project_dPdF_blocks_psd(
    const IsotropicSpectralTangentBlocks &blocks);

  // Assemble the blocks in the column-major vec(F) convention used by Eigen.
  static EigenSupport::M9d assemble_dPdF(
    const EigenSupport::M3d &U,
    const EigenSupport::M3d &V,
    const IsotropicSpectralTangentBlocks &blocks);

  // Apply the same tangent to one matrix perturbation without assembling 9x9.
  static EigenSupport::M3d apply_dPdF(
    const EigenSupport::M3d &U,
    const EigenSupport::M3d &V,
    const IsotropicSpectralTangentBlocks &blocks,
    const EigenSupport::M3d &dF);
};

}  // namespace pgo::SolidDeformationModel
