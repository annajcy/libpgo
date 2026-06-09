#pragma once

namespace pgo
{
namespace SolidDeformationModel
{

enum class DeformationModelElasticMaterial
{
  STABLE_NEO,
  STVK_VOL,
  INV_STVK,
  LINEAR,
  VOLUME,

  HILL_STABLE_NEO,
  HILL_STVK_VOL,
  HILL_STVK,

  STVK,
  MOONEY_RIVLIN,

  KOITER_FABRIC,
  KOITER_STVK,
};

enum class DeformationModelPlasticMaterial
{
  VOLUMETRIC_DOF0 = 0,
  VOLUMETRIC_DOF3 = 1,
  VOLUMETRIC_DOF6 = 2,

  SHELL_FF_DOF0 = 3,
  SHELL_FF_DOF1 = 4,
};

}  // namespace SolidDeformationModel
}  // namespace pgo
