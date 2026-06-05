/*
  Public sampled penalty parameter specifications.
*/

#pragma once

namespace pgo
{
namespace Contact
{
namespace SampledPenalty
{

struct ParametersSpec
{
  double stiffness = 1.0;
  int samples = 1;
  bool enableSelfContact = true;
  bool enableExternalContact = true;
};

struct FrictionParametersSpec
{
  double frictionCoeff = 1.0;
  double velocityEps = 1.0;
};

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
