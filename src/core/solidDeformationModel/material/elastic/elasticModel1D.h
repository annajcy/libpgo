#pragma once

#include "material/elastic/elasticModel.h"

#include <span>

namespace pgo
{
namespace SolidDeformationModel
{

// Scalar one-dimensional material evaluator.
//
// The independent deformation variable is x (a scalar), rather than a 3D
// deformation gradient.  Parameter derivatives are exposed per parameter so
// callers can assemble them into their own parameter-space blocks.
class ElasticModel1D : public ElasticModel
{
public:
  ~ElasticModel1D() override = default;

  virtual double compute_psi(std::span<const double> param, double x) const = 0;
  virtual double compute_dpsi_dx(std::span<const double> param, double x) const = 0;
  virtual double compute_d2psi_dx2(std::span<const double> param, double x) const = 0;

  virtual double compute_dpsi_dparam(
    std::span<const double> param, int i, double x) const = 0;
  virtual double compute_d2psi_dx_dparam(
    std::span<const double> param, int i, double x) const = 0;
  virtual double compute_d2psi_dparam2(
    std::span<const double> param, int i, int j, double x) const = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
