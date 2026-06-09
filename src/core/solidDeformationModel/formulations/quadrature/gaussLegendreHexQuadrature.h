#pragma once

#include "quadrature.h"

namespace pgo
{
namespace SolidDeformationModel
{

// 2x2x2 Gauss-Legendre quadrature for the unit hex [0,1]^3.
// Gauss points at 0.5 +/- 0.5/sqrt(3) in each direction.
// Reference weights = 1/8 per point.
// Used with CubicLinearShapeFunction for hex trilinear deformation gradient formulation.
class GaussLegendreHexQuadrature2 : public Quadrature
{
public:
  static constexpr int kNumPoints = 8;

  int numPoints() const override { return kNumPoints; }
  std::unique_ptr<Quadrature> clone() const override { return std::make_unique<GaussLegendreHexQuadrature2>(*this); }
  void point(int i, double xi[3]) const override;
  double weight(int i) const override;
};

// 4x4x4 Gauss-Legendre quadrature for the unit hex [0,1]^3 (64 points), exact for polynomials up
// to degree 7 per axis. Used with CubicTricubicHermiteShapeFunction, whose deformation gradient is higher
// order than trilinear, so 2x2x2 would under-integrate nonlinear materials.
class GaussLegendreHexQuadrature4 : public Quadrature
{
public:
  static constexpr int kNumPoints = 64;

  int numPoints() const override { return kNumPoints; }
  std::unique_ptr<Quadrature> clone() const override { return std::make_unique<GaussLegendreHexQuadrature4>(*this); }
  void point(int i, double xi[3]) const override;
  double weight(int i) const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
