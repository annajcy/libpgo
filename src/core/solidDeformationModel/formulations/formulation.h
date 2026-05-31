#pragma once

#include <memory>
#include <string_view>

namespace pgo
{
namespace SolidDeformationModel
{

class Basis;
class Quadrature;
class DeformationGradientKernel;
class ShellKernel;

// ============================================================
// Formulation — top-level abstract base
// ============================================================

class Formulation
{
public:
  virtual ~Formulation() = default;
  virtual std::string_view getName() const = 0;
  virtual int getNodesPerElement() const = 0;
  virtual int getLocalDofs() const = 0;
};

// ============================================================
// VolumetricFormulation — owns Basis + Quadrature, creates
// DeformationGradientKernel per element.
// ============================================================

class VolumetricFormulation : public Formulation
{
public:
  VolumetricFormulation(std::unique_ptr<Basis> basis, std::unique_ptr<Quadrature> quad);
  ~VolumetricFormulation() override;

  const Basis &basis() const { return *basis_; }
  const Quadrature &quadrature() const { return *quad_; }

  std::unique_ptr<DeformationGradientKernel> createKernel(const double *restPositions) const;

private:
  std::unique_ptr<Basis> basis_;
  std::unique_ptr<Quadrature> quad_;
};

// ============================================================
// ShellFormulation — creates ShellKernel per element.
// ============================================================

class ShellFormulation : public Formulation
{
public:
  std::unique_ptr<ShellKernel> createKernel(
    const double restX[18], const bool hasVtx[6]) const;
};

// ============================================================
// Intermediate tag classes — one per topology category
// ============================================================

class TetFormulation : public VolumetricFormulation
{
public:
  using VolumetricFormulation::VolumetricFormulation;
};

class CubicFormulation : public VolumetricFormulation
{
public:
  using VolumetricFormulation::VolumetricFormulation;
};

// ============================================================
// Concrete formulations
// ============================================================

class P1TetFormulation : public TetFormulation
{
public:
  P1TetFormulation();
  std::string_view getName() const override;
  int getNodesPerElement() const override;
  int getLocalDofs() const override;
};

class LinearCubicFormulation : public CubicFormulation
{
public:
  LinearCubicFormulation();
  std::string_view getName() const override;
  int getNodesPerElement() const override;
  int getLocalDofs() const override;
};

class KoiterShellFormulation : public ShellFormulation
{
public:
  std::string_view getName() const override;
  int getNodesPerElement() const override;
  int getLocalDofs() const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
