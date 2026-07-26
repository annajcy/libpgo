#pragma once

#include "formulations/formulation/formulation.h"

#include <span>

namespace pgo
{
namespace VolumetricMeshes
{
class VolumetricMesh;
}

namespace SolidDeformationModel
{

class Quadrature;
class ShapeFunction;
class VolumetricElementMapping;
class VolumeDensityField;

class VolumetricFormulation : public Formulation
{
public:
  VolumetricFormulation(std::unique_ptr<ShapeFunction> shapeFunction, std::unique_ptr<Quadrature> quadrature);
  ~VolumetricFormulation() override;

  const ShapeFunction &shapeFunction() const { return *shapeFunction_; }
  const Quadrature &quadrature() const { return *quadrature_; }

  // Quadrature used for mass / body-force assembly. Defaults to the elastic
  // quadrature; formulations whose elastic rule under-integrates rho*N^T*N
  // (tet linear) override this.
  virtual const Quadrature &massQuadrature() const { return *quadrature_; }

  std::unique_ptr<VolumetricElementMapping> createElementMapping(
    std::span<const double> restPositions) const;

  std::unique_ptr<DeformationModel> createElement(
    const SimulationMesh &mesh, int ele,
    std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel) const override;

  // Consistent mass matrix / generalized body force, assembled with the
  // formulation's shape function over massQuadrature(), scattered through
  // the formulation's DofLayout. Density comes from the density field.
  virtual EigenSupport::SpMatD buildMassMatrix(
    const SimulationMesh &mesh, const VolumeDensityField &density) const;
  virtual EigenSupport::VXd buildBodyForce(
    const SimulationMesh &mesh, const EigenSupport::V3d &acceleration,
    const VolumeDensityField &density) const;
  virtual EigenSupport::SpMatD buildSurfaceEmbeddingMatrix(
    const VolumetricMeshes::VolumetricMesh &mesh,
    const EigenSupport::MXd &surfaceVertices) const;

private:
  std::unique_ptr<ShapeFunction> shapeFunction_;
  std::unique_ptr<Quadrature> quadrature_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
