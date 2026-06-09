#pragma once

#include "formulations/formulation/formulation.h"

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

class VolumetricFormulation : public Formulation
{
public:
  VolumetricFormulation(std::unique_ptr<ShapeFunction> shapeFunction, std::unique_ptr<Quadrature> quadrature);
  ~VolumetricFormulation() override;

  const ShapeFunction &shapeFunction() const { return *shapeFunction_; }
  const Quadrature &quadrature() const { return *quadrature_; }

  std::unique_ptr<VolumetricElementMapping> createElementMapping(const double *restPositions) const;

  std::unique_ptr<DeformationModel> createElement(
    const SimulationMesh &mesh, int ele,
    std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel) const override;

  virtual EigenSupport::SpMatD buildMassMatrix(
    const VolumetricMeshes::VolumetricMesh &mesh) const;
  virtual EigenSupport::VXd buildBodyForce(
    const VolumetricMeshes::VolumetricMesh &mesh,
    const EigenSupport::V3d &acceleration) const;
  virtual EigenSupport::SpMatD buildSurfaceEmbeddingMatrix(
    const VolumetricMeshes::VolumetricMesh &mesh,
    const EigenSupport::MXd &surfaceVertices) const;

private:
  std::unique_ptr<ShapeFunction> shapeFunction_;
  std::unique_ptr<Quadrature> quadrature_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
