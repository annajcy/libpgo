#pragma once

#include "EigenDef.h"

#include <memory>
#include <string_view>

namespace pgo
{
namespace SolidDeformationModel
{

class DeformationModel;
class DofLayout;
class ElasticModel;
class PlasticModel;
class SimulationMesh;
enum class SimulationMeshType;

class Formulation
{
public:
  virtual ~Formulation() = default;

  virtual std::string_view getName() const = 0;
  virtual int getNodesPerElement() const = 0;
  virtual int getLocalDofs() const = 0;

  virtual std::unique_ptr<DeformationModel> createElement(
    const SimulationMesh &mesh, int ele,
    std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel) const = 0;

  virtual SimulationMeshType compatibleMeshType() const = 0;

  virtual std::unique_ptr<DofLayout> createDofLayout(const SimulationMesh &mesh) const;
  virtual EigenSupport::VXd buildGlobalRestDofs(const SimulationMesh &mesh) const;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
