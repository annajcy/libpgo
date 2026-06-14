/*
  Contact detection and evaluation-bundle construction for sampled penalty contact.
*/

#pragma once

#include "EigenDef.h"
#include "sampled_penalty/sampledPenaltyEvaluationBundle.h"
#include "sampled_penalty/sampledPenaltySpecs.h"
#include "triMeshGeo.h"

#include <functional>
#include <memory>
#include <vector>

namespace pgo
{
namespace Contact
{
class PointPenetrationEnergy;
class PointTrianglePairCouplingEnergyWithCollision;
class TriangleMeshExternalContactHandler;
class TriangleMeshSelfContactHandler;

namespace SampledPenalty
{

struct SampledPenaltyEnergyConfigurator
{
  std::function<void(PointPenetrationEnergy &)> configureExternal;
  std::function<void(PointTrianglePairCouplingEnergyWithCollision &, EigenSupport::ConstRefVecXd)> configureSelf;
};

class SampledPenaltyContactBuilder
{
public:
  SampledPenaltyContactBuilder(
    const Mesh::TriMeshGeo &surfaceMesh,
    int simulationDofCount,
    const ParametersSpec &params,
    std::vector<Mesh::TriMeshGeo> externalSurfaces,
    std::vector<int> vertexEmbeddingIndices,
    std::vector<double> vertexEmbeddingWeights);

  void updateExternalSurface(int index, const Mesh::TriMeshGeo &surface);

  std::unique_ptr<SampledPenaltyEvaluationBundle> buildFromPositions(
    EigenSupport::ConstRefVecXd x,
    const SampledPenaltyEnergyConfigurator &configurator) const;

private:
  Mesh::TriMeshGeo surfaceMesh_;
  ParametersSpec params_;
  std::vector<Mesh::TriMeshGeo> externalSurfaces_;
  std::vector<int> vertexEmbeddingIndices_;
  std::vector<double> vertexEmbeddingWeights_;
  std::shared_ptr<TriangleMeshExternalContactHandler> externalHandler_;
  std::shared_ptr<TriangleMeshSelfContactHandler> selfHandler_;
};

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
