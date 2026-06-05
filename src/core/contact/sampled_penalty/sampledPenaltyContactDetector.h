/*
  Contact detection and active-set construction for sampled penalty contact.
*/

#pragma once

#include "EigenDef.h"
#include "sampled_penalty/sampledPenaltyActiveSet.h"
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

struct SampledPenaltyActiveEnergyConfigurator
{
  std::function<void(PointPenetrationEnergy &)> configureExternal;
  std::function<void(PointTrianglePairCouplingEnergyWithCollision &, EigenSupport::ConstRefVecXd)> configureSelf;
};

class SampledPenaltyContactDetector
{
public:
  SampledPenaltyContactDetector(
    const Mesh::TriMeshGeo &surfaceMesh,
    int simulationDofCount,
    const ParametersSpec &params,
    std::vector<Mesh::TriMeshGeo> externalSurfaces,
    std::vector<int> vertexEmbeddingIndices,
    std::vector<double> vertexEmbeddingWeights);

  void updateExternalSurface(int index, const Mesh::TriMeshGeo &surface);

  std::unique_ptr<SampledPenaltyActiveSet> buildActiveSet(
    EigenSupport::ConstRefVecXd x,
    const SampledPenaltyActiveEnergyConfigurator &configurator) const;

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
