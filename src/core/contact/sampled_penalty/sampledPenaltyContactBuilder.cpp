/*
  Contact detection and evaluation-bundle construction for sampled penalty contact.
*/

#include "sampled_penalty/sampledPenaltyContactBuilder.h"

#include "sampled_penalty/kernels/pointPenetrationEnergy.h"
#include "sampled_penalty/kernels/pointTrianglePairCouplingEnergyWithCollision.h"
#include "sampled_penalty/kernels/triangleMeshExternalContactHandler.h"
#include "sampled_penalty/kernels/triangleMeshSelfContactHandler.h"

#include <stdexcept>
#include <utility>

namespace pgo
{
namespace Contact
{
namespace SampledPenalty
{
namespace
{
std::vector<Mesh::TriMeshRef> makeSurfaceRefs(std::vector<Mesh::TriMeshGeo> &surfaces)
{
  std::vector<Mesh::TriMeshRef> refs;
  refs.reserve(surfaces.size());
  for (auto &surface : surfaces)
    refs.emplace_back(surface);
  return refs;
}

std::vector<EigenSupport::V3d> flatPositionsToVertices(EigenSupport::ConstRefVecXd x)
{
  if (x.size() % 3 != 0)
    throw std::invalid_argument("SampledPenaltyContactEnergy surface positions must have size 3*n.");

  std::vector<EigenSupport::V3d> vertices;
  vertices.reserve(x.size() / 3);
  for (Eigen::Index i = 0; i < x.size() / 3; i++)
    vertices.emplace_back(x.segment<3>(i * 3));
  return vertices;
}
}  // namespace

SampledPenaltyContactBuilder::SampledPenaltyContactBuilder(
  const Mesh::TriMeshGeo &surfaceMesh,
  int simulationDofCount,
  const ParametersSpec &params,
  std::vector<Mesh::TriMeshGeo> externalSurfaces,
  std::vector<int> vertexEmbeddingIndices,
  std::vector<double> vertexEmbeddingWeights):
  surfaceMesh_(surfaceMesh),
  params_(params),
  externalSurfaces_(std::move(externalSurfaces)),
  vertexEmbeddingIndices_(std::move(vertexEmbeddingIndices)),
  vertexEmbeddingWeights_(std::move(vertexEmbeddingWeights))
{
  if (simulationDofCount <= 0 || simulationDofCount % 3 != 0)
    throw std::invalid_argument("SampledPenaltyContactEnergy requires simulation rest positions with size 3*n.");
  if (params_.samples <= 0)
    throw std::invalid_argument("SampledPenaltyContactEnergy requires a positive sample count.");
  if (params_.stiffness < 0.0)
    throw std::invalid_argument("SampledPenaltyContactEnergy requires non-negative stiffness.");
  if (vertexEmbeddingIndices_.empty() != vertexEmbeddingWeights_.empty())
    throw std::invalid_argument("SampledPenaltyContactEnergy embedding indices and weights must be provided together.");
  if (!vertexEmbeddingIndices_.empty() && vertexEmbeddingIndices_.size() != vertexEmbeddingWeights_.size())
    throw std::invalid_argument("SampledPenaltyContactEnergy embedding indices and weights must have matching sizes.");
  if (vertexEmbeddingIndices_.empty() && surfaceMesh_.numVertices() * 3 > simulationDofCount)
    throw std::invalid_argument("SampledPenaltyContactEnergy surface has more vertex DOFs than the unembedded simulation state.");

  const std::vector<int> *embeddingIndices = vertexEmbeddingIndices_.empty() ? nullptr : &vertexEmbeddingIndices_;
  const std::vector<double> *embeddingWeights = vertexEmbeddingWeights_.empty() ? nullptr : &vertexEmbeddingWeights_;

  if (params_.enableExternalContact && !externalSurfaces_.empty()) {
    std::vector<Mesh::TriMeshRef> externalRefs = makeSurfaceRefs(externalSurfaces_);
    externalHandler_ = std::make_shared<TriangleMeshExternalContactHandler>(
      surfaceMesh_.positions(), surfaceMesh_.triangles(), simulationDofCount,
      externalRefs, params_.samples, embeddingIndices, embeddingWeights);
  }

  if (params_.enableSelfContact) {
    selfHandler_ = std::make_shared<TriangleMeshSelfContactHandler>(
      surfaceMesh_.positions(), surfaceMesh_.triangles(), simulationDofCount,
      params_.samples, embeddingIndices, embeddingWeights);
  }
}

void SampledPenaltyContactBuilder::updateExternalSurface(int index, const Mesh::TriMeshGeo &surface)
{
  if (index < 0 || index >= static_cast<int>(externalSurfaces_.size()))
    throw std::invalid_argument("SampledPenaltyContactEnergy external surface index is out of range.");

  externalSurfaces_[index] = surface;
  if (externalHandler_)
    externalHandler_->updateExternalSurface(index, Mesh::TriMeshRef(externalSurfaces_[index]));
}

std::unique_ptr<SampledPenaltyEvaluationBundle> SampledPenaltyContactBuilder::buildFromPositions(
  EigenSupport::ConstRefVecXd x,
  const SampledPenaltyEnergyConfigurator &configurator) const
{
  auto bundle = std::make_unique<SampledPenaltyEvaluationBundle>();

  if (params_.stiffness <= 0.0)
    return bundle;

  const std::vector<EigenSupport::V3d> surfacePositions = flatPositionsToVertices(x);

  if (externalHandler_) {
    externalHandler_->execute(surfacePositions);
    if (externalHandler_->getNumCollidingSamples() > 0) {
      bundle->externalEnergy = externalHandler_->buildContactEnergy();
      bundle->externalBuffer = bundle->externalEnergy->allocateBuffer();
      bundle->externalEnergy->setBuffer(bundle->externalBuffer);
      configurator.configureExternal(*bundle->externalEnergy);
    }
  }

  if (selfHandler_) {
    selfHandler_->execute(surfacePositions);
    if (!selfHandler_->getCollidingTrianglePair().empty()) {
      selfHandler_->handleContactDCD(0.0, 100);
      bundle->selfEnergy = selfHandler_->buildContactEnergy();
      bundle->selfBuffer = bundle->selfEnergy->allocateBuffer();
      bundle->selfEnergy->setBuffer(bundle->selfBuffer);
      configurator.configureSelf(*bundle->selfEnergy, x);
    }
  }

  return bundle;
}

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
