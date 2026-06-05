#include "contact/sampledPenaltyContact.h"

#include "configFileJSON.h"
#include "sampled_penalty/sampledPenaltyContactEnergy.h"
#include "dynamicStepOptions.h"
#include "energySet.h"
#include "app/config.h"
#include "app/logging.h"
#include "app/session.h"
#include "setup/setup.h"

#include <array>
#include <stdexcept>
#include <string>
#include <utility>

namespace pgo::RunIPCSim
{
namespace
{
namespace ES = pgo::EigenSupport;

struct SampledPenaltyKinematicObject
{
  pgo::Mesh::TriMeshGeo mesh;
  ES::V3d movement = ES::V3d::Zero();
};

std::vector<SampledPenaltyKinematicObject> loadSampledPenaltyKinematicObjects(const pgo::ConfigFileJSON &config, double scale)
{
  std::vector<SampledPenaltyKinematicObject> objects;
  if (!config.exist("external-objects"))
    return objects;

  const auto &externalObjects = config.handle()["external-objects"];
  if (!externalObjects.is_array())
    throw std::invalid_argument("runIPCSim sampled-penalty contact expects `external-objects` to be an array.");

  objects.reserve(externalObjects.size());
  for (const auto &objectJson : externalObjects) {
    if (!objectJson.is_object())
      throw std::invalid_argument("runIPCSim sampled-penalty contact expects each `external-objects[]` entry to be an object.");
    if (!objectJson.contains("filename"))
      throw std::invalid_argument("Missing required field `external-objects[].filename`.");
    if (!objectJson.contains("movement"))
      throw std::invalid_argument("Missing required field `external-objects[].movement`.");

    SampledPenaltyKinematicObject object;
    const std::string filename = config.resolvePath(objectJson["filename"].get<std::string>());
    if (!object.mesh.load(filename))
      throw std::runtime_error("Failed to load external object mesh: " + filename);
    for (int vi = 0; vi < object.mesh.numVertices(); ++vi)
      object.mesh.pos(vi) *= scale;

    const std::array<double, 3> movement = objectJson["movement"].get<std::array<double, 3>>();
    object.movement = ES::V3d(movement[0], movement[1], movement[2]);
    objects.push_back(std::move(object));
  }

  return objects;
}

std::vector<pgo::Mesh::TriMeshGeo> copyObjectMeshes(const std::vector<SampledPenaltyKinematicObject> &objects)
{
  std::vector<pgo::Mesh::TriMeshGeo> meshes;
  meshes.reserve(objects.size());
  for (const auto &object : objects)
    meshes.push_back(object.mesh);
  return meshes;
}

class FrozenSampledPenaltyStaticEnergy final : public NonlinearOptimization::PotentialEnergy
{
public:
  explicit FrozenSampledPenaltyStaticEnergy(
    std::shared_ptr<Contact::SampledPenalty::SampledPenaltyContactEnergy> energy):
    energy_(std::move(energy))
  {
    if (!energy_)
      throw std::invalid_argument("FrozenSampledPenaltyStaticEnergy requires a contact energy.");
  }

  double func(ES::ConstRefVecXd x) const override { return energy_->func(x); }
  void gradient(ES::ConstRefVecXd x, ES::RefVecXd grad) const override { energy_->gradient(x, grad); }
  void hessianInPlace(ES::ConstRefVecXd x, ES::SpMatD &hess) const override { energy_->hessianInPlace(x, hess); }
  void hessianAlloc(ES::SpMatD &hess) const override { energy_->hessianAlloc(hess); }
  void getDOFs(std::vector<int> &dofs) const override { energy_->getDOFs(dofs); }
  int getNumDOFs() const override { return energy_->getNumDOFs(); }
  int isHessianTopologyFixed() const override { return energy_->isHessianTopologyFixed(); }
  NonlinearOptimization::EnergyStateKind stateKind() const override { return energy_->stateKind(); }

private:
  std::shared_ptr<Contact::SampledPenalty::SampledPenaltyContactEnergy> energy_;
};

class SampledPenaltyContactBackend final : public RunIPCSimContactBackend
{
public:
  SampledPenaltyContactBackend(const SampledPenaltyContactConfig &config,
    const pgo::Mesh::TriMeshGeo &surfaceMesh,
    ES::ConstRefVecXd simulationRestPosition,
    std::vector<SampledPenaltyKinematicObject> objects,
    const std::vector<int> &embeddingVertexIndices,
    const std::vector<double> &embeddingWeights):
    config_(config),
    surfaceMesh_(surfaceMesh),
    simulationRestPosition_(simulationRestPosition),
    objects_(std::move(objects)),
    vertexEmbeddingIndices_(embeddingVertexIndices),
    vertexEmbeddingWeights_(embeddingWeights)
  {
    if (config_.stiffness <= 0.0)
      return;

    Contact::SampledPenalty::ParametersSpec params;
    params.stiffness = config_.stiffness;
    params.samples = config_.samples;
    params.enableSelfContact = true;
    params.enableExternalContact = !objects_.empty();

    if (config_.frictionCoeff > 0.0) {
      Contact::SampledPenalty::FrictionParametersSpec friction;
      friction.frictionCoeff = config_.frictionCoeff;
      friction.velocityEps = config_.velocityEps;
      contactEnergy_ = std::make_shared<Contact::SampledPenalty::FrictionalSampledPenaltyContactEnergy>(
        surfaceMesh_, simulationRestPosition_, params, friction, copyObjectMeshes(objects_),
        vertexEmbeddingIndices_, vertexEmbeddingWeights_);
    }
    else {
      contactEnergy_ = std::make_shared<Contact::SampledPenalty::SampledPenaltyContactEnergy>(
        surfaceMesh_, simulationRestPosition_, params, copyObjectMeshes(objects_),
        vertexEmbeddingIndices_, vertexEmbeddingWeights_);
    }
  }

  ~SampledPenaltyContactBackend() override = default;

  ContactBackendKind kind() const override { return ContactBackendKind::SampledPenalty; }
  std::string description() const override { return "sampled-penalty"; }

  void initializeAfterRestart(const RunIPCSimRuntimeConfig &runtimeConfig,
    IpcSimulationContext &context, RunIPCSimSession &session) override
  {
    ES::mv(context.surfaceFromSimulationDispMap, session.u, session.usurf);
    const double denom = runtimeConfig.numSimSteps > 1 ? static_cast<double>(runtimeConfig.numSimSteps - 1) : 1.0;
    const double advanceFrames = static_cast<double>(session.frameStart + 1);
    for (std::size_t oi = 0; oi < objects_.size(); ++oi) {
      const ES::V3d movement = objects_[oi].movement / denom * advanceFrames;
      for (int vi = 0; vi < objects_[oi].mesh.numVertices(); ++vi)
        objects_[oi].mesh.pos(vi) += movement;
      updateExternalSurface(oi);
    }
  }

  void beginFrame(int, const RunIPCSimRuntimeConfig &, IpcSimulationContext &, RunIPCSimSession &) override
  {
  }

  void addForces(int, const RunIPCSimRuntimeConfig &runtimeConfig,
    IpcSimulationContext &context, RunIPCSimSession &session) override
  {
    (void)runtimeConfig;
    (void)context;
    if (contactEnergy_)
      session.transientContactModels.push_back({contactEnergy_, 0.0, 0.0});
  }

  void afterStep(int, const RunIPCSimRuntimeConfig &runtimeConfig,
    IpcSimulationContext &context, RunIPCSimSession &session) override
  {
    ES::mv(context.surfaceFromSimulationDispMap, session.u, session.usurf);

    const double denom = runtimeConfig.numSimSteps > 1 ? static_cast<double>(runtimeConfig.numSimSteps - 1) : 1.0;
    for (std::size_t oi = 0; oi < objects_.size(); ++oi) {
      const ES::V3d movement = objects_[oi].movement / denom;
      for (int vi = 0; vi < objects_[oi].mesh.numVertices(); ++vi)
        objects_[oi].mesh.pos(vi) += movement;
      updateExternalSurface(oi);
    }
  }

  void addStaticEnergies(const RunIPCSimRuntimeConfig &,
    IpcSimulationContext &, std::vector<NonlinearOptimization::EnergySet::Term> &terms) override
  {
    if (!contactEnergy_)
      return;
    if (config_.frictionCoeff > 0.0)
      throw std::invalid_argument("runIPCSim static sampled-penalty contact requires contact-friction-coeff == 0.");
    if (objects_.empty())
      return;

    Contact::SampledPenalty::ParametersSpec params;
    params.stiffness = config_.stiffness;
    params.samples = config_.samples;
    params.enableSelfContact = false;
    params.enableExternalContact = true;
    auto staticExternalEnergy = std::make_shared<Contact::SampledPenalty::SampledPenaltyContactEnergy>(
      surfaceMesh_, simulationRestPosition_, params, copyObjectMeshes(objects_),
      vertexEmbeddingIndices_, vertexEmbeddingWeights_);
    terms.push_back({std::make_shared<FrozenSampledPenaltyStaticEnergy>(staticExternalEnergy), 1.0});
  }

  void logSummary(const IpcSimulationContext &, const RunIPCSimSession &session) const override
  {
    logRunIPCSimMaxStepSummary(session.lastDiagnostics);
  }

private:
  void updateExternalSurface(std::size_t objectIndex)
  {
    if (!contactEnergy_)
      return;
    contactEnergy_->updateExternalSurface(static_cast<int>(objectIndex), objects_[objectIndex].mesh);
  }

  SampledPenaltyContactConfig config_;
  pgo::Mesh::TriMeshGeo surfaceMesh_;
  ES::VXd simulationRestPosition_;
  std::vector<SampledPenaltyKinematicObject> objects_;
  std::vector<int> vertexEmbeddingIndices_;
  std::vector<double> vertexEmbeddingWeights_;
  std::shared_ptr<Contact::SampledPenalty::SampledPenaltyContactEnergy> contactEnergy_;
};
}  // namespace

SampledPenaltyContactConfig parseSampledPenaltyContactConfig(const pgo::ConfigFileJSON &config)
{
  const bool hasContactSample = config.exist("contact-sample");
  const bool hasContactSamples = config.exist("contact-samples");
  if (hasContactSample && hasContactSamples)
    throw std::invalid_argument("runIPCSim sampled-penalty contact accepts either `contact-sample` or `contact-samples`, not both.");

  SampledPenaltyContactConfig parsed;
  parsed.stiffness = config.getDouble("contact-stiffness", 1);
  parsed.samples = hasContactSamples ? config.getInt("contact-samples", 1) : config.getInt("contact-sample", 1);
  parsed.frictionCoeff = config.getDouble("contact-friction-coeff", 1);
  parsed.velocityEps = config.getDouble("contact-vel-eps", 1);

  if (parsed.stiffness < 0.0)
    throw std::invalid_argument("runIPCSim sampled-penalty contact requires non-negative `contact-stiffness`.");
  if (parsed.samples <= 0)
    throw std::invalid_argument("runIPCSim sampled-penalty contact requires positive contact sample count.");
  if (parsed.velocityEps <= 0.0)
    throw std::invalid_argument("runIPCSim sampled-penalty contact requires positive `contact-vel-eps`.");

  return parsed;
}

std::shared_ptr<RunIPCSimContactBackend> makeSampledPenaltyContactBackend(
  const pgo::ConfigFileJSON &config,
  const SampledPenaltyContactConfig &contactConfig,
  const pgo::Mesh::TriMeshGeo &surfaceMesh,
  const std::vector<int> &embeddingVertexIndices,
  const std::vector<double> &embeddingWeights,
  ES::ConstRefVecXd simulationRestPosition,
  double scale)
{
  return std::make_shared<SampledPenaltyContactBackend>(
    contactConfig, surfaceMesh, simulationRestPosition, loadSampledPenaltyKinematicObjects(config, scale),
    embeddingVertexIndices, embeddingWeights);
}
}  // namespace pgo::RunIPCSim
