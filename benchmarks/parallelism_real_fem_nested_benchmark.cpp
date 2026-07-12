#include "deformation/deformationModelAssembler.h"
#include "deformation/deformationModelManager.h"
#include "formulations/formulation/formulations.h"
#include "material/fields/materialParameterFieldInit.h"
#include "parallelism/parallelOptions.h"
#include "parallelism/parallelRuntime.h"
#include "parallelism_benchmark_helpers.h"
#include "pgoLogging.h"
#include "simulation/simulationMesh.h"

#include <benchmark/benchmark.h>

#if defined(PGO_FEM_BENCHMARK_ACCELERATE)
#  include <Accelerate/Accelerate.h>
#elif defined(PGO_FEM_BENCHMARK_MKL)
#  include <mkl.h>
#endif

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{

namespace ES = pgo::EigenSupport;
namespace P = pgo::parallel;
namespace SDM = pgo::SolidDeformationModel;

using pgo::benchmark_helpers::ThreadSampler;
using pgo::benchmark_helpers::adjustedExtraThreads;
using pgo::benchmark_helpers::currentProcessThreadCount;

enum class Policy
{
  Suppress,
  Inherit,
};

enum class FormulationKind
{
  CubicLinear,
  CubicTricubicHermite,
};

enum class EvaluationKind
{
  Energy,
  Gradient,
  Hessian,
  Full,
};

#if defined(PGO_FEM_BENCHMARK_MKL)
constexpr int kRuntimeConcurrency = 16;
#else
constexpr int kRuntimeConcurrency = 8;
#endif

P::ParallelRuntime &benchmarkRuntime()
{
  return P::initializeRuntime({ .maxTbbConcurrency = kRuntimeConcurrency });
}

const char *policyName(Policy policy)
{
  return policy == Policy::Suppress ? "Suppress" : "Inherit";
}

const char *formulationName(FormulationKind formulation)
{
  return formulation == FormulationKind::CubicLinear ? "CubicLinear" : "CubicTricubicHermite";
}

const char *evaluationName(EvaluationKind evaluation)
{
  switch (evaluation) {
  case EvaluationKind::Energy:
    return "Energy";
  case EvaluationKind::Gradient:
    return "Gradient";
  case EvaluationKind::Hessian:
    return "Hessian";
  case EvaluationKind::Full:
    return "Full";
  }
  return "Unknown";
}

P::Options optionsForPolicy(Policy policy)
{
  P::Options options;
  options.grainSize = 1;
  options.nestedKernelPolicy =
    policy == Policy::Suppress ? P::NestedKernelPolicy::Suppress : P::NestedKernelPolicy::Inherit;
  return options;
}

std::shared_ptr<const SDM::SimulationMesh> makeCubicChainMesh(int numElements)
{
  std::vector<double> vertices(static_cast<std::size_t>(numElements + 1) * 4 * 3);
  auto vertexIndex = [](int x, int y, int z) { return x * 4 + z * 2 + y; };
  for (int x = 0; x <= numElements; ++x) {
    for (int z = 0; z < 2; ++z) {
      for (int y = 0; y < 2; ++y) {
        const int vi = vertexIndex(x, y, z);
        vertices[vi * 3 + 0] = static_cast<double>(x);
        vertices[vi * 3 + 1] = static_cast<double>(y);
        vertices[vi * 3 + 2] = static_cast<double>(z);
      }
    }
  }

  std::vector<int> elements(static_cast<std::size_t>(numElements) * 8);
  std::vector<int> materialIndices(static_cast<std::size_t>(numElements), 0);
  for (int e = 0; e < numElements; ++e) {
    const int corners[] = {
      vertexIndex(e, 0, 0), vertexIndex(e + 1, 0, 0),
      vertexIndex(e + 1, 1, 0), vertexIndex(e, 1, 0),
      vertexIndex(e, 0, 1), vertexIndex(e + 1, 0, 1),
      vertexIndex(e + 1, 1, 1), vertexIndex(e, 1, 1),
    };
    std::copy(std::begin(corners), std::end(corners), elements.begin() + e * 8);
  }

  SDM::SimulationMeshENuMaterial material(1200.0, 0.45);
  const SDM::SimulationMeshMaterial *materials[] = { &material };
  return std::shared_ptr<const SDM::SimulationMesh>(new SDM::SimulationMesh(
    (numElements + 1) * 4, vertices.data(), numElements, 8, elements.data(),
    materialIndices.data(), 1, materials, SDM::SimulationMeshType::CUBIC));
}

template<class Formulation>
std::unique_ptr<SDM::DeformationModelAssembler> makeAssembler(
  const std::shared_ptr<const SDM::SimulationMesh> &mesh, const Formulation &formulation)
{
  auto elasticField = SDM::createElasticParameterField(
    *mesh, SDM::DeformationModelElasticMaterial::STABLE_NEO, SDM::ElasticFieldInit{});
  auto plasticField = SDM::createPlasticParameterField(
    *mesh, SDM::DeformationModelPlasticMaterial::VOLUMETRIC_DOF0, SDM::PlasticFieldInit{});
  auto manager = std::make_shared<SDM::DeformationModelManager>(mesh,
    SDM::DeformationModelElasticMaterial::STABLE_NEO,
    SDM::DeformationModelPlasticMaterial::VOLUMETRIC_DOF0,
    formulation, 1, nullptr, nullptr);
  return std::make_unique<SDM::DeformationModelAssembler>(
    std::move(manager), formulation, std::move(elasticField), std::move(plasticField), nullptr);
}

class FemCase
{
public:
  FemCase(FormulationKind formulation, int numElements):
    formulation_(formulation), numElements_(numElements), mesh_(makeCubicChainMesh(numElements))
  {
    if (formulation == FormulationKind::CubicLinear)
      assembler_ = makeAssembler(mesh_, SDM::CubicLinearFormulation{});
    else
      assembler_ = makeAssembler(mesh_, SDM::CubicTricubicHermiteFormulation{});

    absolutePosition_ = makeDeformedPosition(assembler_->getRestPosition());
    gradient_ = ES::VXd::Zero(assembler_->getNumDOFs());
    hessian_ = assembler_->getHessianTemplate();
  }

  double evaluate(EvaluationKind evaluation, const P::Options &options)
  {
    double energy = 0.0;
    if (evaluation == EvaluationKind::Energy || evaluation == EvaluationKind::Full)
      energy = assembler_->computeEnergy(absolutePosition_.data(), options);
    if (evaluation == EvaluationKind::Gradient || evaluation == EvaluationKind::Full)
      assembler_->computeGradient(absolutePosition_.data(), gradient_.data(), options);
    if (evaluation == EvaluationKind::Hessian || evaluation == EvaluationKind::Full)
      assembler_->computeHessian(absolutePosition_.data(), hessian_, options);
    return energy;
  }

  double checksum(EvaluationKind evaluation, double energy) const
  {
    double value = energy;
    if (evaluation == EvaluationKind::Gradient || evaluation == EvaluationKind::Full)
      value += gradient_.squaredNorm() * 1e-6;
    if (evaluation == EvaluationKind::Hessian || evaluation == EvaluationKind::Full)
      value += hessian_.squaredNorm() * 1e-12;
    return value;
  }

  int numDofs() const { return assembler_->getNumDOFs(); }
  int localDofs() const { return formulation_ == FormulationKind::CubicLinear ? 24 : 192; }
  int quadraturePoints() const { return formulation_ == FormulationKind::CubicLinear ? 8 : 64; }
  int numElements() const { return numElements_; }
  Eigen::Index hessianNonZeros() const { return hessian_.nonZeros(); }

private:
  ES::VXd makeDeformedPosition(const ES::VXd &rest) const
  {
    ES::M3d deformation;
    deformation << 1.04, 0.02, 0.0,
      0.01, 0.98, 0.015,
      0.0, 0.01, 1.03;
    const ES::V3d translation(0.03, -0.02, 0.01);
    ES::VXd deformed(rest.size());
    const int modesPerVertex = formulation_ == FormulationKind::CubicLinear ? 1 : 8;
    const int numVertices = static_cast<int>(rest.size()) / (modesPerVertex * 3);
    for (int vertex = 0; vertex < numVertices; ++vertex) {
      for (int mode = 0; mode < modesPerVertex; ++mode) {
        const int offset = (vertex * modesPerVertex + mode) * 3;
        const ES::V3d restMode = rest.segment<3>(offset);
        deformed.segment<3>(offset) =
          deformation * restMode + (mode == 0 ? translation : ES::V3d::Zero());
      }
    }
    return deformed;
  }

  FormulationKind formulation_;
  int numElements_;
  std::shared_ptr<const SDM::SimulationMesh> mesh_;
  std::unique_ptr<SDM::DeformationModelAssembler> assembler_;
  ES::VXd absolutePosition_;
  ES::VXd gradient_;
  ES::SpMatD hessian_;
};

void recordBackendCounters(benchmark::State &state)
{
#if defined(PGO_FEM_BENCHMARK_ACCELERATE)
  state.counters["accelerate_threading"] = static_cast<int>(BLASGetThreading());
#elif defined(PGO_FEM_BENCHMARK_MKL)
  state.counters["mkl_max_threads"] = mkl_get_max_threads();
  state.counters["mkl_dynamic"] = mkl_get_dynamic();
#endif
}

void benchmarkRealFem(benchmark::State &state, FormulationKind formulation,
  EvaluationKind evaluation, Policy policy, int numElements)
{
  benchmarkRuntime();
  pgo::Logging::init();
  FemCase fem(formulation, numElements);
  const P::Options options = optionsForPolicy(policy);

  const int baselineThreads = currentProcessThreadCount();
  int peakThreads = baselineThreads;
  double energy = 0.0;
  for (auto _ : state) {
    state.PauseTiming();
    ThreadSampler sampler;
    sampler.start();
    state.ResumeTiming();

    energy = fem.evaluate(evaluation, options);
    benchmark::ClobberMemory();

    state.PauseTiming();
    peakThreads = std::max(peakThreads, sampler.stop());
    state.ResumeTiming();
  }

  const double checksum = fem.checksum(evaluation, energy);
  benchmark::DoNotOptimize(&checksum);
  const P::RuntimeInfo runtime = P::runtimeInfo();
  state.counters["runtime_concurrency"] = *runtime.maxConcurrency;
  state.counters["tbb_max_allowed_parallelism"] = runtime.effectiveTbbMaxAllowedParallelism;
  state.counters["elements"] = fem.numElements();
  state.counters["local_dofs"] = fem.localDofs();
  state.counters["global_dofs"] = fem.numDofs();
  state.counters["quadrature_points"] = fem.quadraturePoints();
  state.counters["hessian_nonzeros"] = static_cast<double>(fem.hessianNonZeros());
  state.counters["baseline_threads"] = baselineThreads;
  state.counters["peak_threads"] = peakThreads;
  state.counters["extra_threads"] = adjustedExtraThreads(baselineThreads, peakThreads);
  state.counters["policy"] = policy == Policy::Suppress ? 0 : 1;
  state.counters["checksum"] = checksum;
  recordBackendCounters(state);
}

void registerRealFemBenchmarks()
{
  constexpr Policy policies[] = { Policy::Suppress, Policy::Inherit };
  constexpr EvaluationKind evaluations[] = {
    EvaluationKind::Energy,
    EvaluationKind::Gradient,
    EvaluationKind::Hessian,
    EvaluationKind::Full,
  };
  constexpr FormulationKind formulations[] = {
    FormulationKind::CubicLinear,
    FormulationKind::CubicTricubicHermite,
  };

  for (FormulationKind formulation : formulations) {
    const int elementCounts[] = {
      formulation == FormulationKind::CubicLinear ? kRuntimeConcurrency : kRuntimeConcurrency / 2,
      formulation == FormulationKind::CubicLinear ? kRuntimeConcurrency * 4 : kRuntimeConcurrency,
    };
    for (EvaluationKind evaluation : evaluations) {
      for (Policy policy : policies) {
        for (int numElements : elementCounts) {
          const std::string name = std::string("RealFemPolicyDecision/") +
            formulationName(formulation) + "/" + evaluationName(evaluation) + "/" +
            policyName(policy) + "/elements_" + std::to_string(numElements);
          benchmark::RegisterBenchmark(name.c_str(), [=](benchmark::State &state) {
            benchmarkRealFem(state, formulation, evaluation, policy, numElements);
          })->UseRealTime()
            ->Unit(benchmark::kMillisecond);
        }
      }
    }
  }
}

const bool registered = [] {
  registerRealFemBenchmarks();
  return true;
}();

}  // namespace

BENCHMARK_MAIN();
