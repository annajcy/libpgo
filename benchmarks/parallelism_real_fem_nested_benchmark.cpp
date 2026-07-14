#include "deformation/deformationModelAssembler.h"
#include "deformation/deformationModelManager.h"
#include "formulations/formulation/formulations.h"
#include "material/fields/materialParameterFieldInit.h"
#include "parallel/parallelFor.h"
#include "parallel/parallelControl.h"
#include "parallelism_benchmark_helpers.h"
#include "pgoLogging.h"
#include "simulation/simulationMesh.h"

#include <benchmark/benchmark.h>

#include <tbb/parallel_for.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

namespace ES = pgo::EigenSupport;
namespace P = pgo::parallel;
namespace SDM = pgo::SolidDeformationModel;

using pgo::benchmark_helpers::ThreadSampler;
using pgo::benchmark_helpers::CurrentArenaThreadObserver;
using pgo::benchmark_helpers::adjustedExtraThreads;
using pgo::benchmark_helpers::currentProcessThreadCount;

enum class Policy
{
  Bounded1,
  Multi,
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

constexpr int kRuntimeConcurrency = 16;

int benchmarkRuntime()
{
  const char *configured = std::getenv("PGO_BENCHMARK_RUNTIME_CONCURRENCY");
  if (configured == nullptr)
    return P::setMaxConcurrency(kRuntimeConcurrency);

  const int parsed = std::atoi(configured);
  if (parsed <= 0)
    throw std::invalid_argument("PGO_BENCHMARK_RUNTIME_CONCURRENCY must be positive.");
  return P::setMaxConcurrency(parsed);
}

const char *policyName(Policy policy)
{
  switch (policy) {
  case Policy::Bounded1:
    return "Bounded1";
  case Policy::Multi:
    return "Multi";
  }
  return "Unknown";
}

int policyCode(Policy policy)
{
  return static_cast<int>(policy);
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

template<class Fn>
decltype(auto) runWithPolicy(Policy policy, Fn &&fn)
{
  if (policy == Policy::Bounded1)
    return P::withSingleThreadedTbb(std::forward<Fn>(fn));
  return fn();
}

std::shared_ptr<const SDM::SimulationMesh> makeCubicChainMesh(int numElements)
{
  std::vector<double> vertices(static_cast<std::size_t>(numElements + 1) * 4 * 3);
  auto vertexIndex = [](int x, int y, int z) {
    return x * 4 + z * 2 + y;
  };
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
      vertexIndex(e, 0, 0),
      vertexIndex(e + 1, 0, 0),
      vertexIndex(e + 1, 1, 0),
      vertexIndex(e, 1, 0),
      vertexIndex(e, 0, 1),
      vertexIndex(e + 1, 0, 1),
      vertexIndex(e + 1, 1, 1),
      vertexIndex(e, 1, 1),
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

  double evaluate(EvaluationKind evaluation)
  {
    double energy = 0.0;
    if (evaluation == EvaluationKind::Energy || evaluation == EvaluationKind::Full)
      energy = assembler_->computeEnergy(absolutePosition_.data());
    if (evaluation == EvaluationKind::Gradient || evaluation == EvaluationKind::Full)
      assembler_->computeGradient(absolutePosition_.data(), gradient_.data());
    if (evaluation == EvaluationKind::Hessian || evaluation == EvaluationKind::Full)
      assembler_->computeHessian(absolutePosition_.data(), hessian_);
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

void benchmarkRealFem(benchmark::State &state, FormulationKind formulation,
  EvaluationKind evaluation, Policy policy, int numElements, int expectedRuntimeConcurrency)
{
  const int effectiveRuntimeConcurrency = benchmarkRuntime();
  if (effectiveRuntimeConcurrency != expectedRuntimeConcurrency) {
    throw std::logic_error("Benchmark name/runtime concurrency environment mismatch.");
  }
  pgo::Logging::init();
  FemCase fem(formulation, numElements);
  std::atomic<int> observedArenaConcurrency = -1;
  std::atomic<int> observedPeakArenaThreads = 1;
  P::parallelFor(0, 1, [&](int) {
    runWithPolicy(policy, [&] {
      CurrentArenaThreadObserver observer;
      observedArenaConcurrency.store(
        tbb::this_task_arena::max_concurrency(), std::memory_order_relaxed);
      tbb::parallel_for(0, std::max(64, expectedRuntimeConcurrency * 64), [](int i) {
        benchmark::DoNotOptimize(i);
        std::this_thread::yield();
      });
      observedPeakArenaThreads.store(observer.peak(), std::memory_order_relaxed);
    });
  });

  const int baselineThreads = currentProcessThreadCount();
  int peakThreads = baselineThreads;
  double energy = 0.0;
  for (auto _ : state) {
    state.PauseTiming();
    ThreadSampler sampler;
    sampler.start();
    state.ResumeTiming();

    energy = runWithPolicy(policy, [&] {
      return fem.evaluate(evaluation);
    });
    benchmark::ClobberMemory();

    state.PauseTiming();
    peakThreads = std::max(peakThreads, sampler.stop());
    state.ResumeTiming();
  }

  const double checksum = fem.checksum(evaluation, energy);
  benchmark::DoNotOptimize(&checksum);
  state.counters["runtime_concurrency"] = effectiveRuntimeConcurrency;
  state.counters["tbb_max_allowed_parallelism"] = effectiveRuntimeConcurrency;
  state.counters["elements"] = fem.numElements();
  state.counters["local_dofs"] = fem.localDofs();
  state.counters["global_dofs"] = fem.numDofs();
  state.counters["quadrature_points"] = fem.quadraturePoints();
  state.counters["hessian_nonzeros"] = static_cast<double>(fem.hessianNonZeros());
  state.counters["baseline_threads"] = baselineThreads;
  state.counters["peak_threads"] = peakThreads;
  state.counters["extra_threads"] = adjustedExtraThreads(baselineThreads, peakThreads);
  state.counters["policy"] = policyCode(policy);
  state.counters["checksum"] = checksum;
  state.counters["worker_arena_concurrency"] = observedArenaConcurrency.load(std::memory_order_relaxed);
  state.counters["observed_peak_arena_threads"] = observedPeakArenaThreads.load(std::memory_order_relaxed);
}

void registerRealFemBenchmarks()
{
  constexpr Policy policies[] = { Policy::Bounded1, Policy::Multi };
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
            benchmarkRealFem(
              state, formulation, evaluation, policy, numElements, kRuntimeConcurrency);
          })->UseRealTime()
            ->Unit(benchmark::kMillisecond);
        }
      }
    }
  }
}

void registerMklTbbBoundRealFemDecisionBenchmarks()
{
  constexpr Policy policies[] = { Policy::Bounded1, Policy::Multi };
  constexpr EvaluationKind evaluations[] = {
    EvaluationKind::Hessian,
    EvaluationKind::Full,
  };
  constexpr FormulationKind formulations[] = {
    FormulationKind::CubicLinear,
    FormulationKind::CubicTricubicHermite,
  };
  constexpr int runtimeConcurrencies[] = { 1, 4, 8, 16 };

  for (int runtimeConcurrency : runtimeConcurrencies) {
    const std::set<int> elementCounts = {
      1,
      std::max(1, runtimeConcurrency / 2),
      runtimeConcurrency,
      runtimeConcurrency * 4,
    };
    for (FormulationKind formulation : formulations) {
      for (EvaluationKind evaluation : evaluations) {
        for (Policy policy : policies) {
          for (int numElements : elementCounts) {
            const std::string name = std::string("MklTbbBoundRealFemDecision/") +
              formulationName(formulation) + "/" + evaluationName(evaluation) + "/" +
              policyName(policy) + "/runtime_workers_" + std::to_string(runtimeConcurrency) +
              "/elements_" + std::to_string(numElements);
            benchmark::RegisterBenchmark(name.c_str(), [=](benchmark::State &state) {
              benchmarkRealFem(
                state, formulation, evaluation, policy, numElements, runtimeConcurrency);
            })->UseRealTime()
              ->Unit(benchmark::kMillisecond);
          }
        }
      }
    }
  }
}

const bool registered = [] {
  registerRealFemBenchmarks();
  registerMklTbbBoundRealFemDecisionBenchmarks();
  return true;
}();

}  // namespace

BENCHMARK_MAIN();
