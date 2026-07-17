#include "energy/deformationEnergyBuilder.h"
#include "energy/deformationModelEnergy.h"
#include "formulations/formulation/formulations.h"
#include "simulation/simulationMesh.h"

#include "EigenSupport.h"
#include "cubicMesh.h"
#include "parallel/arenaThreadingExecutor.h"
#include "parallel/parallelControl.h"
#include "pgoLogging.h"
#include "solver/newton/newtonSparseSolverBackend.h"

#include <mkl.h>
#include <tbb/global_control.h>
#include <tbb/task_arena.h>
#include <tbb/task_scheduler_observer.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
namespace NO = pgo::NonlinearOptimization;
namespace P = pgo::parallel;
namespace SDM = pgo::SolidDeformationModel;

using Clock = std::chrono::steady_clock;

enum class PreludeKind
{
  None,
  Noop,
  Pardiso,
};

struct CaseSpec
{
  std::string_view name;
  PreludeKind prelude;
  int linearBudget;
};

constexpr CaseSpec kCases[] = {
  { "none", PreludeKind::None, 0 },
  { "noop1", PreludeKind::Noop, 1 },
  { "noop8", PreludeKind::Noop, 8 },
  { "pardiso1", PreludeKind::Pardiso, 1 },
  { "pardiso8", PreludeKind::Pardiso, 8 },
};

struct Arguments
{
  CaseSpec caseSpec;
  std::string meshPath;
  int concurrency;
  int reservedSlots;
  int warmupIterations;
  int measuredIterations;
};

struct Signature
{
  double energy = 0.0;
  double gradientSquaredNorm = 0.0;
  double gradientMaxAbs = 0.0;
  double hessianAbsSum = 0.0;
  double hessianSquaredNorm = 0.0;
  double hessianMaxAbs = 0.0;
};

struct Measurement
{
  double preludeSeconds = 0.0;
  double preludeProcessCpuSeconds = 0.0;
  double evaluationExecuteSeconds = 0.0;
  double evaluationKernelSeconds = 0.0;
  double solveSquaredNorm = 0.0;
  int observedLinearMklBudget = -1;
  int observedEvaluationMklBudget = -1;
  int observedEvaluationArenaConcurrency = -1;
  int preludeWorkerEntries = 0;
  int preludePeakWorkers = 0;
  Signature signature;
};

class PreludeObserver final : public tbb::task_scheduler_observer
{
public:
  int workerEntries() const noexcept { return workerEntries_.load(std::memory_order_relaxed); }
  int peakWorkers() const noexcept { return peakWorkers_.load(std::memory_order_relaxed); }

private:
  void on_scheduler_entry(bool isWorker) override
  {
    if (!isWorker)
      return;
    workerEntries_.fetch_add(1, std::memory_order_relaxed);
    const int active = activeWorkers_.fetch_add(1, std::memory_order_relaxed) + 1;
    int observed = peakWorkers_.load(std::memory_order_relaxed);
    while (active > observed &&
      !peakWorkers_.compare_exchange_weak(observed, active, std::memory_order_relaxed)) {
    }
  }

  void on_scheduler_exit(bool isWorker) override
  {
    if (isWorker)
      activeWorkers_.fetch_sub(1, std::memory_order_relaxed);
  }

  std::atomic<int> workerEntries_{ 0 };
  std::atomic<int> activeWorkers_{ 0 };
  std::atomic<int> peakWorkers_{ 0 };
};

int parseInteger(std::string_view text, std::string_view option, bool allowZero)
{
  const std::string value(text);
  char *end = nullptr;
  const long parsed = std::strtol(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0' || parsed < (allowZero ? 0 : 1) ||
    parsed > INT_MAX) {
    throw std::invalid_argument(std::string(option) + " has an invalid integer value");
  }
  return static_cast<int>(parsed);
}

std::string_view requireValue(int argc, char **argv, std::string_view prefix)
{
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    if (argument.starts_with(prefix))
      return argument.substr(prefix.size());
  }
  throw std::invalid_argument("Missing required option " + std::string(prefix));
}

CaseSpec parseCase(std::string_view name)
{
  for (const CaseSpec &candidate : kCases) {
    if (candidate.name == name)
      return candidate;
  }
  throw std::invalid_argument("Unknown --case value: " + std::string(name));
}

Arguments parseArguments(int argc, char **argv)
{
  return {
    parseCase(requireValue(argc, argv, "--case=")),
    std::string(requireValue(argc, argv, "--mesh=")),
    parseInteger(requireValue(argc, argv, "--concurrency="), "--concurrency", false),
    parseInteger(requireValue(argc, argv, "--reserved-slots="), "--reserved-slots", true),
    parseInteger(requireValue(argc, argv, "--warmup-iterations="), "--warmup-iterations", true),
    parseInteger(requireValue(argc, argv, "--measured-iterations="), "--measured-iterations", false),
  };
}

double secondsBetween(Clock::time_point begin, Clock::time_point end)
{
  return std::chrono::duration<double>(end - begin).count();
}

std::vector<int> fixedTopDofs(const SDM::SimulationMesh &mesh)
{
  double minY = std::numeric_limits<double>::infinity();
  double maxY = -std::numeric_limits<double>::infinity();
  for (int vertex = 0; vertex < mesh.getNumVertices(); ++vertex) {
    double position[3];
    mesh.getVertex(vertex, position);
    minY = std::min(minY, position[1]);
    maxY = std::max(maxY, position[1]);
  }

  const double threshold = maxY - 0.01 * (maxY - minY);
  std::vector<int> fixed;
  for (int vertex = 0; vertex < mesh.getNumVertices(); ++vertex) {
    double position[3];
    mesh.getVertex(vertex, position);
    if (position[1] + 1e-12 >= threshold) {
      fixed.push_back(vertex * 3 + 0);
      fixed.push_back(vertex * 3 + 1);
      fixed.push_back(vertex * 3 + 2);
    }
  }
  if (fixed.empty())
    throw std::runtime_error("The fixed-top boundary selected no vertices");
  return fixed;
}

ES::VXd makePerturbation(int dofs, const std::vector<int> &fixedDofs)
{
  std::vector<unsigned char> isFixed(static_cast<std::size_t>(dofs), 0);
  for (int dof : fixedDofs)
    isFixed.at(static_cast<std::size_t>(dof)) = 1;

  ES::VXd x(dofs);
  for (int dof = 0; dof < dofs; ++dof) {
    x[dof] = isFixed[static_cast<std::size_t>(dof)] ? 0.0 :
                                                      1e-4 * std::sin(0.73 * static_cast<double>(dof + 1));
  }
  return x;
}

Signature makeSignature(double energy, const ES::VXd &gradient, const ES::SpMatD &hessian)
{
  Signature signature;
  signature.energy = energy;
  signature.gradientSquaredNorm = gradient.squaredNorm();
  signature.gradientMaxAbs = gradient.cwiseAbs().maxCoeff();
  for (Eigen::Index index = 0; index < hessian.nonZeros(); ++index) {
    const double value = hessian.valuePtr()[index];
    signature.hessianAbsSum += std::abs(value);
    signature.hessianSquaredNorm += value * value;
    signature.hessianMaxAbs = std::max(signature.hessianMaxAbs, std::abs(value));
  }
  return signature;
}

void requireFinite(const Measurement &measurement)
{
  const double values[] = {
    measurement.preludeSeconds,
    measurement.preludeProcessCpuSeconds,
    measurement.evaluationExecuteSeconds,
    measurement.evaluationKernelSeconds,
    measurement.solveSquaredNorm,
    measurement.signature.energy,
    measurement.signature.gradientSquaredNorm,
    measurement.signature.gradientMaxAbs,
    measurement.signature.hessianAbsSum,
    measurement.signature.hessianSquaredNorm,
    measurement.signature.hessianMaxAbs,
  };
  for (double value : values) {
    if (!std::isfinite(value))
      throw std::runtime_error("A benchmark measurement is not finite");
  }
}

const char *preludeName(PreludeKind kind)
{
  switch (kind) {
  case PreludeKind::None:
    return "none";
  case PreludeKind::Noop:
    return "noop";
  case PreludeKind::Pardiso:
    return "pardiso";
  }
  return "unknown";
}

}  // namespace

int main(int argc, char **argv)
try {
  const Arguments arguments = parseArguments(argc, argv);
  if (arguments.reservedSlots >= arguments.concurrency)
    throw std::invalid_argument("--reserved-slots must be smaller than --concurrency");

  pgo::Logging::init();
  P::GlobalTbbControl globalControl(arguments.concurrency);
  const int effectiveConcurrency = static_cast<int>(tbb::global_control::active_value(
    tbb::global_control::max_allowed_parallelism));

  P::ArenaThreadingExecutor setupExecutor(arguments.concurrency,
    P::ThreadingPolicy{ .mklLocalThreadBudget = 1 }, arguments.reservedSlots);
  P::ArenaThreadingExecutor evaluationExecutor(arguments.concurrency,
    P::ThreadingPolicy{ .mklLocalThreadBudget = 1 }, arguments.reservedSlots);
  P::ArenaThreadingExecutor linearExecutor1(arguments.concurrency,
    P::ThreadingPolicy{ .mklLocalThreadBudget = 1 }, arguments.reservedSlots);
  P::ArenaThreadingExecutor linearExecutor8(arguments.concurrency,
    P::ThreadingPolicy{ .mklLocalThreadBudget = 8 }, arguments.reservedSlots);

  std::shared_ptr<const SDM::SimulationMesh> mesh;
  std::shared_ptr<SDM::DeformationModelEnergy> energy;
  std::unique_ptr<NO::NewtonSparseSolverBackend> pardiso;
  ES::VXd x;
  ES::VXd gradient;
  ES::SpMatD hessian;
  ES::SpMatD reducedHessian;
  ES::VXd rhs;
  ES::VXd solution;
  std::vector<int> fixedDofs;
  double currentEnergy = 0.0;

  setupExecutor.execute([&] {
    pgo::VolumetricMeshes::CubicMesh cubicMesh(arguments.meshPath.c_str());
    mesh = std::shared_ptr<const SDM::SimulationMesh>(SDM::loadCubicMesh(&cubicMesh).release());
    SDM::DeformationModelOptions options;
    options.enforceSPD = true;
    options.enableMaterialMaxStep = false;
    energy = SDM::makeDeformationEnergy(mesh,
      SDM::DeformationModelElasticMaterial::STABLE_NEO,
      SDM::DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
      SDM::CubicLinearFormulation{}, options);
    fixedDofs = fixedTopDofs(*mesh);
    x = makePerturbation(energy->getNumDOFs(), fixedDofs);
    gradient.setZero(energy->getNumDOFs());
    energy->hessianAlloc(hessian);
    currentEnergy = energy->func_grad_hessian(x, gradient, hessian);
    ES::removeRowsCols(hessian, fixedDofs, reducedHessian);
    rhs = ES::VXd::Ones(reducedHessian.rows());
    solution = ES::VXd::Zero(reducedHessian.rows());
    NO::MKLPardisoSparseSolverSelector selector;
    pardiso = selector.build(reducedHessian);
  });

  const int totalIterations = arguments.warmupIterations + arguments.measuredIterations;
  std::vector<Measurement> measurements;
  measurements.reserve(static_cast<std::size_t>(arguments.measuredIterations));

  for (int iteration = 0; iteration < totalIterations; ++iteration) {
    rhs.setOnes();
    solution.setZero();
    Measurement measurement;

    const auto preludeBegin = Clock::now();
    const std::clock_t preludeCpuBegin = std::clock();
    auto runPrelude = [&](P::ArenaThreadingExecutor &executor) {
      executor.execute([&] {
        PreludeObserver observer;
        observer.observe(true);
        measurement.observedLinearMklBudget = mkl_get_max_threads();
        if (arguments.caseSpec.prelude == PreludeKind::Pardiso) {
          if (!pardiso->factorize(reducedHessian))
            throw std::runtime_error("MKL PARDISO factorization failed");
          if (!pardiso->solve(reducedHessian, solution.data(), rhs.data()))
            throw std::runtime_error("MKL PARDISO solve failed");
        }
        observer.observe(false);
        measurement.preludeWorkerEntries = observer.workerEntries();
        measurement.preludePeakWorkers = observer.peakWorkers();
      });
    };
    if (arguments.caseSpec.prelude != PreludeKind::None) {
      if (arguments.caseSpec.linearBudget == 1)
        runPrelude(linearExecutor1);
      else
        runPrelude(linearExecutor8);
    }
    const auto preludeEnd = Clock::now();
    const std::clock_t preludeCpuEnd = std::clock();
    measurement.preludeSeconds = secondsBetween(preludeBegin, preludeEnd);
    measurement.preludeProcessCpuSeconds =
      static_cast<double>(preludeCpuEnd - preludeCpuBegin) / CLOCKS_PER_SEC;

    const auto evaluationExecuteBegin = Clock::now();
    evaluationExecutor.execute([&] {
      measurement.observedEvaluationMklBudget = mkl_get_max_threads();
      measurement.observedEvaluationArenaConcurrency = tbb::this_task_arena::max_concurrency();
      const auto kernelBegin = Clock::now();
      currentEnergy = energy->func_grad_hessian(x, gradient, hessian);
      const auto kernelEnd = Clock::now();
      measurement.evaluationKernelSeconds = secondsBetween(kernelBegin, kernelEnd);
    });
    const auto evaluationExecuteEnd = Clock::now();
    measurement.evaluationExecuteSeconds =
      secondsBetween(evaluationExecuteBegin, evaluationExecuteEnd);
    measurement.solveSquaredNorm = solution.squaredNorm();
    measurement.signature = makeSignature(currentEnergy, gradient, hessian);
    requireFinite(measurement);

    // This untimed reduction prepares the next PARDISO prelude. It is deliberately
    // after evaluation so nothing intervenes between the measured prelude and FGH.
    ES::removeRowsCols(hessian, fixedDofs, reducedHessian);

    if (iteration >= arguments.warmupIterations)
      measurements.push_back(measurement);
  }

  std::cout << std::setprecision(17);
  for (std::size_t index = 0; index < measurements.size(); ++index) {
    const Measurement &measurement = measurements[index];
    std::cout << "PGO_CUBIC_LINEAR_PARDISO_AFTERMATH_RESULT"
              << " case=" << arguments.caseSpec.name
              << " prelude=" << preludeName(arguments.caseSpec.prelude)
              << " iteration=" << index
              << " configured_global_concurrency=" << arguments.concurrency
              << " effective_global_concurrency=" << effectiveConcurrency
              << " configured_arena_concurrency=" << arguments.concurrency
              << " configured_linear_mkl_budget=" << arguments.caseSpec.linearBudget
              << " configured_evaluation_mkl_budget=1"
              << " observed_linear_mkl_budget=" << measurement.observedLinearMklBudget
              << " observed_evaluation_mkl_budget=" << measurement.observedEvaluationMklBudget
              << " observed_evaluation_arena_concurrency=" << measurement.observedEvaluationArenaConcurrency
              << " prelude_worker_entries=" << measurement.preludeWorkerEntries
              << " prelude_peak_workers=" << measurement.preludePeakWorkers
              << " mesh_vertices=" << mesh->getNumVertices()
              << " mesh_elements=" << mesh->getNumElements()
              << " dofs=" << energy->getNumDOFs()
              << " fixed_dofs=" << fixedDofs.size()
              << " reduced_rows=" << reducedHessian.rows()
              << " reduced_nnz=" << reducedHessian.nonZeros()
              << " prelude_seconds=" << measurement.preludeSeconds
              << " prelude_process_cpu_seconds=" << measurement.preludeProcessCpuSeconds
              << " evaluation_execute_seconds=" << measurement.evaluationExecuteSeconds
              << " evaluation_kernel_seconds=" << measurement.evaluationKernelSeconds
              << " energy=" << measurement.signature.energy
              << " gradient_squared_norm=" << measurement.signature.gradientSquaredNorm
              << " gradient_max_abs=" << measurement.signature.gradientMaxAbs
              << " hessian_abs_sum=" << measurement.signature.hessianAbsSum
              << " hessian_squared_norm=" << measurement.signature.hessianSquaredNorm
              << " hessian_max_abs=" << measurement.signature.hessianMaxAbs
              << " solve_squared_norm=" << measurement.solveSquaredNorm
              << '\n';
  }

  setupExecutor.execute([&] { pardiso.reset(); });
  return 0;
}
catch (const std::exception &exception) {
  std::cerr << "cubic_linear_pardiso_aftermath_probe: " << exception.what() << '\n';
  return 1;
}
