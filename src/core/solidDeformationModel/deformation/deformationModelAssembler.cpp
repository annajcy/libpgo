/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "deformation/deformationModelAssembler.h"
#include "deformation/materialMaxStepPolynomialUtils.h"
#include "formulations/formulation/formulation.h"
#include "simulation/simulationMesh.h"
#include "deformation/deformationElement.h"
#include "material/elastic/elasticModel.h"
#include "material/elastic/elasticModelDefinition.h"
#include "material/plastic/plasticModel.h"
#include "material/plastic/plasticModelDefinition.h"
#include "material/runtime/materialBinding.h"
#include "material/runtime/materialState.h"

#include "pgoLogging.h"
#include "EigenSupport.h"
#include "fmtEigen.h"
#include "processMemory.h"

#include <algorithm>
#include <atomic>
#include <set>
#include <stdexcept>
#include <string>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

using namespace pgo::SolidDeformationModel;

namespace ES = pgo::EigenSupport;

namespace
{
std::span<const double> vectorSpan(const ES::VXd &values)
{
  return { values.data(), static_cast<std::size_t>(values.size()) };
}

void validateFormulation(
  SimulationMeshType meshType, const Formulation &formulation)
{
  if (formulation.compatibleMeshType() != meshType)
    throw std::invalid_argument("formulation does not match mesh type");
}

void logMemoryCheckpoint(const char *stage)
{
  constexpr double bytesPerMiB = 1024.0 * 1024.0;
  const pgo::Profiling::ProcessMemoryUsage usage =
    pgo::Profiling::recordProcessMemoryProfileCounters(stage);
  SPDLOG_LOGGER_INFO(pgo::Logging::lgr(),
    "Process memory checkpoint stage={} currentMiB={:.2f} peakMiB={:.2f}",
    stage, usage.residentBytes / bytesPerMiB, usage.peakResidentBytes / bytesPerMiB);
}

void validateFiniteValues(std::span<const double> values, Eigen::Index count, const char *label)
{
  for (Eigen::Index i = 0; i < count; i++) {
    int fpclass = std::fpclassify(values[i]);
    if (fpclass == FP_INFINITE || fpclass == FP_NAN) {
      SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Encounter weird {} numbers at {}: {}", label, i, values[i]);
      throw std::logic_error("Encounter weird numbers.");
    }
  }
}

void warnIllegalInitialState(pgo::SolidDeformationModel::SimulationMeshType meshType, int elementId, int locationId, double phi0, double eps)
{
  if (locationId >= 0) {
    SPDLOG_LOGGER_WARN(pgo::Logging::lgr(),
      "material max step encountered illegal initial state on meshType={} element={} location={} : phi(0)={} <= eps={}. Returning recovery clamp {}.",
      meshTypeName(meshType), elementId, locationId, phi0, eps, pgo::SolidDeformationModel::kMaterialMaxStepMinClamp);
  }
  else {
    SPDLOG_LOGGER_WARN(pgo::Logging::lgr(),
      "material max step encountered illegal initial state on meshType={} element={} : phi(0)={} <= eps={}. Returning recovery clamp {}.",
      meshTypeName(meshType), elementId, phi0, eps, pgo::SolidDeformationModel::kMaterialMaxStepMinClamp);
  }
}

}  // namespace

DeformationModelAssembler::Element::Element(
  std::unique_ptr<DeformationElement> deformation_, int localDofs,
  int maxMaterialLocations, int maxMaterialParams):
  deformation(std::move(deformation_)),
  localPosition(localDofs),
  localDirection(localDofs),
  localGradient(localDofs),
  localParameterGradient(maxMaterialParams),
  localMatrixData(static_cast<std::size_t>(localDofs * localDofs)),
  materialLocationValues(static_cast<std::size_t>(
    std::max(16, maxMaterialLocations)))
{
  if (!deformation)
    throw std::invalid_argument(
      "DeformationModelAssembler element must not be null.");
}

DeformationModelAssembler::DeformationModelAssembler(
  const SimulationMesh &mesh,
  const MaterialBinding &materialBinding,
  const Formulation &formulation,
  bool projectHessianPSD,
  std::span<const double> elementWeights_):
  meshType_(mesh.getElementType()),
  numVertices_(mesh.getNumVertices())
{
  if (materialBinding.numElements() != mesh.getNumElements())
    throw std::invalid_argument(
      "DeformationModelAssembler material binding element count does not match mesh.");
  if (mesh.getNumElements() <= 0)
    throw std::invalid_argument(
      "DeformationModelAssembler mesh must contain at least one element.");
  validateFormulation(mesh.getElementType(), formulation);

  dofLayout_ = formulation.createDofLayout(mesh);
  restDofs_ = formulation.buildGlobalRestDofs(mesh);
  nele = mesh.getNumElements();
  localDOFs = dofLayout_->numLocalDofs(0);
  numDOFs = dofLayout_->numGlobalDofs();
  numElasticParams_ = materialBinding.elastic().numOptimizableChannels();
  numPlasticParams_ = materialBinding.plastic().numOptimizableChannels();
  for (int ele = 1; ele < nele; ++ele) {
    if (dofLayout_->numLocalDofs(ele) != localDOFs)
      throw std::invalid_argument(
        "A formulation must use a fixed local DOF count across all elements.");
  }

  if (!elementWeights_.empty()) {
    if (elementWeights_.size() != static_cast<std::size_t>(nele))
      throw std::invalid_argument("DeformationModelAssembler element weight count does not match the mesh.");
    elementWeights.assign(elementWeights_.begin(), elementWeights_.end());
  }
  else {
    elementWeights.assign(nele, 1);
  }

  initializeElements(mesh, materialBinding, formulation,
    DeformationElementConstructionOptions{ projectHessianPSD });
  for (int ele = 0; ele < nele; ele++)
    dofLayout_->getDofGroups(ele, elements_[ele].dofGroups);
  logMemoryCheckpoint("assembler.after_element_cache_setup");

  SPDLOG_LOGGER_INFO(Logging::lgr(), "Assembler elementwise parameter channels:{},{}",
    numElasticParams_, numPlasticParams_);

  // Hessian template.
  std::set<SparseBlockKey> hessianBlocks;
  dofLayout_->collectSparseBlockPairs(nele, hessianBlocks);
  logMemoryCheckpoint("assembler.after_unique_hessian_block_collection");

  buildSparseMatrixTemplate(
    numDOFs, hessianBlocks, hessianAssembly_.matrixTemplate);
  logMemoryCheckpoint("assembler.after_compressed_hessian_template");

  dofLayout_->buildAllSparseBlockOffsets(nele,
    hessianAssembly_.matrixTemplate,
    hessianAssembly_.elementBlockOffsets);
  logMemoryCheckpoint("assembler.after_element_block_offsets");
}

void DeformationModelAssembler::initializeElements(
  const SimulationMesh &mesh,
  const MaterialBinding &binding,
  const Formulation &formulation,
  DeformationElementConstructionOptions options)
{
  SPDLOG_LOGGER_INFO(
    pgo::Logging::lgr(), "Initializing deformation elements...");

  const auto &elastic = binding.elastic();
  const auto &plastic = binding.plastic();
  const int numElasticFixed = elastic.numFixedChannels();
  const int numPlasticFixed = plastic.numFixedChannels();
  const auto &elasticFixed = elastic.fixedValues();
  const auto &plasticFixed = plastic.fixedValues();

  std::vector<std::unique_ptr<DeformationElement>> deformations(nele);
  tbb::parallel_for(0, nele, [&](int ele) {
    const MaterialFrame &frame = binding.materialFrames()[ele];
    const std::span<const double> elasticValues(
      numElasticFixed ? elasticFixed.data() + ele * numElasticFixed : nullptr,
      static_cast<std::size_t>(numElasticFixed));
    const std::span<const double> plasticValues(
      numPlasticFixed ? plasticFixed.data() + ele * numPlasticFixed : nullptr,
      static_cast<std::size_t>(numPlasticFixed));
    auto elasticModel = elastic.definition()->createModel(elasticValues, frame);
    auto plasticModel = plastic.definition()->createModel(plasticValues, frame);
    deformations[ele] = formulation.createElement(
      mesh, ele, std::move(elasticModel), std::move(plasticModel), options);
  });

  int maxMaterialLocations = 0;
  for (int ele = 0; ele < nele; ++ele) {
    if (!deformations[ele])
      throw std::runtime_error("Formulation returned a null deformation element.");
    if (deformations[ele]->getNumElasticParameters() != numElasticParams_ ||
      deformations[ele]->getNumPlasticParameters() != numPlasticParams_)
      throw std::invalid_argument(
        "Deformation elements have inconsistent optimizable parameter channels.");
    maxMaterialLocations = std::max(
      maxMaterialLocations, deformations[ele]->getNumMaterialLocations());
  }

  const int maxMaterialParams =
    std::max(numElasticParams_, numPlasticParams_);
  elements_.reserve(nele);
  for (auto &deformation : deformations)
    elements_.emplace_back(std::move(deformation), localDOFs,
      maxMaterialLocations, maxMaterialParams);
}

DeformationModelAssembler::~DeformationModelAssembler() = default;

const DeformationElement &DeformationModelAssembler::element(
  int elementId) const
{
  if (elementId < 0 || elementId >= nele)
    throw std::out_of_range("Deformation element index is out of range.");
  return *elements_[elementId].deformation;
}

void DeformationModelAssembler::validateMaterialState(
  const MaterialStateView &state) const
{
  if (state.elasticValues().size() != static_cast<std::size_t>(getNumElasticGlobalParams()))
    throw std::invalid_argument("MaterialStateView elastic value count does not match the assembler.");
  if (state.plasticValues().size() != static_cast<std::size_t>(getNumPlasticGlobalParams()))
    throw std::invalid_argument("MaterialStateView plastic value count does not match the assembler.");
}

void DeformationModelAssembler::validatePositionSpan(
  std::span<const double> x, const char *label) const
{
  if (x.size() != static_cast<std::size_t>(numDOFs))
    throw std::invalid_argument(
      std::string("DeformationModelAssembler ") + label + " has unexpected size.");
}

void DeformationModelAssembler::gatherPosition(
  std::span<const double> x, Element &scratch) const
{
  std::fill(scratch.localPosition.data(), scratch.localPosition.data() + localDOFs, 0.0);
  for (const DofGroup &group : scratch.dofGroups) {
    for (int i = 0; i < group.size; i++)
      scratch.localPosition[group.localStart + i] = x[group.globalDof(i)];
  }
}

std::span<const double> DeformationModelAssembler::elasticValues(
  int elementId, const MaterialStateView &state) const
{
  return state.elasticValues().subspan(
    static_cast<std::size_t>(elementId * numElasticParams_),
    numElasticParams_);
}

std::span<const double> DeformationModelAssembler::plasticValues(
  int elementId, const MaterialStateView &state) const
{
  return state.plasticValues().subspan(
    static_cast<std::size_t>(elementId * numPlasticParams_),
    numPlasticParams_);
}

double DeformationModelAssembler::compute_E(
  std::span<const double> x, MaterialStateView state) const
{
  validatePositionSpan(x, "position vector");
  validateMaterialState(state);

  auto localEnergyFunc = [this, x, &state](int ele) {
    auto &scratch = elements_[ele];
    scratch.energy = 0.0;
    if (elementWeights[ele] == 0)
      return;

    gatherPosition(x, scratch);
    double energy = scratch.deformation->computeEnergy(
      vectorSpan(scratch.localPosition), elasticValues(ele, state),
      plasticValues(ele, state));

    scratch.energy = energy * elementWeights[ele];
  };

  tbb::parallel_for(0, nele, localEnergyFunc);

  double energyAll = 0;
  for (int ele = 0; ele < nele; ele++)
    energyAll += elements_[ele].energy;

  return energyAll;
}

DeformationModelAssembler::MaterialMaxStepObservation DeformationModelAssembler::computeMaxStepObservation(std::span<const double> x, std::span<const double> dx) const
{
  validatePositionSpan(x, "position vector");
  validatePositionSpan(dx, "direction vector");
  MaterialMaxStepObservation observation;
  const SimulationMeshType meshType = meshType_;

  for (int ele = 0; ele < nele; ele++) {
    if (elementWeights[ele] == 0) {
      continue;
    }

    auto &scratch = elements_[ele];
    std::fill(scratch.localPosition.data(), scratch.localPosition.data() + localDOFs, 0.0);
    std::fill(scratch.localDirection.data(), scratch.localDirection.data() + localDOFs, 0.0);
    for (const DofGroup &group : scratch.dofGroups) {
      for (int i = 0; i < group.size; i++) {
        scratch.localPosition[group.localStart + i] = x[group.globalDof(i)];
        scratch.localDirection[group.localStart + i] = dx[group.globalDof(i)];
      }
    }

    const DeformationElement::LocalMaxStepResult localResult =
      (*elements_[ele].deformation).computeLocalMaxStepSize(std::span<const double>(scratch.localPosition.data(), scratch.localPosition.size()), std::span<const double>(scratch.localDirection.data(), scratch.localDirection.size()));
    if (localResult.alpha < observation.alpha) {
      observation.alpha = localResult.alpha;
      observation.limitingElementId = ele;
      observation.limitingLocationId = localResult.locationId;
    }
    if (localResult.illegalInitialState) {
      observation.hasIllegalInitialState = true;
      warnIllegalInitialState(meshType, ele, localResult.locationId, localResult.phi0, localResult.eps);
    }

    if (observation.alpha <= kMaterialMaxStepMinClamp) {
      break;
    }
  }

  return observation;
}

double DeformationModelAssembler::computeMaxStepSize(std::span<const double> x, std::span<const double> dx) const
{
  return computeMaxStepObservation(x, dx).alpha;
}

void DeformationModelAssembler::compute_dE_dx(
  std::span<const double> x, MaterialStateView state, ES::RefVecXd grad) const
{
  validatePositionSpan(x, "position vector");
  validateMaterialState(state);
  if (grad.size() != numDOFs)
    throw std::invalid_argument("DeformationModelAssembler gradient has unexpected size.");
  grad.setZero();
  auto localGradFunc = [this, x, &state, &grad](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elements_[ele];
    gatherPosition(x, scratch);
    scratch.deformation->computeDisplacementGradient(
      vectorSpan(scratch.localPosition), elasticValues(ele, state),
      plasticValues(ele, state), scratch.localGradient);
    scratch.localGradient *= elementWeights[ele];

    for (int i = 0; i < localDOFs; i++) {
      if (!std::isfinite(scratch.localGradient[i])) {
        SPDLOG_LOGGER_ERROR(Logging::lgr(), "Ele: {}", ele);
        SPDLOG_LOGGER_ERROR(Logging::lgr(), "Encounter weird numbers.\nGrad:\n{}\n;x:{}\n",
          scratch.localGradient, scratch.localPosition);
      }
    }

    for (const DofGroup &group : scratch.dofGroups) {
      for (int i = 0; i < group.size; i++) {
        std::atomic_ref<double> atomicGrad(grad[group.globalDof(i)]);
        atomicGrad.fetch_add(scratch.localGradient[group.localStart + i]);
      }
    }
  };

  tbb::parallel_for(0, nele, localGradFunc);

  validateFiniteValues(std::span<const double>(grad.data(), static_cast<std::size_t>(grad.size())),
    numDOFs, "gradient");
}

void DeformationModelAssembler::compute_d2E_dx2(
  std::span<const double> x, MaterialStateView state, EigenSupport::SpMatD &hess) const
{
  validatePositionSpan(x, "position vector");
  validateMaterialState(state);
  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  auto localHessFunc = [this, x, &state, &hess](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elements_[ele];
    gatherPosition(x, scratch);

    ES::Mp<ES::MXd> localK(scratch.localMatrixData.data(), localDOFs, localDOFs);
    scratch.deformation->computeDisplacementHessian(
      vectorSpan(scratch.localPosition), elasticValues(ele, state),
      plasticValues(ele, state), localK);

    localK *= elementWeights[ele];

    const auto &blocks = hessianAssembly_.elementBlockOffsets[ele];
    for (const auto &block : blocks) {
      for (int localCol = 0; localCol < block.colSize; localCol++) {
        const int srcCol = block.colLocalStart + localCol;
        for (int localRow = 0; localRow < block.rowSize; localRow++) {
          const int srcRow = block.rowLocalStart + localRow;
          const std::ptrdiff_t offset = block.offset(localRow, localCol);
          std::atomic_ref<double> hessRef(hess.valuePtr()[offset]);
          hessRef.fetch_add(localK(srcRow, srcCol));
        }
      }
    }
  };

  tbb::parallel_for(0, nele, localHessFunc);

  validateFiniteValues(std::span<const double>(hess.valuePtr(), hess.nonZeros()), hess.nonZeros(), "Hessian");
}

int DeformationModelAssembler::getNumElasticGlobalParams() const
{
  return nele * numElasticParams_;
}

int DeformationModelAssembler::getNumPlasticGlobalParams() const
{
  return nele * numPlasticParams_;
}

void DeformationModelAssembler::compute_dE_dp(
  std::span<const double> x, MaterialStateView state, ES::RefVecXd grad) const
{
  validatePositionSpan(x, "position vector");
  validateMaterialState(state);
  const int numPlasticGlobalParams = getNumPlasticGlobalParams();
  if (grad.size() != numPlasticGlobalParams)
    throw std::invalid_argument("DeformationModelAssembler plastic gradient has unexpected size.");
  grad.setZero();

  if (numPlasticParams_ == 0)
    return;

  auto localGradFunc = [this, x, &state, &grad](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elements_[ele];
    gatherPosition(x, scratch);
    auto localGrad = scratch.localParameterGradient.head(numPlasticParams_);
    scratch.deformation->computePlasticGradient(
      vectorSpan(scratch.localPosition), elasticValues(ele, state),
      plasticValues(ele, state), localGrad);
    localGrad *= elementWeights[ele];

    grad.segment(ele * numPlasticParams_, numPlasticParams_) = localGrad;
  };

  tbb::parallel_for(0, nele, localGradFunc);

  validateFiniteValues(std::span<const double>(grad.data(), static_cast<std::size_t>(grad.size())),
    numPlasticGlobalParams, "plastic gradient");
}

void DeformationModelAssembler::compute_dE_de(
  std::span<const double> x, MaterialStateView state, ES::RefVecXd grad) const
{
  validatePositionSpan(x, "position vector");
  validateMaterialState(state);
  const int numElasticGlobalParams = getNumElasticGlobalParams();
  if (grad.size() != numElasticGlobalParams)
    throw std::invalid_argument("DeformationModelAssembler elastic gradient has unexpected size.");
  grad.setZero();

  if (numElasticParams_ == 0)
    return;

  auto localGradFunc = [this, x, &state, &grad](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elements_[ele];
    gatherPosition(x, scratch);
    auto localGrad = scratch.localParameterGradient.head(numElasticParams_);
    scratch.deformation->computeElasticGradient(
      vectorSpan(scratch.localPosition), elasticValues(ele, state),
      plasticValues(ele, state), localGrad);
    localGrad *= elementWeights[ele];

    grad.segment(ele * numElasticParams_, numElasticParams_) = localGrad;
  };

  tbb::parallel_for(0, nele, localGradFunc);

  validateFiniteValues(std::span<const double>(grad.data(), static_cast<std::size_t>(grad.size())),
    numElasticGlobalParams, "elastic gradient");
}

void DeformationModelAssembler::computePlasticMaterialVJP(
  std::span<const double> absolutePositions,
  std::span<const double> adjoint,
  MaterialStateView state,
  std::span<double> output) const
{
  validateMaterialState(state);
  assembleMaterialVJP(
    absolutePositions, adjoint, state, false, output,
    "plastic material VJP");
}

void DeformationModelAssembler::computeElasticMaterialVJP(
  std::span<const double> absolutePositions,
  std::span<const double> adjoint,
  MaterialStateView state,
  std::span<double> output) const
{
  validateMaterialState(state);
  assembleMaterialVJP(
    absolutePositions, adjoint, state, true, output,
    "elastic material VJP");
}

void DeformationModelAssembler::computeVonMisesStresses(
  std::span<const double> x, MaterialStateView state, std::span<double> elementStresses) const
{
  validatePositionSpan(x, "position vector");
  validateMaterialState(state);
  if (nele == 0)
    return;
  if (elementStresses.size() < static_cast<std::size_t>(nele))
    throw std::invalid_argument(
      "Von Mises stress output must not be null.");
  std::fill(elementStresses.begin(), elementStresses.begin() + nele, 0.0);

  auto localStressFunc = [this, x, &state, elementStresses](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elements_[ele];
    gatherPosition(x, scratch);

    std::fill(scratch.materialLocationValues.begin(), scratch.materialLocationValues.end(), 0.0);
    int nPt = 0;
    try {
      nPt = scratch.deformation->computeVonMisesStress(
        vectorSpan(scratch.localPosition), elasticValues(ele, state),
        plasticValues(ele, state), std::span<double>(scratch.materialLocationValues.data(), scratch.materialLocationValues.size()));
    }
    catch (const UnsupportedDeformationDiagnosticError &e) {
      throw UnsupportedDeformationDiagnosticError(
        "Element " + std::to_string(ele) + ": " + e.what());
    }

    if (nPt <= 0 ||
      nPt > static_cast<int>(scratch.materialLocationValues.size())) {
      throw std::runtime_error(
        "Element " + std::to_string(ele) +
        " returned an invalid von Mises stress sample count: " +
        std::to_string(nPt) + ".");
    }
    elementStresses[ele] = *std::max_element(
      scratch.materialLocationValues.begin(),
      scratch.materialLocationValues.begin() + nPt);
  };

  tbb::parallel_for(0, nele, localStressFunc);
}

void DeformationModelAssembler::computeMaxStrains(
  std::span<const double> x, MaterialStateView state, std::span<double> elementStrain) const
{
  validatePositionSpan(x, "position vector");
  validateMaterialState(state);
  if (nele == 0)
    return;
  if (elementStrain.size() < static_cast<std::size_t>(nele))
    throw std::invalid_argument(
      "Maximum strain output must not be null.");
  std::fill(elementStrain.begin(), elementStrain.begin() + nele, 0.0);

  auto localStrainFunc = [this, x, &state, elementStrain](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elements_[ele];
    gatherPosition(x, scratch);

    std::fill(scratch.materialLocationValues.begin(), scratch.materialLocationValues.end(), 0.0);
    int nPt = 0;
    try {
      nPt = scratch.deformation->computeMaxStrain(
        vectorSpan(scratch.localPosition), elasticValues(ele, state),
        plasticValues(ele, state), std::span<double>(scratch.materialLocationValues.data(), scratch.materialLocationValues.size()));
    }
    catch (const UnsupportedDeformationDiagnosticError &e) {
      throw UnsupportedDeformationDiagnosticError(
        "Element " + std::to_string(ele) + ": " + e.what());
    }

    if (nPt <= 0 ||
      nPt > static_cast<int>(scratch.materialLocationValues.size())) {
      throw std::runtime_error(
        "Element " + std::to_string(ele) +
        " returned an invalid maximum strain sample count: " +
        std::to_string(nPt) + ".");
    }
    elementStrain[ele] = *std::max_element(
      scratch.materialLocationValues.begin(),
      scratch.materialLocationValues.begin() + nPt);
  };

  tbb::parallel_for(0, nele, localStrainFunc);
}

// ── Private helpers ──────────────────────────────────────────────────────────

void DeformationModelAssembler::assembleMaterialVJP(
  std::span<const double> absolutePositions,
  std::span<const double> adjoint,
  MaterialStateView state,
  bool elastic,
  std::span<double> output,
  const char *label) const
{
  validatePositionSpan(absolutePositions, "position vector");
  validatePositionSpan(adjoint, "material VJP adjoint");
  const int numParams = elastic ? numElasticParams_ : numPlasticParams_;
  const int numGlobalParams = nele * numParams;
  if (output.size() != static_cast<std::size_t>(numGlobalParams))
    throw std::invalid_argument(
      "DeformationModelAssembler material VJP output has unexpected size.");
  std::fill(output.begin(), output.end(), 0.0);

  if (numParams == 0)
    return;

  auto localFunc = [this, absolutePositions, adjoint, &state, output,
                     numParams, elastic](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elements_[ele];
    gatherPosition(absolutePositions, scratch);
    std::fill(
      scratch.localDirection.data(),
      scratch.localDirection.data() + localDOFs, 0.0);
    for (const DofGroup &group : scratch.dofGroups)
      for (int i = 0; i < group.size; ++i)
        scratch.localDirection[group.localStart + i] =
          adjoint[group.globalDof(i)];

    auto localVJP = scratch.localParameterGradient.head(numParams);
    if (elastic) {
      scratch.deformation->computeElasticVJP(
        vectorSpan(scratch.localPosition), elasticValues(ele, state),
        plasticValues(ele, state), vectorSpan(scratch.localDirection), localVJP);
    }
    else {
      scratch.deformation->computePlasticVJP(
        vectorSpan(scratch.localPosition), elasticValues(ele, state),
        plasticValues(ele, state), vectorSpan(scratch.localDirection), localVJP);
    }
    localVJP *= elementWeights[ele];
    std::copy_n(localVJP.data(), numParams, output.data() + ele * numParams);
  };

  tbb::parallel_for(0, nele, localFunc);

  validateFiniteValues(output, output.size(), label);
}
