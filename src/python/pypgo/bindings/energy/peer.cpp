#include "peer.h"

#include "eigen_numpy.h"
#include "evaluation.h"
#include "potentialEnergy.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstdint>
#include <memory>
#include <string>

namespace nb = nanobind;
namespace NO = pgo::NonlinearOptimization;

// ── PyPotentialEnergy member functions (delegate through virtual) ──────────

int PyPotentialEnergy::numDofs() const
{
  return potentialEnergyHandle()->getNumDOFs();
}

nb::ndarray<nb::numpy, std::int64_t> PyPotentialEnergy::dofs() const
{
  const auto energy = potentialEnergyHandle();
  auto d = NO::dofsOf(*energy);
  auto storage = new std::vector<std::int64_t>(d.begin(), d.end());
  nb::capsule owner(storage, [](void *p) noexcept {
    delete static_cast<std::vector<std::int64_t> *>(p);
  });
  return nb::ndarray<nb::numpy, std::int64_t>(
    storage->data(),
    { storage->size() },
    owner);
}

std::string PyPotentialEnergy::stateKind() const
{
  switch (potentialEnergyHandle()->stateKind()) {
    case NO::EnergyStateKind::Displacement:
      return "displacement";
    default:
      return "generic";
  }
}

double PyPotentialEnergy::value(nb::ndarray<nb::numpy, const double> x) const
{
  auto xMap = pgo::python::ndarrayToVectorMapXd(x);
  double result;
  {
    nb::gil_scoped_release release;
    result = NO::evaluateValue(*potentialEnergyHandle(), xMap);
  }
  return result;
}

nb::ndarray<nb::numpy, double> PyPotentialEnergy::gradient(
  nb::ndarray<nb::numpy, const double> x) const
{
  auto xMap = pgo::python::ndarrayToVectorMapXd(x);
  pgo::EigenSupport::VXd grad;
  {
    nb::gil_scoped_release release;
    grad = NO::evaluateGradient(*potentialEnergyHandle(), xMap);
  }
  return pgo::python::vectorXdToNdarray(std::move(grad));
}

PySparseMatrix PyPotentialEnergy::hessian(
  nb::ndarray<nb::numpy, const double> x) const
{
  auto xMap = pgo::python::ndarrayToVectorMapXd(x);
  pgo::EigenSupport::SpMatD H;
  {
    nb::gil_scoped_release release;
    H = NO::evaluateHessian(*potentialEnergyHandle(), xMap);
  }
  return PySparseMatrix(std::move(H));
}

NO::StepConstraint PyPotentialEnergy::maxStep(
  nb::ndarray<nb::numpy, const double> x,
  nb::ndarray<nb::numpy, const double> dx) const
{
  auto xMap = pgo::python::ndarrayToVectorMapXd(x);
  auto dxMap = pgo::python::ndarrayToVectorMapXd(dx);
  NO::StepConstraint result;
  {
    nb::gil_scoped_release release;
    result = NO::evaluateMaxStep(*potentialEnergyHandle(), xMap, dxMap);
  }
  return result;
}

nb::ndarray<nb::numpy, double> PyPotentialEnergy::zeroState() const
{
  int n = potentialEnergyHandle()->getNumDOFs();
  auto data = new std::vector<double>(static_cast<size_t>(n), 0.0);
  nb::capsule owner(data, [](void *p) noexcept {
    delete static_cast<std::vector<double> *>(p);
  });
  return nb::ndarray<nb::numpy, double>(
    data->data(),
    { data->size() },
    owner);
}

std::string PyPotentialEnergy::repr() const
{
  return "PotentialEnergy(" + std::to_string(potentialEnergyHandle()->getNumDOFs()) + " DOFs)";
}

// ── PyOwnedPotentialEnergy ─────────────────────────────────────────────────

PyOwnedPotentialEnergy::PyOwnedPotentialEnergy(
  std::shared_ptr<const NO::PotentialEnergy> energy)
  : energy_(std::move(energy))
{
}

std::shared_ptr<const NO::PotentialEnergy>
PyOwnedPotentialEnergy::potentialEnergyHandle() const
{
  return energy_;
}

// ── PyVertexAttachmentEnergy ────────────────────────────────────────────────

PyVertexAttachmentEnergy::PyVertexAttachmentEnergy(
  std::shared_ptr<pgo::ConstraintPotentialEnergies::MultipleVertexPulling> energy)
  : energy_(std::move(energy))
{
}

std::shared_ptr<const NO::PotentialEnergy>
PyVertexAttachmentEnergy::potentialEnergyHandle() const
{
  return energy_;
}

void PyVertexAttachmentEnergy::setTargetPositions(
  nb::ndarray<nb::numpy, const double> targetPositions)
{
  auto target = pgo::python::ndarrayToVectorXd(targetPositions);
  energy_->setTargetPositions(std::move(target));
}
