#pragma once

#include "eigen_numpy.h"
#include "../sparse/core.h"

#include "evaluation.h"
#include "potentialEnergy.h"
#include "multiVertexPullingSoftConstraints.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <memory>
#include <string>
#include <vector>

namespace nb = nanobind;
namespace NO = pgo::NonlinearOptimization;

class PyPotentialEnergy
{
public:
  virtual ~PyPotentialEnergy() = default;
  virtual std::shared_ptr<const NO::PotentialEnergy> potentialEnergyHandle() const = 0;

  int numDofs() const;
  nb::ndarray<nb::numpy, std::int64_t> dofs() const;
  std::string stateKind() const;
  double value(nb::ndarray<nb::numpy, const double> x) const;
  nb::ndarray<nb::numpy, double> gradient(nb::ndarray<nb::numpy, const double> x) const;
  PySparseMatrix hessian(nb::ndarray<nb::numpy, const double> x) const;
  NO::StepConstraint maxStep(
    nb::ndarray<nb::numpy, const double> x,
    nb::ndarray<nb::numpy, const double> dx) const;
  nb::ndarray<nb::numpy, double> zeroState() const;
  std::string repr() const;
};

// Generic peer for energies with no type-specific behavior
// (LinearEnergy, QuadraticEnergy, ConstraintPenalty, ConstraintViolationPenalty).
class PyOwnedPotentialEnergy final : public PyPotentialEnergy
{
public:
  explicit PyOwnedPotentialEnergy(std::shared_ptr<const NO::PotentialEnergy> energy);
  std::shared_ptr<const NO::PotentialEnergy> potentialEnergyHandle() const override;

private:
  std::shared_ptr<const NO::PotentialEnergy> energy_;
};

// Typed peer for VertexAttachment — saves concrete MultipleVertexPulling type
// to avoid dynamic_cast in setTargetPositions.
class PyVertexAttachmentEnergy final : public PyPotentialEnergy
{
public:
  explicit PyVertexAttachmentEnergy(
    std::shared_ptr<pgo::ConstraintPotentialEnergies::MultipleVertexPulling> energy);
  std::shared_ptr<const NO::PotentialEnergy> potentialEnergyHandle() const override;
  void setTargetPositions(nb::ndarray<nb::numpy, const double> targetPositions);

private:
  std::shared_ptr<pgo::ConstraintPotentialEnergies::MultipleVertexPulling> energy_;
};
