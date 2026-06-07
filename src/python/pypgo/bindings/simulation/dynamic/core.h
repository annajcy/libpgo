#pragma once

#include "../../energy/peer.h"
#include "../../solver/core.h"

#include "dynamicState.h"
#include "dynamicStepper.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace nb = nanobind;

class PyDynamicStepper
{
public:
  virtual ~PyDynamicStepper() = default;
  virtual pgo::Simulation::DynamicStepperKind kind() const = 0;
  virtual double trbdf2Gamma() const { return 0.5; }
};

class PyBackwardEulerDynamicStepper final : public PyDynamicStepper
{
public:
  pgo::Simulation::DynamicStepperKind kind() const override
  {
    return pgo::Simulation::DynamicStepperKind::ImplicitEuler;
  }
};

class PyTRBDF2DynamicStepper final : public PyDynamicStepper
{
public:
  explicit PyTRBDF2DynamicStepper(double gamma = 0.5);
  pgo::Simulation::DynamicStepperKind kind() const override
  {
    return pgo::Simulation::DynamicStepperKind::TRBDF2;
  }
  double trbdf2Gamma() const override { return gamma_; }
  double gamma() const { return gamma_; }

private:
  double gamma_;
};

// Low-level dynamic simulation peer: owns a DynamicStepper plus the current
// state.  Evaluation energy is supplied as a PyPotentialEnergy peer; each step
// is driven by a PyOptimizer peer (Optimizer base).
class PyDynamicSimulation
{
public:
  PyDynamicSimulation(
    int numDofs,
    std::vector<int> massRows, std::vector<int> massCols, std::vector<double> massVals,
    std::shared_ptr<PyPotentialEnergy> energy,
    double massDamping, double stiffnessDamping,
    nb::ndarray<nb::numpy, const double> displacement,
    nb::ndarray<nb::numpy, const double> velocity,
    nb::ndarray<nb::numpy, const double> acceleration,
    double timestep,
    std::shared_ptr<PyDynamicStepper> integrator,
    std::vector<int> fixedDofs);

  nb::dict step(nb::ndarray<nb::numpy, const double> externalForce,
    nb::ndarray<nb::numpy, const double> fixedValues, bool hasFixedValues,
    PyOptimizer &optimizer);

  nb::ndarray<nb::numpy, double> displacement() const;
  nb::ndarray<nb::numpy, double> velocity() const;
  nb::ndarray<nb::numpy, double> acceleration() const;
  std::uint64_t timestepId() const;
  double time() const;
  int numDofs() const;

private:
  int n_;
  pgo::Simulation::DynamicState state_;
  std::unique_ptr<pgo::Simulation::DynamicStepper> stepper_;
};
