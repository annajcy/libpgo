#pragma once

#include "../../energy/peer.h"
#include "../../solver/core.h"

#include "dynamicState.h"
#include "dynamicStepper.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace nb = nanobind;

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
    const std::string &integrator,
    std::vector<int> fixedDofs,
    double gamma);

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
