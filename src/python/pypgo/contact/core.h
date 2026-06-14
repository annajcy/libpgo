#pragma once

#include "../energy/peer.h"

#include "contactEnergyFactory.h"
#include "statefulContactEnergy.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstdint>
#include <memory>
#include <string>

namespace nb = nanobind;
namespace CT = pgo::Contact;

// Surface peer.  Holds the contact surface spec (rest vertices plus the
// surface-from-simulation displacement map) consumed by the energy factories.
class PyContactSurface
{
public:
  explicit PyContactSurface(CT::ContactSurfaceSpec spec);

  const CT::ContactSurfaceSpec &spec() const;

  int numSurfaceVertices() const;
  int numSurfaceDofs() const;
  int numSimulationDofs() const;

private:
  CT::ContactSurfaceSpec spec_;
};

// Long-lived stateful contact energy peer.  Inherits PyPotentialEnergy so the
// Python facade stores the concrete peer directly in `_handle`; evaluation
// dispatches through potentialEnergyHandle() and the step-aware surface
// (begin_step / is_step_dependent) lives on the same object.
class PyStatefulContactEnergy : public PyPotentialEnergy
{
public:
  explicit PyStatefulContactEnergy(std::shared_ptr<CT::StatefulContactEnergy> energy);

  std::shared_ptr<const NO::PotentialEnergy> potentialEnergyHandle() const override;

  bool isStepDependent() const;
  void beginStep(double time, double timestep, nb::object previousX) const;

protected:
  std::shared_ptr<CT::StatefulContactEnergy> energy_;
};

class PySampledPenaltyContactEnergy final : public PyStatefulContactEnergy
{
public:
  using PyStatefulContactEnergy::PyStatefulContactEnergy;
};

class PyIPCContactEnergy final : public PyStatefulContactEnergy
{
public:
  using PyStatefulContactEnergy::PyStatefulContactEnergy;
  void setMovingObstacleTime(double time) const;
};

// ── Factories (implemented in contact/core.cpp) ──────────────────────────

PyContactSurface createContactSurfaceIdentity(nb::ndarray<nb::numpy, const double> restVertices);

PyContactSurface createContactSurfaceEmbedded(
  nb::ndarray<nb::numpy, const double> restVertices,
  const PySparseMatrix &surfaceFromSimulationDispMap);

std::shared_ptr<PyPotentialEnergy> createFloorEnergy(
  const PyContactSurface &surface,
  const std::string &axis,
  const std::string &side,
  double height,
  double stiffness);

void setFloorEnergyHeight(const PyPotentialEnergy &energy, double height);

std::shared_ptr<PySampledPenaltyContactEnergy> createSampledPenaltyEnergy(
  const PyContactSurface &surface,
  nb::ndarray<nb::numpy, const std::int64_t> surfaceTriangles,
  double stiffness,
  int samples,
  bool enableSelfContact,
  bool enableExternalContact,
  nb::object frictionCoeff,
  nb::object velocityEps,
  nb::object obstacleSpecs);

std::shared_ptr<PyIPCContactEnergy> createIPCEnergy(
  const PyContactSurface &surface,
  nb::ndarray<nb::numpy, const std::int64_t> surfaceTriangles,
  double dhat,
  double dhatExternal,
  double kappa,
  double epsEE,
  double slackness,
  double ccdThickness,
  nb::object obstacleSpecs);
