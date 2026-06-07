#include "core.h"

#include "floor/floorContactEnergy.h"
#include "eigen_numpy.h"
#include "ipc/ipcContactEnergy.h"
#include "sampled_penalty/sampledPenaltyContactEnergy.h"
#include "../sparse/core.h"
#include "stepAwareEnergy.h"
#include "stepDependentEnergy.h"

#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <stdexcept>
#include <utility>
#include <vector>

namespace ES = pgo::EigenSupport;
namespace NO = pgo::NonlinearOptimization;

namespace
{

ES::SpMatD makeIdentityMap(int rows)
{
  std::vector<ES::TripletD> entries;
  entries.reserve(static_cast<size_t>(rows));
  for (int i = 0; i < rows; ++i)
    entries.emplace_back(i, i, 1.0);

  ES::SpMatD out(rows, rows);
  out.setFromTriplets(entries.begin(), entries.end());
  return out;
}

ES::SpMatD sparseMatrixToEigen(const PySparseMatrix &A)
{
  auto coo = A.toCOO();
  const auto &rowIndices = std::get<0>(coo);
  const auto &colIndices = std::get<1>(coo);
  const auto &values = std::get<2>(coo);

  std::vector<ES::TripletD> triplets;
  triplets.reserve(values.size());
  for (size_t i = 0; i < values.size(); ++i)
    triplets.emplace_back(rowIndices[i], colIndices[i], values[i]);

  ES::SpMatD matrix(A.rows(), A.cols());
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  return matrix;
}

ES::MXi ndarrayToMatrixXi(nb::ndarray<nb::numpy, const std::int64_t> array, const char *name)
{
  if (array.dtype() != nb::dtype<std::int64_t>())
    throw nb::value_error((std::string(name) + " must have dtype int64").c_str());
  if (array.ndim() != 2)
    throw nb::value_error((std::string(name) + " must have shape (m, n)").c_str());

  ES::MXi out(
    static_cast<Eigen::Index>(array.shape(0)),
    static_cast<Eigen::Index>(array.shape(1)));
  for (size_t r = 0; r < array.shape(0); ++r) {
    for (size_t c = 0; c < array.shape(1); ++c) {
      out(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) =
        static_cast<int>(array.data()[static_cast<std::int64_t>(r) * array.stride(0) +
                                      static_cast<std::int64_t>(c) * array.stride(1)]);
    }
  }
  return out;
}

CT::FloorAxis parseFloorAxis(const std::string &axis)
{
  if (axis == "x")
    return CT::FloorAxis::X;
  if (axis == "y")
    return CT::FloorAxis::Y;
  if (axis == "z")
    return CT::FloorAxis::Z;
  throw nb::value_error("axis must be one of 'x', 'y', or 'z'");
}

CT::FloorSide parseFloorSide(const std::string &side)
{
  if (side == "keep_above")
    return CT::FloorSide::KeepAbove;
  if (side == "keep_below")
    return CT::FloorSide::KeepBelow;
  throw nb::value_error("side must be 'keep_above' or 'keep_below'");
}

CT::SampledPenaltyContactSpec makeSampledPenaltyParams(
  double stiffness,
  int samples,
  bool enableSelfContact,
  bool enableExternalContact)
{
  CT::SampledPenaltyContactSpec params;
  params.stiffness = stiffness;
  params.samples = samples;
  params.enableSelfContact = enableSelfContact;
  params.enableExternalContact = enableExternalContact;
  return params;
}

ES::V3d objectToVector3d(nb::object value, const char *name)
{
  const ES::VXd vec = pgo::python::ndarrayToVectorXd(
    nb::cast<nb::ndarray<nb::numpy, const double>>(value));
  if (vec.size() != 3)
    throw nb::value_error((std::string(name) + " must be a 3-vector").c_str());
  return ES::V3d(vec[0], vec[1], vec[2]);
}

std::vector<CT::ObstacleSpec> parseObstacleSpecs(nb::object obstacleSpecs)
{
  std::vector<CT::ObstacleSpec> obstacles;
  if (obstacleSpecs.is_none())
    return obstacles;

  nb::iterable iterable = nb::borrow<nb::iterable>(obstacleSpecs);
  for (nb::handle item : iterable) {
    nb::object spec = nb::borrow<nb::object>(item);
    const std::string kind = nb::cast<std::string>(nb::getattr(spec, "kind"));
    const ES::MXd restVertices = pgo::python::ndarrayToMatrixXd(
      nb::cast<nb::ndarray<nb::numpy, const double>>(nb::getattr(spec, "rest_vertices")));
    const ES::MXi triangles = ndarrayToMatrixXi(
      nb::cast<nb::ndarray<nb::numpy, const std::int64_t>>(nb::getattr(spec, "triangles")),
      "obstacle triangles");

    if (kind == "static") {
      CT::StaticObstacleSpec staticSpec;
      staticSpec.restVertices = restVertices;
      staticSpec.triangles = triangles;
      obstacles.emplace_back(std::move(staticSpec));
      continue;
    }

    if (kind == "linear_velocity") {
      CT::LinearMovingObstacleSpec movingSpec;
      movingSpec.restVertices = restVertices;
      movingSpec.triangles = triangles;
      movingSpec.velocity = objectToVector3d(nb::getattr(spec, "velocity"), "obstacle velocity");
      movingSpec.t0 = nb::cast<double>(nb::getattr(spec, "t0"));
      obstacles.emplace_back(std::move(movingSpec));
      continue;
    }

    throw nb::value_error(("Unsupported contact obstacle kind: " + kind).c_str());
  }

  return obstacles;
}

}  // namespace

// ── PyContactSurface ─────────────────────────────────────────────────────

PyContactSurface::PyContactSurface(CT::ContactSurfaceSpec spec):
  spec_(std::move(spec))
{
}

const CT::ContactSurfaceSpec &PyContactSurface::spec() const { return spec_; }

int PyContactSurface::numSurfaceVertices() const { return static_cast<int>(spec_.restVertices.rows()); }
int PyContactSurface::numSurfaceDofs() const { return static_cast<int>(spec_.restVertices.rows() * 3); }
int PyContactSurface::numSimulationDofs() const { return static_cast<int>(spec_.surfaceFromSimulationDispMap.cols()); }

// ── PyStatefulContactEnergy hierarchy ────────────────────────────────────

PyStatefulContactEnergy::PyStatefulContactEnergy(std::shared_ptr<CT::StatefulContactEnergy> energy):
  energy_(std::move(energy))
{
  if (!energy_)
    throw std::invalid_argument("PyStatefulContactEnergy requires a contact energy.");
}

std::shared_ptr<const NO::PotentialEnergy> PyStatefulContactEnergy::potentialEnergyHandle() const
{
  return energy_;
}

bool PyStatefulContactEnergy::isStepDependent() const
{
  return dynamic_cast<const NO::StepDependentEnergy *>(energy_.get()) != nullptr;
}

void PyStatefulContactEnergy::beginStep(double time, double timestep, nb::object previousX) const
{
  auto *stepAware = dynamic_cast<NO::StepAwareEnergy *>(energy_.get());
  if (!stepAware)
    return;

  NO::StepState state;
  state.time = time;
  state.timestep = timestep;

  ES::VXd previous;
  if (!previousX.is_none()) {
    previous = pgo::python::ndarrayToVectorXd(nb::cast<nb::ndarray<nb::numpy, const double>>(previousX));
    state.previousX = &previous;
  }

  stepAware->beginStep(state);
}

void PyIPCContactEnergy::setMovingObstacleTime(double t) const
{
  auto *ipcEnergy = dynamic_cast<CT::IPC::IPCContactEnergy *>(energy_.get());
  if (!ipcEnergy)
    throw std::runtime_error("set_moving_obstacle_time is only available on IPCEnergy.");
  ipcEnergy->setMovingObstacleTime(t);
}

// ── Factories ────────────────────────────────────────────────────────────

PyContactSurface createContactSurfaceIdentity(nb::ndarray<nb::numpy, const double> restVertices)
{
  CT::ContactSurfaceSpec spec;
  spec.restVertices = pgo::python::ndarrayToMatrixXd(restVertices);
  if (spec.restVertices.cols() != 3)
    throw nb::value_error("rest_vertices must have shape (n, 3)");
  spec.surfaceFromSimulationDispMap = makeIdentityMap(static_cast<int>(spec.restVertices.rows() * 3));
  return PyContactSurface(std::move(spec));
}

PyContactSurface createContactSurfaceEmbedded(
  nb::ndarray<nb::numpy, const double> restVertices,
  const PySparseMatrix &surfaceFromSimulationDispMap)
{
  CT::ContactSurfaceSpec spec;
  spec.restVertices = pgo::python::ndarrayToMatrixXd(restVertices);
  if (spec.restVertices.cols() != 3)
    throw nb::value_error("rest_vertices must have shape (n, 3)");
  spec.surfaceFromSimulationDispMap = sparseMatrixToEigen(surfaceFromSimulationDispMap);
  if (spec.surfaceFromSimulationDispMap.rows() != spec.restVertices.rows() * 3)
    throw nb::value_error("surface_from_simulation must have 3 * num_surface_vertices rows");
  return PyContactSurface(std::move(spec));
}

std::shared_ptr<PyPotentialEnergy> createFloorEnergy(
  const PyContactSurface &surface,
  const std::string &axis,
  const std::string &side,
  double height,
  double stiffness)
{
  CT::FloorContactSpec floor;
  floor.axis = parseFloorAxis(axis);
  floor.side = parseFloorSide(side);
  floor.height = height;
  floor.stiffness = stiffness;
  return std::make_shared<PyOwnedPotentialEnergy>(CT::createFloorEnergy(surface.spec(), floor));
}

void setFloorEnergyHeight(const PyPotentialEnergy &energy, double height)
{
  auto *floor = dynamic_cast<CT::Floor::FloorContactEnergy *>(
    const_cast<NO::PotentialEnergy *>(energy.potentialEnergyHandle().get()));
  if (!floor)
    throw std::runtime_error("set_floor_height is only available on FloorEnergy.");
  floor->setFloorHeight(height);
}

std::shared_ptr<PySampledPenaltyContactEnergy> createSampledPenaltyEnergy(
  const PyContactSurface &surface,
  nb::ndarray<nb::numpy, const std::int64_t> surfaceTriangles,
  double stiffness,
  int samples,
  bool enableSelfContact,
  bool enableExternalContact)
{
  auto triangles = ndarrayToMatrixXi(surfaceTriangles, "surface_triangles");
  auto energy = CT::SampledPenalty::createSampledPenaltyEnergy(
    surface.spec(), triangles,
    makeSampledPenaltyParams(stiffness, samples, enableSelfContact, enableExternalContact));
  return std::make_shared<PySampledPenaltyContactEnergy>(std::move(energy));
}

std::shared_ptr<PyIPCContactEnergy> createIPCEnergy(
  const PyContactSurface &surface,
  nb::ndarray<nb::numpy, const std::int64_t> surfaceTriangles,
  double dhat,
  double dhatExternal,
  double kappa,
  double epsEE,
  double slackness,
  double ccdThickness,
  nb::object obstacleSpecs)
{
  auto triangles = ndarrayToMatrixXi(surfaceTriangles, "surface_triangles");
  CT::IPCContactSpec params;
  params.dhat = dhat;
  params.dhatExternal = dhatExternal;
  params.kappa = kappa;
  params.epsEE = epsEE;
  params.slackness = slackness;
  params.ccdThickness = ccdThickness;

  auto energy = CT::IPC::createIPCEnergy(
    surface.spec(),
    triangles,
    params,
    parseObstacleSpecs(std::move(obstacleSpecs)));
  return std::make_shared<PyIPCContactEnergy>(std::move(energy));
}

std::shared_ptr<PyFrictionalSampledPenaltyContactEnergy> createFrictionalSampledPenaltyEnergy(
  const PyContactSurface &surface,
  nb::ndarray<nb::numpy, const std::int64_t> surfaceTriangles,
  double stiffness,
  int samples,
  bool enableSelfContact,
  bool enableExternalContact,
  double frictionCoeff,
  double velocityEps)
{
  auto triangles = ndarrayToMatrixXi(surfaceTriangles, "surface_triangles");
  CT::FrictionContactSpec friction;
  friction.frictionCoeff = frictionCoeff;
  friction.velocityEps = velocityEps;
  auto energy = CT::SampledPenalty::createFrictionalSampledPenaltyEnergy(
    surface.spec(), triangles,
    makeSampledPenaltyParams(stiffness, samples, enableSelfContact, enableExternalContact),
    friction);
  return std::make_shared<PyFrictionalSampledPenaltyContactEnergy>(std::move(energy));
}
