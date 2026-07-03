#include "core.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

void init_contact_bindings(nb::module_ &m)
{
  nb::class_<PyContactSurface>(m, "PyContactSurface")
    .def_prop_ro("num_surface_vertices", &PyContactSurface::numSurfaceVertices)
    .def_prop_ro("num_surface_dofs", &PyContactSurface::numSurfaceDofs)
    .def_prop_ro("num_simulation_dofs", &PyContactSurface::numSimulationDofs);

  nb::class_<PyStatefulContactEnergy, PyPotentialEnergy>(m, "PyStatefulContactEnergy")
    .def_prop_ro("is_step_dependent", &PyStatefulContactEnergy::isStepDependent)
    .def("begin_step", &PyStatefulContactEnergy::beginStep,
      nb::arg("time"),
      nb::arg("timestep"),
      nb::arg("previous_x") = nb::none());

  nb::class_<PySampledPenaltyContactEnergy, PyStatefulContactEnergy>(m, "PySampledPenaltyContactEnergy");

  nb::class_<PyIPCContactEnergy, PyStatefulContactEnergy>(m, "PyIPCContactEnergy")
    .def("set_moving_obstacle_time", &PyIPCContactEnergy::setMovingObstacleTime, nb::arg("time"));

  m.def("_create_contact_surface_identity", &createContactSurfaceIdentity,
    nb::arg("rest_vertices"));
  m.def("_create_contact_surface_embedded", &createContactSurfaceEmbedded,
    nb::arg("rest_vertices"),
    nb::arg("surface_from_simulation"));
  m.def("_create_floor_contact_energy", &createFloorEnergy,
    nb::arg("surface"),
    nb::arg("axis"),
    nb::arg("side"),
    nb::arg("height"),
    nb::arg("stiffness"));
  m.def("_set_floor_contact_height", &setFloorEnergyHeight,
    nb::arg("energy"),
    nb::arg("height"));
  m.def("_create_sampled_penalty_contact_energy", &createSampledPenaltyEnergy,
    nb::arg("surface"),
    nb::arg("surface_triangles"),
    nb::arg("stiffness"),
    nb::arg("samples"),
    nb::arg("enable_self_contact"),
    nb::arg("enable_external_contact"),
    nb::arg("friction_coeff") = nb::none(),
    nb::arg("velocity_eps") = nb::none(),
    nb::arg("obstacles") = nb::none());
  m.def("_create_ipc_contact_energy", &createIPCEnergy,
    nb::arg("surface"),
    nb::arg("surface_triangles"),
    nb::arg("dhat"),
    nb::arg("dhat_external"),
    nb::arg("kappa"),
    nb::arg("eps_ee"),
    nb::arg("slackness"),
    nb::arg("ccd_thickness"),
    nb::arg("obstacles") = nb::none());
}
