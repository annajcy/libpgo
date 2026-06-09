#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/shared_ptr.h>

#include "core.h"
#include "evaluation.h"

namespace nb = nanobind;
using namespace pgo;

void init_energy_bindings(nb::module_ &m)
{
  // ── StepConstraint ─────────────────────────────────────────────

  nb::class_<PyStepConstraint>(m, "StepConstraint")
    .def_prop_ro("alpha", &PyStepConstraint::alpha)
    .def_prop_ro("clamped", &PyStepConstraint::clamped)
    .def_prop_ro("source", &PyStepConstraint::source);

  // ── PotentialEnergy handle ─────────────────────────────────────
  //
  // Polymorphic base for all energy peers.  Each concrete Python energy
  // type stores a derived peer in _handle.  Evaluation dispatches through
  // the virtual potentialEnergyHandle() method.

  nb::class_<PyPotentialEnergy>(m, "PyPotentialEnergy")
    .def("__repr__", &PyPotentialEnergy::repr)
    .def_prop_ro("num_dofs", &PyPotentialEnergy::numDofs)
    .def("dofs", &PyPotentialEnergy::dofs)
    .def_prop_ro("state_kind", &PyPotentialEnergy::stateKind)
    .def("value", &PyPotentialEnergy::value, nb::arg("x"))
    .def("gradient", &PyPotentialEnergy::gradient, nb::arg("x"))
    .def("hessian", &PyPotentialEnergy::hessian, nb::arg("x"))
    .def("max_step", &PyPotentialEnergy::maxStep, nb::arg("x"), nb::arg("dx"))
    .def("zero_state", &PyPotentialEnergy::zeroState);

  nb::class_<PyOwnedPotentialEnergy, PyPotentialEnergy>(m, "PyOwnedPotentialEnergy");

  nb::class_<PyVertexAttachmentEnergy, PyPotentialEnergy>(m, "PyVertexAttachmentEnergy")
    .def("set_target_positions", &PyVertexAttachmentEnergy::setTargetPositions);

  // ── PyDeformationEnergy ────────────────────────────────────────

  nb::class_<PyDeformationEnergy, PyPotentialEnergy>(m, "PyDeformationEnergy")
    .def("rest_position", &PyDeformationEnergy::restPosition)
    .def_prop_ro("num_vertices", &PyDeformationEnergy::numVertices)
    .def_prop_ro("num_elastic_params", &PyDeformationEnergy::numElasticParams)
    .def_prop_ro("num_plastic_params", &PyDeformationEnergy::numPlasticParams)
    .def_prop_ro("num_elastic_dofs", &PyDeformationEnergy::numElasticDofs)
    .def_prop_ro("num_plastic_dofs", &PyDeformationEnergy::numPlasticDofs)
    .def_prop_ro("elastic_model", &PyDeformationEnergy::elasticModel)
    .def_prop_ro("plastic_model", &PyDeformationEnergy::plasticModel)
    .def_prop_ro("elastic_field", &PyDeformationEnergy::elasticField)
    .def_prop_ro("plastic_field", &PyDeformationEnergy::plasticField)
    .def("set_elastic_values", &PyDeformationEnergy::setElasticValues, nb::arg("values"))
    .def("set_plastic_values", &PyDeformationEnergy::setPlasticValues, nb::arg("values"))
    .def("plastic_gradient", &PyDeformationEnergy::plasticGradient, nb::arg("displacement"))
    .def("plastic_hessian", &PyDeformationEnergy::plasticHessian, nb::arg("displacement"))
    .def("plastic_jacobian", &PyDeformationEnergy::plasticJacobian, nb::arg("displacement"));

  nb::class_<PyParameterField>(m, "PyParameterField")
    .def_prop_ro("domain", &PyParameterField::domain)
    .def_prop_ro("model", &PyParameterField::model)
    .def_prop_ro("num_elements", &PyParameterField::numElements)
    .def_prop_ro("num_value_rows", &PyParameterField::numValueRows)
    .def_prop_ro("num_channels", &PyParameterField::numChannels)
    .def("values", &PyParameterField::values)
    .def("set_values", &PyParameterField::setValues, nb::arg("values"));

  m.def("_elastic_num_channels", &elasticNumChannels,
    nb::arg("mesh_core"),
    nb::arg("elastic_model"));

  // Unified deformation energy factory (public API entry point).
  m.def("_create_deformation_energy", &createDeformationEnergy,
    nb::arg("mesh_core"),
    nb::arg("elastic_model"),
    nb::arg("elastic_values").none(),
    nb::arg("plastic_model"),
    nb::arg("plastic_values").none(),
    nb::arg("elastic_field_type") = "elementwise",
    nb::arg("plastic_field_type") = "elementwise",
    nb::arg("formulation"),
    nb::arg("enforce_spd") = true,
    nb::arg("enable_material_max_step") = true);

  m.def("_create_plastic_material_energy", &createPlasticMaterialEnergy,
    nb::arg("deformation_energy_core"),
    nb::arg("fixed_displacement"));

  // Private/experimental — minimal QuadraticPotentialEnergy factory for
  // PotentialEnergy-handle tests.
  m.def("_create_quadratic_energy_for_test", &createQuadraticEnergyForTest,
    nb::arg("rows"),
    nb::arg("cols"),
    nb::arg("row_indices"),
    nb::arg("col_indices"),
    nb::arg("values"));

  // ── LinearEnergy / QuadraticEnergy factories ──────────────────────

  m.def("_create_linear_energy", &createLinearEnergy,
    nb::arg("b"));

  m.def("_create_constraint_penalty", &createConstraintPenalty,
    nb::arg("constraints"),
    nb::arg("weight") = 1.0);

  m.def("_create_constraint_violation_penalty", &createConstraintViolationPenalty,
    nb::arg("constraints"),
    nb::arg("lower"),
    nb::arg("upper"),
    nb::arg("weight") = 1.0);

  m.def("_create_quadratic_energy_from_sparse", &createQuadraticEnergyFromSparse,
    nb::arg("A"));

  m.def("_create_quadratic_energy_from_sparse_with_b", &createQuadraticEnergyFromSparseWithB,
    nb::arg("A"), nb::arg("b"));

  m.def("_create_quadratic_energy_from_coo", &createQuadraticEnergyFromCOO,
    nb::arg("rows"),
    nb::arg("cols"),
    nb::arg("row_indices"),
    nb::arg("col_indices"),
    nb::arg("values"));

  m.def("_create_quadratic_energy_from_coo_with_b", &createQuadraticEnergyFromCOOWithB,
    nb::arg("rows"),
    nb::arg("cols"),
    nb::arg("row_indices"),
    nb::arg("col_indices"),
    nb::arg("values"),
    nb::arg("b"));

  // ── VertexAttachment factory ──────────────────────────────────────

  m.def("_create_vertex_attachment", &createVertexAttachment,
    nb::arg("num_dofs"),
    nb::arg("rows"),
    nb::arg("cols"),
    nb::arg("k_row_indices"),
    nb::arg("k_col_indices"),
    nb::arg("k_values"),
    nb::arg("rest_positions"),
    nb::arg("vertex_indices"),
    nb::arg("target_positions"),
    nb::arg("coeff") = 1.0,
    nb::arg("is_displacement") = true);

  // ── EnergySet ──────────────────────────────────────────────────────
  //
  // Inherits PyPotentialEnergy so _handle is the concrete peer and
  // num_dofs / value / gradient / etc. are inherited from the base.

  nb::class_<PyEnergySet, PyPotentialEnergy>(m, "PyEnergySet")
    .def("__repr__", &PyEnergySet::repr)
    .def_prop_ro("num_terms", &PyEnergySet::numTerms)
    .def("term", &PyEnergySet::term, nb::arg("i"))
    .def("set_weight", &PyEnergySet::setWeight, nb::arg("i"), nb::arg("w"));

  m.def("_create_energy_set", &createEnergySet, nb::arg("terms"));
}
