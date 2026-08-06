#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/shared_ptr.h>

#include "core.h"
#include "../fem/formulation/core.h"
#include "energy/evaluation.h"

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

  nb::class_<PyDeformationEnergyOperator>(m, "PyDeformationEnergyOperator")
    .def("rest_state", &PyDeformationEnergyOperator::restState)
    .def("vertex_rest_positions", &PyDeformationEnergyOperator::vertexRestPositions)
    .def_prop_ro("num_vertices", &PyDeformationEnergyOperator::numVertices)
    .def_prop_ro("num_elements", &PyDeformationEnergyOperator::numElements)
    .def_prop_ro("num_elastic_params", &PyDeformationEnergyOperator::numElasticParams)
    .def_prop_ro("num_plastic_params", &PyDeformationEnergyOperator::numPlasticParams)
    .def_prop_ro("num_elastic_values", &PyDeformationEnergyOperator::numElasticValues)
    .def_prop_ro("num_plastic_values", &PyDeformationEnergyOperator::numPlasticValues)
    .def_prop_ro("num_dofs", &PyDeformationEnergyOperator::numDofs)
    .def("value", &PyDeformationEnergyOperator::value, nb::arg("displacement"), nb::arg("material_state"))
    .def("gradient", &PyDeformationEnergyOperator::gradient, nb::arg("displacement"), nb::arg("material_state"))
    .def("hessian", &PyDeformationEnergyOperator::hessian, nb::arg("displacement"), nb::arg("material_state"))
    .def("zero_state", &PyDeformationEnergyOperator::zeroState)
    .def("dE_de", &PyDeformationEnergyOperator::dE_de, nb::arg("displacement"), nb::arg("material_state"))
    .def("element_von_mises_stresses", &PyDeformationEnergyOperator::elementVonMisesStresses, nb::arg("displacement"), nb::arg("material_state"))
    .def("dE_dp", &PyDeformationEnergyOperator::dE_dp, nb::arg("displacement"), nb::arg("material_state"))
    .def("elastic_material_vjp", &PyDeformationEnergyOperator::elasticMaterialVJP,
      nb::arg("displacement"), nb::arg("material_state"), nb::arg("adjoint"))
    .def("plastic_material_vjp", &PyDeformationEnergyOperator::plasticMaterialVJP,
      nb::arg("displacement"), nb::arg("material_state"), nb::arg("adjoint"));

  nb::class_<PyDeformationPotentialEnergy, PyPotentialEnergy>(
    m, "PyDeformationPotentialEnergy")
    .def_prop_ro("energy_operator", &PyDeformationPotentialEnergy::energyOperator)
    .def_prop_ro("material_state", &PyDeformationPotentialEnergy::materialState);

  nb::class_<PyMaterialState>(m, "PyMaterialState")
    .def_prop_ro(
      "elastic_values", &PyMaterialState::elasticValues,
      nb::rv_policy::move)
    .def_prop_ro(
      "plastic_values", &PyMaterialState::plasticValues,
      nb::rv_policy::move)
    .def("with_elastic_values", &PyMaterialState::withElasticValues,
      nb::arg("values"))
    .def("with_plastic_values", &PyMaterialState::withPlasticValues,
      nb::arg("values"));

  nb::class_<PyMaterialFrames>(m, "PyMaterialFrames")
    .def_prop_ro("num_elements", &PyMaterialFrames::numElements);
  nb::class_<PyMaterialBinding>(m, "PyMaterialBinding");

  m.def("_make_material_frames",
    &makeMaterialFrames, nb::arg("frame_values"));
  m.def("_make_material_frames_from_primary_axes",
    &makeMaterialFramesFromPrimaryAxes, nb::arg("axes"));

  m.def("_create_material_binding", &createMaterialBinding,
    nb::arg("elastic_definition"), nb::arg("num_elements"),
    nb::arg("elastic_fixed_values"), nb::arg("plastic_definition"),
    nb::arg("plastic_fixed_values"),
    nb::arg("material_frames").none() = nb::none());
  m.def("_create_material_state", &createMaterialState,
    nb::arg("elastic_values"), nb::arg("plastic_values"));
  m.def("_create_deformation_energy_operator", &createDeformationEnergyOperator,
    nb::arg("mesh"), nb::arg("material_binding"), nb::arg("formulation"),
    nb::arg("element_weights").none() = nb::none(), nb::arg("project_hessian_psd") = true);
  m.def("_create_deformation_potential_energy", &createDeformationPotentialEnergy,
    nb::arg("energy_operator"), nb::arg("material_state"));

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
