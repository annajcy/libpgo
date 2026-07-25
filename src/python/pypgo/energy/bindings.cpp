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

  // ── PyDeformationEnergy ────────────────────────────────────────

  nb::class_<PyDeformationEnergy, PyPotentialEnergy>(m, "PyDeformationEnergy")
    .def("rest_state", &PyDeformationEnergy::restState)
    .def("vertex_rest_positions", &PyDeformationEnergy::vertexRestPositions)
    .def_prop_ro("num_vertices", &PyDeformationEnergy::numVertices)
    .def_prop_ro("num_elastic_params", &PyDeformationEnergy::numElasticParams)
    .def_prop_ro("num_plastic_params", &PyDeformationEnergy::numPlasticParams)
    .def_prop_ro("num_elastic_dofs", &PyDeformationEnergy::numElasticDofs)
    .def_prop_ro("num_plastic_dofs", &PyDeformationEnergy::numPlasticDofs)
    .def_prop_ro("elastic_model", &PyDeformationEnergy::elasticModel)
    .def_prop_ro("plastic_model", &PyDeformationEnergy::plasticModel)
    .def_prop_ro("parameters", &PyDeformationEnergy::parameters)
    .def("dE_de", &PyDeformationEnergy::dE_de, nb::arg("displacement"))
    .def("element_von_mises_stresses", &PyDeformationEnergy::elementVonMisesStresses, nb::arg("displacement"))
    .def("d2E_de2", &PyDeformationEnergy::d2E_de2, nb::arg("displacement"))
    .def("d2E_dpde", &PyDeformationEnergy::d2E_dpde, nb::arg("displacement"))
    .def("dE_dp", &PyDeformationEnergy::dE_dp, nb::arg("displacement"))
    .def("d2E_dp2", &PyDeformationEnergy::d2E_dp2, nb::arg("displacement"))
    .def("d2E_dude", &PyDeformationEnergy::d2E_dude, nb::arg("displacement"))
    .def("d2E_dudp", &PyDeformationEnergy::d2E_dudp, nb::arg("displacement"));

  nb::class_<PyParameterDofLayout>(m, "PyParameterDofLayout");
  nb::class_<PyMaterialChannelMapping>(m, "PyMaterialChannelMapping");

  nb::class_<PyMaterialParameterRef>(m, "PyMaterialParameterRef")
    .def_prop_ro("name", &PyMaterialParameterRef::name)
    .def_prop_ro("channel", &PyMaterialParameterRef::channel);

  nb::class_<PyMaterialParameterBlock>(m, "PyMaterialParameterBlock")
    .def_prop_ro("num_channels", &PyMaterialParameterBlock::numChannels)
    .def_prop_ro("num_local_dofs", &PyMaterialParameterBlock::numLocalDofs)
    .def_prop_ro("num_global_dofs", &PyMaterialParameterBlock::numGlobalDofs)
    .def_prop_ro("num_value_rows", &PyMaterialParameterBlock::numValueRows)
    .def_prop_ro("channel_names", &PyMaterialParameterBlock::channelNames)
    .def("parameter", &PyMaterialParameterBlock::parameter, nb::arg("name"));

  nb::class_<PyMaterialParameterSpace>(m, "PyMaterialParameterSpace")
    .def_prop_ro("elastic", &PyMaterialParameterSpace::elastic)
    .def_prop_ro("plastic", &PyMaterialParameterSpace::plastic);

  nb::class_<PyMaterialParameters>(m, "PyMaterialParameters")
    .def_prop_ro("space", &PyMaterialParameters::space)
    .def_prop_ro(
      "elastic_values", &PyMaterialParameters::elasticValues,
      nb::rv_policy::move)
    .def_prop_ro(
      "plastic_values", &PyMaterialParameters::plasticValues,
      nb::rv_policy::move)
    .def("set_elastic_values", &PyMaterialParameters::setElasticValues, nb::arg("values"))
    .def("set_plastic_values", &PyMaterialParameters::setPlasticValues, nb::arg("values"))
    .def("_same_space", &PyMaterialParameters::sameSpace, nb::arg("other"));

  m.def("_make_elementwise_parameter_dof_layout",
    &makeElementwiseParameterDofLayout);
  m.def("_make_constant_parameter_dof_layout",
    &makeConstantParameterDofLayout);
  m.def("_make_identity_material_channel_mapping",
    &makeIdentityMaterialChannelMapping);

  m.def("_create_material_parameter_space", &createMaterialParameterSpace,
    nb::arg("mesh_core"), nb::arg("elastic_model"), nb::arg("elastic_layout"),
    nb::arg("elastic_mapping"), nb::arg("plastic_model"), nb::arg("plastic_layout"),
    nb::arg("plastic_mapping"));
  m.def("_create_default_material_parameters", &createDefaultMaterialParameters,
    nb::arg("mesh_core"), nb::arg("elastic_model"), nb::arg("plastic_model"));
  m.def("_create_material_parameters", &createMaterialParameters,
    nb::arg("space"), nb::arg("elastic_values"), nb::arg("plastic_values"));
  m.def("_create_deformation_energy_with_parameters", &createDeformationEnergyWithParameters,
    nb::arg("mesh_core"), nb::arg("elastic_model"), nb::arg("plastic_model"),
    nb::arg("material_parameters"), nb::arg("formulation"),
    nb::arg("element_weights").none() = nb::none(), nb::arg("project_hessian_psd") = true,
    nb::arg("enable_material_max_step") = true);

  m.def("_create_plastic_material_energy", &createPlasticMaterialEnergy,
    nb::arg("deformation_energy_core"),
    nb::arg("fixed_displacement"));

  m.def("_create_elastic_material_energy", &createElasticMaterialEnergy,
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
