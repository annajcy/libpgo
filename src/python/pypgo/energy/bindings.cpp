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
    .def_prop_ro("elastic_definition", &PyDeformationEnergy::elasticDefinition)
    .def_prop_ro("plastic_definition", &PyDeformationEnergy::plasticDefinition)
    .def_prop_ro("optimizable_parameters", &PyDeformationEnergy::optimizableParameters)
    .def("dE_de", &PyDeformationEnergy::dE_de, nb::arg("displacement"))
    .def("element_von_mises_stresses", &PyDeformationEnergy::elementVonMisesStresses, nb::arg("displacement"))
    .def("d2E_de2", &PyDeformationEnergy::d2E_de2, nb::arg("displacement"))
    .def("d2E_dpde", &PyDeformationEnergy::d2E_dpde, nb::arg("displacement"))
    .def("dE_dp", &PyDeformationEnergy::dE_dp, nb::arg("displacement"))
    .def("d2E_dp2", &PyDeformationEnergy::d2E_dp2, nb::arg("displacement"))
    .def("d2E_dude", &PyDeformationEnergy::d2E_dude, nb::arg("displacement"))
    .def("d2E_dudp", &PyDeformationEnergy::d2E_dudp, nb::arg("displacement"));

  nb::class_<PyParameterLayout>(m, "PyParameterLayout")
    .def_prop_ro("num_elements", &PyParameterLayout::numElements)
    .def_prop_ro(
      "num_local_parameters", &PyParameterLayout::numLocalParameters)
    .def_prop_ro(
      "num_global_parameters", &PyParameterLayout::numGlobalParameters)
    .def_prop_ro("num_value_rows", &PyParameterLayout::numValueRows);
  nb::class_<PyMaterialEvaluator>(m, "PyMaterialEvaluator")
    .def_prop_ro("num_parameters", &PyMaterialEvaluator::numParameters)
    .def_prop_ro("num_channels", &PyMaterialEvaluator::numChannels);
  nb::class_<
    PyDifferentiableMaterialEvaluator,
    PyMaterialEvaluator>(
    m, "PyDifferentiableMaterialEvaluator");

  nb::class_<PyOptimizableParameterRef>(m, "PyOptimizableParameterRef")
    .def_prop_ro("name", &PyOptimizableParameterRef::name)
    .def_prop_ro(
      "parameter_index", &PyOptimizableParameterRef::parameterIndex);
  nb::class_<PyOptimizableMaterialChannelRef>(
    m, "PyOptimizableMaterialChannelRef")
    .def_prop_ro("name", &PyOptimizableMaterialChannelRef::name)
    .def_prop_ro(
      "channel_index", &PyOptimizableMaterialChannelRef::channelIndex);

  nb::class_<PyOptimizableParameterField>(m, "PyOptimizableParameterField")
    .def_prop_ro(
      "num_material_channels",
      &PyOptimizableParameterField::numMaterialChannels)
    .def_prop_ro(
      "num_local_parameters",
      &PyOptimizableParameterField::numLocalParameters)
    .def_prop_ro(
      "num_global_parameters",
      &PyOptimizableParameterField::numGlobalParameters)
    .def_prop_ro("num_value_rows", &PyOptimizableParameterField::numValueRows)
    .def_prop_ro("parameter_names", &PyOptimizableParameterField::parameterNames)
    .def_prop_ro("layout", &PyOptimizableParameterField::layout)
    .def_prop_ro("evaluator", &PyOptimizableParameterField::evaluator)
    .def("parameter", &PyOptimizableParameterField::parameter, nb::arg("name"));

  nb::class_<PyOptimizableParameters>(m, "PyOptimizableParameters")
    .def_prop_ro("elastic_field", &PyOptimizableParameters::elasticField)
    .def_prop_ro("plastic_field", &PyOptimizableParameters::plasticField)
    .def_prop_ro(
      "elastic_values", &PyOptimizableParameters::elasticValues,
      nb::rv_policy::move)
    .def_prop_ro(
      "plastic_values", &PyOptimizableParameters::plasticValues,
      nb::rv_policy::move)
    .def("set_elastic_values", &PyOptimizableParameters::setElasticValues, nb::arg("values"))
    .def("set_plastic_values", &PyOptimizableParameters::setPlasticValues, nb::arg("values"))
    .def("set_values", &PyOptimizableParameters::setValues,
      nb::arg("elastic_values"), nb::arg("plastic_values"))
    .def("_same_fields", &PyOptimizableParameters::sameFields, nb::arg("other"));

  nb::class_<PyFixedParameterField>(m, "PyFixedParameterField")
    .def_prop_ro("parameter_names", &PyFixedParameterField::parameterNames)
    .def_prop_ro("num_elements", &PyFixedParameterField::numElements)
    .def_prop_ro(
      "num_local_parameters", &PyFixedParameterField::numLocalParameters)
    .def_prop_ro(
      "num_global_parameters", &PyFixedParameterField::numGlobalParameters)
    .def_prop_ro("num_value_rows", &PyFixedParameterField::numValueRows)
    .def_prop_ro(
      "num_material_channels",
      &PyFixedParameterField::numMaterialChannels)
    .def_prop_ro("layout", &PyFixedParameterField::layout)
    .def_prop_ro("evaluator", &PyFixedParameterField::evaluator);
  nb::class_<PyMaterialAssignment>(m, "PyMaterialAssignment")
    .def_prop_ro("optimizable_parameters", &PyMaterialAssignment::optimizableParameters);

  nb::class_<PyElasticParameterization>(m, "PyElasticParameterization")
    .def_prop_ro("definition", &PyElasticParameterization::definition)
    .def_prop_ro("fixed_field", &PyElasticParameterization::fixedField)
    .def_prop_ro(
      "optimizable_field", &PyElasticParameterization::optimizableField)
    .def_prop_ro("fixed_channel_names", &PyElasticParameterization::fixedChannelNames)
    .def_prop_ro(
      "optimizable_channel_names",
      &PyElasticParameterization::optimizableChannelNames)
    .def(
      "optimizable_channel", &PyElasticParameterization::optimizableChannel,
      nb::arg("name"))
    .def_prop_ro("num_elements", &PyElasticParameterization::numElements);
  nb::class_<PyPlasticParameterization>(m, "PyPlasticParameterization")
    .def_prop_ro("definition", &PyPlasticParameterization::definition)
    .def_prop_ro("fixed_field", &PyPlasticParameterization::fixedField)
    .def_prop_ro(
      "optimizable_field", &PyPlasticParameterization::optimizableField)
    .def_prop_ro("fixed_channel_names", &PyPlasticParameterization::fixedChannelNames)
    .def_prop_ro(
      "optimizable_channel_names",
      &PyPlasticParameterization::optimizableChannelNames)
    .def(
      "optimizable_channel", &PyPlasticParameterization::optimizableChannel,
      nb::arg("name"))
    .def_prop_ro("num_elements", &PyPlasticParameterization::numElements)
    .def_prop_ro("dofs", &PyPlasticParameterization::dofs);
  nb::class_<PyMaterialParameterization>(m, "PyMaterialParameterization")
    .def_prop_ro("elastic", &PyMaterialParameterization::elastic)
    .def_prop_ro("plastic", &PyMaterialParameterization::plastic)
    .def_prop_ro("num_elements", &PyMaterialParameterization::numElements);
  nb::class_<PyMaterialParameterData>(m, "PyMaterialParameterData")
    .def_prop_ro("elastic_fixed_values", &PyMaterialParameterData::elasticFixedValues,
      nb::rv_policy::move)
    .def_prop_ro("elastic_initial_optimizable_values",
      &PyMaterialParameterData::elasticInitialOptimizableValues, nb::rv_policy::move)
    .def_prop_ro("plastic_fixed_values", &PyMaterialParameterData::plasticFixedValues,
      nb::rv_policy::move)
    .def_prop_ro("plastic_initial_optimizable_values",
      &PyMaterialParameterData::plasticInitialOptimizableValues, nb::rv_policy::move);

  m.def("_make_elementwise_parameter_layout",
    &makeElementwiseParameterLayout,
    nb::arg("num_elements"), nb::arg("num_local_parameters"));
  m.def("_make_constant_parameter_layout",
    &makeConstantParameterLayout,
    nb::arg("num_elements"), nb::arg("num_local_parameters"));
  m.def("_make_identity_material_evaluator",
    &makeIdentityMaterialEvaluator, nb::arg("num_parameters"));

  m.def("_create_optimizable_parameter_field", &createOptimizableParameterField,
    nb::arg("parameter_names"), nb::arg("layout"), nb::arg("evaluator"));
  m.def("_create_fixed_parameter_field", &createFixedParameterField,
    nb::arg("parameter_names"), nb::arg("layout"), nb::arg("evaluator"));
  m.def("_create_elastic_parameterization", &createElasticParameterization,
    nb::arg("definition"), nb::arg("fixed_field"),
    nb::arg("optimizable_field"));
  m.def("_create_plastic_parameterization", &createPlasticParameterization,
    nb::arg("definition"), nb::arg("fixed_field"),
    nb::arg("optimizable_field"));
  m.def("_create_material_parameterization", &createMaterialParameterization,
    nb::arg("elastic"), nb::arg("plastic"));
  m.def("_project_material_parameter_data", &projectMaterialParameterData,
    nb::arg("asset"), nb::arg("parameterization"));
  m.def("_create_material_parameter_data", &createMaterialParameterData,
    nb::arg("elastic_fixed_values"), nb::arg("elastic_initial_optimizable_values"),
    nb::arg("plastic_fixed_values"), nb::arg("plastic_initial_optimizable_values"));
  m.def("_project_material_parameter_data_from_imported_data",
    &projectMaterialParameterDataFromImportedData,
    nb::arg("source"), nb::arg("parameterization"));
  m.def("_resolve_material_input", &resolveMaterialInput,
    nb::arg("source"), nb::arg("name"));
  m.def("_pack_material_element_inputs", &packMaterialElementInputs,
    nb::arg("layout"), nb::arg("element_local_values"));
  m.def("_validate_material_parameter_data", &validateMaterialParameterData,
    nb::arg("parameterization"), nb::arg("data"));
  m.def("_create_material_assignment_from_parameterization",
    &createMaterialAssignmentFromParameterization,
    nb::arg("asset"), nb::arg("parameterization"), nb::arg("data"));
  m.def("_create_deformation_energy", &createDeformationEnergy,
    nb::arg("assignment"), nb::arg("formulation"),
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
