#include "shellFormulation.h"

#include "mass/shellArealDensityField.h"
#include "material/core/materialParameters.h"
#include "deformation/shell/shellDeformationModel.h"
#include "simulation/simulationMesh.h"

#include "EigenSupport.h"

#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{
namespace
{
namespace ES = EigenSupport;

template<class Derived, class Base>
std::unique_ptr<Derived> checkedMaterialCast(
  std::unique_ptr<Base> model, const char *message)
{
  if (Derived *typed = dynamic_cast<Derived *>(model.get())) {
    model.release();
    return std::unique_ptr<Derived>(typed);
  }

  throw std::invalid_argument(message);
}

double triangleRestArea(const SimulationMesh &mesh, int ele)
{
  const ES::V3d &a = mesh.getVertex(ele, 0);
  const ES::V3d &b = mesh.getVertex(ele, 1);
  const ES::V3d &c = mesh.getVertex(ele, 2);
  return 0.5 * ((b - a).cross(c - a)).norm();
}
}  // namespace

SimulationMeshType ShellFormulation::compatibleMeshType() const
{
  return SimulationMeshType::SHELL;
}

std::unique_ptr<DeformationModel> ShellFormulation::createElement(
  const SimulationMesh &mesh, int ele,
  std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel) const
{
  ES::V18d restPosition = ES::V18d::Zero();
  std::array<bool, 6> hasVtx;
  for (int j = 0; j < 6; j++) {
    if (mesh.getVertexIndex(ele, j) < 0) {
      hasVtx[j] = false;
    }
    else {
      hasVtx[j] = true;
      restPosition.segment<3>(3 * j) = mesh.getVertex(ele, j);
    }
  }

  auto mapping = createElementMapping(restPosition, hasVtx);
  return std::make_unique<ShellDeformationModel>(
    std::move(mapping),
    checkedMaterialCast<ElasticModel2DFundamentalForms>(
      std::move(elasticModel),
      "ShellFormulation requires ElasticModel2DFundamentalForms."),
    checkedMaterialCast<PlasticModel2DFundamentalForms>(
      std::move(plasticModel),
      "ShellFormulation requires PlasticModel2DFundamentalForms."));
}

EigenSupport::SpMatD ShellFormulation::buildMassMatrix(
  const SimulationMesh &mesh, const ShellArealDensityField &arealDensity,
  MaterialParameterEvaluationView state) const
{
  if (mesh.getElementType() != compatibleMeshType()) {
    throw std::invalid_argument("mesh type is incompatible with this formulation");
  }

  arealDensity.validate(mesh.getNumElements());
  auto evaluation = arealDensity.evaluator(std::move(state));

  std::vector<ES::TripletD> entries;
  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    const double m = evaluation.value(ele, 0) * triangleRestArea(mesh, ele) / 3.0;
    for (int j = 0; j < 3; j++) {
      const int v = mesh.getVertexIndex(ele, j);
      for (int d = 0; d < 3; d++)
        entries.emplace_back(v * 3 + d, v * 3 + d, m);
    }
  }

  const int numDofs = mesh.getNumVertices() * 3;
  ES::SpMatD M(numDofs, numDofs);
  M.setFromTriplets(entries.begin(), entries.end());
  return M;
}

EigenSupport::VXd ShellFormulation::buildBodyForce(
  const SimulationMesh &mesh, const EigenSupport::V3d &acceleration,
  const ShellArealDensityField &arealDensity, MaterialParameterEvaluationView state) const
{
  if (mesh.getElementType() != compatibleMeshType()) {
    throw std::invalid_argument("mesh type is incompatible with this formulation");
  }

  arealDensity.validate(mesh.getNumElements());
  auto evaluation = arealDensity.evaluator(std::move(state));

  ES::VXd f = ES::VXd::Zero(mesh.getNumVertices() * 3);
  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    const double m = evaluation.value(ele, 0) * triangleRestArea(mesh, ele) / 3.0;
    for (int j = 0; j < 3; j++) {
      const int v = mesh.getVertexIndex(ele, j);
      f.segment<3>(v * 3) += m * acceleration;
    }
  }
  return f;
}

EigenSupport::SpMatD ShellFormulation::buildBodyForceParameterJacobian(
  const SimulationMesh &mesh, const EigenSupport::V3d &acceleration,
  const ShellArealDensityField &arealDensity, MaterialParameterEvaluationView state) const
{
  if (mesh.getElementType() != compatibleMeshType()) {
    throw std::invalid_argument("mesh type is incompatible with this formulation");
  }
  arealDensity.validate(mesh.getNumElements());
  const MaterialParameterRef *dependency = arealDensity.parameterDependency();
  if (dependency == nullptr) {
    throw std::invalid_argument(
      "buildBodyForceParameterJacobian requires a parameter-dependent areal density field");
  }
  if (state.empty()) {
    throw std::invalid_argument(
      "parameter-dependent areal density requires material parameter state");
  }

  if (&dependency->field() != &state.space().elastic()) {
    throw std::invalid_argument(
      "parameter-dependent areal density must depend on the evaluation space elastic field");
  }

  auto evaluation = arealDensity.evaluator(std::move(state));
  const MaterialParameterRef &parameter = *dependency;
  const auto &layout = parameter.field().dofLayout();
  const int numLocal = layout.numLocalDofs();
  ES::VXd dRho(numLocal);
  std::vector<ES::TripletD> entries;
  entries.reserve(static_cast<size_t>(mesh.getNumElements()) * numLocal * 9);

  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    evaluation.localParameterDerivative(
      ele, 0, dRho);
    const double areaThird = triangleRestArea(mesh, ele) / 3.0;
    for (int k = 0; k < numLocal; k++) {
      if (dRho[k] == 0.0)
        continue;
      const int col = layout.globalDof(ele, k);
      const double s = dRho[k] * areaThird;
      for (int j = 0; j < 3; j++) {
        const int v = mesh.getVertexIndex(ele, j);
        for (int d = 0; d < 3; d++)
          entries.emplace_back(v * 3 + d, col, s * acceleration[d]);
      }
    }
  }

  ES::SpMatD J(mesh.getNumVertices() * 3, layout.numGlobalDofs());
  J.setFromTriplets(entries.begin(), entries.end());
  return J;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
