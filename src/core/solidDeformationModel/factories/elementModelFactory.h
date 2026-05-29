#pragma once

#include "../formulations/formulationTraits.h"
#include "../formulations/formulationConcepts.h"
#include "../formulations/elements/koiterShellElementModel.h"
#include "../deformationModel.h"
#include "../simulationMesh.h"
#include "EigenSupport.h"
#include "deformationModelManager.h"

#include <concepts>
#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

// ElementModelFactory
//
// Creates per-element DeformationModel instances using the formulation's
// traits-selected Kernel and ElementModel types.

class ElementModelFactory
{
public:
  // Create the element FEM for one element using FormulationTraits::ElementModel.
  template<class Formulation>
  static DeformationModel *create(
    const SimulationMesh &mesh,
    int ele,
    ElasticModel *elasticModel,
    PlasticModel *plasticModel,
    DeformationModelElasticMaterial elasticMaterialType);
};

// ============================================================
// Template implementation
// ============================================================

template<class Formulation>
DeformationModel *ElementModelFactory::create(
  const SimulationMesh &mesh,
  int ele,
  ElasticModel *elasticModel,
  PlasticModel *plasticModel,
  DeformationModelElasticMaterial elasticMaterialType)
{
  if constexpr (std::same_as<Formulation, TetP1>)
  {
    using ElementModel = typename FormulationTraits<TetP1>::ElementModel;
    ES::V12d restPosition;
    for (int j = 0; j < 4; j++) {
      ES::V3d p;
      mesh.getVertex(ele, j, p.data());
      restPosition.segment<3>(j * 3) = p;
    }
    return new ElementModel(restPosition.data(), elasticModel, plasticModel);
  }
  else if constexpr (std::same_as<Formulation, HexTrilinear>)
  {
    using ElementModel = typename FormulationTraits<HexTrilinear>::ElementModel;
    ES::V24d restPosition;
    for (int j = 0; j < 8; j++) {
      ES::V3d p;
      mesh.getVertex(ele, j, p.data());
      restPosition.segment<3>(j * 3) = p;
    }
    return new ElementModel(restPosition.data(), elasticModel, plasticModel);
  }
  else if constexpr (std::same_as<Formulation, ShellKoiter>)
  {
    using ElementModel = typename FormulationTraits<ShellKoiter>::ElementModel;
    ES::V18d restPosition;
    bool hasVtx[6];
    for (int j = 0; j < 6; j++) {
      if (mesh.getVertexIndex(ele, j) < 0) {
        hasVtx[j] = false;
        restPosition.segment<3>(3 * j).setZero();
      }
      else {
        hasVtx[j] = true;
        ES::V3d p;
        mesh.getVertex(ele, j, p.data());
        restPosition.segment<3>(3 * j) = p;
      }
    }

    if (elasticMaterialType == DeformationModelElasticMaterial::KOITER_FABRIC ||
        elasticMaterialType == DeformationModelElasticMaterial::KOITER_STVK) {
      return new ElementModel(restPosition.data(), hasVtx, elasticModel, plasticModel);
    }
    else {
      throw std::logic_error("unsupported elastic material for shell element");
    }
  }
  else
  {
    throw std::logic_error("ElementModelFactory: unsupported formulation");
  }
}

}  // namespace SolidDeformationModel
}  // namespace pgo
