#pragma once

#include "../formulations/formulationTraits.h"
#include "../formulations/formulationConcepts.h"
#include "../deformationModel.h"
#include "../simulationMesh.h"
#include "../koiterDeformationModel.h"
#include "EigenSupport.h"

#include <concepts>
#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

// ElementModelFactory
//
// Creates per-element DeformationModel instances using the formulation's
// traits-selected Kernel and ElementModel types. ShellKoiter routes to the
// existing KoiterDeformationModel path.

class ElementModelFactory
{
public:
  // Create the element FEM for one element.
  // Uses FormulationTraits<Formulation>::ElementModel for volumetric formulations
  // or the KoiterDeformationModel path for ShellKoiter.
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
    ES::V18d restPosition;
    for (int j = 0; j < 6; j++) {
      ES::V3d p;
      if (mesh.getVertexIndex(ele, j) < 0) {
        p = ES::V3d(-10496, -10496, -10496);
      }
      else {
        mesh.getVertex(ele, j, p.data());
      }
      restPosition.segment<3>(3 * j) = p;
    }

    if (elasticMaterialType == DeformationModelElasticMaterial::KOITER_FABRIC ||
        elasticMaterialType == DeformationModelElasticMaterial::KOITER_STVK) {
      return new KoiterDeformationModel(
        restPosition.data(), restPosition.data() + 3, restPosition.data() + 6,
        restPosition.data() + 9, restPosition.data() + 12,
        restPosition.data() + 15, elasticModel, plasticModel);
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
