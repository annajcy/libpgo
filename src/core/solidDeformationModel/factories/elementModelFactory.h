#pragma once

#include "../formulations/formulationTraits.h"
#include "../formulations/formulationConcepts.h"
#include "../formulations/elements/deformationGradientElementModel.h"
#include "../formulations/elements/koiterShellElementModel.h"
#include "../formulations/elements/parameterizedMaterialBlock.h"
#include "../formulations/basis/tetP1Basis.h"
#include "../formulations/basis/hexTrilinearBasis.h"
#include "../formulations/quadrature/tetP1DefaultQuadrature.h"
#include "../formulations/quadrature/gaussLegendreHexQuadrature.h"
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

class ElementModelFactory
{
public:
  // Block-based creation (production path).
  template<class Formulation>
  static std::unique_ptr<DeformationModel> create(
    const SimulationMesh &mesh,
    int ele,
    const ElasticBlock &elasticBlock,
    const PlasticBlock &plasticBlock,
    DeformationModelElasticMaterial elasticMaterialType);

  // Legacy creation — forwards to block-based with raw pointer blocks.
  template<class Formulation>
  static std::unique_ptr<DeformationModel> create(
    const SimulationMesh &mesh,
    int ele,
    ElasticModel *elasticModel,
    PlasticModel *plasticModel,
    DeformationModelElasticMaterial elasticMaterialType);
};

// ============================================================
// Block-based implementation
// ============================================================

template<class Formulation>
std::unique_ptr<DeformationModel> ElementModelFactory::create(
  const SimulationMesh &mesh,
  int ele,
  const ElasticBlock &elasticBlock,
  const PlasticBlock &plasticBlock,
  DeformationModelElasticMaterial elasticMaterialType)
{
  if constexpr (std::same_as<Formulation, TetP1>)
  {
    ES::V12d restPosition;
    for (int j = 0; j < 4; j++) {
      ES::V3d p;
      mesh.getVertex(ele, j, p.data());
      restPosition.segment<3>(j * 3) = p;
    }
    TetP1Basis basis;
    TetP1DefaultQuadrature quad;
    return std::make_unique<DeformationGradientElementModel>(
      ele, restPosition.data(), basis, quad, elasticBlock, plasticBlock);
  }
  else if constexpr (std::same_as<Formulation, HexTrilinear>)
  {
    ES::V24d restPosition;
    for (int j = 0; j < 8; j++) {
      ES::V3d p;
      mesh.getVertex(ele, j, p.data());
      restPosition.segment<3>(j * 3) = p;
    }
    HexTrilinearBasis basis;
    GaussLegendreHexQuadrature2 quad;
    return std::make_unique<DeformationGradientElementModel>(
      ele, restPosition.data(), basis, quad, elasticBlock, plasticBlock);
  }
  else if constexpr (std::same_as<Formulation, ShellKoiter>)
  {
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
      return std::make_unique<KoiterShellElementModel>(
        ele, restPosition.data(), hasVtx, elasticBlock, plasticBlock);
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

// ============================================================
// Legacy overload — delegates to block-based
// ============================================================

template<class Formulation>
std::unique_ptr<DeformationModel> ElementModelFactory::create(
  const SimulationMesh &mesh,
  int ele,
  ElasticModel *elasticModel,
  PlasticModel *plasticModel,
  DeformationModelElasticMaterial elasticMaterialType)
{
  if constexpr (std::same_as<Formulation, TetP1>)
  {
    ES::V12d restPosition;
    for (int j = 0; j < 4; j++) {
      ES::V3d p;
      mesh.getVertex(ele, j, p.data());
      restPosition.segment<3>(j * 3) = p;
    }
    TetP1Basis basis;
    TetP1DefaultQuadrature quad;
    return std::make_unique<DeformationGradientElementModel>(
      restPosition.data(), basis, quad, elasticModel, plasticModel);
  }
  else if constexpr (std::same_as<Formulation, HexTrilinear>)
  {
    ES::V24d restPosition;
    for (int j = 0; j < 8; j++) {
      ES::V3d p;
      mesh.getVertex(ele, j, p.data());
      restPosition.segment<3>(j * 3) = p;
    }
    HexTrilinearBasis basis;
    GaussLegendreHexQuadrature2 quad;
    return std::make_unique<DeformationGradientElementModel>(
      restPosition.data(), basis, quad, elasticModel, plasticModel);
  }
  else if constexpr (std::same_as<Formulation, ShellKoiter>)
  {
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
      return std::make_unique<KoiterShellElementModel>(
        restPosition.data(), hasVtx, elasticModel, plasticModel);
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
