/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "constraints/prescribedPrincipleStressConstraintFunctions.h"
#include "deformation/deformationModelManager.h"
#include "material/runtime/materialState.h"
#include "simulation/simulationMesh.h"

#include "deformation/volume/volumetricDeformationModel.h"

#include "svdDerivatives.h"
#include "pgoLogging.h"

#include <algorithm>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <utility>

using namespace pgo;
using namespace pgo::SolidDeformationModel;

using TetFEM = VolumetricDeformationModel;

PrescribedPrincipleStressConstraintFunctions::
  PrescribedPrincipleStressConstraintFunctions(
    int nAll, int doff, std::span<const int> elementIDs,
    const DeformationModelManager &tmdmm,
    MaterialState materialState):
  ConstraintFunctions(nAll),
  dofStart(doff),
  tetMeshDMM(tmdmm),
  materialState_(std::move(materialState))
{
  if (materialState_.empty())
    throw std::invalid_argument(
      "PrescribedPrincipleStressConstraintFunctions requires material state.");
  const auto &elasticField = materialState_.elasticField();
  const auto &plasticField = materialState_.plasticField();
  const SimulationMesh &mesh = tetMeshDMM.getMesh();
  if (elasticField.mapping().numChannels() !=
    tetMeshDMM.getNumElasticParameters())
    throw std::invalid_argument(
      "Constraint elastic parameter channels do not match the deformation model.");
  if (plasticField.mapping().numChannels() !=
    tetMeshDMM.getNumPlasticParameters())
    throw std::invalid_argument(
      "Constraint plastic parameter channels do not match the deformation model.");
  if (elasticField.layout().numElements() != mesh.getNumElements() ||
    plasticField.layout().numElements() != mesh.getNumElements())
    throw std::invalid_argument(
      "Constraint optimizable parameter layouts do not match the mesh.");

  elements.assign(elementIDs.begin(), elementIDs.end());
  targetPrincipleStress.resize(static_cast<Eigen::Index>(elements.size()) * 3);

  elementData_.reserve(elements.size());
  for (std::size_t ei = 0; ei < elements.size(); ei++) {
    if (elements[ei] < 0 || elements[ei] >= mesh.getNumElements())
      throw std::out_of_range(
        "Prescribed principal stress constraint element is out of range.");
    const auto &dm = tetMeshDMM.getDeformationModel(elements[ei]);
    const auto *tetFEM = dynamic_cast<const TetFEM *>(&dm);
    if (tetFEM == nullptr)
      throw std::invalid_argument(
        "Prescribed principal stress constraints require volumetric deformation models.");
    elementData_.emplace_back(*tetFEM);
  }

  std::vector<ES::TripletD> entries;
  for (std::size_t i = 0; i < elements.size(); i++) {
    for (int r = 0; r < 3; r++) {
      for (int j = 0; j < 4; j++) {
        for (int dof = 0; dof < 3; dof++) {
          entries.emplace_back(static_cast<int>(i) * 3 + r, dofStart + tetMeshDMM.getMesh().getVertexIndex(elements[i], j) * 3 + dof, 1.0);
        }
      }
    }
  }
  jacobianTemplate.resize(static_cast<int>(elements.size()) * 3, nAll);
  jacobianTemplate.setFromTriplets(entries.begin(), entries.end());
  ES::buildEntryMap(jacobianTemplate, jacEntries);

  entries.clear();
  for (std::size_t i = 0; i < elements.size(); i++) {
    for (int vi = 0; vi < 4; vi++) {
      for (int dofi = 0; dofi < 3; dofi++) {
        int row = tetMeshDMM.getMesh().getVertexIndex(elements[i], vi) * 3 + dofi;
        for (int vj = 0; vj < 4; vj++) {
          for (int dofj = 0; dofj < 3; dofj++) {
            int col = tetMeshDMM.getMesh().getVertexIndex(elements[i], vj) * 3 + dofj;
            entries.emplace_back(dofStart + row, dofStart + col, 1.0);
          }
        }
      }
    }
  }

  lambdahTemplate.resize(nAll, nAll);
  lambdahTemplate.setFromTriplets(entries.begin(), entries.end());
  ES::buildEntryMap(lambdahTemplate, hessEntries);

  hessLocks = std::vector<tbb::spin_mutex>(nAll);
}

VolumetricDeformationModelEvaluator &
PrescribedPrincipleStressConstraintFunctions::prepareElement(
  int elementID, MaterialStateView state,
  ElementData &data) const
{
  const std::span<const double> elasticParameters =
    state.evaluateElement(
      state.elasticField(), elementID, data.numMaterialLocations,
      data.elasticParameters);
  const std::span<const double> plasticParameters =
    state.evaluateElement(
      state.plasticField(), elementID, data.numMaterialLocations,
      data.plasticParameters);

  data.evaluator->prepare(
    std::span<const double>(data.localp.data(),
      static_cast<std::size_t>(data.numDOFs)),
    elasticParameters, plasticParameters);
  return *data.evaluator;
}

void PrescribedPrincipleStressConstraintFunctions::setTargetPHat(
  std::span<const double> phat)
{
  const std::size_t expected = elements.size() * 3;
  if (phat.size() != expected)
    throw std::invalid_argument(
      "Prescribed principal stress target has unexpected size.");
  targetPrincipleStress = Eigen::Map<const EigenSupport::VXd>(
    phat.data(), static_cast<Eigen::Index>(phat.size()));
}

// g = S(P) - Pbar
void PrescribedPrincipleStressConstraintFunctions::func(ES::ConstRefVecXd x, ES::RefVecXd g) const
{
  const MaterialStateView state = materialState_.view();
  for (int i = 0; i < (int)elements.size(); i++) {
    auto &scratch = elementData_[i];
    ES::V18d &localp = scratch.localp;
    ES::V3d Phat = targetPrincipleStress.segment<3>(i * 3);
    int eleID = elements[i];
    for (int j = 0; j < tetMeshDMM.getMesh().getNumElementVertices(); j++) {
      int vid = tetMeshDMM.getMesh().getVertexIndex(eleID, j);
      ES::V3d vtxp;
      xToPosFunc(x.segment<3>(dofStart + vid * 3), dofStart + vid * 3, vtxp);
      localp.segment<3>(j * 3) = vtxp;
    }

    VolumetricDeformationModelEvaluator &evaluator =
      prepareElement(eleID, state, scratch);

    ES::M3d P = evaluator.compute_P(0);

    ES::V3d S;
    NonlinearOptimization::SVDDerivatives::unorderedSquareMatrixSVD3(P, S);

    g.segment<3>(i * 3) = S - Phat;
  }
}

void PrescribedPrincipleStressConstraintFunctions::computeForceFromTargetPHat(ES::ConstRefVecXd x, ES::RefVecXd fext) const
{
  fext.setZero();
  const MaterialStateView state = materialState_.view();

  for (int i = 0; i < (int)elements.size(); i++) {
    auto &scratch = elementData_[i];
    ES::V18d &localp = scratch.localp;
    ES::V3d Phat = targetPrincipleStress.segment<3>(i * 3);
    int eleID = elements[i];
    for (int j = 0; j < tetMeshDMM.getMesh().getNumElementVertices(); j++) {
      int vid = tetMeshDMM.getMesh().getVertexIndex(eleID, j);
      ES::V3d vtxp;
      xToPosFunc(x.segment<3>(dofStart + vid * 3), dofStart + vid * 3, vtxp);
      localp.segment<3>(j * 3) = vtxp;
    }

    VolumetricDeformationModelEvaluator &evaluator =
      prepareElement(eleID, state, scratch);

    ES::M3d P = evaluator.compute_P(0);

    ES::V3d S;
    ES::M3d U, V;
    NonlinearOptimization::SVDDerivatives::unorderedSquareMatrixSVD3(P, S, &U, &V);

    ES::M3d P1 = U * Phat.asDiagonal() * V.transpose();
    // ES::M3d P1 = Phat.asDiagonal();
    ES::V12d f;
    f.setZero();

    evaluator.computeForceFromP(0, P1, f);
    for (int j = 0; j < tetMeshDMM.getMesh().getNumElementVertices(); j++) {
      int vid = tetMeshDMM.getMesh().getVertexIndex(eleID, j);
      fext.segment<3>(vid * 3) = f.segment<3>(j * 3);
    }
  }
}

double PrescribedPrincipleStressConstraintFunctions::computeSurfaceNormalTractionFromElement(ES::ConstRefVecXd x, const ES::V3d &n, int eleID) const
{
  const auto it = std::find(elements.begin(), elements.end(), eleID);
  if (it == elements.end())
    throw std::out_of_range("Element is not part of the principal stress constraint.");
  const std::size_t elementIndex = static_cast<std::size_t>(std::distance(elements.begin(), it));
  auto &scratch = elementData_[elementIndex];
  ES::V18d &localp = scratch.localp;

  for (int j = 0; j < tetMeshDMM.getMesh().getNumElementVertices(); j++) {
    int vid = tetMeshDMM.getMesh().getVertexIndex(eleID, j);
    ES::V3d vtxp;
    xToPosFunc(x.segment<3>(dofStart + vid * 3), dofStart + vid * 3, vtxp);
    localp.segment<3>(j * 3) = vtxp;
  }
  VolumetricDeformationModelEvaluator &evaluator =
    prepareElement(eleID, materialState_.view(), scratch);

  ES::M3d P = evaluator.compute_P(0);

  return (P * n).dot(n);
}

// dg/dx = dS/dP dP/dF dF/dx
void PrescribedPrincipleStressConstraintFunctions::jacobian(ES::ConstRefVecXd x, ES::SpMatD &jac) const
{
  const MaterialStateView state = materialState_.view();
  for (int i = 0; i < (int)elements.size(); i++) {
    auto &scratch = elementData_[i];
    ES::V18d &localp = scratch.localp;
    ES::V3d Phat = targetPrincipleStress.segment<3>(i * 3);
    int eleID = elements[i];
    for (int j = 0; j < tetMeshDMM.getMesh().getNumElementVertices(); j++) {
      int vid = tetMeshDMM.getMesh().getVertexIndex(eleID, j);
      ES::V3d vtxp;
      xToPosFunc(x.segment<3>(dofStart + vid * 3), dofStart + vid * 3, vtxp);
      localp.segment<3>(j * 3) = vtxp;
    }

    VolumetricDeformationModelEvaluator &evaluator =
      prepareElement(eleID, state, scratch);

    ES::M3d P = evaluator.compute_P(0);

    ES::V3d S;
    ES::M3d U, V;
    NonlinearOptimization::SVDDerivatives::unorderedSquareMatrixSVD3(P, S, &U, &V);

    ES::M3x9d dSdPi;
    for (int i = 0; i < 9; i++) {
      ES::M3d dP;
      dP.setZero();
      dP.data()[i] = 1.0;
      ES::V3d dS;
      NonlinearOptimization::SVDDerivatives::unorderedSquareMatrixSVD3Derivatices(
        P, S, U, V, dP, dS);
      dSdPi.col(i) = dS;
    }

    ES::M9d dPdF;
    ES::M9x12d dFdx;
    dPdF = evaluator.compute_dP_dF(0);
    evaluator.compute_dF_dx(0, dFdx);

    ES::M9x12d dPdx = dPdF * dFdx;

    ES::M3x12d dSdx = ES::M3x12d::Zero();

    // dS/dPi dPi/dx
    for (int i = 0; i < 9; i++) {
      dSdx += dSdPi.col(i) * dPdx.row(i);
    }

    for (int ci = 0; ci < 12; ci++) {
      for (int ri = 0; ri < 3; ri++) {
        int vid = ci / 3;
        int dof = ci % 3;

        auto it = jacEntries.find(std::make_pair(i * 3 + ri, tetMeshDMM.getMesh().getVertexIndex(eleID, vid) * 3 + dof));
        PGO_ALOG(it != jacEntries.end());

        jac.valuePtr()[it->second] = dSdx(ri, ci);
      }
    }
  }
}

// dg/dx = dSdP dPdx
// d2g/dx2 = dPdx d2SdP2 dPdx + dSdP d2Pdx2
void PrescribedPrincipleStressConstraintFunctions::hessianInPlace(ES::ConstRefVecXd x, ES::ConstRefVecXd lambda, ES::SpMatD &hess) const
{
  const MaterialStateView state = materialState_.view();
  for (int ei = 0; ei < (int)elements.size(); ei++) {
    auto &scratch = elementData_[ei];
    ES::V18d &localp = scratch.localp;
    int eleID = elements[ei];
    for (int j = 0; j < tetMeshDMM.getMesh().getNumElementVertices(); j++) {
      int vid = tetMeshDMM.getMesh().getVertexIndex(eleID, j);
      ES::V3d vtxp;
      xToPosFunc(x.segment<3>(dofStart + vid * 3), dofStart + vid * 3, vtxp);
      localp.segment<3>(j * 3) = vtxp;
    }

    VolumetricDeformationModelEvaluator &evaluator =
      prepareElement(eleID, state, scratch);

    ES::M3d P = evaluator.compute_P(0);

    ES::V3d S;
    ES::M3d U, V;
    NonlinearOptimization::SVDDerivatives::unorderedSquareMatrixSVD3(P, S, &U, &V);

    ES::M3x81d d2SdPidPj;
    for (int i = 0; i < 9; i++) {
      ES::M3d dP;
      dP.setZero();
      dP.data()[i] = 1.0;

      ES::V3d dS;
      NonlinearOptimization::SVDDerivatives::unorderedSquareMatrixSVD3Derivatices(
        P, S, U, V, dP, dS);

      for (int j = 0; j < 9; j++) {
        ES::M3d d2P;
        d2P.setZero();
        ES::V3d d2S;

        NonlinearOptimization::SVDDerivatives::unorderedSquareMatrixSVD3Derivatices(P, S, U, V,
          dP, dS, nullptr, nullptr,
          &d2P, &d2S);
        d2SdPidPj.col(i * 9 + j) = d2S;
      }
    }

    ES::M9d dPdF;
    ES::M9x12d dFdx;
    dPdF = evaluator.compute_dP_dF(0);
    evaluator.compute_dF_dx(0, dFdx);
    ES::M9x12d dPdx = dPdF * dFdx;

    ES::M12d hessLocal;
    hessLocal.setZero();

    ES::V3d lambdaLocal = lambda.segment<3>(ei * 3);

    // d2g/dx2 = dPdx d2SdP2 dPdx + dSdP d2Pdx2
    // second term is omit
    for (int si = 0; si < 3; si++) {
      for (int i = 0; i < 9; i++) {
        ES::V12d dPidx = dPdx.row(i);

        for (int j = 0; j < 9; j++) {
          ES::V12d dPjdx = dPdx.row(j);

          ES::M12d h;
          ES::tensorProduct(h, dPjdx, dPidx);
          hessLocal += d2SdPidPj(si, i * 9 + j) * lambdaLocal[si] * h;
        }
      }
    }

    for (int ci = 0; ci < 12; ci++) {
      for (int ri = 0; ri < 12; ri++) {
        int vi = ci / 3;
        int dofi = ci % 3;

        int vj = ri / 3;
        int dofj = ri % 3;

        int globalRow = tetMeshDMM.getMesh().getVertexIndex(eleID, vj) * 3 + dofj;
        int globalCol = tetMeshDMM.getMesh().getVertexIndex(eleID, vi) * 3 + dofi;

        auto it = hessEntries.find(std::make_pair(globalRow, globalCol));
        PGO_ALOG(it != hessEntries.end());

        hessLocks[globalRow].lock();

        hess.valuePtr()[it->second] += hessLocal(ri, ci);

        hessLocks[globalRow].unlock();
      }
    }
  }
}
