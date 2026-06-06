set(SOLID_DEFORMATION_MODEL_HEADERS
  elastic/elasticModel.h
  elastic/elasticModel3DDeformationGradient.h
  elastic/elasticModelStableNeoHookeanMaterial.h
  elastic/elasticModelInvariantBasedMaterial.h
  elastic/elasticModelLinearMaterial.h
  elastic/elasticModelHillTypeMaterial.h
  elastic/elasticModelVolumeMaterial.h
  elastic/elasticModelCombinedMaterial.h
  elastic/elasticModel3DMooneyRivlin.h
  elastic/elasticModel3DSTVKMaterial.h


  elastic/elasticModel1DQuadratic.h
  elastic/elasticModel1DCubicSpline.h

  elastic/elasticModel2DFundamentalForms.h
  elastic/elasticModel2DFundamentalFormsFabric.h
  elastic/elasticModel2DFundamentalFormsSTVK.h

  elastic/invariantBasedMaterial.h
  elastic/invariantBasedMaterialStVK.h

  plastic/plasticModel.h
  plastic/plasticModel3DDeformationGradient.h
  plastic/plasticModel3DConstant.h
  plastic/plasticModel3D3DOF.h
  plastic/plasticModel3D6DOF.h

  plastic/plasticModel2DFundamentalForms.h
  plastic/plasticModel2DFundamentalFormsUniformStretch.h

  deformation/deformationModel.h
  deformation/materialMaxStepPolynomialUtils.h
  deformation/deformationModelManager.h

  deformation/deformationModelAssembler.h
  deformation/deformationModelAssemblerCacheData.h
  deformation/deformationModelState.h
  deformation/hessianMatrixHandle.h

  energy/deformationModelEnergy.h
  energy/plasticMaterialEnergy.h
  energy/deformationEnergyBuilder.h

  simulation/simulationMesh.h
  simulation/simulationMeshMaterial.h
  constraints/tetVolumeConstraintFunctions.h
  simulation/tetMeshOccupation.h

  constraints/segmentChainConstraintFunctions.h
  constraints/segmentBinormalConstraintFunctions.h

  constraints/prescribedPrincipleStressConstraintFunctions.h

  simulation/generateTetMeshMatrix.h

  formulations/formulation.h
  formulations/basis/basis.h
  formulations/basis/tetP1Basis.h
  formulations/basis/hexTrilinearBasis.h
  formulations/basis/hexTricubicHermiteBasis.h

  formulations/quadrature/quadrature.h
  formulations/quadrature/tetP1DefaultQuadrature.h
  formulations/quadrature/gaussLegendreHexQuadrature.h

  formulations/kernels/volumetricKernel.h
  formulations/kernels/shellKernel.h
  formulations/kernels/koiterShellKernel.h

  formulations/elements/volumetricDeformationModel.h
  formulations/elements/volumetricDeformationModelCacheData.h
  formulations/elements/shellDeformationModel.h
  formulations/elements/shellDeformationModelCacheData.h

  formulations/geometry/tetP1Geometry.h

  formulations/dof/dofLayout.h
  formulations/dof/vertex3DofLayout.h
  formulations/dof/hexTricubicHermiteDofLayout.h

  formulations/parameters/parameterField.h
  formulations/parameters/constantParameterField.h
  formulations/parameters/elementwiseParameterField.h

  elastic/elasticModelFactory.h
  plastic/plasticModelFactory.h
)

set(SOLID_DEFORMATION_MODEL_SOURCES
  elastic/elasticModelInvariantBasedMaterial.cpp
  elastic/elasticModelStableNeoHookeanMaterial.cpp
  elastic/elasticModelLinearMaterial.cpp
  elastic/elasticModelHillTypeMaterial.cpp
  elastic/elasticModelVolumeMaterial.cpp
  elastic/elasticModel3DMooneyRivlin.cpp
  elastic/elasticModel3DSTVKMaterial.cpp

  elastic/elasticModel1DQuadratic.cpp
  elastic/elasticModel1DCubicSpline.cpp

  elastic/elasticModel2DFundamentalFormsFabric.cpp
  elastic/elasticModel2DFundamentalFormsSTVK.cpp

  elastic/invariantBasedMaterialStVK.cpp

  plastic/plasticModel3DConstant.cpp
  plastic/plasticModel3D3DOF.cpp
  plastic/plasticModel3D6DOF.cpp

  plastic/plasticModel2DFundamentalFormsUniformStretch.cpp

  deformation/materialMaxStepPolynomialUtils.cpp

  formulations/formulation.cpp

  formulations/basis/tetP1Basis.cpp
  formulations/basis/hexTrilinearBasis.cpp
  formulations/basis/hexTricubicHermiteBasis.cpp

  formulations/quadrature/tetP1DefaultQuadrature.cpp
  formulations/quadrature/gaussLegendreHexQuadrature.cpp

  formulations/kernels/volumetricKernel.cpp
  formulations/kernels/koiterShellKernel.cpp

  formulations/elements/volumetricDeformationModel.cpp
  formulations/elements/volumetricDeformationModelCacheData.cpp
  formulations/elements/shellDeformationModel.cpp

  formulations/dof/vertex3DofLayout.cpp
  formulations/dof/hexTricubicHermiteDofLayout.cpp

  formulations/parameters/constantParameterField.cpp
  formulations/parameters/elementwiseParameterField.cpp


  deformation/deformationModelManager.cpp

  deformation/deformationModelAssembler.cpp
  deformation/deformationModelState.cpp

  energy/deformationModelEnergy.cpp
  energy/plasticMaterialEnergy.cpp
  energy/deformationEnergyBuilder.cpp

  elastic/elasticModelFactory.cpp
  plastic/plasticModelFactory.cpp

  simulation/simulationMesh.cpp
  constraints/tetVolumeConstraintFunctions.cpp
  simulation/tetMeshOccupation.cpp

  constraints/segmentChainConstraintFunctions.cpp
  constraints/segmentBinormalConstraintFunctions.cpp

  constraints/prescribedPrincipleStressConstraintFunctions.cpp

  simulation/generateTetMeshMatrix.cpp
)

set(SOLID_DEFORMATION_MODEL_DEPS
  pgoLogging basicAlgorithms eigenSupport nonlinearOptimization configFileJSON volumetricMesh interpolationCoordinates
  Eigen3::Eigen TBB::tbb fmt::fmt-header-only
)

foreach(tgt ${SOLID_DEFORMATION_MODEL_DEPS})
  if(NOT TARGET ${tgt})
    message(STATUS "solidDeformationModel is not included. Missing: ${tgt}")
    return()
  endif()
endforeach()

add_libpgo_lib(solidDeformationModel "${SOLID_DEFORMATION_MODEL_SOURCES}" "${SOLID_DEFORMATION_MODEL_HEADERS}")

target_link_libraries(solidDeformationModel PUBLIC ${SOLID_DEFORMATION_MODEL_DEPS})
