set(SOLID_DEFORMATION_MODEL_HEADERS
  material/elastic/elasticModel.h
  material/elastic/elasticModel3DDeformationGradient.h
  material/elastic/elasticModelStableNeoHookeanMaterial.h
  material/elastic/elasticModelInvariantBasedMaterial.h
  material/elastic/elasticModelLinearMaterial.h
  material/elastic/elasticModelHillTypeMaterial.h
  material/elastic/elasticModelVolumeMaterial.h
  material/elastic/elasticModelCombinedMaterial.h
  material/elastic/elasticModel3DMooneyRivlin.h
  material/elastic/elasticModel3DSTVKMaterial.h


  material/elastic/elasticModel1DQuadratic.h
  material/elastic/elasticModel1DCubicSpline.h

  material/elastic/elasticModel2DFundamentalForms.h
  material/elastic/elasticModel2DFundamentalFormsFabric.h
  material/elastic/elasticModel2DFundamentalFormsSTVK.h

  material/elastic/invariantBasedMaterial.h
  material/elastic/invariantBasedMaterialStVK.h

  material/plastic/plasticModel.h
  material/plastic/plasticModel3DDeformationGradient.h
  material/plastic/plasticModel3DConstant.h
  material/plastic/plasticModel3D3DOF.h
  material/plastic/plasticModel3D6DOF.h

  material/plastic/plasticModel2DFundamentalForms.h
  material/plastic/plasticModel2DFundamentalFormsUniformStretch.h

  material/materialTypes.h

  deformation/deformationModel.h
  deformation/materialMaxStepPolynomialUtils.h
  deformation/deformationModelManager.h

  deformation/deformationModelAssembler.h
  deformation/deformationModelAssemblerCacheData.h
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

  formulations/kinematics/volumetricKinematics.h
  formulations/kinematics/shellKinematics.h
  formulations/kinematics/koiterShellKinematics.h

  deformation/volume/volumetricDeformationModel.h
  deformation/volume/volumetricDeformationModelCacheData.h
  deformation/shell/shellDeformationModel.h
  deformation/shell/shellDeformationModelCacheData.h

  formulations/geometry/tetP1Geometry.h

  formulations/dof/dofLayout.h
  formulations/dof/vertex3DofLayout.h
  formulations/dof/hexTricubicHermiteDofLayout.h

  material/fields/parameterField.h
  material/fields/materialParameterFieldInit.h
  material/fields/constantParameterField.h
  material/fields/elementwiseParameterField.h

  material/elastic/elasticModelFactory.h
  material/plastic/plasticModelFactory.h
)

set(SOLID_DEFORMATION_MODEL_SOURCES
  material/elastic/elasticModelInvariantBasedMaterial.cpp
  material/elastic/elasticModelStableNeoHookeanMaterial.cpp
  material/elastic/elasticModelLinearMaterial.cpp
  material/elastic/elasticModelHillTypeMaterial.cpp
  material/elastic/elasticModelVolumeMaterial.cpp
  material/elastic/elasticModel3DMooneyRivlin.cpp
  material/elastic/elasticModel3DSTVKMaterial.cpp

  material/elastic/elasticModel1DQuadratic.cpp
  material/elastic/elasticModel1DCubicSpline.cpp

  material/elastic/elasticModel2DFundamentalFormsFabric.cpp
  material/elastic/elasticModel2DFundamentalFormsSTVK.cpp

  material/elastic/invariantBasedMaterialStVK.cpp

  material/plastic/plasticModel3DConstant.cpp
  material/plastic/plasticModel3D3DOF.cpp
  material/plastic/plasticModel3D6DOF.cpp

  material/plastic/plasticModel2DFundamentalFormsUniformStretch.cpp

  deformation/materialMaxStepPolynomialUtils.cpp

  formulations/formulation.cpp

  formulations/basis/tetP1Basis.cpp
  formulations/basis/hexTrilinearBasis.cpp
  formulations/basis/hexTricubicHermiteBasis.cpp

  formulations/quadrature/tetP1DefaultQuadrature.cpp
  formulations/quadrature/gaussLegendreHexQuadrature.cpp

  formulations/kinematics/volumetricKinematics.cpp
  formulations/kinematics/koiterShellKinematics.cpp

  deformation/volume/volumetricDeformationModel.cpp
  deformation/volume/volumetricDeformationModelCacheData.cpp
  deformation/shell/shellDeformationModel.cpp

  formulations/dof/vertex3DofLayout.cpp
  formulations/dof/hexTricubicHermiteDofLayout.cpp

  material/fields/constantParameterField.cpp
  material/fields/elementwiseParameterField.cpp
  material/fields/materialParameterFieldInit.cpp


  deformation/deformationModelManager.cpp

  deformation/deformationModelAssembler.cpp

  energy/deformationModelEnergy.cpp
  energy/plasticMaterialEnergy.cpp
  energy/deformationEnergyBuilder.cpp

  material/elastic/elasticModelFactory.cpp
  material/plastic/plasticModelFactory.cpp

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
