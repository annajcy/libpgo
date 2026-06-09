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
  energy/elasticMaterialEnergy.h
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

  formulations/formulation/formulation.h
  formulations/formulation/formulations.h
  formulations/formulation/volumetricFormulation/volumetricFormulation.h
  formulations/formulation/volumetricFormulation/tetFormulation.h
  formulations/formulation/volumetricFormulation/cubicFormulation.h
  formulations/formulation/volumetricFormulation/tetLinearFormulation.h
  formulations/formulation/volumetricFormulation/cubicLinearFormulation.h
  formulations/formulation/volumetricFormulation/cubicTricubicHermiteFormulation.h
  formulations/formulation/shellFormulation/shellFormulation.h
  formulations/formulation/shellFormulation/koiterShellFormulation.h
  formulations/shapeFunction/shapeFunction.h
  formulations/shapeFunction/tetLinearShapeFunction.h
  formulations/shapeFunction/cubicLinearShapeFunction.h
  formulations/shapeFunction/cubicTricubicHermiteShapeFunction.h

  formulations/quadrature/quadrature.h
  formulations/quadrature/tetLinearDefaultQuadrature.h
  formulations/quadrature/gaussLegendreHexQuadrature.h

  deformation/volume/volumetricElementMapping.h
  deformation/shell/shellElementMapping.h
  deformation/shell/koiterShellElementMapping.h

  deformation/volume/volumetricDeformationModel.h
  deformation/volume/volumetricDeformationModelCacheData.h
  deformation/shell/shellDeformationModel.h
  deformation/shell/shellDeformationModelCacheData.h

  formulations/dof/dofLayout.h
  formulations/dof/vertex3DofLayout.h
  formulations/dof/cubicTricubicHermiteDofLayout.h

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

  formulations/formulation/formulation.cpp
  formulations/formulation/volumetricFormulation/volumetricFormulation.cpp
  formulations/formulation/volumetricFormulation/tetFormulation.cpp
  formulations/formulation/volumetricFormulation/cubicFormulation.cpp
  formulations/formulation/volumetricFormulation/tetLinearFormulation.cpp
  formulations/formulation/volumetricFormulation/cubicLinearFormulation.cpp
  formulations/formulation/volumetricFormulation/cubicTricubicHermiteFormulation.cpp
  formulations/formulation/shellFormulation/shellFormulation.cpp
  formulations/formulation/shellFormulation/koiterShellFormulation.cpp

  formulations/shapeFunction/tetLinearShapeFunction.cpp
  formulations/shapeFunction/cubicLinearShapeFunction.cpp
  formulations/shapeFunction/cubicTricubicHermiteShapeFunction.cpp

  formulations/quadrature/tetLinearDefaultQuadrature.cpp
  formulations/quadrature/gaussLegendreHexQuadrature.cpp

  deformation/volume/volumetricElementMapping.cpp
  deformation/shell/koiterShellElementMapping.cpp

  deformation/volume/volumetricDeformationModel.cpp
  deformation/volume/volumetricDeformationModelCacheData.cpp
  deformation/shell/shellDeformationModel.cpp

  formulations/dof/vertex3DofLayout.cpp
  formulations/dof/cubicTricubicHermiteDofLayout.cpp

  material/fields/constantParameterField.cpp
  material/fields/elementwiseParameterField.cpp
  material/fields/materialParameterFieldInit.cpp


  deformation/deformationModelManager.cpp

  deformation/deformationModelAssembler.cpp

  energy/deformationModelEnergy.cpp
  energy/elasticMaterialEnergy.cpp
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
