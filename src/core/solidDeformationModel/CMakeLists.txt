set(SOLID_DEFORMATION_MODEL_HEADERS
  material/elastic/elasticModel.h
  material/elastic/elasticModel3DDeformationGradient.h
  material/elastic/deformationGradient/spectral/elasticModel3DIsotropicPrincipalStretch.h
  material/elastic/deformationGradient/spectral/elasticModel3DValanisLandel.h
  material/elastic/deformationGradient/spectral/elasticModel3DSystematicPoking.h
  material/elastic/deformationGradient/spectral/isotropicSpectralTangent.h
  material/elastic/elasticModelStableNeoHookeanMaterial.h
  material/elastic/elasticModel3DNeoHookean.h
  material/elastic/deformationGradient/spectral/elasticModelStableNeoHookeanPrincipalStretch.h
  material/elastic/elasticModelInvariantBasedMaterial.h
  material/elastic/elasticModelLinearMaterial.h
  material/elastic/elasticModelHillTypeMaterial.h
  material/elastic/elasticModelVolumeMaterial.h
  material/elastic/elasticModelCombinedMaterial.h
  material/elastic/elasticModel3DMooneyRivlin.h
  material/elastic/elasticModel3DSTVKMaterial.h
  material/elastic/elasticModel1D.h
  material/elastic/elasticModel1DZero.h
  material/elastic/elasticModel1DFixedParameters.h
  material/elastic/elasticModel1DScaled.h
  material/elastic/elasticModel1DLogSquared.h
  material/elastic/elasticModel1DIntegratedLinearCurvatureSpline.h
  material/elastic/elasticModel1DQuadratic.h
  material/elastic/elasticModel1DCubicSpline.h
  material/elastic/spline/integratedLinearCurvatureSpline.h

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

  deformation/deformationModel.h
  deformation/deformationModelEvaluator.h
  deformation/hessianProjection.h
  deformation/materialMaxStepPolynomialUtils.h
  deformation/deformationModelManager.h

  deformation/deformationModelAssembler.h
  deformation/deformationModelAssemblerElementWorkspace.h
  deformation/hessianMatrixHandle.h

  energy/deformationEnergyOperator.h
  energy/deformationPotentialEnergy.h

  simulation/simulationMesh.h
  simulation/simulationMeshBase.h
  simulation/simulationMeshVolume.h
  simulation/simulationMeshTriangle.h
  simulation/simulationMeshEdgeQuad.h
  simulation/simulationMeshShell.h
  simulation/import/simulationImportResult.h
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
  formulations/quadrature/tetDegree2Quadrature.h

  deformation/volume/volumetricElementMapping.h
  deformation/shell/shellElementMapping.h
  deformation/shell/koiterShellElementMapping.h

  deformation/volume/volumetricDeformationModel.h
  deformation/volume/volumetricDeformationModelEvaluator.h
  deformation/volume/volumetricDeformationModelCacheData.h
  deformation/shell/shellDeformationModel.h
  deformation/shell/shellDeformationModelEvaluator.h
  deformation/shell/shellDeformationModelCacheData.h

  formulations/dof/dofLayout.h
  formulations/dof/perVertexDofLayout.h
  formulations/dof/vertex3DofLayout.h
  formulations/dof/cubicTricubicHermiteDofLayout.h

  material/parameterization/parameterLayout.h
  material/model/materialSchema.h
  material/model/elasticModelDefinition.h
  material/model/plasticModelDefinition.h
  material/parameterization/materialChannelMapping.h
  material/parameterization/materialParameterField.h
  material/runtime/materialBinding.h
  material/runtime/materialState.h
  material/runtime/optimizableParameterRef.h
  material/frame/materialFrameField.h
  material/data/materialParameterData.h
  material/data/namedMaterialInputData.h
  material/parameterization/materialParameterization.h
  material/projection/materialInputProjection.h
  material/projection/importedMaterialFrames.h
  material/import/importedMaterialCatalog.h
  material/import/volumeMaterialImporter.h

  mass/elementScalarField.h
  mass/volumeDensityField.h
  mass/shellArealDensityField.h
)

set(SOLID_DEFORMATION_MODEL_SOURCES
  material/elastic/deformationGradient/spectral/elasticModel3DIsotropicPrincipalStretch.cpp
  material/elastic/deformationGradient/spectral/elasticModel3DValanisLandel.cpp
  material/elastic/deformationGradient/spectral/elasticModel3DSystematicPoking.cpp
  material/elastic/deformationGradient/spectral/isotropicSpectralTangent.cpp
  material/elastic/elasticModelInvariantBasedMaterial.cpp
  material/elastic/elasticModelStableNeoHookeanMaterial.cpp
  material/elastic/elasticModel3DNeoHookean.cpp
  material/elastic/deformationGradient/spectral/elasticModelStableNeoHookeanPrincipalStretch.cpp
  material/elastic/elasticModelLinearMaterial.cpp
  material/elastic/elasticModelHillTypeMaterial.cpp
  material/elastic/elasticModelVolumeMaterial.cpp
  material/elastic/elasticModel3DMooneyRivlin.cpp
  material/elastic/elasticModel3DSTVKMaterial.cpp
  material/elastic/elasticModelCombinedMaterial.cpp
  material/elastic/elasticModel1DZero.cpp
  material/elastic/elasticModel1DFixedParameters.cpp
  material/elastic/elasticModel1DScaled.cpp
  material/elastic/elasticModel1DLogSquared.cpp
  material/elastic/elasticModel1DIntegratedLinearCurvatureSpline.cpp
  material/elastic/elasticModel1DQuadratic.cpp
  material/elastic/elasticModel1DCubicSpline.cpp
  material/elastic/spline/integratedLinearCurvatureSpline.cpp
  material/elastic/elasticModel2DFundamentalFormsFabric.cpp
  material/elastic/elasticModel2DFundamentalFormsSTVK.cpp

  material/elastic/invariantBasedMaterialStVK.cpp

  material/plastic/plasticModel3DConstant.cpp
  material/plastic/plasticModel3D3DOF.cpp
  material/plastic/plasticModel3D6DOF.cpp

  material/plastic/plasticModel2DFundamentalFormsUniformStretch.cpp

  material/model/materialSchema.cpp

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

  formulations/dof/dofLayout.cpp

  formulations/quadrature/tetLinearDefaultQuadrature.cpp
  formulations/quadrature/gaussLegendreHexQuadrature.cpp
  formulations/quadrature/tetDegree2Quadrature.cpp

  deformation/volume/volumetricElementMapping.cpp
  deformation/shell/koiterShellElementMapping.cpp

  deformation/volume/volumetricDeformationModel.cpp
  deformation/volume/volumetricDeformationModelEvaluator.cpp
  deformation/volume/volumetricDeformationModelCacheData.cpp
  deformation/shell/shellDeformationModel.cpp
  deformation/shell/shellDeformationModelEvaluator.cpp
  deformation/shell/shellDeformationModelCacheData.cpp

  material/parameterization/parameterLayout.cpp
  material/parameterization/materialChannelMapping.cpp
  material/parameterization/materialParameterField.cpp
  material/parameterization/materialParameterization.cpp
  material/runtime/materialState.cpp
  material/runtime/optimizableParameterRef.cpp
  material/frame/materialFrameField.cpp
  material/projection/materialInputProjection.cpp
  material/projection/importedMaterialFrames.cpp
  material/import/importedMaterialCatalog.cpp
  material/import/volumeMaterialImporter.cpp
  material/data/namedMaterialInputData.cpp


  deformation/deformationModelManager.cpp

  deformation/deformationModelAssembler.cpp
  deformation/deformationModelAssemblerElementWorkspace.cpp

  energy/deformationEnergyOperator.cpp
  energy/deformationPotentialEnergy.cpp

  material/plastic/plasticModel2DFundamentalForms.cpp

  mass/elementScalarField.cpp
  mass/volumeDensityField.cpp
  mass/shellArealDensityField.cpp

  simulation/simulationMeshCore.cpp
  simulation/import/simulationImportResult.cpp
  simulation/simulationMeshVolume.cpp
  simulation/simulationMeshTriangle.cpp
  simulation/simulationMeshShell.cpp
  constraints/tetVolumeConstraintFunctions.cpp
  simulation/tetMeshOccupation.cpp

  constraints/segmentChainConstraintFunctions.cpp
  constraints/segmentBinormalConstraintFunctions.cpp

  constraints/prescribedPrincipleStressConstraintFunctions.cpp

  simulation/generateTetMeshMatrix.cpp
)

set(SOLID_DEFORMATION_MODEL_DEPS
  pgoLogging basicAlgorithms eigenSupport profiling nonlinearOptimization configFileJSON volumetricMesh interpolationCoordinates
  Eigen3::Eigen TBB::tbb fmt::fmt-header-only
)

pgo_require_targets(solidDeformationModel TARGETS ${SOLID_DEFORMATION_MODEL_DEPS})

add_libpgo_lib(solidDeformationModel "${SOLID_DEFORMATION_MODEL_SOURCES}" "${SOLID_DEFORMATION_MODEL_HEADERS}")

target_link_libraries(solidDeformationModel PUBLIC ${SOLID_DEFORMATION_MODEL_DEPS})
