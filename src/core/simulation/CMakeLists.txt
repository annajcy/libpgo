set(SIMULATION_HEADERS
  dynamicState.h
  dynamicStepOptions.h
  dynamicStepper.h
  dynamicStepService.h
  common/dynamicStepperUtils.h
  common/rayleighDampingAssembly.h
  common/stageResidual.h
  backwardEuler/backwardEulerStageBuilder.h
  backwardEuler/backwardEulerStepper.h
  trbdf2/trbdf2StageBuilder.h
  trbdf2/trbdf2Stepper.h
)

set(SIMULATION_SOURCES
  dynamicStepOptions.cpp
  dynamicStepService.cpp
  common/dynamicStepperUtils.cpp
  common/rayleighDampingAssembly.cpp
  common/stageResidual.cpp
  backwardEuler/backwardEulerStageBuilder.cpp
  backwardEuler/backwardEulerStepper.cpp
  trbdf2/trbdf2StageBuilder.cpp
  trbdf2/trbdf2Stepper.cpp
)

set(SIMULATION_DEPS constraintPotentialEnergies solidDeformationModel contact genericPotentialEnergies)

foreach(tgt ${SIMULATION_DEPS})
  if(NOT TARGET ${tgt})
    message(STATUS "simulation is not included. Missing: ${tgt}")
    return()
  endif()
endforeach()

add_libpgo_lib(simulation "${SIMULATION_SOURCES}" "${SIMULATION_HEADERS}")

target_link_libraries(simulation PUBLIC ${SIMULATION_DEPS})
