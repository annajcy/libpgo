set(SIMULATION_HEADERS
  timeIntegrator.h
  timeIntegratorSolver.h
  dynamicState.h
  dynamicStepOptions.h
  rayleighDampingAssembly.h
  stageResidual.h
  implicitEulerStageBuilder.h
  trbdf2StageBuilder.h
  dynamicStepper.h
  dynamicStepService.h
)

set(SIMULATION_SOURCES
  timeIntegrator.cpp
  timeIntegratorSolver.cpp
  dynamicStepOptions.cpp
  rayleighDampingAssembly.cpp
  stageResidual.cpp
  implicitEulerStageBuilder.cpp
  trbdf2StageBuilder.cpp
  dynamicStepper.cpp
  dynamicStepService.cpp
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
