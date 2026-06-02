set(NONLINEAROPTIMIZATION_HEADERS
  automaticDifferentiation_autodiff.h

  evaluation.h

  energySet.h
  potentialEnergy.h
  lineSearchAwareEnergy.h
  solver/common/solveDiagnostics.h
  solver/common/solverResult.h
  solver/service/optimizationService.h
  solver/service/optimizationBackend.h
  solver/newton/newtonLineSearchPolicy.h
  solver/newton/newtonSparseSolverBackend.h

  potentialEnergyFromConstraintFunctions.h

  constraintFunction.h constraintFunctions.h constraintFunctionsAssember.h
  linearConstraintFunctions.h

  solver/external/common/nonlinearProblem.h lagrangian.h
  solver/newton/NewtonSolver.h solver/legacy/minimizeEnergy.h
  finiteDifference.h
  determinantDerivatives.h polarDecompositionDerivatives.h naturalCubicSplineDerivatives.h svdDerivatives.h
  barrierFunction.h
  naturalCubicSplineFitting.h
  solver/newton/lineSearch.h
)

set(NONLINEAROPTIMIZATION_SOURCES
  evaluation.cpp
  energySet.cpp
  solver/common/solverResult.cpp
  solver/service/optimizationService.cpp
  solver/service/optimizationBackend.cpp
  solver/newton/newtonLineSearchPolicy.cpp
  solver/newton/newtonSparseSolverBackend.cpp

  constraintFunctions.cpp constraintFunctionsAssember.cpp
  linearConstraintFunctions.cpp

  potentialEnergyFromConstraintFunctions.cpp
  solver/external/common/nonlinearProblem.cpp lagrangian.cpp
  solver/newton/NewtonSolver.cpp solver/legacy/minimizeEnergy.cpp
  finiteDifference.cpp
  determinantDerivatives.cpp polarDecompositionDerivatives.cpp naturalCubicSplineDerivatives.cpp svdDerivatives.cpp
  barrierFunction.cpp
  naturalCubicSplineFitting.cpp
  solver/newton/lineSearch.cpp
)

if(TARGET Knitro::Knitro)
  set(NONLINEAROPTIMIZATION_HEADERS ${NONLINEAROPTIMIZATION_HEADERS} solver/external/knitro/knitroProblem.h solver/external/knitro/knitroOptimizer.h)
  set(NONLINEAROPTIMIZATION_SOURCES ${NONLINEAROPTIMIZATION_SOURCES} solver/external/knitro/knitroProblem.cpp solver/external/knitro/knitroOptimizer.cpp)
endif()

if(TARGET Ipopt::Core)
  set(NONLINEAROPTIMIZATION_HEADERS ${NONLINEAROPTIMIZATION_HEADERS} solver/external/ipopt/IpoptProblem.h solver/external/ipopt/IpoptOptimizer.h)
  set(NONLINEAROPTIMIZATION_SOURCES ${NONLINEAROPTIMIZATION_SOURCES} solver/external/ipopt/IpoptProblem.cpp solver/external/ipopt/IpoptOptimizer.cpp)
endif()

set(NONLINEAROPTIMIZATION_DEPS eigenSupport pgoLogging autodiff::autodiff profiling)

foreach(tgt ${NONLINEAROPTIMIZATION_DEPS})
  if(NOT TARGET ${tgt})
    message(STATUS "nonlinearOptimization is not included. Missing: ${tgt}")
    return()
  endif()
endforeach()

add_libpgo_lib(nonlinearOptimization "${NONLINEAROPTIMIZATION_SOURCES}" "${NONLINEAROPTIMIZATION_HEADERS}")

foreach(lib ${NONLINEAROPTIMIZATION_DEPS})
  target_link_libraries(nonlinearOptimization PUBLIC ${lib})
endforeach()

if(TARGET Knitro::Knitro)
  target_link_libraries(nonlinearOptimization PRIVATE Knitro::Knitro)
endif()

if(TARGET Ipopt::Core)
  target_link_libraries(nonlinearOptimization PRIVATE Ipopt::Core)
endif()
