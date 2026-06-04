set(NONLINEAROPTIMIZATION_HEADERS
  automaticDifferentiation_autodiff.h

  evaluation.h

  energySet.h
  potentialEnergy.h
  lineSearchAwareEnergy.h
  evaluationStateAwareEnergy.h
  stepDependentEnergy.h
  stepAwareEnergy.h
  solver/common/solveDiagnostics.h
  solver/common/solverResult.h
  solver/service/optimizationProblem.h
  solver/service/optimizationResult.h
  solver/service/optimizer.h
  solver/service/optimizerUtils.h
  solver/service/optimizationService.h
  solver/newton/newtonLineSearchPolicy.h
  solver/newton/newtonSparseSolverBackend.h
  solver/newton/NewtonOptimizer.h
  solver/ipopt/IpoptOptimizer.h
  solver/knitro/KnitroOptimizer.h

  constraints/potentialEnergyFromConstraintFunctions.h

  constraints/constraintEvaluation.h
  constraints/constraintFunction.h constraints/constraintFunctions.h constraints/constraintSet.h
  constraints/linearConstraintFunctions.h

  solver/common/nonlinearProblem.h lagrangian.h
  solver/newton/NewtonSolver.h
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
  solver/service/optimizerUtils.cpp
  solver/newton/newtonLineSearchPolicy.cpp
  solver/newton/newtonSparseSolverBackend.cpp
  solver/newton/NewtonOptimizer.cpp
  solver/ipopt/IpoptOptimizer.cpp
  solver/knitro/KnitroOptimizer.cpp

  constraints/constraintEvaluation.cpp
  constraints/constraintFunctions.cpp constraints/constraintSet.cpp
  constraints/linearConstraintFunctions.cpp

  constraints/potentialEnergyFromConstraintFunctions.cpp
  solver/common/nonlinearProblem.cpp lagrangian.cpp
  solver/newton/NewtonSolver.cpp
  finiteDifference.cpp
  determinantDerivatives.cpp polarDecompositionDerivatives.cpp naturalCubicSplineDerivatives.cpp svdDerivatives.cpp
  barrierFunction.cpp
  naturalCubicSplineFitting.cpp
  solver/newton/lineSearch.cpp
)

if(TARGET Knitro::Knitro)
  set(NONLINEAROPTIMIZATION_HEADERS ${NONLINEAROPTIMIZATION_HEADERS} solver/knitro/knitroProblem.h solver/knitro/knitroSolverWrapper.h)
  set(NONLINEAROPTIMIZATION_SOURCES ${NONLINEAROPTIMIZATION_SOURCES} solver/knitro/knitroProblem.cpp solver/knitro/knitroSolverWrapper.cpp)
endif()

if(TARGET Ipopt::Core)
  set(NONLINEAROPTIMIZATION_HEADERS ${NONLINEAROPTIMIZATION_HEADERS} solver/ipopt/IpoptProblem.h solver/ipopt/IpoptSolverWrapper.h)
  set(NONLINEAROPTIMIZATION_SOURCES ${NONLINEAROPTIMIZATION_SOURCES} solver/ipopt/IpoptProblem.cpp solver/ipopt/IpoptSolverWrapper.cpp)
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
