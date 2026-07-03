set(GENERIC_POTENTIAL_ENERGIES_HEADERS
  quadraticPotentialEnergy.h linearPotentialEnergy.h
  laplacianProblem.h
)

set(GENERIC_POTENTIAL_ENERGIES_SOURCES
  quadraticPotentialEnergy.cpp linearPotentialEnergy.cpp
  laplacianProblem.cpp
)

set(GENERIC_POTENTIAL_ENERGIES_DEPS nonlinearOptimization mesh)

pgo_require_targets(genericPotentialEnergies TARGETS ${GENERIC_POTENTIAL_ENERGIES_DEPS})

add_libpgo_lib(genericPotentialEnergies "${GENERIC_POTENTIAL_ENERGIES_SOURCES}" "${GENERIC_POTENTIAL_ENERGIES_HEADERS}")

target_link_libraries(genericPotentialEnergies PUBLIC ${GENERIC_POTENTIAL_ENERGIES_DEPS})
