set(PGO_GLOBAL_LIBRARY_TARGETS
  macros  
  basicAlgorithms
  profiling
  pgoLogging
  parallel
  basicIO
  eigenSupport
  configFileJSON
  mesh
  nonlinearOptimization
  genericPotentialEnergies
  geometryPotentialEnergies
  constraintPotentialEnergies
  contact
  volumetricMesh
  interpolationCoordinates
  solidDeformationModel
  simulation
  cgalInterface
  geogramInterface
  libiglInterface
  tetgenInterface
  volumetricMeshMeshing
  animationIO
  CACHE INTERNAL ""
)

if(PGO_ENABLE_CUDA)
  list(APPEND PGO_GLOBAL_LIBRARY_TARGETS cudaUtilities)
endif()

foreach(lib ${PGO_GLOBAL_LIBRARY_TARGETS})
  add_subdirectory(core/${lib})
endforeach()

add_subdirectory(core/implicitSurface)

if(PGO_UI)
  add_subdirectory(render)
endif()

if(PGO_ENABLE_PYTHON)
  add_subdirectory(python)
endif()
