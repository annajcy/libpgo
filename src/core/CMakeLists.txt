include(${CMAKE_CURRENT_SOURCE_DIR}/utils/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/scene/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/energy/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/solve/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/external/CMakeLists.txt)

# Keep target order unchanged to preserve dependency availability checks in child CMake files.
set(PGO_GLOBAL_LIBRARY_TARGETS
  macros
  basicAlgorithms
  fileService
  pgoLogging
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
  animationIO
  CACHE INTERNAL ""
)

foreach(lib ${PGO_GLOBAL_LIBRARY_TARGETS})
  if(lib IN_LIST PGO_UTILS_TARGETS)
    add_subdirectory(utils/${lib})
  elseif(lib IN_LIST PGO_SCENE_TARGETS)
    add_subdirectory(scene/${lib})
  elseif(lib IN_LIST PGO_ENERGY_TARGETS)
    add_subdirectory(energy/${lib})
  elseif(lib IN_LIST PGO_SOLVE_TARGETS)
    add_subdirectory(solve/${lib})
  elseif(lib IN_LIST PGO_EXTERNAL_TARGETS)
    add_subdirectory(external/${lib})
  else()
    message(FATAL_ERROR "Unknown core library target: ${lib}")
  endif()
endforeach()
