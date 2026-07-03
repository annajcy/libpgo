if(TARGET openvdb_static OR TARGET OpenVDB::openvdb)
  return()
endif()

message(STATUS "Loading OpenVDB...")

if(PGO_CHECK_CONDA)
  set(CANDIDATE_PREFIX "")
  foreach(_PGO_CONDA_ROOT "$ENV{PREFIX}" "$ENV{CONDA_PREFIX}")
    if("${_PGO_CONDA_ROOT}" STREQUAL "")
      continue()
    endif()

    if(WIN32)
      set(_PGO_CONDA_PREFIX "${_PGO_CONDA_ROOT}/Library")
    else()
      set(_PGO_CONDA_PREFIX "${_PGO_CONDA_ROOT}")
    endif()

    if(EXISTS "${_PGO_CONDA_PREFIX}")
      set(CANDIDATE_PREFIX "${_PGO_CONDA_PREFIX}")
      break()
    endif()
  endforeach()

  if(EXISTS "${CANDIDATE_PREFIX}")
    message(STATUS "OpenVDB conda prefix: ${CANDIDATE_PREFIX}")
    list(PREPEND CMAKE_PREFIX_PATH "${CANDIDATE_PREFIX}")

    # OpenVDB's conda package is built against conda Boost. Keep its dependency
    # checks inside the same conda prefix.
    set(BOOST_ROOT "${CANDIDATE_PREFIX}" CACHE PATH "Boost prefix for OpenVDB" FORCE)
    set(BOOST_INCLUDEDIR "${CANDIDATE_PREFIX}/include" CACHE PATH "Boost include directory for OpenVDB" FORCE)
    set(Boost_INCLUDE_DIR "${CANDIDATE_PREFIX}/include" CACHE PATH "Boost include directory for OpenVDB" FORCE)
    set(Boost_INCLUDE_DIRS "${CANDIDATE_PREFIX}/include" CACHE STRING "Boost include directories for OpenVDB" FORCE)
    set(Boost_USE_STATIC_LIBS OFF)
    set(Boost_USE_STATIC_LIBS OFF CACHE BOOL "Use conda static Boost libraries for OpenVDB" FORCE)
    set(OPENVDB_USE_STATIC_LIBS OFF)
    set(OPENVDB_USE_STATIC_LIBS OFF CACHE BOOL "Use conda static OpenVDB libraries" FORCE)
    set(Boost_NO_SYSTEM_PATHS ON CACHE BOOL "Restrict OpenVDB Boost lookup to conda" FORCE)
    set(Boost_NO_BOOST_CMAKE OFF CACHE BOOL "Allow OpenVDB to find conda Boost config" FORCE)
    set(BOOST_LIBRARYDIR "${CANDIDATE_PREFIX}/lib" CACHE PATH "Boost library directory for OpenVDB" FORCE)

    foreach(CANDIDATE_CONFIG_DIR
        "${CANDIDATE_PREFIX}/lib/cmake/OpenVDB"
        "${CANDIDATE_PREFIX}/lib/cmake/openvdb"
        "${CANDIDATE_PREFIX}/share/cmake/OpenVDB"
        "${CANDIDATE_PREFIX}/share/cmake/openvdb")
      if(EXISTS "${CANDIDATE_CONFIG_DIR}/OpenVDBConfig.cmake"
          OR EXISTS "${CANDIDATE_CONFIG_DIR}/openvdb-config.cmake")
        set(OpenVDB_DIR "${CANDIDATE_CONFIG_DIR}" CACHE PATH "OpenVDB CMake package directory" FORCE)
        break()
      elseif(EXISTS "${CANDIDATE_CONFIG_DIR}/FindOpenVDB.cmake")
        list(PREPEND CMAKE_MODULE_PATH "${CANDIDATE_CONFIG_DIR}")
        break()
      endif()
    endforeach()
  endif()
endif()

find_package(OpenVDB REQUIRED)

message(STATUS "Done.")
