# =================== external ====================
if(MSVC)
  find_program(POWERSHELL_PATH powershell REQUIRED)
  message(STATUS "Powershell: ${POWERSHELL_PATH}")

  get_filename_component(VCPKG_TOOLCHAIN_FILE_DIR "${CMAKE_TOOLCHAIN_FILE}" DIRECTORY)
  set(APPLOCAL_FILE_PATH "${VCPKG_TOOLCHAIN_FILE_DIR}/msbuild/applocal.ps1")
  message(STATUS "applocal.ps1: ${APPLOCAL_FILE_PATH}")

  if(EXISTS ${APPLOCAL_FILE_PATH})
    set(PGO_HAS_APPLOCAL ON)
    message(STATUS "Found.")
  else()
    set(PGO_HAS_APPLOCAL OFF)
    message(STATUS "Not using vcpkg.")
  endif()
endif()

set(PGO_BINARY_ROOT ${CMAKE_CURRENT_BINARY_DIR})

macro(pgo_require_targets module_name)
  cmake_parse_arguments(PGO_REQUIRE "" "" "TARGETS" ${ARGN})

  foreach(tgt ${PGO_REQUIRE_TARGETS})
    if(NOT TARGET ${tgt})
      message(STATUS "${module_name} is not included. Missing: ${tgt}")
      return()
    endif()
  endforeach()
endmacro()

function(add_libpgo_lib tgt sources headers)
  if(PGO_CREATE_MISSING_FILE)
    foreach(src ${sources})
      if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${src})
        message(STATUS "Create file ${CMAKE_CURRENT_SOURCE_DIR}/${src}")
        file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/${src} "")
      endif()
    endforeach()

    foreach(h ${headers})
      if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${h})
        message(STATUS "Create file ${CMAKE_CURRENT_SOURCE_DIR}/${h}")
        file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/${h} "")
      endif()
    endforeach()
  endif()

  add_library(${tgt} STATIC ${sources} ${headers})

  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    set_target_properties(${tgt} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${PGO_BINARY_ROOT}/lib)
    set_target_properties(${tgt} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PGO_BINARY_ROOT}/lib)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set_target_properties(${tgt} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${PGO_BINARY_ROOT}/lib/Debug)
    set_target_properties(${tgt} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PGO_BINARY_ROOT}/lib/Release)
  else()
    message(FATAL_ERROR "Unsupported compiler")
  endif()

  target_include_directories(${tgt} PUBLIC ./)
  target_link_libraries(${tgt} PRIVATE compilation_flag)

  if(PGO_RELEASE_MODE_DEBUG)
    target_link_libraries(${tgt} PRIVATE compilation_flag_for_debug)
  endif()

  set_property(TARGET ${tgt} PROPERTY FOLDER libraries)

  # set(PGO_GLOBAL_LIBRARY_TARGETS ${PGO_GLOBAL_LIBRARY_TARGETS} ${tgt} CACHE INTERNAL "global library targets")
endfunction()
