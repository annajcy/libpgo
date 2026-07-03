include_guard(GLOBAL)

include(FetchContent)

function(pgo_dep_option name type value doc)
  set(${name} ${value} CACHE ${type} "${doc}" FORCE)
endfunction()

function(pgo_add_third_party name)
  cmake_parse_arguments(PGO_THIRD_PARTY
    "ADD_POPULATED_SUBDIRECTORY"
    "STATUS;DONE_MESSAGE;FETCH_MODE;POPULATE_REASON;POST_FETCH"
    "TARGETS;FETCHCONTENT_ARGS"
    ${ARGN})

  foreach(_pgo_third_party_target IN LISTS PGO_THIRD_PARTY_TARGETS)
    if(TARGET ${_pgo_third_party_target})
      return()
    endif()
  endforeach()

  if(PGO_THIRD_PARTY_STATUS)
    message(STATUS "${PGO_THIRD_PARTY_STATUS}")
  endif()

  FetchContent_Declare(${name} ${PGO_THIRD_PARTY_FETCHCONTENT_ARGS})
  if(NOT PGO_THIRD_PARTY_FETCH_MODE)
    set(PGO_THIRD_PARTY_FETCH_MODE MAKE_AVAILABLE)
  endif()

  if(PGO_THIRD_PARTY_FETCH_MODE STREQUAL "MAKE_AVAILABLE")
    pgo_fetch_make_available(${name})
  elseif(PGO_THIRD_PARTY_FETCH_MODE STREQUAL "POPULATE")
    if(NOT PGO_THIRD_PARTY_POPULATE_REASON)
      set(PGO_THIRD_PARTY_POPULATE_REASON "${name} source tree is prepared before use")
    endif()
    pgo_fetch_populate_compat(${name} "${PGO_THIRD_PARTY_POPULATE_REASON}")
  else()
    message(FATAL_ERROR "Unsupported pgo_add_third_party FETCH_MODE: ${PGO_THIRD_PARTY_FETCH_MODE}")
  endif()

  if(PGO_THIRD_PARTY_POST_FETCH)
    cmake_language(CALL ${PGO_THIRD_PARTY_POST_FETCH})
  endif()

  if(PGO_THIRD_PARTY_ADD_POPULATED_SUBDIRECTORY)
    pgo_add_populated_subdirectory(${name})
  endif()

  if(PGO_THIRD_PARTY_DONE_MESSAGE)
    message(STATUS "${PGO_THIRD_PARTY_DONE_MESSAGE}")
  else()
    message(STATUS "Done.")
  endif()
endfunction()

macro(pgo_reuse_existing_fetchcontent_source name)
  string(TOLOWER "${name}" _pgo_dep_name_lower)
  string(TOUPPER "${name}" _pgo_dep_name_upper)
  set(_pgo_dep_source_var "FETCHCONTENT_SOURCE_DIR_${_pgo_dep_name_upper}")
  set(_pgo_dep_existing_source "${FETCHCONTENT_BASE_DIR}/${_pgo_dep_name_lower}-src")
  if("${${_pgo_dep_source_var}}" STREQUAL ""
      AND EXISTS "${_pgo_dep_existing_source}/CMakeLists.txt")
    # ponytail: reuse an already-populated source tree; remove if CMake stops
    # redownloading URL deps when only the download stamp is stale.
    set(${_pgo_dep_source_var} "${_pgo_dep_existing_source}")
    set(${_pgo_dep_source_var} "${_pgo_dep_existing_source}" CACHE PATH "Use existing ${name} FetchContent source tree" FORCE)
  endif()
endmacro()

macro(pgo_fetch_make_available name)
  pgo_reuse_existing_fetchcontent_source(${name})
  FetchContent_MakeAvailable(${name})
endmacro()

macro(pgo_fetch_populate_compat name reason)
  message(STATUS "Using FetchContent_Populate(${name}): ${reason}")
  pgo_reuse_existing_fetchcontent_source(${name})
  FetchContent_GetProperties(${name})

  if(NOT ${name}_POPULATED)
    cmake_policy(PUSH)
    if(POLICY CMP0169)
      cmake_policy(SET CMP0169 OLD)
    endif()
    FetchContent_Populate(${name})
    cmake_policy(POP)
  endif()
endmacro()

macro(pgo_add_populated_subdirectory name)
  if(NOT DEFINED ${name}_SOURCE_DIR OR NOT DEFINED ${name}_BINARY_DIR)
    message(FATAL_ERROR "FetchContent source/binary variables are not defined for ${name}.")
  endif()

  add_subdirectory("${${name}_SOURCE_DIR}" "${${name}_BINARY_DIR}" EXCLUDE_FROM_ALL)
endmacro()

function(pgo_apply_patch source_dir patch_file reason)
  if(NOT EXISTS "${patch_file}")
    message(FATAL_ERROR "Patch file does not exist: ${patch_file}")
  endif()

  find_package(Git QUIET)
  if(NOT GIT_FOUND)
    message(FATAL_ERROR "Git is required to apply patch: ${patch_file}")
  endif()

  get_filename_component(_pgo_patch_name "${patch_file}" NAME)
  get_filename_component(_pgo_patch_ceiling "${source_dir}" DIRECTORY)
  set(_pgo_patch_git
    "${CMAKE_COMMAND}" -E env "GIT_CEILING_DIRECTORIES=${_pgo_patch_ceiling}"
    "${GIT_EXECUTABLE}" -C "${source_dir}")

  execute_process(
    COMMAND ${_pgo_patch_git} apply --check "${patch_file}"
    RESULT_VARIABLE _pgo_patch_check_result
    OUTPUT_VARIABLE _pgo_patch_check_output
    ERROR_VARIABLE _pgo_patch_check_error
  )

  if(_pgo_patch_check_result EQUAL 0)
    message(STATUS "Applying patch ${_pgo_patch_name}: ${reason}")
    execute_process(
      COMMAND ${_pgo_patch_git} apply "${patch_file}"
      RESULT_VARIABLE _pgo_patch_apply_result
      OUTPUT_VARIABLE _pgo_patch_apply_output
      ERROR_VARIABLE _pgo_patch_apply_error
    )
    if(NOT _pgo_patch_apply_result EQUAL 0)
      message(FATAL_ERROR "Failed to apply ${patch_file}:\n${_pgo_patch_apply_error}")
    endif()
    return()
  endif()

  execute_process(
    COMMAND ${_pgo_patch_git} apply --reverse --check "${patch_file}"
    RESULT_VARIABLE _pgo_patch_reverse_check_result
    OUTPUT_VARIABLE _pgo_patch_reverse_check_output
    ERROR_VARIABLE _pgo_patch_reverse_check_error
  )

  if(_pgo_patch_reverse_check_result EQUAL 0)
    message(STATUS "Patch already applied ${_pgo_patch_name}: ${reason}")
  else()
    message(FATAL_ERROR
      "Patch ${patch_file} does not apply cleanly to ${source_dir}.\n"
      "Forward check:\n${_pgo_patch_check_error}\n"
      "Reverse check:\n${_pgo_patch_reverse_check_error}")
  endif()
endfunction()

function(pgo_copy_file source_file target_file)
  if(NOT EXISTS "${source_file}")
    message(FATAL_ERROR "Source file does not exist: ${source_file}")
  endif()

  file(COPY_FILE "${source_file}" "${target_file}" ONLY_IF_DIFFERENT)
endfunction()

function(pgo_replace_in_file target_file old_text new_text)
  file(READ "${target_file}" _pgo_file_contents)
  string(FIND "${_pgo_file_contents}" "${new_text}" _pgo_already_patched_index)
  if(NOT _pgo_already_patched_index EQUAL -1)
    return()
  endif()

  string(FIND "${_pgo_file_contents}" "${old_text}" _pgo_match_index)
  if(_pgo_match_index EQUAL -1)
    message(FATAL_ERROR "Failed to patch ${target_file}: expected snippet was not found.")
  endif()

  string(REPLACE "${old_text}" "${new_text}" _pgo_file_contents "${_pgo_file_contents}")
  file(WRITE "${target_file}" "${_pgo_file_contents}")
endfunction()
