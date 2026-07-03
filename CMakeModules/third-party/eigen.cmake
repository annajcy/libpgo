if(NOT TARGET Eigen3::Eigen)
  if(PGO_CHECK_CONDA AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
    if(WIN32)
      string(REPLACE "\\" "/" _pgo_eigen_prefix "$ENV{CONDA_PREFIX}/Library")
    else()
      set(_pgo_eigen_prefix "$ENV{CONDA_PREFIX}")
    endif()

    if(DEFINED Eigen3_DIR)
      string(FIND "${Eigen3_DIR}" "${_pgo_eigen_prefix}" _pgo_eigen_cached_prefix)
      if(NOT _pgo_eigen_cached_prefix EQUAL 0)
        unset(Eigen3_DIR CACHE)
      endif()
    endif()

    find_package(Eigen3 CONFIG REQUIRED
      PATHS "${_pgo_eigen_prefix}"
      NO_DEFAULT_PATH)
  else()
    find_package(Eigen3 CONFIG REQUIRED)
  endif()
endif()

get_property(aliased_target TARGET Eigen3::Eigen PROPERTY ALIASED_TARGET)
if("${aliased_target}" STREQUAL "")
  set(REAL_TGT Eigen3::Eigen)
else()
  set(REAL_TGT ${aliased_target})
endif()

if(TARGET MKL::MKL)
  target_link_libraries(${REAL_TGT} INTERFACE MKL::MKL)
  target_compile_definitions(${REAL_TGT} INTERFACE EIGEN_DONT_PARALLELIZE)
  target_compile_definitions(${REAL_TGT} INTERFACE EIGEN_USE_MKL_ALL)
  target_compile_definitions(${REAL_TGT} INTERFACE EIGEN_MKL_NO_DIRECT_CALL)
endif()

target_compile_definitions(${REAL_TGT} INTERFACE EIGEN_MAX_ALIGN_BYTES=32)
