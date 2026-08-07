if(TARGET Eigen3::Eigen)
else()
  message(STATUS "Loading eigen...")

  set(BUILD_TESTING OFF CACHE BOOL "eigen build test" FORCE)
  set(BUILD_EXAMPLES OFF CACHE BOOL "eigen build examples" FORCE)
  set(EIGEN_BUILD_DOC OFF CACHE BOOL "eigen build documentation" FORCE)
  set(EIGEN_BUILD_CMAKE_PACKAGE ON CACHE BOOL "eigen build cmake package" FORCE)

  include(FetchContent)
  FetchContent_Declare(
    Eigen3
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
    OVERRIDE_FIND_PACKAGE
    EXCLUDE_FROM_ALL
    DOWNLOAD_EXTRACT_TIMESTAMP ON
  )

  # Eigen 3.4 configures its optional BLAS target even when BUILD_TESTING is
  # disabled. On Windows, CheckLanguage may pick up an unrelated MinGW
  # gfortran from PATH and then try to combine it with the active MSVC
  # toolchain. libpgo does not use Eigen's BLAS target, so prevent only that
  # optional probe while preserving an explicitly configured Fortran compiler.
  if(WIN32 AND NOT DEFINED CMAKE_Fortran_COMPILER)
    set(CMAKE_Fortran_COMPILER NOTFOUND)
    set(_pgo_suppressed_eigen_fortran_probe TRUE)
  endif()

  FetchContent_MakeAvailable(Eigen3)

  if(_pgo_suppressed_eigen_fortran_probe)
    unset(CMAKE_Fortran_COMPILER)
    unset(_pgo_suppressed_eigen_fortran_probe)
  endif()

  message(STATUS "Done.")
endif()

get_property(aliased_target TARGET Eigen3::Eigen PROPERTY ALIASED_TARGET)
if(aliased_target)
  set(REAL_TGT "${aliased_target}")
else()
  set(REAL_TGT "Eigen3::Eigen")
endif()

if(TARGET MKL::MKL)
  target_link_libraries(compilation_flag INTERFACE MKL::MKL)
  target_compile_definitions(compilation_flag INTERFACE
    EIGEN_USE_MKL_ALL
    EIGEN_MKL_NO_DIRECT_CALL)
endif()

if(APPLE)
  if(TARGET PGO::AccelerateBLAS)
    target_link_libraries(compilation_flag INTERFACE PGO::AccelerateBLAS)
  endif()
  target_compile_definitions(compilation_flag INTERFACE EIGEN_USE_BLAS)
endif()

target_compile_definitions(${REAL_TGT} INTERFACE EIGEN_MAX_ALIGN_BYTES=32)
