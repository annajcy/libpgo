function(_pgo_setup_cgal)
  set(CGAL_SOURCE_DIR "${CMAKE_BINARY_DIR}/_deps/cgal-src")
  set(CGAL_BINARY_DIR "${CMAKE_BINARY_DIR}/_deps/cgal-build")

  set(MODIFIED_FILE "${CMAKE_SOURCE_DIR}/CMakeModules/patches/CGAL_SetupBoost.cmake")
  set(CGAL_SETUP_BOOST_FILE "${CGAL_SOURCE_DIR}/cmake/modules/CGAL_SetupBoost.cmake")
  if(NOT EXISTS "${CGAL_SETUP_BOOST_FILE}")
    message(FATAL_ERROR "CGAL setup file not found: ${CGAL_SETUP_BOOST_FILE}")
  endif()

  pgo_copy_file("${MODIFIED_FILE}" "${CGAL_SETUP_BOOST_FILE}")

  # gmp for windows
  if(WIN32)
    if(PGO_CHECK_CONDA AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
      set(TGT_FILE "$ENV{CONDA_PREFIX}/Library/bin/gmp-10.dll")

      if(NOT EXISTS ${TGT_FILE})
        file(COPY_FILE "${CMAKE_SOURCE_DIR}/third-party/gmp-msvc/release/gmp-10.dll" "${TGT_FILE}")
      endif()

      set(TGT_FILE "$ENV{CONDA_PREFIX}/Library/bin/gmpxx-4.dll")

      if(NOT EXISTS ${TGT_FILE})
        file(COPY_FILE "${CMAKE_SOURCE_DIR}/third-party/gmp-msvc/release/gmpxx-4.dll" "${TGT_FILE}")
      endif()

      set(TGT_FILE "$ENV{CONDA_PREFIX}/Library/bin/mpfr-6.dll")

      if(NOT EXISTS ${TGT_FILE})
        file(COPY_FILE "${CMAKE_SOURCE_DIR}/third-party/mpfr-msvc/release/mpfr-6.dll" "${TGT_FILE}")
      endif()
    endif()
  endif()

  list(APPEND CMAKE_MODULE_PATH "${CGAL_SOURCE_DIR}/cmake/modules")

  # CGAL is header-only here but hard-depends on GMP/MPFR (+GMPXX). Conda builds
  # must never leak Homebrew or system library install names into the extension.
  if(PGO_CHECK_CONDA)
    if(WIN32)
      set(GMP_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/third-party/gmp-msvc" CACHE PATH "MSVC GMP include directory" FORCE)
      set(MPFR_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/third-party/mpfr-msvc" CACHE PATH "MSVC MPFR include directory" FORCE)
      set(GMPXX_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/third-party/gmp-msvc" CACHE PATH "MSVC GMPXX include directory" FORCE)
      set(GMP_LIBRARIES "${CMAKE_SOURCE_DIR}/third-party/gmp-msvc/release/gmp.lib" CACHE FILEPATH "MSVC GMP library" FORCE)
      set(GMP_LIBRARY_RELEASE "${GMP_LIBRARIES}" CACHE FILEPATH "MSVC GMP release library" FORCE)
      set(GMP_LIBRARY_DEBUG "${GMP_LIBRARIES}" CACHE FILEPATH "MSVC GMP debug library" FORCE)
      set(MPFR_LIBRARIES "${CMAKE_SOURCE_DIR}/third-party/mpfr-msvc/release/mpfr.lib" CACHE FILEPATH "MSVC MPFR library" FORCE)
      set(GMPXX_LIBRARIES "${CMAKE_SOURCE_DIR}/third-party/gmp-msvc/release/gmpxx.lib" CACHE FILEPATH "MSVC GMPXX library" FORCE)
    else()
      set(_PGO_CGAL_CONDA_PREFIX "")
      foreach(_PGO_CGAL_PREFIX_CANDIDATE IN ITEMS "$ENV{PREFIX}" "$ENV{CONDA_PREFIX}")
        if(NOT "${_PGO_CGAL_PREFIX_CANDIDATE}" STREQUAL "" AND EXISTS "${_PGO_CGAL_PREFIX_CANDIDATE}/include")
          set(_PGO_CGAL_CONDA_PREFIX "${_PGO_CGAL_PREFIX_CANDIDATE}")
          break()
        endif()
      endforeach()

      if(_PGO_CGAL_CONDA_PREFIX)
        list(PREPEND CMAKE_PREFIX_PATH "${_PGO_CGAL_CONDA_PREFIX}")
        set(GMP_INCLUDE_DIR "${_PGO_CGAL_CONDA_PREFIX}/include" CACHE PATH "Conda GMP include directory" FORCE)
        set(MPFR_INCLUDE_DIR "${_PGO_CGAL_CONDA_PREFIX}/include" CACHE PATH "Conda MPFR include directory" FORCE)
        set(GMPXX_INCLUDE_DIR "${_PGO_CGAL_CONDA_PREFIX}/include" CACHE PATH "Conda GMPXX include directory" FORCE)

        find_library(GMP_LIBRARIES NAMES gmp libgmp-10 gmp-10
          PATHS "${_PGO_CGAL_CONDA_PREFIX}/lib" NO_DEFAULT_PATH)
        find_library(MPFR_LIBRARIES NAMES mpfr libmpfr-6 mpfr-6
          PATHS "${_PGO_CGAL_CONDA_PREFIX}/lib" NO_DEFAULT_PATH)
        find_library(GMPXX_LIBRARIES NAMES gmpxx libgmpxx-4 gmpxx-4
          PATHS "${_PGO_CGAL_CONDA_PREFIX}/lib" NO_DEFAULT_PATH)

        set(GMP_LIBRARIES "${GMP_LIBRARIES}" CACHE FILEPATH "Conda GMP library" FORCE)
        set(GMP_LIBRARY_RELEASE "${GMP_LIBRARIES}" CACHE FILEPATH "Conda GMP release library" FORCE)
        set(GMP_LIBRARY_DEBUG "${GMP_LIBRARIES}" CACHE FILEPATH "Conda GMP debug library" FORCE)
        set(MPFR_LIBRARIES "${MPFR_LIBRARIES}" CACHE FILEPATH "Conda MPFR library" FORCE)
        set(GMPXX_LIBRARIES "${GMPXX_LIBRARIES}" CACHE FILEPATH "Conda GMPXX library" FORCE)
      endif()
    endif()
  endif()

  find_package(GMP QUIET)
  find_package(MPFR QUIET)
  find_package(GMPXX QUIET)

  pgo_dep_option(CGAL_WITH_GMPXX BOOL ON "Enable CGAL GMPXX support")
  pgo_dep_option(CGAL_ENABLE_TESTING BOOL OFF "disable testing")
  find_package(CGAL CONFIG COMPONENTS Core REQUIRED PATHS ${CGAL_SOURCE_DIR} NO_DEFAULT_PATH)

  include("${CGAL_MODULES_DIR}/CGAL_TBB_support.cmake")
endfunction()

pgo_add_third_party(CGAL
  TARGETS CGAL::CGAL
  STATUS "Downloading CGAL..."
  FETCH_MODE POPULATE
  POPULATE_REASON "CGAL source tree is patched before find_package(CGAL CONFIG ... NO_DEFAULT_PATH)"
  POST_FETCH _pgo_setup_cgal
  FETCHCONTENT_ARGS
    URL https://github.com/CGAL/cgal/releases/download/v6.0.1/CGAL-6.0.1-library.zip
    URL_HASH SHA256=7b0bb231d57261491722b7a0950f8026e17a08ae4a93315495cfb9d91faa31e3
    DOWNLOAD_EXTRACT_TIMESTAMP ON
)
