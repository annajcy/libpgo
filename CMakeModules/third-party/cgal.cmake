if(TARGET CGAL::CGAL)
  return()
endif()

message(STATUS "Downloading CGAL...")
include(FetchContent)

FetchContent_Declare(
  CGAL
  URL https://github.com/CGAL/cgal/releases/download/v6.0.1/CGAL-6.0.1-library.zip
  URL_HASH SHA256=7b0bb231d57261491722b7a0950f8026e17a08ae4a93315495cfb9d91faa31e3
  DOWNLOAD_EXTRACT_TIMESTAMP ON
)

FetchContent_GetProperties(CGAL)

if(NOT CGAL_POPULATED)
  FetchContent_Populate(CGAL)
endif()

if(NOT CGAL_SOURCE_DIR)
  if(NOT FETCHCONTENT_BASE_DIR)
    set(FETCHCONTENT_BASE_DIR "${CMAKE_BINARY_DIR}/_deps")
  endif()
  set(CGAL_SOURCE_DIR "${FETCHCONTENT_BASE_DIR}/cgal-src")
endif()

set(MODIFIED_FILE "${CMAKE_SOURCE_DIR}/CMakeModules/patches/CGAL_SetupBoost.cmake")
set(TARGET_FILE "${CGAL_SOURCE_DIR}/cmake/modules/CGAL_SetupBoost.cmake")

file(READ "${MODIFIED_FILE}" content)
file(WRITE "${TARGET_FILE}" "${content}")

set(CGAL_WITH_GMPXX ON CACHE BOOL "" FORCE)
set(CGAL_ENABLE_TESTING OFF CACHE BOOL "disable testing" FORCE)
find_package(CGAL CONFIG COMPONENTS Core REQUIRED PATHS ${CGAL_SOURCE_DIR} NO_DEFAULT_PATH)

message(STATUS "cgal module path: ${CGAL_MODULES_DIR}")
message(STATUS "CGAL GMP libraries: ${GMP_LIBRARIES}")
message(STATUS "CGAL GMPXX libraries: ${GMPXX_LIBRARIES}")
message(STATUS "CGAL MPFR libraries: ${MPFR_LIBRARIES}")
message(STATUS "TBB: ${TBB_FOUND}")
include("${CGAL_MODULES_DIR}/CGAL_TBB_support.cmake")
