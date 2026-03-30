if(DEFINED LIBPGO_CONAN_DEPS_INCLUDED)
  return()
endif()
set(LIBPGO_CONAN_DEPS_INCLUDED ON)

message(STATUS "Resolving Conan-managed packages...")

function(_libpgo_try_find_package package_name)
  if(ARGC GREATER 1)
    find_package(${package_name} QUIET CONFIG COMPONENTS ${ARGN})
  else()
    find_package(${package_name} QUIET CONFIG)
  endif()

  if(${package_name}_FOUND)
    message(STATUS "Conan package found: ${package_name}")
  else()
    message(STATUS "Conan package missing: ${package_name} (legacy fallback may be used)")
  endif()
endfunction()

function(_libpgo_add_ns_alias alias_target real_target)
  if(NOT TARGET ${alias_target} AND TARGET ${real_target})
    add_library(${alias_target} INTERFACE IMPORTED)
    target_link_libraries(${alias_target} INTERFACE ${real_target})
  endif()
endfunction()

# ── Core dependencies ────────────────────────────────────────────────────────
_libpgo_try_find_package(TBB)
_libpgo_try_find_package(onetbb)
_libpgo_try_find_package(Eigen3)
_libpgo_try_find_package(fmt)
_libpgo_try_find_package(nlohmann_json)
_libpgo_try_find_package(spdlog)
_libpgo_try_find_package(argparse)
_libpgo_try_find_package(tinyobjloader)
_libpgo_try_find_package(stb)
_libpgo_try_find_package(autodiff)

# ── Optional: Geometry stack ─────────────────────────────────────────────────
if(PGO_FEATURE_GEOMETRY_STACK)
  _libpgo_try_find_package(boost)
  _libpgo_try_find_package(Boost)
  _libpgo_try_find_package(CGAL Core)
  _libpgo_try_find_package(Ceres)
  _libpgo_try_find_package(NLopt)
  _libpgo_try_find_package(nlopt)
  _libpgo_try_find_package(SuiteSparse)
endif()

if(PGO_FEATURE_PYTHON)
  _libpgo_try_find_package(pybind11)
endif()

if(PGO_FEATURE_LIBIGL)
  _libpgo_try_find_package(libigl)
endif()

if(PGO_FEATURE_GEOGRAM)
  _libpgo_try_find_package(geogram)
endif()

# ── Optional: Feature-switch dependencies ────────────────────────────────────
if(PGO_FEATURE_ANIMATION_IO)
  _libpgo_try_find_package(Alembic)
  _libpgo_try_find_package(Imath)
endif()

if(PGO_FEATURE_GMSH)
  _libpgo_try_find_package(gmsh)
endif()

if(PGO_FEATURE_ARPACK)
  _libpgo_try_find_package(arpackng)
endif()

# ── Vendored private-recipe dependencies ─────────────────────────────────────
_libpgo_try_find_package(tetgen)
_libpgo_try_find_package(CCD_SafeCCD)
_libpgo_try_find_package(CCD_ExactCCD)
_libpgo_try_find_package(ASA)

# ── Testing ──────────────────────────────────────────────────────────────────
if(PGO_ENABLE_TESTS)
  _libpgo_try_find_package(GTest)
endif()

# ── Compatibility aliases ────────────────────────────────────────────────────
# Target name remapping so that existing CMakeLists.txt in src/ keeps working
# without changes to target_link_libraries() calls.

_libpgo_add_ns_alias(TBB::tbb onetbb::onetbb)
_libpgo_add_ns_alias(onetbb::onetbb TBB::tbb)
_libpgo_add_ns_alias(Boost::boost boost::boost)
_libpgo_add_ns_alias(boost::boost Boost::boost)
_libpgo_add_ns_alias(NLopt::nlopt nlopt::nlopt)
_libpgo_add_ns_alias(nlopt::nlopt NLopt::nlopt)

# Conan's top-level boost::boost target aggregates every enabled component,
# including Boost.Test. Some downstream packages (for example CGAL) depend on
# boost::boost expecting header-only semantics, so normalize it back to the
# headers-only component to avoid leaking unrelated link libraries.
if(TARGET boost::boost AND TARGET Boost::boost)
  set_property(TARGET boost::boost PROPERTY INTERFACE_LINK_LIBRARIES Boost::boost)
elseif(TARGET boost::boost AND TARGET Boost::headers)
  set_property(TARGET boost::boost PROPERTY INTERFACE_LINK_LIBRARIES Boost::headers)
endif()

if(TARGET Ceres::ceres)
  _libpgo_add_ns_alias(Ceres::Ceres Ceres::ceres)
elseif(TARGET Ceres::Ceres)
  _libpgo_add_ns_alias(Ceres::ceres Ceres::Ceres)
endif()

if(TARGET fmt::fmt)
  _libpgo_add_ns_alias(fmt::fmt-header-only fmt::fmt)
endif()

if(TARGET spdlog::spdlog)
  _libpgo_add_ns_alias(spdlog::spdlog_header_only spdlog::spdlog)
endif()

# tiny_obj_loader: Conan target → plain "tiny_obj_loader" used in src/
if(TARGET tinyobjloader::tinyobjloader_double AND NOT TARGET tiny_obj_loader)
  add_library(tiny_obj_loader INTERFACE)
  target_link_libraries(tiny_obj_loader INTERFACE tinyobjloader::tinyobjloader_double)
elseif(TARGET tinyobjloader::tinyobjloader AND NOT TARGET tiny_obj_loader)
  add_library(tiny_obj_loader INTERFACE)
  target_link_libraries(tiny_obj_loader INTERFACE tinyobjloader::tinyobjloader)
  target_compile_definitions(tiny_obj_loader INTERFACE TINYOBJLOADER_USE_DOUBLE)
endif()

# stb
if(TARGET stb::stb AND NOT TARGET stb)
  add_library(stb INTERFACE)
  target_link_libraries(stb INTERFACE stb::stb)
endif()

# CGAL TBB support shim
if(TARGET CGAL::CGAL AND TARGET TBB::tbb AND NOT TARGET CGAL::TBB_support)
  add_library(CGAL::TBB_support INTERFACE IMPORTED)
  target_link_libraries(CGAL::TBB_support INTERFACE TBB::tbb)
endif()

# tetgen: Conan target → bare "tetgen" used in src/
if(TARGET tetgen::tetgen AND NOT TARGET tetgen)
  add_library(tetgen INTERFACE)
  target_link_libraries(tetgen INTERFACE tetgen::tetgen)
endif()

# CCD: Conan targets → aliased targets used in src/
if(TARGET CCD::SafeCCD AND NOT TARGET CCD_SafeCCD)
  add_library(CCD_SafeCCD INTERFACE)
  target_link_libraries(CCD_SafeCCD INTERFACE CCD::SafeCCD)
endif()
if(TARGET CCD::Exact AND NOT TARGET CCD_ExactCCD)
  add_library(CCD_ExactCCD INTERFACE)
  target_link_libraries(CCD_ExactCCD INTERFACE CCD::Exact)
endif()

# ASA: Conan target → bare "ASA_lib" used in src/
if(TARGET ASA::ASA AND NOT TARGET ASA_lib)
  add_library(ASA_lib INTERFACE)
  target_link_libraries(ASA_lib INTERFACE ASA::ASA)
endif()

# geogram: Conan target → bare "geogram" used in src/
if(TARGET geogram::geogram AND NOT TARGET geogram)
  add_library(geogram INTERFACE)
  target_link_libraries(geogram INTERFACE geogram::geogram)
endif()

# Gmsh: Conan target → aliased target used in src/
if(TARGET Gmsh::Gmsh)
  # Already matches the expected target name.
elseif(TARGET gmsh::gmsh AND NOT TARGET Gmsh::Gmsh)
  _libpgo_add_ns_alias(Gmsh::Gmsh gmsh::gmsh)
endif()
