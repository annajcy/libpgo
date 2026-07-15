# ================== third party ==========================
find_package(Threads REQUIRED)

if(PGO_UI)
  find_package(Vulkan REQUIRED)

  if(PGO_RENDER_USE_ASSIMP)
    find_package(assimp REQUIRED)
  endif()

  include(${PROJECT_SOURCE_DIR}/CMakeModules/third-party/glfw.cmake)
  include(${PROJECT_SOURCE_DIR}/CMakeModules/third-party/glm.cmake)
endif()

add_subdirectory(third-party)

include(CMakeModules/third-party/tbb.cmake)
include(CMakeModules/third-party/mkl.cmake)
include(CMakeModules/third-party/backward.cmake)

if(APPLE)
  include(CMakeModules/third-party/accelerate.cmake)
endif()

include(CMakeModules/third-party/eigen.cmake)
include(CMakeModules/third-party/fmt.cmake)
include(CMakeModules/third-party/nlohmann_json.cmake)
include(CMakeModules/third-party/spdlog.cmake)
include(CMakeModules/third-party/autodiff.cmake)
include(CMakeModules/third-party/argparse.cmake)

if(PGO_BUILD_BENCHMARKS)
  include(CMakeModules/third-party/benchmark.cmake)
endif()

if(PGO_ENABLE_PYTHON OR PGO_ENABLE_FULL)
  include(CMakeModules/third-party/ceres.cmake)
  include(CMakeModules/third-party/boost.cmake)
  include(CMakeModules/third-party/cgal.cmake)
endif()

if(PGO_ENABLE_FULL)
  include(CMakeModules/third-party/geogram.cmake)
endif()

if(PGO_ENABLE_FULL OR PGO_ENABLE_PYTHON)
  include(CMakeModules/third-party/libigl.cmake)

  igl_include(copyleft core)
  igl_include(copyleft cgal)
  
  if(PGO_ENABLE_ALEMBIC)
    include(CMakeModules/third-party/alembic.cmake)
  endif()

  if(PGO_ENABLE_GMSH)
    include(CMakeModules/third-party/gmsh.cmake)
  endif()
endif()

if(PGO_ENABLE_OPENVDB)
  include(CMakeModules/third-party/openvdb.cmake)
endif()

if(PGO_TET_MESHER_USE_TET_WILD)
  include(CMakeModules/third-party/ftetwild.cmake)
endif()

if(PGO_ENABLE_PYTHON)
  include(CMakeModules/third-party/nanobind.cmake)
endif()

if(TARGET MKL::MKL)
  target_compile_definitions(compilation_flag INTERFACE PGO_HAS_MKL)
endif()

if(TARGET CGAL::CGAL)
  target_compile_definitions(compilation_flag INTERFACE PGO_HAS_CGAL)
endif()

if(TARGET Ceres::ceres)
  target_compile_definitions(compilation_flag INTERFACE PGO_HAS_CERES)
endif()

if(PGO_OPT_USE_IPOPT)
  find_package(Ipopt)

  if(TARGET Ipopt::Core)
    target_compile_definitions(compilation_flag INTERFACE PGO_HAS_IPOPT)
  endif()
endif()

if(PGO_OPT_USE_KNITRO)
  include(CMakeModules/third-party/knitro.cmake)

  if(TARGET Knitro::Knitro)
    target_compile_definitions(compilation_flag INTERFACE PGO_HAS_KNITRO)
  endif()
endif()

if(PGO_HAS_ORIG_PARDISO)
  include(CMakeModules/third-party/pardiso.cmake)

  if(TARGET Pardiso::Pardiso)
    target_compile_definitions(compilation_flag INTERFACE PGO_HAS_ORIG_PARDISO)
  endif()
endif()
