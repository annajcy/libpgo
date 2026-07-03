set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Export compile_commands.json for clangd" FORCE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(PGO_USE_MKL "Use MKL" OFF)
option(PGO_ENABLE_PYTHON "Build python module" OFF)
option(PGO_BUILD_C_API "Build C API library (pgo_c)" OFF)
option(PGO_ENABLE_FULL "Enable all functionalities" OFF)
option(PGO_ENABLE_CUDA "Enable CUDA" OFF)
option(PGO_BUILD_SUBPROJECTS "Include subprojects" OFF)
option(PGO_CHECK_CONDA "Check conda" OFF)

option(PGO_UI "Build pgo ui" OFF)
option(PGO_RENDER_USE_ASSIMP "Use Assimp" OFF)
option(PGO_RELEASE_MODE_DEBUG "Enable debug in release mode" OFF)
option(PGO_IGNORE_DEBUG_FLAG "Ignore debug flag" OFF)
option(PGO_CREATE_MISSING_FILE "Create file when it is missing" OFF)
option(PGO_HAS_ORIG_PARDISO "Use original pardiso" OFF)
option(PGO_OPT_USE_IPOPT "Use IPOPT" OFF)
option(PGO_OPT_USE_KNITRO "Use KNITRO" OFF)
option(PGO_MKL_LINK_DYNAMIC "MKL Link" ON)
option(PGO_ENABLE_ALEMBIC "Enable Alembic support" OFF)
option(PGO_ENABLE_OPENVDB "Enable OpenVDB support" OFF)
option(PGO_ENABLE_GMSH "Enable GMSH support" OFF)
option(PGO_TET_MESHER_USE_TET_WILD "Enable fTetWild backend for volumetric meshing" OFF)
option(PGO_BUILD_TESTING "Build libpgo tests" OFF)
option(PGO_NATIVE_OPTIMIZATION "Use host-native CPU optimization flags" ON)
option(PGO_ENABLE_OPENMP "Enable OpenMP compile/link flags and backend" ON)
# Statically embedding libstdc++/libgcc is fatal for a Python extension: the .so then
# carries a *private* libstdc++ while numpy/openvdb/vtk in the same interpreter use the
# shared one. Two libstdc++ copies share no locale-facet / RTTI state, so
# std::istringstream / std::filesystem crash. This project builds and runs inside conda
# (every shipped artifact is a conda package, which links the shared conda libstdc++),
# so default OFF. Opt in (-DPGO_STATIC_LIBSTDCXX=ON) only to ship a standalone binary to
# a non-conda host with an older system libstdc++.
option(PGO_STATIC_LIBSTDCXX "Statically link libstdc++/libgcc (GNU); opt-in for non-conda portable binaries" OFF)
