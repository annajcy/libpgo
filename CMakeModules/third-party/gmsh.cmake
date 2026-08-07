if(TARGET Gmsh::Gmsh)
  return()
endif()

message(STATUS "Downloading GMSH...")
include(FetchContent)

# Build only the static GMSH API library that libpgo links for .msh loading.
# Disable GUI, CAD, solver, and optional dependency features so the build is
# self-contained and does not leak Homebrew/system libraries into wheels.
set(DEFAULT OFF CACHE INTERNAL "GMSH default-value switch" FORCE)
set(ENABLE_BUILD_LIB ON CACHE BOOL "Build static GMSH library" FORCE)
set(ENABLE_BUILD_SHARED OFF CACHE BOOL "Build shared GMSH library" FORCE)
set(ENABLE_BUILD_DYNAMIC OFF CACHE BOOL "Build dynamic GMSH executable" FORCE)
set(ENABLE_GRAPHICS OFF CACHE BOOL "Disable graphics" FORCE)
set(ENABLE_OPENGL OFF CACHE BOOL "Disable OpenGL" FORCE)
set(ENABLE_OSMESA OFF CACHE BOOL "Disable OSMesa" FORCE)
set(ENABLE_FLTK OFF CACHE BOOL "Disable FLTK GUI" FORCE)
set(ENABLE_CAIRO OFF CACHE BOOL "Disable Cairo" FORCE)
set(ENABLE_OCC OFF CACHE BOOL "Disable OpenCASCADE" FORCE)
set(ENABLE_OCC_CAF OFF CACHE BOOL "Disable OpenCASCADE CAF" FORCE)
set(ENABLE_ONELAB OFF CACHE BOOL "Disable ONELAB" FORCE)
set(ENABLE_ONELAB_METAMODEL OFF CACHE BOOL "Disable ONELAB metamodels" FORCE)
set(ENABLE_MPI OFF CACHE BOOL "Disable MPI" FORCE)
set(ENABLE_PETSC OFF CACHE BOOL "Disable PETSc" FORCE)
set(ENABLE_SLEPC OFF CACHE BOOL "Disable SLEPc" FORCE)
set(ENABLE_MUMPS OFF CACHE BOOL "Disable MUMPS" FORCE)
set(ENABLE_OPENMP OFF CACHE BOOL "Disable GMSH OpenMP" FORCE)
set(ENABLE_OPENACC OFF CACHE BOOL "Disable OpenACC" FORCE)
set(ENABLE_GMP OFF CACHE BOOL "Disable GMP-dependent Kbipack" FORCE)
set(ENABLE_EIGEN OFF CACHE BOOL "Disable Eigen" FORCE)
set(ENABLE_BLAS_LAPACK OFF CACHE BOOL "Disable BLAS/LAPACK" FORCE)
set(ENABLE_MESH ON CACHE BOOL "Enable mesh module" FORCE)
set(ENABLE_POST OFF CACHE BOOL "Disable post-processing module" FORCE)
set(ENABLE_SOLVER OFF CACHE BOOL "Disable built-in solvers" FORCE)
set(ENABLE_PARSER OFF CACHE BOOL "Disable GEO parser" FORCE)
set(ENABLE_PLUGINS OFF CACHE BOOL "Disable plugins" FORCE)
set(ENABLE_DINTEGRATION OFF CACHE BOOL "Disable discrete integration" FORCE)
set(ENABLE_DOMHEX OFF CACHE BOOL "Disable DomHex" FORCE)
set(ENABLE_UNTANGLE OFF CACHE BOOL "Disable Untangle" FORCE)
set(ENABLE_NII2MESH OFF CACHE BOOL "Disable Nii2mesh" FORCE)
set(ENABLE_BLOSSOM OFF CACHE BOOL "Disable Blossom" FORCE)
set(ENABLE_ALGLIB OFF CACHE BOOL "Disable ALGLIB" FORCE)
set(ENABLE_KBIPACK OFF CACHE BOOL "Disable Kbipack" FORCE)
set(ENABLE_MATHEX OFF CACHE BOOL "Disable MathEx" FORCE)
set(ENABLE_TINYXML2 OFF CACHE BOOL "Disable TinyXML2" FORCE)
set(ENABLE_TETGENBR OFF CACHE BOOL "Disable TetGen/BR" FORCE)
set(ENABLE_HXT OFF CACHE BOOL "Disable HXT" FORCE)
set(ENABLE_METIS OFF CACHE BOOL "Disable Metis" FORCE)
set(ENABLE_NETGEN OFF CACHE BOOL "Disable Netgen" FORCE)
set(ENABLE_MMG OFF CACHE BOOL "Disable Mmg" FORCE)
set(ENABLE_BAMG OFF CACHE BOOL "Disable Bamg" FORCE)
set(ENABLE_ANN OFF CACHE BOOL "Disable ANN" FORCE)
set(ENABLE_VOROPP OFF CACHE BOOL "Disable Voro++" FORCE)
set(ENABLE_QUADTRI OFF CACHE BOOL "Disable QuadTri" FORCE)
set(ENABLE_QUADMESHINGTOOLS OFF CACHE BOOL "Disable QuadMeshingTools" FORCE)
set(ENABLE_WINSLOWUNTANGLER OFF CACHE BOOL "Disable WinslowUntangler" FORCE)
set(ENABLE_OPTHOM OFF CACHE BOOL "Disable high-order optimization" FORCE)
set(ENABLE_REVOROPT OFF CACHE BOOL "Disable Revoropt" FORCE)
set(ENABLE_MED OFF CACHE BOOL "Disable MED" FORCE)
set(ENABLE_CGNS OFF CACHE BOOL "Disable CGNS" FORCE)
set(ENABLE_GETDP OFF CACHE BOOL "Disable GetDP" FORCE)
set(ENABLE_POPPLER OFF CACHE BOOL "Disable Poppler" FORCE)
set(ENABLE_P4EST OFF CACHE BOOL "Disable p4est" FORCE)
set(ENABLE_MESQUITE OFF CACHE BOOL "Disable Mesquite" FORCE)
set(ENABLE_TCMALLOC OFF CACHE BOOL "Disable TCMalloc" FORCE)
set(ENABLE_TOUCHBAR OFF CACHE BOOL "Disable Apple Touch Bar" FORCE)
set(ENABLE_OS_SPECIFIC_INSTALL OFF CACHE BOOL "Disable OS-specific install" FORCE)
set(ENABLE_TESTS OFF CACHE BOOL "Disable GMSH tests" FORCE)
set(ENABLE_WRAP_PYTHON OFF CACHE BOOL "Disable Python wrappers" FORCE)
set(ENABLE_WRAP_JAVA OFF CACHE BOOL "Disable Java wrappers" FORCE)
set(ENABLE_NUMPY OFF CACHE BOOL "Disable NumPy private API" FORCE)
set(ENABLE_PETSC4PY OFF CACHE BOOL "Disable petsc4py" FORCE)
set(ENABLE_PRIVATE_API OFF CACHE BOOL "Disable private API" FORCE)

FetchContent_Declare(
  gmsh
  URL https://gmsh.info/src/gmsh-4.13.1-source.tgz
  URL_HASH SHA256=77972145f431726026d50596a6a44fb3c1c95c21255218d66955806b86edbe8d
  DOWNLOAD_EXTRACT_TIMESTAMP ON
  EXCLUDE_FROM_ALL
)

FetchContent_MakeAvailable(gmsh)

set(_pgo_gmsh_target)
foreach(_pgo_candidate IN ITEMS GMSH::GMSH lib shared gmsh)
  if(TARGET ${_pgo_candidate})
    set(_pgo_gmsh_target ${_pgo_candidate})
    break()
  endif()
endforeach()

if(NOT _pgo_gmsh_target)
  message(FATAL_ERROR
    "GMSH FetchContent build completed without a usable library target.")
endif()

add_library(Gmsh::Gmsh INTERFACE IMPORTED GLOBAL)
target_link_libraries(Gmsh::Gmsh INTERFACE ${_pgo_gmsh_target})
target_include_directories(Gmsh::Gmsh INTERFACE "${gmsh_SOURCE_DIR}/api")

if(WIN32)
  # GMSH's static library uses Winsock (send/gethostname) and the multimedia
  # joystick API even in a minimal build. Its own CMake only links these into
  # the executable/shared targets, so add them to the imported interface.
  target_link_libraries(Gmsh::Gmsh INTERFACE ws2_32 winmm)
endif()

message(STATUS "GMSH target: ${_pgo_gmsh_target}")
