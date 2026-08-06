if(APPLE)
  if(PGO_USE_MKL)
    message(WARNING "MKL is not supported on macOS; forcing PGO_USE_MKL=OFF.")
    set(PGO_USE_MKL OFF CACHE BOOL "Use MKL" FORCE)
  endif()
  set(BLA_VENDOR "Apple" CACHE STRING "BLAS vendor" FORCE)
else()
  if(NOT PGO_USE_MKL)
    message(WARNING "MKL is required on non-macOS builds; forcing PGO_USE_MKL=ON.")
    set(PGO_USE_MKL ON CACHE BOOL "Use MKL" FORCE)
  endif()
endif()

if(APPLE)
  if(PGO_ENABLE_CUDA)
    message(WARNING "CUDA is not supported on macOS; forcing PGO_ENABLE_CUDA=OFF.")
    set(PGO_ENABLE_CUDA OFF CACHE BOOL "Enable CUDA" FORCE)
  endif()

  # Homebrew installs GMP/MPFR/OpenVDB/GMSH under a prefix that is not on the
  # default compiler/linker search path on Apple Silicon.
  set(PGO_HOMEBREW_PREFIX "" CACHE PATH "Homebrew installation prefix")
  if(NOT PGO_HOMEBREW_PREFIX)
    find_program(PGO_BREW_EXECUTABLE brew)
    if(PGO_BREW_EXECUTABLE)
      execute_process(
        COMMAND ${PGO_BREW_EXECUTABLE} --prefix
        OUTPUT_VARIABLE PGO_HOMEBREW_PREFIX
        OUTPUT_STRIP_TRAILING_WHITESPACE)
      set(PGO_HOMEBREW_PREFIX "${PGO_HOMEBREW_PREFIX}" CACHE PATH "Homebrew installation prefix" FORCE)
    endif()
  endif()

  if(PGO_HOMEBREW_PREFIX AND EXISTS "${PGO_HOMEBREW_PREFIX}/include")
    message(STATUS "Adding Homebrew prefix to global search paths: ${PGO_HOMEBREW_PREFIX}")
    include_directories(SYSTEM "${PGO_HOMEBREW_PREFIX}/include")
    link_directories("${PGO_HOMEBREW_PREFIX}/lib")
    list(APPEND CMAKE_PREFIX_PATH "${PGO_HOMEBREW_PREFIX}")
  endif()
endif()

# The active uv virtual environment provides Python-distributed native
# dependencies (oneMKL/oneTBB on Linux and Windows). Standard finders already
# search CMAKE_PREFIX_PATH; pythonDependencies.cmake prepends the venv there.
if(NOT "$ENV{VIRTUAL_ENV}" STREQUAL "" AND IS_DIRECTORY "$ENV{VIRTUAL_ENV}")
  message(STATUS "Active virtual environment: $ENV{VIRTUAL_ENV}")
endif()
