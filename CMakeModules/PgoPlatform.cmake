if(APPLE)
  if(PGO_USE_MKL)
    message(WARNING "MKL is not supported on macOS; forcing PGO_USE_MKL=OFF.")
    set(PGO_USE_MKL OFF CACHE BOOL "Use MKL" FORCE)
  endif()

  if(PGO_ENABLE_CUDA)
    message(WARNING "CUDA is not supported on macOS; forcing PGO_ENABLE_CUDA=OFF.")
    set(PGO_ENABLE_CUDA OFF CACHE BOOL "Enable CUDA" FORCE)
  endif()

  if(NOT PGO_CHECK_CONDA)
    # Homebrew installs GMP/MPFR (required by CGAL, ftetwild, ...) under a prefix
    # that is not on the default compiler/linker search path on Apple Silicon.
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
endif()

# Conda builds (PGO_CHECK_CONDA=ON): make standard finders search the active
# conda environment. Package-specific finders historically hardcoded their own
# $CONDA_PREFIX hints, but find_package(BLAS) relies on the default search path.
if(PGO_CHECK_CONDA AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  if(WIN32)
    # CMake >=4.3 try_compile writes CMAKE_PREFIX_PATH/CMAKE_MODULE_PATH into
    # generated CMakeLists.txt files without escaping backslashes, causing syntax
    # errors when Windows-style paths (e.g. C:\Users\…) appear as string values.
    # Normalise to forward slashes — CMake handles them natively on Windows too.
    string(REPLACE "\\" "/" _conda_prefix "$ENV{CONDA_PREFIX}")
    list(PREPEND CMAKE_PREFIX_PATH "${_conda_prefix}/Library" "${_conda_prefix}")
    list(PREPEND CMAKE_LIBRARY_PATH "${_conda_prefix}/Library/lib" "${_conda_prefix}/Library/bin" "${_conda_prefix}/Library/mingw-w64/lib")
  else()
    list(PREPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
    list(PREPEND CMAKE_LIBRARY_PATH "$ENV{CONDA_PREFIX}/lib")
  endif()
  message(STATUS "Conda build: prepended $ENV{CONDA_PREFIX} to CMAKE_PREFIX_PATH")

  # Help the custom FindBLAS.cmake (CMakeModules/FindBLAS.cmake) locate BLAS
  # libraries inside the conda environment.  Without these hints the module
  # falls back to probing system paths and fails on macOS/Windows.
  if(PGO_USE_MKL)
    # MKL provides its own CMake package config — tell FindBLAS to use it.
    set(BLA_VENDOR "Intel10_64lp")
  else()
    # OpenBLAS and other conda BLAS backends ship .pc files.  Prefer
    # pkg-config so we do not rely on the manual library-search fallback.
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
      set(BLA_PREFER_PKGCONFIG ON)
      set(BLA_PKGCONFIG_BLAS "openblas")
    endif()
  endif()

  if(PGO_ENABLE_PYTHON AND NOT DEFINED Python_EXECUTABLE)
    if(WIN32)
      find_program(_pgo_conda_python NAMES python.exe python
        PATHS "$ENV{CONDA_PREFIX}" "$ENV{CONDA_PREFIX}/Scripts"
        NO_DEFAULT_PATH)
    else()
      find_program(_pgo_conda_python NAMES python3 python
        PATHS "$ENV{CONDA_PREFIX}/bin"
        NO_DEFAULT_PATH)
    endif()
    if(_pgo_conda_python)
      set(Python_EXECUTABLE "${_pgo_conda_python}" CACHE FILEPATH "Python executable for pypgo bindings" FORCE)
      message(STATUS "Conda build: using Python executable ${Python_EXECUTABLE}")
    endif()
  endif()

  if(APPLE)
    set(_pgo_openblas_vers "$ENV{CONDA_PREFIX}/lib/libopenblas.0.dylib")
    set(_pgo_openblas_plain "$ENV{CONDA_PREFIX}/lib/libopenblas.dylib")
    if(EXISTS "${_pgo_openblas_vers}" AND NOT EXISTS "${_pgo_openblas_plain}")
      execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
        "libopenblas.0.dylib" "${_pgo_openblas_plain}")
      message(STATUS "Created missing OpenBLAS symlink: ${_pgo_openblas_plain} -> libopenblas.0.dylib")
    endif()
  endif()
endif()
