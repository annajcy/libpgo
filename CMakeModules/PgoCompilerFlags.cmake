set(PGO_OPENMP_FOUND OFF)
if(PGO_ENABLE_OPENMP)
  find_package(OpenMP)
  if(OpenMP_CXX_FOUND OR OPENMP_FOUND)
    set(PGO_OPENMP_FOUND ON)
    set(OPENMP_FLAG "${OpenMP_CXX_FLAGS}")
  endif()
else()
  message(STATUS "OpenMP disabled by PGO_ENABLE_OPENMP=OFF")
endif()

# openmp may cause mkl pardiso error
include(Find_AVX)
CHECK_FOR_AVX()

# basic debug flags
add_library(compilation_flag_for_debug INTERFACE)
add_flag(compilation_flag_for_debug MSVC Release INTERFACE /ZI)
add_flag(compilation_flag_for_debug MSVC Release INTERFACE /Oi-)
add_flag(compilation_flag_for_debug MSVC Release INTERFACE /Ob0)
add_def(compilation_flag_for_debug MSVC Debug INTERFACE _DEBUG)
add_def(compilation_flag_for_debug MSVC Release INTERFACE NDEBUG)

add_flag_poxis(compilation_flag_for_debug INTERFACE -ggdb3)
add_flag_poxis(compilation_flag_for_debug INTERFACE -O0)
add_flag_poxis(compilation_flag_for_debug INTERFACE -fsanitize=address)
add_flag_poxis(compilation_flag_for_debug INTERFACE -fsanitize=undefined)

add_flag(compilation_flag_for_debug GNU All INTERFACE -fsanitize=leak)
add_flag(compilation_flag_for_debug Clang All INTERFACE -fsanitize=leak)

add_library(compilation_flag INTERFACE)
add_library(cuda_compilation_flag INTERFACE)

if(PGO_OPENMP_FOUND)
  target_compile_definitions(compilation_flag INTERFACE USE_OPENMP)
endif()

message(STATUS "PGO compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "OS: ${CMAKE_SYSTEM_NAME}")

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "^(Apple)?Clang$")
  message(STATUS "CMake build type: ${CMAKE_BUILD_TYPE}")

  if(CMAKE_BUILD_TYPE STREQUAL "")
    message(STATUS "CMake build type is empty and will be set to Release")
    set(CMAKE_BUILD_TYPE Release)
  endif()

  if(CMAKE_GENERATOR STREQUAL "Ninja")
    add_flag_poxis(compilation_flag INTERFACE -fdiagnostics-color=always)
  endif()

elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  message(STATUS "Windows platform...")
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")

  if(CMAKE_GENERATOR STREQUAL Ninja)
    message(STATUS "use ninja")
  else()
    set(CMAKE_CONFIGURATION_TYPES Debug Release)
  endif()
else()
  message(FATAL_ERROR "Unsupported compiler")
endif()

# basic compilation flags
add_flag_poxis(compilation_flag INTERFACE -Wall)
add_flag_poxis(compilation_flag INTERFACE -Wextra)
add_flag_poxis(compilation_flag INTERFACE -frounding-math)
add_flag_poxis(compilation_flag INTERFACE -fvisibility=hidden)
if(PGO_NATIVE_OPTIMIZATION)
  add_flag_poxis(compilation_flag INTERFACE -march=native)
  add_flag_poxis(compilation_flag INTERFACE -mtune=native)
endif()
add_flag(compilation_flag GNU All INTERFACE -Wno-template-body)

if(PGO_NATIVE_OPTIMIZATION)
  add_link_flag_poxis(compilation_flag -march=native)
  add_link_flag_poxis(compilation_flag -mtune=native)
endif()

# static gcc — opt-in only (see PGO_STATIC_LIBSTDCXX); never enable for the Python module
if(PGO_STATIC_LIBSTDCXX)
  add_flag(compilation_flag GNU All INTERFACE -static-libstdc++)
  add_flag(compilation_flag GNU All INTERFACE -static-libgcc)
  add_link_flag(compilation_flag GNU All -static-libstdc++)
  add_link_flag(compilation_flag GNU All -static-libgcc)
endif()

# if has openmp flags
if(PGO_OPENMP_FOUND)
  add_flag(compilation_flag GNU All INTERFACE ${OPENMP_FLAG})
  add_flag(compilation_flag Clang All INTERFACE ${OPENMP_FLAG})
  add_link_flag(compilation_flag GNU All ${OPENMP_FLAG})
  add_link_flag(compilation_flag Clang All ${OPENMP_FLAG})
  add_flag(compilation_flag MSVC All INTERFACE ${OPENMP_FLAG})
endif()

# msvc flags
add_def(compilation_flag MSVC All INTERFACE __BASE_FILE__=__FILE__)
add_flag(compilation_flag MSVC All INTERFACE /bigobj)
add_flag(compilation_flag MSVC All INTERFACE /Zc:__cplusplus)
add_flag_cuda(cuda_compilation_flag MSVC All INTERFACE "-Xcompiler=/bigobj")

if(PGO_NATIVE_OPTIMIZATION)
  if(HAVE_AVX512_EXTENSIONS)
    add_flag(compilation_flag MSVC All INTERFACE /arch:AVX512)
    add_flag_cuda(cuda_compilation_flag MSVC All INTERFACE "-Xcompiler=/arch:AVX512")
  elseif(HAVE_AVX2_EXTENSIONS)
    add_flag(compilation_flag MSVC All INTERFACE /arch:AVX2)
    add_flag_cuda(cuda_compilation_flag MSVC All INTERFACE "-Xcompiler=/arch:AVX2")
  elseif(HAVE_AVX_EXTENSIONS)
    add_flag(compilation_flag MSVC All INTERFACE /arch:AVX)
    add_flag_cuda(cuda_compilation_flag MSVC All INTERFACE "-Xcompiler=/arch:AVX")
  endif()
endif()

add_flag(compilation_flag MSVC All INTERFACE /MP)
add_flag_cuda(cuda_compilation_flag MSVC All INTERFACE "-Xcompiler=/MP")

# set(cuda_compilation_flag "${cuda_compilation_flag};/MP")
add_def(compilation_flag MSVC All INTERFACE NOMINMAX)
add_def(compilation_flag MSVC All INTERFACE _USE_MATH_DEFINES)
add_def(compilation_flag MSVC All INTERFACE _CRT_SECURE_NO_WARNINGS)
add_def(compilation_flag MSVC All INTERFACE HAVE_STRUCT_TIMESPEC)
add_def(compilation_flag MSVC All INTERFACE GLOG_NO_ABBREVIATED_SEVERITIES)

if(PGO_ENABLE_CUDA)
  find_package(CUDAToolkit REQUIRED)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "Debug Mode..")

  add_flag_poxis(compilation_flag INTERFACE -O0)
  add_flag_poxis(compilation_flag INTERFACE -ggdb3)
  add_link_flag_poxis(compilation_flag -ggdb3)
else()
  message(STATUS "Release Mode..")

  add_flag_poxis(compilation_flag INTERFACE -O3)
  add_flag_poxis(compilation_flag INTERFACE -ggdb3)
  add_link_flag_poxis(compilation_flag -ggdb3)
endif()
