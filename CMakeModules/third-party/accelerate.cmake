if(NOT APPLE)
  return()
endif()

execute_process(
  COMMAND xcrun --sdk macosx --show-sdk-path
  OUTPUT_VARIABLE PGO_MACOS_SDK_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  COMMAND_ERROR_IS_FATAL ANY)

find_library(PGO_ACCELERATE_FRAMEWORK
  NAMES Accelerate
  REQUIRED)
find_library(PGO_ACCELERATE_BLAS_RUNTIME
  NAMES blas
  PATHS "${PGO_MACOS_SDK_PATH}/usr/lib"
  NO_DEFAULT_PATH
  REQUIRED)

add_library(pgo_accelerate_blas INTERFACE)
add_library(PGO::AccelerateBLAS ALIAS pgo_accelerate_blas)
target_link_libraries(pgo_accelerate_blas INTERFACE
  "${PGO_ACCELERATE_FRAMEWORK}"
  "${PGO_ACCELERATE_BLAS_RUNTIME}")
