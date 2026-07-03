project(cudaUtilities LANGUAGES CXX CUDA)

set(CUDA_UTILITIES_HEADERS
  cudaUtils.h
  cudaMat.h
  cudaVec.h
  cudaInit.h
  cudaLinearMappedMemory.h
)

set(CUDA_UTILITIES_SOURCES
  cudaInit.cu
)

if(NOT TARGET CUDA::cublas OR NOT TARGET CUDA::cusparse OR NOT TARGET CUDA::cudart)
  message(STATUS "cudaUtilities is not included. Missing: cuda")
  return()
endif()

set(CUDA_UTILITIES_DEPS pgoLogging CUDA::cublas CUDA::cusparse CUDA::cudart)

pgo_require_targets(cudaUtilities TARGETS ${CUDA_UTILITIES_DEPS})

add_libpgo_lib(cudaUtilities "${CUDA_UTILITIES_SOURCES}" "${CUDA_UTILITIES_HEADERS}")

target_link_libraries(cudaUtilities PUBLIC ${CUDA_UTILITIES_DEPS})
