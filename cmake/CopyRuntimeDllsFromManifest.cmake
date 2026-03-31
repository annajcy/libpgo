if(NOT DEFINED libpgo_runtime_manifest)
  message(FATAL_ERROR "libpgo_runtime_manifest is required")
endif()

if(NOT DEFINED libpgo_runtime_output_dir)
  message(FATAL_ERROR "libpgo_runtime_output_dir is required")
endif()

if(NOT EXISTS "${libpgo_runtime_manifest}")
  return()
endif()

include("${libpgo_runtime_manifest}")

if(NOT DEFINED libpgo_target_runtime_dlls OR libpgo_target_runtime_dlls STREQUAL "")
  return()
endif()

foreach(libpgo_runtime_dll IN LISTS libpgo_target_runtime_dlls)
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${libpgo_runtime_dll}" "${libpgo_runtime_output_dir}"
    COMMAND_ERROR_IS_FATAL ANY
  )
endforeach()
