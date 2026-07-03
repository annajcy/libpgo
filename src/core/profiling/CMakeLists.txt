set(PROFILING_HEADERS
  processMemory.h
  scopedProfileSection.h
  threadRuntime.h
)

set(PROFILING_SOURCES
  processMemory.cpp
  scopedProfileSection.cpp
  threadRuntime.cpp
)

add_libpgo_lib(profiling "${PROFILING_SOURCES}" "${PROFILING_HEADERS}")

if(WIN32)
  target_link_libraries(profiling PRIVATE Psapi)
endif()
