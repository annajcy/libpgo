set(AVAILABLE_LIBS)

foreach(lib ${PGO_GLOBAL_LIBRARY_TARGETS})
  if(TARGET ${lib})
    list(APPEND AVAILABLE_LIBS ${lib})
  endif()
endforeach()

add_libpgo_tools(cubicMesher cubicMesher.cpp "${AVAILABLE_LIBS}")
target_link_libraries(cubicMesher PRIVATE argparse::argparse)