set(FILESERVICE_HEADERS
  fileService.h
)

set(FILESERVICE_SOURCES
  fileService.cpp
)

add_libpgo_lib(fileService "${FILESERVICE_SOURCES}" "${FILESERVICE_HEADERS}")
