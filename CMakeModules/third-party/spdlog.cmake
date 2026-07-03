if(TARGET spdlog::spdlog_header_only)
  return()
endif()

pgo_dep_option(SPDLOG_FMT_EXTERNAL_HO BOOL ON "Use the existing header-only fmt target in spdlog")

pgo_add_third_party(spdlog
  TARGETS spdlog::spdlog_header_only
  STATUS "Loading spdlog..."
  FETCHCONTENT_ARGS
    URL https://github.com/gabime/spdlog/archive/refs/tags/v1.15.2.zip
    EXCLUDE_FROM_ALL
    DOWNLOAD_EXTRACT_TIMESTAMP ON
)
