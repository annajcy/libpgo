pgo_add_third_party(fmt
  TARGETS fmt::fmt-header-only
  STATUS "Loading fmtlib..."
  FETCHCONTENT_ARGS
    URL https://github.com/fmtlib/fmt/archive/refs/tags/11.1.4.zip
    EXCLUDE_FROM_ALL
    DOWNLOAD_EXTRACT_TIMESTAMP ON
)
