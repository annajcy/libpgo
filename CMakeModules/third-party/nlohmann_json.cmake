pgo_add_third_party(nlohmann_json
  TARGETS nlohmann_json::nlohmann_json
  STATUS "Loading nlohmann_json..."
  FETCHCONTENT_ARGS
    URL https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.tar.gz
    EXCLUDE_FROM_ALL
    DOWNLOAD_EXTRACT_TIMESTAMP ON
)
