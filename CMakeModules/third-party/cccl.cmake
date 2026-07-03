pgo_add_third_party(cccl
  TARGETS CCCL::CCCL
  STATUS "Loading cccl..."
  FETCHCONTENT_ARGS
  # GIT_REPOSITORY https://github.com/NVIDIA/cccl.git
  # GIT_TAG 8306d992387dfafa9e7d81387b4bf2c1c30ee8bf
    URL https://github.com/NVIDIA/cccl/releases/download/v3.1.3/cccl-src-v3.1.3.zip
    EXCLUDE_FROM_ALL
    DOWNLOAD_EXTRACT_TIMESTAMP ON
)
