add_executable(cubic_linear_pardiso_aftermath_probe
  cubic_linear_pardiso_aftermath_probe.cpp
)

target_link_libraries(cubic_linear_pardiso_aftermath_probe
  PRIVATE
    solidDeformationModel
    nonlinearOptimization
    eigenSupport
    volumetricMesh
    parallel
    MKL::MKL
    TBB::tbb
)
