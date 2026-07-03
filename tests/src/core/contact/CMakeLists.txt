foreach(test_name
  ipcPairGenerator
  ipcContactAssembler
  spatialHashGrid
  ipcGeometry
  surfaceIPCTopology
  surfaceIPCSelfBroadPhase
  surfaceIPCExternalBroadPhase
  surfaceIPCMaxStep
  surfaceIPCExternalMaxStep
  surfaceIPCBarrierAssembler
  ipcActiveSetCache
  floorContactEnergy
  contactEnergyFactory
  surfaceDofMap
  sampledPenaltyContactEnergy
  sampledPenaltyInternals
)
  pgo_add_gtest(${test_name}_gtest
    SOURCES ${test_name}_gtest.cpp
    DEPS contact
  )
endforeach()

pgo_add_gtest(ipcContactEnergy_gtest
  SOURCES ipcContactEnergy_gtest.cpp
  DEPS contact profiling
)
