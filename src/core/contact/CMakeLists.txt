set(CONTACT_HEADERS
  sampled_penalty/kernels/CCDKernel.h
  sampled_penalty/kernels/contactEnergyUtilities.h
  sampled_penalty/kernels/pointPenetrationEnergy.h
  sampled_penalty/kernels/pointTrianglePairCouplingEnergyWithCollision.h
  sampled_penalty/kernels/triangleMeshExternalContactHandler.h
  sampled_penalty/kernels/triangleMeshSelfContactDetection.h
  sampled_penalty/kernels/triangleMeshSelfContactHandler.h
  sampled_penalty/sampledPenaltyEvaluationBundle.h
  sampled_penalty/sampledPenaltyContactEnergy.h
  sampled_penalty/sampledPenaltyContactBuilder.h
  sampled_penalty/sampledPenaltyContactEvaluator.h
  sampled_penalty/sampledPenaltyFrictionState.h
  sampled_penalty/sampledPenaltySpecs.h
  contactEnergyFactory.h
  surfaceDofMap.h
  statefulContactEnergy.h
  floor/floorContactEnergy.h
  ipc/ipcActiveSetCache.h
  ipc/ipcContactAssembler.h
  ipc/ipcContactEnergy.h
  ipc/ipcPairGenerator.h
  ipc/broadPhase/spatialHashGrid.h
  ipc/broadPhase/surfaceIPCBroadPhase.h
  ipc/core/surfaceIPCSelfBarrierAssembler.h
  ipc/core/surfaceIPCExternalBarrierAssembler.h
  ipc/core/surfaceIPCBarrierKernels.h
  ipc/core/surfaceIPCMaxStep.h
  ipc/core/surfaceIPCActiveSet.h
  ipc/core/surfaceIPCPairs.h
  ipc/external/obstacleSurface.h
  ipc/external/obstaclePoseCache.h
  ipc/geometry/ipcBarrier.h
  ipc/geometry/ipcCCD.h
  ipc/geometry/ipcDistancePrimitives.h
  ipc/geometry/ipcHessianProjection.h
  ipc/profiling/surfaceIPCProfiling.h
  ipc/topology/surfaceIPCTopology.h
)

set(CONTACT_SOURCES
  sampled_penalty/kernels/CCDKernel.cpp
  sampled_penalty/kernels/contactEnergyUtilities.cpp
  sampled_penalty/kernels/pointPenetrationEnergy.cpp
  sampled_penalty/kernels/pointTrianglePairCouplingEnergyWithCollision.cpp
  sampled_penalty/kernels/triangleMeshExternalContactHandler.cpp
  sampled_penalty/kernels/triangleMeshSelfContactDetection.cpp
  sampled_penalty/kernels/triangleMeshSelfContactHandler.cpp
  sampled_penalty/sampledPenaltyEvaluationBundle.cpp
  sampled_penalty/sampledPenaltyContactEnergy.cpp
  sampled_penalty/sampledPenaltyContactBuilder.cpp
  sampled_penalty/sampledPenaltyContactEvaluator.cpp
  sampled_penalty/sampledPenaltyFrictionState.cpp
  contactEnergyFactory.cpp
  surfaceDofMap.cpp
  floor/floorContactEnergy.cpp
  ipc/ipcActiveSetCache.cpp
  ipc/ipcContactAssembler.cpp
  ipc/ipcContactEnergy.cpp
  ipc/ipcPairGenerator.cpp
  ipc/broadPhase/spatialHashGrid.cpp
  ipc/broadPhase/surfaceIPCBroadPhase.cpp
  ipc/broadPhase/surfaceIPCSelfBroadPhase.cpp
  ipc/broadPhase/surfaceIPCExternalBroadPhase.cpp
  ipc/core/surfaceIPCSelfBarrierAssembler.cpp
  ipc/core/surfaceIPCExternalBarrierAssembler.cpp
  ipc/core/surfaceIPCBarrierKernels.cpp
  ipc/core/surfaceIPCMaxStep.cpp
  ipc/geometry/ipcBarrier.cpp
  ipc/geometry/ipcCCD.cpp
  ipc/external/obstacleSurface.cpp
  ipc/external/obstaclePoseCache.cpp
  ipc/geometry/ipcDistancePrimitives.cpp
  ipc/geometry/ipcHessianProjection.cpp
  ipc/topology/surfaceIPCTopology.cpp
)

set(CONTACT_PUBLIC_DEPS mesh nonlinearOptimization profiling)
set(CONTACT_PRIVATE_DEPS CCD::SafeCCD CCD::Exact)

foreach(tgt ${CONTACT_PUBLIC_DEPS} ${CONTACT_PRIVATE_DEPS})
  if(NOT TARGET ${tgt})
    message(STATUS "selfContact is not included. Missing: ${tgt}")
    return()
  endif()
endforeach()

add_libpgo_lib(contact "${CONTACT_SOURCES}" "${CONTACT_HEADERS}")

target_link_libraries(contact PUBLIC ${CONTACT_PUBLIC_DEPS})
target_link_libraries(contact PRIVATE ${CONTACT_PRIVATE_DEPS})
