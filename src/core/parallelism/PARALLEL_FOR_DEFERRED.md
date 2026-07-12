# tbb::parallel_for V1 Inventory

This document is the human-readable audit view of `PARALLEL_FOR_INVENTORY.json`.
The JSON manifest is the source of truth for the static guard.

## Scope and baseline

- Scan scope: C/C++ source and headers below `src/`, excluding `src/core/parallelism/**`.
- Scanner semantics: comments and string/character literals are ignored.
- Original textual baseline: 160 references in 39 production files.
- Phase 1 active baseline: 159 calls in 39 files.
- Final V1 active baseline: 84 deferred calls in 25 files after 75 migrations.
- Removed dead reference: the commented-out `tbb::parallel_for` in `multiVertexPullingSoftConstraintsPOrder.cpp` (original line 84).
- Stable guard key: per-file active-call count. Informational line numbers may move.

## Classification summary

| Classification | Calls |
|---|---:|
| `migrate-v1` | 0 |
| `typed-index` | 23 |
| `chunk-scratch` | 0 |
| `partitioner` | 49 |
| `tls-coupled` | 12 |
| `reduce` | 0 |
| `non-index-range` | 0 |
| **Total** | **84** |

Explicit partitioners: `default`=35, `static`=49.

Index types: `ES::IDX/IDX`=3, `Eigen::Index`=20, `int`=52, `size_t`=7, `std::size_t`=2.

A deferred category is the primary reason the call cannot move in V1. The coupling column
retains secondary TLS/scratch information even when typed index or partitioner is primary.

## Per-call inventory

| ID | Location | Index type | Partitioner | Coupling | Classification | Rationale |
|---|---|---|---|---|---|---|
| `src/core/constraintPotentialEnergies/barycentricCoordinateSlidingSoftConstraints.cpp#1` | `src/core/constraintPotentialEnergies/barycentricCoordinateSlidingSoftConstraints.cpp:90` | `int` | `static` | TLS/combinable local state | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/constraintPotentialEnergies/barycentricCoordinateSlidingSoftConstraints.cpp#2` | `src/core/constraintPotentialEnergies/barycentricCoordinateSlidingSoftConstraints.cpp:139` | `int` | `static` | shared state protected by existing concurrency primitive | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/constraintPotentialEnergies/barycentricCoordinateSlidingSoftConstraints.cpp#3` | `src/core/constraintPotentialEnergies/barycentricCoordinateSlidingSoftConstraints.cpp:207` | `int` | `static` | shared state protected by existing concurrency primitive | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/constraintPotentialEnergies/barycentricCoordinateSlidingSoftConstraints.cpp#4` | `src/core/constraintPotentialEnergies/barycentricCoordinateSlidingSoftConstraints.cpp:251` | `int` | `static` | shared state protected by existing concurrency primitive | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/constraintPotentialEnergies/barycentricCoordinateSlidingSoftConstraints.cpp#5` | `src/core/constraintPotentialEnergies/barycentricCoordinateSlidingSoftConstraints.cpp:338` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/constraintPotentialEnergies/multiVertexPullingSoftConstraints.cpp#1` | `src/core/constraintPotentialEnergies/multiVertexPullingSoftConstraints.cpp:104` | `size_t` | `default` | none observed | `typed-index` | size_t range cannot be passed to the int-only V1 API without narrowing |
| `src/core/constraintPotentialEnergies/multiVertexPullingSoftConstraints.cpp#2` | `src/core/constraintPotentialEnergies/multiVertexPullingSoftConstraints.cpp:126` | `size_t` | `default` | none observed | `typed-index` | size_t range cannot be passed to the int-only V1 API without narrowing |
| `src/core/constraintPotentialEnergies/multiVertexPullingSoftConstraintsPOrder.cpp#1` | `src/core/constraintPotentialEnergies/multiVertexPullingSoftConstraintsPOrder.cpp:125` | `int` | `static` | shared state protected by existing concurrency primitive | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/constraintPotentialEnergies/multiVertexPullingSoftConstraintsPOrder.cpp#2` | `src/core/constraintPotentialEnergies/multiVertexPullingSoftConstraintsPOrder.cpp:191` | `int` | `static` | shared state protected by existing concurrency primitive | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/constraintPotentialEnergies/multiVertexPullingSoftConstraintsPOrder.cpp#3` | `src/core/constraintPotentialEnergies/multiVertexPullingSoftConstraintsPOrder.cpp:287` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/constraintPotentialEnergies/multiVertexSlidingSoftConstraints.cpp#1` | `src/core/constraintPotentialEnergies/multiVertexSlidingSoftConstraints.cpp:80` | `int` | `static` | TLS/combinable local state | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/constraintPotentialEnergies/multiVertexSlidingSoftConstraints.cpp#2` | `src/core/constraintPotentialEnergies/multiVertexSlidingSoftConstraints.cpp:121` | `int` | `static` | shared state protected by existing concurrency primitive | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/constraintPotentialEnergies/multiVertexSlidingSoftConstraints.cpp#3` | `src/core/constraintPotentialEnergies/multiVertexSlidingSoftConstraints.cpp:168` | `int` | `static` | shared state protected by existing concurrency primitive | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/constraintPotentialEnergies/multiVertexSlidingSoftConstraints.cpp#4` | `src/core/constraintPotentialEnergies/multiVertexSlidingSoftConstraints.cpp:199` | `int` | `static` | shared state protected by existing concurrency primitive | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/constraintPotentialEnergies/multiVertexSlidingSoftConstraints.cpp#5` | `src/core/constraintPotentialEnergies/multiVertexSlidingSoftConstraints.cpp:270` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/contact/ipc/broadPhase/surfaceIPCBroadPhaseInternal.h#1` | `src/core/contact/ipc/broadPhase/surfaceIPCBroadPhaseInternal.h:192` | `int` | `default` | TLS/combinable local state | `tls-coupled` | loop owns or consumes TLS/combinable state that must migrate with its merge lifecycle |
| `src/core/contact/ipc/core/surfaceIPCSelfBarrierAssembler.cpp#1` | `src/core/contact/ipc/core/surfaceIPCSelfBarrierAssembler.cpp:141` | `Eigen::Index` | `default` | none observed | `typed-index` | Eigen::Index range cannot be passed to the int-only V1 API without narrowing |
| `src/core/contact/ipc/core/surfaceIPCSelfBarrierAssembler.cpp#2` | `src/core/contact/ipc/core/surfaceIPCSelfBarrierAssembler.cpp:215` | `Eigen::Index` | `default` | none observed | `typed-index` | Eigen::Index range cannot be passed to the int-only V1 API without narrowing |
| `src/core/contact/ipc/core/surfaceIPCSelfBarrierAssembler.cpp#3` | `src/core/contact/ipc/core/surfaceIPCSelfBarrierAssembler.cpp:392` | `int` | `default` | TLS/combinable local state | `tls-coupled` | loop owns or consumes TLS/combinable state that must migrate with its merge lifecycle |
| `src/core/contact/ipc/core/surfaceIPCSelfBarrierAssembler.cpp#4` | `src/core/contact/ipc/core/surfaceIPCSelfBarrierAssembler.cpp:409` | `int` | `default` | TLS/combinable local state | `tls-coupled` | loop owns or consumes TLS/combinable state that must migrate with its merge lifecycle |
| `src/core/contact/sampled_penalty/kernels/pointPenetrationEnergy.cpp#1` | `src/core/contact/sampled_penalty/kernels/pointPenetrationEnergy.cpp:193` | `int` | `static` | TLS/combinable local state | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/contact/sampled_penalty/kernels/pointPenetrationEnergy.cpp#2` | `src/core/contact/sampled_penalty/kernels/pointPenetrationEnergy.cpp:262` | `int` | `static` | shared state protected by existing concurrency primitive | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/contact/sampled_penalty/kernels/pointPenetrationEnergy.cpp#3` | `src/core/contact/sampled_penalty/kernels/pointPenetrationEnergy.cpp:373` | `int` | `static` | shared state protected by existing concurrency primitive | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/contact/sampled_penalty/kernels/pointPenetrationEnergy.cpp#4` | `src/core/contact/sampled_penalty/kernels/pointPenetrationEnergy.cpp:420` | `int` | `static` | shared state protected by existing concurrency primitive | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/contact/sampled_penalty/kernels/triangleMeshExternalContactHandler.cpp#1` | `src/core/contact/sampled_penalty/kernels/triangleMeshExternalContactHandler.cpp:78` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/contact/sampled_penalty/kernels/triangleMeshExternalContactHandler.cpp#2` | `src/core/contact/sampled_penalty/kernels/triangleMeshExternalContactHandler.cpp:112` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/contact/sampled_penalty/kernels/triangleMeshExternalContactHandler.cpp#3` | `src/core/contact/sampled_penalty/kernels/triangleMeshExternalContactHandler.cpp:437` | `int` | `default` | TLS/combinable local state | `tls-coupled` | loop owns or consumes TLS/combinable state that must migrate with its merge lifecycle |
| `src/core/contact/sampled_penalty/kernels/triangleMeshExternalContactHandler.cpp#4` | `src/core/contact/sampled_penalty/kernels/triangleMeshExternalContactHandler.cpp:628` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/contact/sampled_penalty/kernels/triangleMeshSelfContactDetection.cpp#1` | `src/core/contact/sampled_penalty/kernels/triangleMeshSelfContactDetection.cpp:84` | `size_t` | `default` | none observed | `typed-index` | size_t range cannot be passed to the int-only V1 API without narrowing |
| `src/core/contact/sampled_penalty/kernels/triangleMeshSelfContactDetection.cpp#2` | `src/core/contact/sampled_penalty/kernels/triangleMeshSelfContactDetection.cpp:109` | `size_t` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/contact/sampled_penalty/kernels/triangleMeshSelfContactHandler.cpp#1` | `src/core/contact/sampled_penalty/kernels/triangleMeshSelfContactHandler.cpp:115` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/contact/sampled_penalty/kernels/triangleMeshSelfContactHandler.cpp#2` | `src/core/contact/sampled_penalty/kernels/triangleMeshSelfContactHandler.cpp:149` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/contact/sampled_penalty/kernels/triangleMeshSelfContactHandler.cpp#3` | `src/core/contact/sampled_penalty/kernels/triangleMeshSelfContactHandler.cpp:523` | `int` | `default` | TLS/combinable local state | `tls-coupled` | loop owns or consumes TLS/combinable state that must migrate with its merge lifecycle |
| `src/core/contact/sampled_penalty/kernels/triangleMeshSelfContactHandler.cpp#4` | `src/core/contact/sampled_penalty/kernels/triangleMeshSelfContactHandler.cpp:798` | `size_t` | `static` | TLS/combinable local state | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/contact/sampled_penalty/kernels/triangleMeshSelfContactHandler.cpp#5` | `src/core/contact/sampled_penalty/kernels/triangleMeshSelfContactHandler.cpp:861` | `size_t` | `static` | TLS/combinable local state | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/contact/sampled_penalty/kernels/triangleMeshSelfContactHandler.cpp#6` | `src/core/contact/sampled_penalty/kernels/triangleMeshSelfContactHandler.cpp:917` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/contact/surfaceDofMap.cpp#1` | `src/core/contact/surfaceDofMap.cpp:91` | `Eigen::Index` | `default` | none observed | `typed-index` | Eigen::Index range cannot be passed to the int-only V1 API without narrowing |
| `src/core/contact/surfaceDofMap.cpp#2` | `src/core/contact/surfaceDofMap.cpp:152` | `Eigen::Index` | `default` | none observed | `typed-index` | Eigen::Index range cannot be passed to the int-only V1 API without narrowing |
| `src/core/contact/surfaceDofMap.cpp#3` | `src/core/contact/surfaceDofMap.cpp:258` | `Eigen::Index` | `default` | none observed | `typed-index` | Eigen::Index range cannot be passed to the int-only V1 API without narrowing |
| `src/core/contact/surfaceDofMap.cpp#4` | `src/core/contact/surfaceDofMap.cpp:367` | `Eigen::Index` | `default` | TLS/combinable local state | `typed-index` | Eigen::Index range cannot be passed to the int-only V1 API without narrowing |
| `src/core/contact/surfaceDofMap.cpp#5` | `src/core/contact/surfaceDofMap.cpp:496` | `Eigen::Index` | `default` | none observed | `typed-index` | Eigen::Index range cannot be passed to the int-only V1 API without narrowing |
| `src/core/contact/surfaceDofMap.cpp#6` | `src/core/contact/surfaceDofMap.cpp:515` | `Eigen::Index` | `default` | none observed | `typed-index` | Eigen::Index range cannot be passed to the int-only V1 API without narrowing |
| `src/core/eigenSupport/EigenSupport.cpp#1` | `src/core/eigenSupport/EigenSupport.cpp:406` | `ES::IDX/IDX` | `default` | TLS/combinable local state | `typed-index` | ES::IDX/IDX range cannot be passed to the int-only V1 API without narrowing |
| `src/core/eigenSupport/EigenSupport.cpp#2` | `src/core/eigenSupport/EigenSupport.cpp:820` | `Eigen::Index` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/eigenSupport/EigenSupport.cpp#3` | `src/core/eigenSupport/EigenSupport.cpp:1180` | `Eigen::Index` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/eigenSupport/EigenSupport.cpp#4` | `src/core/eigenSupport/EigenSupport.cpp:1197` | `Eigen::Index` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/eigenSupport/EigenSupport.cpp#5` | `src/core/eigenSupport/EigenSupport.cpp:1218` | `Eigen::Index` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/eigenSupport/EigenSupport.cpp#6` | `src/core/eigenSupport/EigenSupport.cpp:1234` | `ES::IDX/IDX` | `default` | none observed | `typed-index` | ES::IDX/IDX range cannot be passed to the int-only V1 API without narrowing |
| `src/core/eigenSupport/EigenSupport.cpp#7` | `src/core/eigenSupport/EigenSupport.cpp:1250` | `Eigen::Index` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/eigenSupport/EigenSupport.cpp#8` | `src/core/eigenSupport/EigenSupport.cpp:1270` | `ES::IDX/IDX` | `default` | none observed | `typed-index` | ES::IDX/IDX range cannot be passed to the int-only V1 API without narrowing |
| `src/core/geometryPotentialEnergies/centerOfMassMatchingEnergy.cpp#1` | `src/core/geometryPotentialEnergies/centerOfMassMatchingEnergy.cpp:99` | `int` | `default` | TLS/combinable local state | `tls-coupled` | loop owns or consumes TLS/combinable state that must migrate with its merge lifecycle |
| `src/core/geometryPotentialEnergies/centerOfMassMatchingEnergy.cpp#2` | `src/core/geometryPotentialEnergies/centerOfMassMatchingEnergy.cpp:135` | `int` | `default` | TLS/combinable local state | `tls-coupled` | loop owns or consumes TLS/combinable state that must migrate with its merge lifecycle |
| `src/core/geometryPotentialEnergies/centerOfMassMatchingEnergy.cpp#3` | `src/core/geometryPotentialEnergies/centerOfMassMatchingEnergy.cpp:260` | `int` | `default` | TLS/combinable local state | `tls-coupled` | loop owns or consumes TLS/combinable state that must migrate with its merge lifecycle |
| `src/core/geometryPotentialEnergies/smoothRSEnergy.cpp#1` | `src/core/geometryPotentialEnergies/smoothRSEnergy.cpp:428` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/geometryPotentialEnergies/smoothRSEnergy.cpp#2` | `src/core/geometryPotentialEnergies/smoothRSEnergy.cpp:497` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/geometryPotentialEnergies/smoothRSEnergy.cpp#3` | `src/core/geometryPotentialEnergies/smoothRSEnergy.cpp:821` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/interpolationCoordinates/GreenCoordinates.cpp#1` | `src/core/interpolationCoordinates/GreenCoordinates.cpp:74` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/interpolationCoordinates/GreenCoordinates.cpp#2` | `src/core/interpolationCoordinates/GreenCoordinates.cpp:123` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/interpolationCoordinates/meanValueCoordinates.cpp#1` | `src/core/interpolationCoordinates/meanValueCoordinates.cpp:52` | `int` | `static` | chunk-local scratch | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/interpolationCoordinates/meanValueCoordinates.cpp#2` | `src/core/interpolationCoordinates/meanValueCoordinates.cpp:143` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/mesh/boundingVolumeTree.cpp#1` | `src/core/mesh/boundingVolumeTree.cpp:594` | `int` | `default` | TLS/combinable local state | `tls-coupled` | loop owns or consumes TLS/combinable state that must migrate with its merge lifecycle |
| `src/core/mesh/triMeshPseudoNormal.cpp#1` | `src/core/mesh/triMeshPseudoNormal.cpp:162` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/mesh/triMeshSampler.cpp#1` | `src/core/mesh/triMeshSampler.cpp:91` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/mesh/triMeshSampler.cpp#2` | `src/core/mesh/triMeshSampler.cpp:109` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/nonlinearOptimization/energy/energySet.cpp#1` | `src/core/nonlinearOptimization/energy/energySet.cpp:115` | `Eigen::Index` | `default` | none observed | `typed-index` | Eigen::Index range cannot be passed to the int-only V1 API without narrowing |
| `src/core/nonlinearOptimization/energy/energySet.cpp#2` | `src/core/nonlinearOptimization/energy/energySet.cpp:152` | `Eigen::Index` | `default` | none observed | `typed-index` | Eigen::Index range cannot be passed to the int-only V1 API without narrowing |
| `src/core/nonlinearOptimization/energy/energySet.cpp#3` | `src/core/nonlinearOptimization/energy/energySet.cpp:167` | `Eigen::Index` | `default` | none observed | `typed-index` | Eigen::Index range cannot be passed to the int-only V1 API without narrowing |
| `src/core/nonlinearOptimization/energy/energySet.cpp#4` | `src/core/nonlinearOptimization/energy/energySet.cpp:205` | `Eigen::Index` | `default` | none observed | `typed-index` | Eigen::Index range cannot be passed to the int-only V1 API without narrowing |
| `src/core/nonlinearOptimization/energy/energySet.cpp#5` | `src/core/nonlinearOptimization/energy/energySet.cpp:225` | `Eigen::Index` | `default` | none observed | `typed-index` | Eigen::Index range cannot be passed to the int-only V1 API without narrowing |
| `src/core/nonlinearOptimization/energy/energySet.cpp#6` | `src/core/nonlinearOptimization/energy/energySet.cpp:262` | `Eigen::Index` | `default` | none observed | `typed-index` | Eigen::Index range cannot be passed to the int-only V1 API without narrowing |
| `src/core/nonlinearOptimization/energy/energySet.cpp#7` | `src/core/nonlinearOptimization/energy/energySet.cpp:288` | `Eigen::Index` | `default` | none observed | `typed-index` | Eigen::Index range cannot be passed to the int-only V1 API without narrowing |
| `src/core/nonlinearOptimization/energy/energySet.cpp#8` | `src/core/nonlinearOptimization/energy/energySet.cpp:309` | `std::size_t` | `default` | none observed | `typed-index` | std::size_t range cannot be passed to the int-only V1 API without narrowing |
| `src/core/solidDeformationModel/constraints/segmentBinormalConstraintFunctions.cpp#1` | `src/core/solidDeformationModel/constraints/segmentBinormalConstraintFunctions.cpp:172` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/solidDeformationModel/constraints/segmentBinormalConstraintFunctions.cpp#2` | `src/core/solidDeformationModel/constraints/segmentBinormalConstraintFunctions.cpp:200` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/solidDeformationModel/constraints/segmentBinormalConstraintFunctions.cpp#3` | `src/core/solidDeformationModel/constraints/segmentBinormalConstraintFunctions.cpp:248` | `int` | `static` | shared state protected by existing concurrency primitive | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/solidDeformationModel/constraints/segmentChainConstraintFunctions.cpp#1` | `src/core/solidDeformationModel/constraints/segmentChainConstraintFunctions.cpp:108` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/solidDeformationModel/constraints/segmentChainConstraintFunctions.cpp#2` | `src/core/solidDeformationModel/constraints/segmentChainConstraintFunctions.cpp:121` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/solidDeformationModel/constraints/segmentChainConstraintFunctions.cpp#3` | `src/core/solidDeformationModel/constraints/segmentChainConstraintFunctions.cpp:140` | `int` | `static` | shared state protected by existing concurrency primitive | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/solidDeformationModel/formulations/dof/dofLayout.cpp#1` | `src/core/solidDeformationModel/formulations/dof/dofLayout.cpp:78` | `std::size_t` | `default` | none observed | `typed-index` | std::size_t range cannot be passed to the int-only V1 API without narrowing |
| `src/core/solidDeformationModel/simulation/generateTetMeshMatrix.cpp#1` | `src/core/solidDeformationModel/simulation/generateTetMeshMatrix.cpp:83` | `int` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/solidDeformationModel/simulation/generateTetMeshMatrix.cpp#2` | `src/core/solidDeformationModel/simulation/generateTetMeshMatrix.cpp:129` | `size_t` | `static` | none observed | `partitioner` | explicit static partitioner is not represented by pgo::parallel V1 |
| `src/core/solidDeformationModel/simulation/generateTetMeshMatrix.cpp#3` | `src/core/solidDeformationModel/simulation/generateTetMeshMatrix.cpp:141` | `int` | `default` | TLS/combinable local state | `tls-coupled` | loop owns or consumes TLS/combinable state that must migrate with its merge lifecycle |
| `src/core/solidDeformationModel/simulation/tetMeshOccupation.cpp#1` | `src/core/solidDeformationModel/simulation/tetMeshOccupation.cpp:42` | `int` | `default` | shared RNG requires per-worker or per-index state | `tls-coupled` | shared std::mt19937 state is not concurrency-safe; migrate only with explicit local RNG ownership |
| `src/core/solidDeformationModel/simulation/tetMeshOccupation.cpp#2` | `src/core/solidDeformationModel/simulation/tetMeshOccupation.cpp:120` | `int` | `default` | shared RNG requires per-worker or per-index state | `tls-coupled` | shared std::mt19937 state is not concurrency-safe; migrate only with explicit local RNG ownership |

## Guard update protocol

When a call is migrated or deliberately deferred differently, update the JSON entry in the same
change. A new production `tbb::parallel_for` must first be classified and added to the manifest;
otherwise `tests/check_tbb_parallel_for_inventory.py` fails. Tests, benchmarks, and the
`src/core/parallelism` backend are outside this guard's production scan scope.

