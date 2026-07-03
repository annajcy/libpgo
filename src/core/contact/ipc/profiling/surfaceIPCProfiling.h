#pragma once

#include <string_view>

namespace pgo::Contact::SurfaceIPCProfileSections
{

inline constexpr std::string_view kPairBuildStatic = "contact.surface.pair_build.static";
inline constexpr std::string_view kPairBuildSwept = "contact.surface.pair_build.swept";
inline constexpr std::string_view kPairBuildSelfAABB = "contact.surface.pair_build.self_aabb";
inline constexpr std::string_view kPairBuildSelfPTHashQuery = "contact.surface.pair_build.self_pt_hash_query";
inline constexpr std::string_view kPairBuildSelfPTHashInsert = "contact.surface.pair_build.self_pt_hash_insert";
inline constexpr std::string_view kPairBuildSelfPTQuery = "contact.surface.pair_build.self_pt_query";
inline constexpr std::string_view kPairBuildSelfPTHashCandidates = "contact.surface.pair_build.self_pt.hash_candidates";
inline constexpr std::string_view kPairBuildSelfPTDistanceTests = "contact.surface.pair_build.self_pt.distance_tests";
inline constexpr std::string_view kPairBuildSelfPTAcceptedPairs = "contact.surface.pair_build.self_pt.accepted_pairs";
inline constexpr std::string_view kPairBuildSelfEEHashQuery = "contact.surface.pair_build.self_ee_hash_query";
inline constexpr std::string_view kPairBuildSelfEEHashInsert = "contact.surface.pair_build.self_ee_hash_insert";
inline constexpr std::string_view kPairBuildSelfEEQuery = "contact.surface.pair_build.self_ee_query";
inline constexpr std::string_view kPairBuildSelfEEHashCandidates = "contact.surface.pair_build.self_ee.hash_candidates";
inline constexpr std::string_view kPairBuildSelfEEDistanceTests = "contact.surface.pair_build.self_ee.distance_tests";
inline constexpr std::string_view kPairBuildSelfEEAcceptedPairs = "contact.surface.pair_build.self_ee.accepted_pairs";
inline constexpr std::string_view kPairBuildExternal = "contact.surface.pair_build.external";
inline constexpr std::string_view kPairBuildExternalAABB = "contact.surface.pair_build.external_aabb";
inline constexpr std::string_view kPairBuildExternalPT = "contact.surface.pair_build.external_pt";
inline constexpr std::string_view kPairBuildExternalTP = "contact.surface.pair_build.external_tp";
inline constexpr std::string_view kPairBuildExternalEE = "contact.surface.pair_build.external_ee";
inline constexpr std::string_view kPairBuildExternalOverlappingObstacles = "contact.surface.pair_build.external.overlapping_obstacles";
inline constexpr std::string_view kPairBuildExternalPTHashCandidates = "contact.surface.pair_build.external_pt.hash_candidates";
inline constexpr std::string_view kPairBuildExternalPTDistanceTests = "contact.surface.pair_build.external_pt.distance_tests";
inline constexpr std::string_view kPairBuildExternalPTAcceptedPairs = "contact.surface.pair_build.external_pt.accepted_pairs";
inline constexpr std::string_view kPairBuildExternalTPHashCandidates = "contact.surface.pair_build.external_tp.hash_candidates";
inline constexpr std::string_view kPairBuildExternalTPDistanceTests = "contact.surface.pair_build.external_tp.distance_tests";
inline constexpr std::string_view kPairBuildExternalTPAcceptedPairs = "contact.surface.pair_build.external_tp.accepted_pairs";
inline constexpr std::string_view kPairBuildExternalEEHashCandidates = "contact.surface.pair_build.external_ee.hash_candidates";
inline constexpr std::string_view kPairBuildExternalEEDistanceTests = "contact.surface.pair_build.external_ee.distance_tests";
inline constexpr std::string_view kPairBuildExternalEEAcceptedPairs = "contact.surface.pair_build.external_ee.accepted_pairs";
inline constexpr std::string_view kMaxStepPT = "contact.surface.max_step_pt";
inline constexpr std::string_view kMaxStepEE = "contact.surface.max_step_ee";
inline constexpr std::string_view kMaxStepSelfPTHashCandidates = "contact.surface.max_step.self_pt.hash_candidates";
inline constexpr std::string_view kMaxStepSelfPTCCDTests = "contact.surface.max_step.self_pt.ccd_tests";
inline constexpr std::string_view kMaxStepSelfEEHashCandidates = "contact.surface.max_step.self_ee.hash_candidates";
inline constexpr std::string_view kMaxStepSelfEECCDTests = "contact.surface.max_step.self_ee.ccd_tests";
inline constexpr std::string_view kMaxStepExternalOverlappingObstacles = "contact.surface.max_step.external.overlapping_obstacles";
inline constexpr std::string_view kMaxStepExternalPTHashCandidates = "contact.surface.max_step.external_pt.hash_candidates";
inline constexpr std::string_view kMaxStepExternalPTCCDTests = "contact.surface.max_step.external_pt.ccd_tests";
inline constexpr std::string_view kMaxStepExternalTPHashCandidates = "contact.surface.max_step.external_tp.hash_candidates";
inline constexpr std::string_view kMaxStepExternalTPCCDTests = "contact.surface.max_step.external_tp.ccd_tests";
inline constexpr std::string_view kMaxStepExternalEEHashCandidates = "contact.surface.max_step.external_ee.hash_candidates";
inline constexpr std::string_view kMaxStepExternalEECCDTests = "contact.surface.max_step.external_ee.ccd_tests";
inline constexpr std::string_view kEnergy = "contact.surface.energy";
inline constexpr std::string_view kGradient = "contact.surface.gradient";
inline constexpr std::string_view kHessian = "contact.surface.hessian";
inline constexpr std::string_view kCombined = "contact.surface.combined";
inline constexpr std::string_view kBuildActiveSet = "contact.surface.build_active_set";
inline constexpr std::string_view kActiveSetEnergy = "contact.surface.active_set_energy";
inline constexpr std::string_view kActiveSetGradient = "contact.surface.active_set_gradient";
inline constexpr std::string_view kActiveSetHessian = "contact.surface.active_set_hessian";
inline constexpr std::string_view kActiveSetCombined = "contact.surface.active_set_combined";
inline constexpr std::string_view kActiveSetSelfCombined = "contact.surface.active_set_combined.self";
inline constexpr std::string_view kActiveSetSelfPTCombined = "contact.surface.active_set_combined.self_pt";
inline constexpr std::string_view kActiveSetSelfEECombined = "contact.surface.active_set_combined.self_ee";
inline constexpr std::string_view kActiveSetSelfTripletAlloc =
  "contact.surface.active_set_combined.self.triplet_alloc";
inline constexpr std::string_view kActiveSetSelfSetFromTriplets =
  "contact.surface.active_set_combined.self.set_from_triplets";
inline constexpr std::string_view kActiveSetSelfDirectRowAssembly =
  "contact.surface.active_set_combined.self.direct_row_assembly";
inline constexpr std::string_view kActiveSetSelfThreadRowMerge =
  "contact.surface.active_set_combined.self.thread_row_merge";
inline constexpr std::string_view kActiveSetSelfRowSortReduce =
  "contact.surface.active_set_combined.self.row_sort_reduce";
inline constexpr std::string_view kActiveSetSelfDirectSparseFill =
  "contact.surface.active_set_combined.self.direct_sparse_fill";
inline constexpr std::string_view kActiveSetSelfPTPairCount = "contact.surface.active_set_combined.self_pt.pairs";
inline constexpr std::string_view kActiveSetSelfEEPairCount = "contact.surface.active_set_combined.self_ee.pairs";
inline constexpr std::string_view kActiveSetSelfTripletSlots =
  "contact.surface.active_set_combined.self.triplet_slots";
inline constexpr std::string_view kActiveSetSelfHessianNnz =
  "contact.surface.active_set_combined.self.hessian_nnz";
inline constexpr std::string_view kActiveSetSelfDirectRowContributions =
  "contact.surface.active_set_combined.self.direct_row_contributions";
inline constexpr std::string_view kActiveSetSelfDirectActiveRows =
  "contact.surface.active_set_combined.self.direct_active_rows";
inline constexpr std::string_view kActiveSetExternalCombined = "contact.surface.active_set_combined.external";
inline constexpr std::string_view kActiveSetExternalPTCombined = "contact.surface.active_set_combined.external_pt";
inline constexpr std::string_view kActiveSetExternalTPCombined = "contact.surface.active_set_combined.external_tp";
inline constexpr std::string_view kActiveSetExternalEECombined = "contact.surface.active_set_combined.external_ee";
inline constexpr std::string_view kActiveSetExternalTripletAlloc =
  "contact.surface.active_set_combined.external.triplet_alloc";
inline constexpr std::string_view kActiveSetExternalSetFromTriplets =
  "contact.surface.active_set_combined.external.set_from_triplets";
inline constexpr std::string_view kActiveSetExternalHessianAdd =
  "contact.surface.active_set_combined.external.hessian_add";
inline constexpr std::string_view kActiveSetExternalPTPairCount = "contact.surface.active_set_combined.external_pt.pairs";
inline constexpr std::string_view kActiveSetExternalTPPairCount = "contact.surface.active_set_combined.external_tp.pairs";
inline constexpr std::string_view kActiveSetExternalEEPairCount = "contact.surface.active_set_combined.external_ee.pairs";
inline constexpr std::string_view kActiveSetExternalTripletSlots =
  "contact.surface.active_set_combined.external.triplet_slots";
inline constexpr std::string_view kActiveSetExternalHessianNnz =
  "contact.surface.active_set_combined.external.hessian_nnz";
inline constexpr std::string_view kWrapperSync = "contact.wrapper.sync";
inline constexpr std::string_view kFloorPostPass = "contact.wrapper.floor_post_pass";
inline constexpr std::string_view kAdapterFunc = "contact.adapter.func";
inline constexpr std::string_view kAdapterGradient = "contact.adapter.gradient";
inline constexpr std::string_view kAdapterHessianDirect = "contact.adapter.hessian_direct";
inline constexpr std::string_view kAdapterMaxStep = "contact.adapter.max_step";
inline constexpr std::string_view kAdapterMapToSurface = "contact.adapter.map_to_surface";
inline constexpr std::string_view kAdapterPullbackGradient = "contact.adapter.pullback_gradient";
inline constexpr std::string_view kAdapterPullbackHessian = "contact.adapter.pullback_hessian";
inline constexpr std::string_view kAdapterPullbackHessianValidate = "contact.adapter.pullback_hessian.validate";
inline constexpr std::string_view kAdapterPullbackHessianMultiplySurfaceHessianMap =
  "contact.adapter.pullback_hessian.multiply_surface_hessian_map";
inline constexpr std::string_view kAdapterPullbackHessianRowBuild =
  "contact.adapter.pullback_hessian.row_build";
inline constexpr std::string_view kAdapterPullbackHessianRowMerge =
  "contact.adapter.pullback_hessian.row_merge";
inline constexpr std::string_view kAdapterPullbackHessianWorkspacePrepare =
  "contact.adapter.pullback_hessian.workspace_prepare";
inline constexpr std::string_view kAdapterPullbackHessianOutputFill =
  "contact.adapter.pullback_hessian.output_fill";
inline constexpr std::string_view kAdapterPullbackHessianDirectFillPrepare =
  "contact.adapter.pullback_hessian.direct_fill_prepare";
inline constexpr std::string_view kAdapterPullbackHessianDirectFillValues =
  "contact.adapter.pullback_hessian.direct_fill_values";
inline constexpr std::string_view kAdapterPullbackHessianMapRows = "contact.adapter.pullback_hessian.map_rows";
inline constexpr std::string_view kAdapterPullbackHessianMapCols = "contact.adapter.pullback_hessian.map_cols";
inline constexpr std::string_view kAdapterPullbackHessianMapNnz = "contact.adapter.pullback_hessian.map_nnz";
inline constexpr std::string_view kAdapterPullbackHessianSurfaceHessianNnz =
  "contact.adapter.pullback_hessian.surface_hessian_nnz";
inline constexpr std::string_view kAdapterPullbackHessianTmpNnz = "contact.adapter.pullback_hessian.tmp_nnz";
inline constexpr std::string_view kAdapterPullbackHessianSimulationHessianNnz =
  "contact.adapter.pullback_hessian.simulation_hessian_nnz";
inline constexpr std::string_view kAdapterPullbackHessianMapRowNnzMin =
  "contact.adapter.pullback_hessian.map_row_nnz_min";
inline constexpr std::string_view kAdapterPullbackHessianMapRowNnzMax =
  "contact.adapter.pullback_hessian.map_row_nnz_max";
inline constexpr std::string_view kAdapterPullbackHessianMapRowNnzTotal =
  "contact.adapter.pullback_hessian.map_row_nnz_total";
inline constexpr std::string_view kAdapterPullbackHessianMapRowNnzNonzeroRows =
  "contact.adapter.pullback_hessian.map_row_nnz_nonzero_rows";
inline constexpr std::string_view kAdapterPullbackHessianEnabled =
  "contact.adapter.pullback_hessian.enabled";
inline constexpr std::string_view kAdapterPullbackHessianOutputNnz =
  "contact.adapter.pullback_hessian.output_nnz";
inline constexpr std::string_view kAdapterPullbackHessianContributionCount =
  "contact.adapter.pullback_hessian.contribution_count";
inline constexpr std::string_view kAdapterPullbackHessianActiveOutputRows =
  "contact.adapter.pullback_hessian.active_output_rows";
inline constexpr std::string_view kAdapterPullbackHessianMergeStreamCount =
  "contact.adapter.pullback_hessian.merge_stream_count";
inline constexpr std::string_view kAdapterPullbackHessianMergeOutputNnz =
  "contact.adapter.pullback_hessian.merge_output_nnz";
inline constexpr std::string_view kAdapterPullbackHessianWorkspaceReusedRows =
  "contact.adapter.pullback_hessian.workspace_reused_rows";
inline constexpr std::string_view kAdapterPullbackHessianDirectFillEnabled =
  "contact.adapter.pullback_hessian.direct_fill_enabled";
inline constexpr std::string_view kAdapterPullbackHessianDirectFillNnz =
  "contact.adapter.pullback_hessian.direct_fill_nnz";
inline constexpr std::string_view kAdapterPullbackHessianDirectFillRows =
  "contact.adapter.pullback_hessian.direct_fill_rows";

}  // namespace pgo::Contact::SurfaceIPCProfileSections
