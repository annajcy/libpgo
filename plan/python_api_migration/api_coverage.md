# Python API 覆盖矩阵

## 目的

本文档把当前 `libpgo` / `runIPCSim` / 旧 Python wrapper 曾经暴露的能力，映射到未来 Python-first API。它用于决定：

- 哪些能力必须恢复。
- 哪些能力应该先重构 C++ 边界再绑定。
- 每项能力属于哪个 milestone。
- 应该用哪些 Python parity tests 锁定行为。

## 模块边界

未来公开 API 先按以下模块组织：

| Python 模块 | 职责 |
| --- | --- |
| `pypgo.mesh_geo` | geometry-only mesh：`TriMeshGeo`、`VolumeMeshGeo`、`TetMeshGeo`、`CubicMeshGeo`、拓扑/几何数组访问 |
| `pypgo.mesh` | M1 提供 `MaterialSpec`；后续提供 FEM/simulation mesh：带材料/region/set 的 volume mesh、由 `mesh_geo + material spec` 构建 |
| `pypgo.sparse` | 稀疏矩阵对象、COO/CSR/PyTorch sparse 转换、矩阵向量乘 |
| `pypgo.fem` | tet/cubic/shell FEM 常用矩阵、质量矩阵、嵌入矩阵、插值权重 |
| `pypgo.geometry` | 最近点、barycentric、距离查询、BVH 包装 |
| `pypgo.energy` | PotentialEnergy、组合能量、材料能量、约束能量 |
| `pypgo.solver` | Newton solve、future minimize、solver 参数、结果和 diagnostics |
| `pypgo.contact` | IPC、floor、obstacle、sampled penalty contact energy |
| `pypgo.sim` | config/context/static solve/dynamic loop/output/session |
| `pypgo.io` | OBJ/VEG/state/stress/Alembic 等文件 I/O 的 canonical 入口 |
| `pypgo.torch` | PyTorch tensor/sparse/autograd adapter |

## 覆盖矩阵

### Package 和构建

| 当前能力 | 当前 owner | 未来 Python API | Milestone | Parity test | C++ boundary quality | C++ refactor plan |
| --- | --- | --- | --- | --- | --- | --- |
| Python package scaffold | `pypgo/__init__.py` | `import pypgo` | M0 | `tests/pypgo/test_package_scaffold.py` | bind-ready | 无 |
| 私有 native binding 入口 | `src/python/pypgo/CMakeLists.txt` | `import pypgo._core` | M1 | `tests/pypgo/test_core_import.py` | needs facade | 新增小型 binding TUs，不恢复单文件 wrapper |
| 旧 C-style Python API 已移除 | 已删除旧 wrapper | 不提供 `pypgo.legacy` | M0 | `tests/pypgo/test_package_scaffold.py` | bind-ready | 无 |

### Mesh Geo、Simulation Mesh 和 I/O

模块边界约定：

- `pypgo.mesh_geo` 负责 geometry-only 对象、拓扑和数组访问。它不表达材料、region、set、FEM model 或 solver state。
- `pypgo.mesh` 保留给 FEM/simulation mesh。M1 只先提供 `MaterialSpec`，因为 `.veg` 文件自带 material 信息；后续 simulation mesh 应由 `pypgo.mesh_geo.VolumeMeshGeo` 加 `pypgo.mesh.MaterialSpec` 构建，而不是从裸数组直接构建第二套几何真值。
- 文件格式解析和写出统一放在 `pypgo.io`。M1 使用显式 geometry I/O：`.veg` 使用 `pypgo.io.read_veg_geo(path) -> (volume_geo, material_spec)` / `pypgo.io.write_veg_geo(path, volume_geo, material_spec=None)`，OBJ 使用 `pypgo.io.read_obj_geo(path)` / `pypgo.io.write_obj_geo(path, surface_geo)`。
- 后续如果要提供完整 `.veg` model I/O，必须先决定 `read_veg` 是返回 geometry-only、full simulation model，还是拆成 `read_veg_geo` / `read_veg_model`；M1/M2 不引入 ambiguous `read_veg`。

| 当前能力 | 当前 owner | 未来 Python API | Milestone | Parity test | C++ boundary quality | C++ refactor plan |
| --- | --- | --- | --- | --- | --- | --- |
| 共享 volume geometry storage | 新增 `pgo::Mesh::VolumeMeshGeo` | private `_core.VolumeMeshGeoCore` / public `pypgo.mesh_geo.VolumeMeshGeo` | M1 | `tests/src/core/mesh/volumeMeshGeo_gtest.cpp`；`tests/pypgo/test_mesh_geo.py::test_volume_mesh_geo_constructs_from_tets` | needs refactor | 新增 geometry truth：flat cells + explicit `VolumeCellType`，tet/cubic façade 都组合它 |
| `TetMeshGeo::tets()` 读访问 | `pgo::Mesh::TetMeshGeo` | `pypgo.mesh_geo.TetMeshGeo.tets` | M1 | `tests/src/core/mesh/volumeMeshGeo_gtest.cpp`；`tests/pypgo/test_mesh_geo.py::test_tet_mesh_geo_preserves_tet_api` | API change | `tets()` 改成 `TetCellsView`，call sites 改接 view adapter；保留 `tet(i)` / `tetVtxID(i,j)` |
| 由 NumPy 数组构造三角几何网格 | `pgo::Mesh::TriMeshGeo` / C API `pgo_create_trimeshgeo` | `pypgo.mesh_geo.TriMeshGeo(vertices, faces)` | M1 | `tests/pypgo/test_mesh_geo.py::test_trimesh_geo_constructs_from_numpy_arrays_and_returns_copies` | needs facade | 绑定面只接受 `(n,3)` / `(m,3)`，隐藏 C handle |
| 由 NumPy 数组构造 tet 几何网格 | `pgo::Mesh::TetMeshGeo` / C API `pgo_create_tetmeshgeo` | `pypgo.mesh_geo.VolumeMeshGeo.from_tets(vertices, tets)` / `pypgo.mesh_geo.TetMeshGeo(vertices, tets)` | M1 | `tests/pypgo/test_mesh_geo.py::test_volume_mesh_geo_constructs_from_tets` | needs refactor | `TetMeshGeo` 内部组合 `VolumeMeshGeo`，`tets()` 返回 view adapter |
| 由 NumPy 数组构造 cubic 几何网格 | 新增 `pgo::Mesh::CubicMeshGeo` / `pgo::VolumetricMeshes::CubicMesh` | `pypgo.mesh_geo.VolumeMeshGeo.from_cubes(vertices, cubes)` / `pypgo.mesh_geo.CubicMeshGeo(vertices, cubes)` | M1 | `tests/pypgo/test_mesh_geo.py::test_volume_mesh_geo_constructs_from_cubes` | needs refactor | 新增独立 `CubicMeshGeo`，与 tet 共享 `VolumeMeshGeo` |
| `.veg` material spec | `pgo::VolumetricMeshes::VolumetricMesh::Material` and regions | `pypgo.mesh.MaterialSpec` | M1 | `tests/pypgo/test_io.py::test_read_veg_geo_returns_material_spec` | needs facade | M1 至少保留 common ENu material 和 roundtrip 所需 payload；无法安全表示的 multi-material/region/set 必须明确报错；`mesh_geo` 不承载材料 |
| 从 geometry 构建 FEM/simulation volume mesh | `pgo::VolumetricMeshes::VolumetricMesh`, `TetMesh`, `CubicMesh` | `pypgo.mesh.VolumeMesh.from_geo(geo, material)` | M2/M4 | `tests/pypgo/test_mesh.py::test_volume_mesh_builds_from_geo_and_material` | needs refactor | `VolumetricMesh` 内部几何存储逐步迁移到 `Mesh::VolumeMeshGeo`；材料/region/set 留在 simulation mesh |
| 加载 `.veg` geometry mesh | `pgo::VolumetricMeshes::TetMesh`, `CubicMesh` setup path | `pypgo.io.read_veg_geo(path) -> tuple[pypgo.mesh_geo.VolumeMeshGeo, pypgo.mesh.MaterialSpec]` | M1 | `tests/pypgo/test_io.py::test_read_veg_geo_returns_volume_mesh_geo_and_material_spec` | needs facade | loader 返回 `cell_type`、`vertices`、`cells` 和 material spec；不要假设所有 `.veg` 都是 tet |
| 保存 `.veg` geometry mesh | `pgo::VolumetricMeshes::*::save` / new geometry writer | `pypgo.io.write_veg_geo(path, volume_geo, material_spec=None)` | M1 | `tests/pypgo/test_io.py::test_write_veg_geo_roundtrips_volume_mesh_geo_and_material` | needs facade | `pypgo.io` 负责格式写出；没有 material_spec 时使用明确默认材料或抛出清晰错误 |
| 加载 OBJ surface geometry | `pgo::Mesh::TriMeshGeo::load` | `pypgo.io.read_obj_geo(path) -> pypgo.mesh_geo.TriMeshGeo` | M1 | `tests/pypgo/test_io.py::test_read_obj_geo_returns_trimesh_geo` | bind-ready | 绑定 file loader service，返回 `pypgo.mesh_geo.TriMeshGeo` |
| 保存 OBJ surface geometry | `RunIPCSimOutput::writeSurface` / `TriMeshGeo` 写出路径 | `pypgo.io.write_obj_geo(path, surface_geo)` | M1 | `tests/pypgo/test_io.py::test_write_obj_geo_roundtrips_trimesh_geo` | needs facade | 需要稳定公开 OBJ writer 或复用 `TriMeshGeo` 写出路径；不把 writer 挂到 geometry object 上 |
| 读取/写出 deform state `.u` | `EigenSupport::readMatrix/writeMatrix`, `RunIPCSimOutput` | `pypgo.io.read_state`, `pypgo.io.write_state` | M5 | `tests/pypgo/test_sim_output.py::test_state_io_layout` | bind-ready | 绑定矩阵 I/O service |

### Sparse 和 FEM building blocks

| 当前能力 | 当前 owner | 未来 Python API | Milestone | Parity test | C++ boundary quality | C++ refactor plan |
| --- | --- | --- | --- | --- | --- | --- |
| 稀疏矩阵 nnz/COO 导出 | `EigenSupport::SpMatD`, 旧 wrapper `SparseMatrix` | `SparseMatrix.nnz`, `SparseMatrix.to_coo()` | M2 | `tests/pypgo/test_sparse.py::test_sparse_exports_numpy_coo` | needs facade | 新增 `SparseMatrixHandle` 或 value wrapper，返回 NumPy COO |
| 稀疏矩阵乘向量 | C API `pgo_sp_mv` | `SparseMatrix @ np.ndarray` / `SparseMatrix.matvec(x)` | M2 | `tests/pypgo/test_sparse.py::test_sparse_matvec_matches_scipy` | bind-ready | 使用 Eigen sparse mv adapter |
| 二次型 `0.5 * x^T A x` | C API `pgo_conjugate_mv` | `SparseMatrix.quadratic(x)` | M2 | `tests/pypgo/test_sparse.py::test_sparse_quadratic_matches_numpy` | bind-ready | 使用 Eigen dot adapter |
| Tet Laplacian | `TetMeshMatrix::generateBasicElementLaplacianMatrix` | `pypgo.fem.tet_laplacian(tet, face_neighbor=False, repeat=1, scale=False)` | M2 | `tests/pypgo/test_fem_matrices.py::test_tet_laplacian_exports_coo` | bind-ready | 小型 service function |
| Tet gradient | `TetMeshMatrix::generateGradientMatrix` | `pypgo.fem.tet_gradient(tet)` | M2 | `tests/pypgo/test_fem_matrices.py::test_tet_gradient_shape` | bind-ready | 小型 service function |
| Tet per-element gradient | `TetMeshMatrix::generateElementGradientMatrix` | `pypgo.fem.tet_gradient_per_element(tet)` | M2 | `tests/pypgo/test_fem_matrices.py::test_tet_gradient_per_element_shape` | bind-ready | 返回 NumPy `(num_tets, 9, 12)`，不暴露 Eigen storage order |
| Tet biharmonic gradient | 旧 wrapper `create_tet_biharmonic_gradient_matrix` | `pypgo.fem.tet_biharmonic_gradient(tet, face_neighbor=True, scale=False)` | M2 | `tests/pypgo/test_fem_matrices.py::test_tet_biharmonic_gradient_exports_coo` | needs facade | 将旧 wrapper 中组合逻辑抽成 C++ service，避免 Python 重写 Eigen 乘法 |
| Mass matrix | `generateMassMatrix.*`, `RunIPCSim` setup | `pypgo.fem.mass_matrix(mesh_or_context)` | M4 | `tests/pypgo/test_fem_matrices.py::test_mass_matrix_shape` | needs refactor | 从 setup 中抽出 service，支持 in-memory mesh/context |
| Surface-to-volume embedding | `BarycentricCoordinates::generateInterpolationMatrix`, volume setup | `pypgo.fem.embedding_matrix(surface, volume)` | M4 | `tests/pypgo/test_embedding.py::test_embedding_matches_cpp_baseline` | needs facade | 绑定明确输入/输出的 embedding service |

### Geometry queries

| 当前能力 | 当前 owner | 未来 Python API | Milestone | Parity test | C++ boundary quality | C++ refactor plan |
| --- | --- | --- | --- | --- | --- | --- |
| Triangle mesh closest distance | `TriMeshBVTree::closestTriangleQuery`, C API `pgo_trimesh_closest_distances` | `pypgo.geometry.closest_points(surface, points)` | M2 | `tests/pypgo/test_geometry.py::test_trimesh_closest_distances` | needs facade | 返回 distances、triangle ids，隐藏 BVTree 生命周期 |
| Tet barycentric weights | `TetMeshBVTree`, `getTetBarycentricWeights`, C API `pgo_tetmesh_barycentric_weights` | `pypgo.geometry.tet_barycentric_weights(tet, points)` | M2 | `tests/pypgo/test_geometry.py::test_tet_barycentric_weights` | needs facade | 返回 weights `(n,4)` 和 element ids `(n,)` |
| Mesh remeshing / segmentation | `cgalInterface`, C API 中的未完成接口 | `pypgo.geometry.remesh_isotropic`, `pypgo.geometry.segment_mesh` | M9 | `tests/pypgo/test_geometry_optional.py` | needs refactor | 当前 C API segmentation 未真正实现，后置 |

### Energy 和 solver

| 当前能力 | 当前 owner | 未来 Python API | Milestone | Parity test | C++ boundary quality | C++ refactor plan |
| --- | --- | --- | --- | --- | --- | --- |
| `PotentialEnergy` handle / future nanobind trampoline | `NonlinearOptimization::PotentialEnergy` | `pypgo.energy.PotentialEnergy` | M3/M9+ | `tests/pypgo/test_energy.py::test_builtin_energy_solves`；future: `test_python_defined_energy_solves` | needs small facade | M3 先服务 solver-facing energy；`state_kind` 由 C++ `EnergyStateKind` 映射；完整 Python programming API 和 nanobind trampoline 见 `future_work.md` |
| 多能量组合 | `PotentialEnergies` | `pypgo.energy.EnergySet` | M3 | `tests/pypgo/test_energy.py::test_energy_set_weights_terms` | needs refactor | C++ `PotentialEnergies` 迁移到 constructor-complete `EnergySet`，不暴露 add/init |
| Newton solver | `NewtonSolver` through `optimizationService.h` | `pypgo.solver.solve_newton` | M3 | `tests/pypgo/test_solver.py::test_newton_solves_quadratic__warm_start` | needs facade | C++ 长期边界是 `minimize(problem, x0, NewtonOptions)`；Python 只暴露 `solve_newton`，不暴露 stateful `NewtonSolver` / `SolverParam` |
| General minimize | `EnergyOptimizer::minimize` | future `pypgo.solver.minimize` | post-M3 | future constrained solver tests | needs facade | M3 不绑定；等 bounds/constraints 和 backend-specific options 稳定后再设计，不复用旧宽签名 |
| Solver status/result | `SolverResult`, `SolveStatus`, `solveDiagnostics`, `OptimizationResult` | `pypgo.solver.SolverResult`, `pypgo.solver.SolveStatus`, `pypgo.solver.SolveDiagnostics` | M3 | `tests/pypgo/test_solver.py::test_status_roundtrip_for_all_values` | bind-ready | `SolverResult` 保持 solver 语义；`OptimizationResult` 拥有 `x` 和 final objective；Python result 是 value object |
| Smooth RS energy | `SmoothRSEnergy` behind MKL | `pypgo.energy.SmoothRS` | M9 | optional MKL-gated test | needs refactor | MKL-gated，延后到核心 sim path 之后 |

### Contact 和 IPC

| 当前能力 | 当前 owner | 未来 Python API | Milestone | Parity test | C++ boundary quality | C++ refactor plan |
| --- | --- | --- | --- | --- | --- | --- |
| IPC surface energy | `EmbeddedSurfaceIPCPotentialEnergy` | `pypgo.contact.IPCEnergy` | M3 | `tests/pypgo/test_contact.py::test_ipc_energy_constructs` | needs facade | 通过 `contactEnergyFactory` / `IPCContactEnergy` 构造，Python 只包装统一 energy handle |
| Floor energy | `EmbeddedSurfaceFloorPotentialEnergy` | `pypgo.contact.FloorEnergy` | M3 | `tests/pypgo/test_contact.py::test_floor_energy_constructs` | needs facade | 通过 `contactEnergyFactory::createFloorEnergy` 构造，Python enum/string adapter |
| Floor config parsing | `setup/floorSetup.cpp` | `pypgo.sim.RunSimConfig.floors` | M4 | `tests/pypgo/test_config.py::test_floor_config_parity` | needs facade | 将 floor parser service 化，不暴露 setup internals |
| Moving floor | `IpcFloorMotionState`, `floorHeightAtFrame` | `pypgo.contact.MovingFloor`, `floor.height_at(frame)` | M6 | `tests/pypgo/test_dynamic_sim.py::test_moving_floor_stress` | bind-ready | 绑定 value object + helper |
| Obstacle setup | `setup/obstacleSetup.cpp` | `pypgo.contact.Obstacle`, config loader support | M4/M6 | `tests/pypgo/test_config.py::test_obstacle_config_parity` | needs facade | 抽 obstacle parser/value object |
| Sampled penalty contact energy | `contact/legacy_penalty/*`, `contact/legacyPenaltyContact.cpp` | `pypgo.contact.SampledPenaltyEnergy` / future config `contact_model="sampled_penalty"` | M3/M4-M6 | `tests/pypgo/test_contact.py::test_sampled_penalty_energy_constructs`；future config parity | needs refactor | 重命名 legacy penalty 为 sampled penalty contact model；不暴露 handler classes；config migration 保留 JSON 行为 |

### Run-sim config/context/output

| 当前能力 | 当前 owner | 未来 Python API | Milestone | Parity test | C++ boundary quality | C++ refactor plan |
| --- | --- | --- | --- | --- | --- | --- |
| Runtime config parsing | `parseRunIPCSimRuntimeConfig` | `pypgo.sim.RunSimConfig.from_file(path)` | M4 | `tests/pypgo/test_config.py::test_runtime_config_fields_match_cpp` | bind-ready | 绑定 value object 或重新实现 Python parser 并用 C++ tests 对齐 |
| Path resolution relative to config | `ConfigFileJSON` | `RunSimConfig.resolve_path(key)` | M4 | `tests/pypgo/test_config.py::test_relative_paths_resolve` | bind-ready | 复用 `ConfigFileJSON` 或 Python parser 对齐 |
| Shell IPC setup | `buildShellIpcSimulation` | `pypgo.sim.from_config(shell_path)` | M4 | `tests/pypgo/test_sim_context.py::test_shell_context_builds` | needs facade | 不直接暴露 `IpcSimulationContext` 字段，先加 context handle |
| Tet volume IPC setup | `buildVolumeIpcSimulation` | `pypgo.sim.from_config(tet_path)` | M4 | `tests/pypgo/test_sim_context.py::test_tet_context_builds` | needs facade | context handle |
| Cubic volume IPC setup | `buildVolumeIpcSimulation` | `pypgo.sim.from_config(cubic_path)` | M4 | `tests/pypgo/test_sim_context.py::test_cubic_context_builds` | needs facade | context handle |
| Output path layout | `RunIPCSimOutput` | `pypgo.sim.OutputLayout` | M5 | `tests/pypgo/test_sim_output.py::test_output_paths_match_cli` | bind-ready | 绑定或 Python 重写 path rule |
| State/surface/stress frame writing | `RunIPCSimOutput` | `SimulationFrame.write(output)` | M5/M6 | `tests/pypgo/test_sim_output.py` | needs facade | output service 接受 frame state value object |
| Static solve | `runIPCSimStaticSolve` | `pypgo.sim.solve_static` | M5 | `tests/pypgo/test_static_sim.py` | needs facade | 抽 `runStaticSolve(context, output/options)` service |
| Dynamic loop | `runIPCSimLoop` | `pypgo.sim.DynamicSimulation.run` | M6 | `tests/pypgo/test_dynamic_sim.py` | needs refactor | 抽 per-step API，让 Python 编排 timestep |
| Restart from `.u` | `RunIPCSimOutput::loadLatestRestartState`, `restoreRestartStateIfRequested` | `DynamicSimulation.restore_latest()` | M6 | `tests/pypgo/test_dynamic_sim.py::test_restart_from_u` | bind-ready | 绑定 restart service 或 Python I/O 复现 |
| CLI log/profiling | `RunIPCSimRunScope`, CLI log redirect | `pypgo.tools.run_sim --log`, `RunResult.log_path` | M7 | `tests/pypgo/test_python_cli.py::test_log_flag_writes_log` | needs facade | 保持 C++ logger 初始化为 service |

### Python CLI、batch 和研究生态

| 当前能力 | 当前 owner | 未来 Python API | Milestone | Parity test | C++ boundary quality | C++ refactor plan |
| --- | --- | --- | --- | --- | --- | --- |
| `runIPCSim` executable | `src/tools/sim/runIPCSim` | `pypgo-run-sim`, `python -m pypgo.tools.run_sim` | M7 | `tests/pypgo/test_python_cli.py` | needs facade | Python CLI 调用 `pypgo.sim`，C++ CLI 保留参考 |
| Batch runner | `scripts/run_sim_batch.py`, `examples/ipc/ipc_batch.json` | `pypgo.tools.run_batch` | M7 | `tests/pypgo/test_run_batch.py` | no C++ binding | Python 侧重写，复用 config runner |
| Legacy shell standalone script | 当前主 `runIPCSim --legacy` 不支持 shell legacy | `pypgo.tools.run_legacy_shell_sim` 或独立脚本 | M9 | `tests/pypgo/test_legacy_shell_script.py` | needs refactor | 后续单独设计 legacy sim 脚本；不放进 `pypgo.sim.from_config(..., contact_model="sampled_penalty")` |
| Animation conversion to Alembic | `convertAnimation`, old wrapper `convert_animation_to_abc` | `pypgo.io.convert_animation_to_abc` | M7/M9 | `tests/pypgo/test_animation_io.py` | needs facade | Alembic optional，延后恢复 |
| PyTorch energy value | 无稳定公开 Python-first API | `pypgo.torch.energy_value` | M8 | `tests/pypgo/test_torch_energy.py` | needs facade | 建立 CPU tensor/NumPy bridge + energy adapter |
| PyTorch sparse conversion | 无稳定公开 Python-first API | `SparseMatrix.to_torch_sparse_coo()` | M8 | `tests/pypgo/test_sparse_torch.py` | bind-ready | adapter only，PyTorch lazy import |

## 明确暂缓项

| 能力 | 暂缓原因 | 目标 milestone |
| --- | --- | --- |
| CGAL segmentation | C API 中当前实现未真正输出结果 | M9 |
| Isotropic remeshing | 依赖 CGAL，非 run-sim 主路径 | M9 |
| Smooth RS energy | MKL-gated，非 run-sim 主路径 | M9 |
| Alembic conversion | 依赖可选 animationIO/Alembic，先迁移 simulation runner | M7/M9 |
| Legacy shell simulation | 不进入 Python 主 run-sim 路径；后续单独做 legacy sim 脚本 | M9 |
| Solver-level differentiable simulation | 难度高，先做 energy-level PyTorch autograd | M8 之后 |
| Mitsuba-style Python programming/plugin API | 需要先稳定 Python data/sim API，再决定哪些 C++ abstract base 值得通过 nanobind trampoline 暴露 | M9 之后 |
| Python kernel JIT / Dr.Jit-like tracing backend | 高复杂度 research API；先保证 batch array contract 和 compiled C++ backend 可替换 | M9 之后 |
| User-compiled energy/contact kernels | 需要稳定 kernel ABI、lifetime 和构建/缓存策略 | M9 之后 |

## M0 覆盖结论

- 当前 `runIPCSim` 用户可见主路径已分配到 M4-M7。
- 旧 Python wrapper 曾经覆盖的 mesh/sparse/FEM/solver/contact 能力已分配到 M2-M3。
- NumPy 互操作被前置到 M1/M2。
- `IpcSimulationContext`、dynamic loop、output frame writing 等边界标记为 `needs facade` 或 `needs refactor`，不能直接机械绑定。
