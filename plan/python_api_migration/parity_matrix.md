# Run-Sim 迁移 Parity Matrix

## 目的

本文档定义 Python 迁移期间必须保持一致的行为样本。`runIPCSim` C++ CLI 是迁移期 parity oracle；Python runner 必须在代表性场景中匹配它的 config 解析、输出布局、成功/失败语义和关键日志/diagnostics。

## 通用 parity 规则

- 输出目录布局必须保持：
  - `states/deform%04d.u`
  - `surface/ret%04d.obj`
  - `stress/von_mises%04d.json`
  - `runIPCSim.log`
- `restart-from-u=false` 时清理 output root；`restart-from-u=true` 时保留已有 state 并从最近 frame 恢复。
- static mode 只接受 Newton `Converged`；失败时不写 partial static frame。
- dynamic mode 保持当前 timestep 接受策略：`Converged`、`MaxIterations`、`StepTooSmall` 仍按现有 C++ 语义处理。
- Python config loader 必须按 config 文件所在目录解析相对路径。
- 所有 parity tests 优先用小 fixture 或 lite example，避免把全量长仿真放进常规测试。

## Config 和输出基础

| 场景 | 当前覆盖 | Python parity test | Milestone | 成功标准 |
| --- | --- | --- | --- | --- |
| required/optional runtime fields | `RunIPCSimConfigGTest.RuntimeConfigParsesRequiredAndOptionalFields` | `tests/pypgo/test_config.py::test_runtime_config_fields` | M4 | `g`, `init-vel`, `timestep`, `scale`, `solver-*`, `dump-interval`, `output`, optional flags 全部一致 |
| static mode parsing | `RunIPCSimConfigGTest.RuntimeConfigAcceptsStaticMode` | `tests/pypgo/test_config.py::test_static_mode_parses` | M4 | `sim-type=static` 映射到 static enum/string |
| invalid sim-type | `parseRunIPCSimRuntimeConfig` | `tests/pypgo/test_config.py::test_invalid_sim_type_rejected` | M4 | 错误语义和消息方向一致 |
| output path layout | `RunIPCSimOutputGTest.OutputPathsPreserveCurrentLayout` | `tests/pypgo/test_sim_output.py::test_output_layout_paths` | M5 | path names 和四位 frame padding 一致 |
| config relative path resolution | `ConfigFileJSON` tests / fixtures | `tests/pypgo/test_config.py::test_relative_paths_resolve_from_config_dir` | M4 | 相对 mesh/fixed/output 路径按 config 目录解析 |

## Shell IPC

| 场景 | Config / fixture | 当前特性 | Python parity test | Milestone | 成功标准 |
| --- | --- | --- | --- | --- | --- |
| shell hang dynamic | `examples/ipc/shell/shell-hang/shell-ipc.json` | shell IPC, fixed vertices, `ipc-heuristic=true` | `tests/pypgo/test_dynamic_sim.py::test_shell_hang_one_step` | M6 | 写出 frame 0 state/surface，输出布局一致 |
| shell drop dynamic | `examples/ipc/shell/shell-drop/shell-ipc.json` | external obstacle, shell IPC | `tests/pypgo/test_dynamic_sim.py::test_shell_drop_obstacle_setup` | M6 | obstacle context 构建成功，一步运行成功 |
| shell static | generated static config from shell fixture | static Newton solve | `tests/pypgo/test_static_sim.py::test_static_shell_writes_state_and_surface` | M5 | `deform0000.u` 和 `ret0000.obj` 存在 |
| shell floor | generated config with `floors[]` | floor energy/logging | `tests/pypgo/test_dynamic_sim.py::test_shell_floor_one_step` | M6 | floor 创建成功并写出 frame 0 |
| shell ignores legacy contact fields | current C++ test generated config | IPC path ignores legacy contact fields | `tests/pypgo/test_config.py::test_shell_ignores_legacy_contact_fields` | M4 | 默认 backend 仍为 IPC，不误走 penalty |
| shell rejects surface pressure | generated shell config | surface pressure volume-only | `tests/pypgo/test_config.py::test_shell_rejects_surface_pressure` | M4 | Python 与 C++ 同样拒绝 |

## Tet volume IPC

| 场景 | Config / fixture | 当前特性 | Python parity test | Milestone | 成功标准 |
| --- | --- | --- | --- | --- | --- |
| tet box hang dynamic | `examples/ipc/tet/box-hang/box-ipc.json` | tet volume IPC, fixed vertices | `tests/pypgo/test_dynamic_sim.py::test_tet_box_hang_one_step` | M6 | frame 0 state/surface 存在 |
| tet box squash dynamic | `examples/ipc/tet/box-squash/box-ipc.json` | material max-step disabled, von Mises | `tests/pypgo/test_dynamic_sim.py::test_tet_squash_von_mises` | M6 | stress JSON 存在且 schema 匹配 |
| tet static | generated static tet config | static volume solve | `tests/pypgo/test_static_sim.py::test_static_tet_writes_state_and_surface` | M5 | state/surface frame 0 存在 |
| tet zero timestep | generated config `num-timestep=0` | output dirs only | `tests/pypgo/test_dynamic_sim.py::test_tet_zero_timestep_creates_dirs_only` | M6 | dirs 存在，frame 文件不存在 |
| tet one timestep | generated tet config | dynamic one-step smoke | `tests/pypgo/test_dynamic_sim.py::test_tet_one_step_writes_frame_zero` | M6 | frame 0 state/surface 存在，无 flat legacy output |
| tet missing `ipc-dhat`/`ipc-kappa` | generated invalid config | required IPC params | `tests/pypgo/test_config.py::test_tet_missing_ipc_params_fails` | M4 | setup 失败 |
| tet rejects `ipc-heuristic` | generated invalid config | shell-only heuristic | `tests/pypgo/test_config.py::test_tet_rejects_ipc_heuristic` | M4 | setup 失败 |
| tet non-unit scale | generated config | scale applied to volume/surface | `tests/pypgo/test_dynamic_sim.py::test_tet_non_unit_scale_one_step` | M6 | one-step 成功 |
| tet moving upper floor | generated config | moving floor + stress | `tests/pypgo/test_dynamic_sim.py::test_tet_moving_floor_stress` | M6 | frame 1 state/stress 存在 |
| tet surface pressure | generated config | surface pressure force + stress | `tests/pypgo/test_dynamic_sim.py::test_tet_surface_pressure_stress` | M6 | state/surface/stress 存在 |

## Cubic volume IPC

| 场景 | Config / fixture | 当前特性 | Python parity test | Milestone | 成功标准 |
| --- | --- | --- | --- | --- | --- |
| cubic box hang dynamic | `examples/ipc/cubic/box-hang/box-ipc.json` | cubic volume IPC | `tests/pypgo/test_dynamic_sim.py::test_cubic_box_hang_one_step` | M6 | frame 0 state/surface 存在 |
| cubic box squash dynamic | `examples/ipc/cubic/box-squash/box-ipc.json` | squash, material max-step disabled | `tests/pypgo/test_dynamic_sim.py::test_cubic_squash_one_step` | M6 | one-step 成功 |
| cubic static with floor | generated static cubic floor config | floor + static solve | `tests/pypgo/test_static_sim.py::test_static_cubic_floor_writes_state_and_surface` | M5 | state/surface frame 0 存在 |
| cubic one timestep | generated cubic config | dynamic one-step smoke | `tests/pypgo/test_dynamic_sim.py::test_cubic_one_step_writes_frame_zero` | M6 | frame 0 state/surface 存在 |
| cubic floor one timestep | generated cubic floor config | floor backend | `tests/pypgo/test_dynamic_sim.py::test_cubic_floor_one_step` | M6 | frame 0 state/surface 存在 |
| cubic rejects shell-only material | generated invalid config | `koiter-stvk` shell-only | `tests/pypgo/test_config.py::test_cubic_rejects_shell_material` | M4 | setup 失败 |
| cubic non-unit scale | generated config | scale applied | `tests/pypgo/test_dynamic_sim.py::test_cubic_non_unit_scale_one_step` | M6 | one-step 成功 |

## Volume legacy penalty backend

| 场景 | Config / fixture | 当前特性 | Python parity test | Milestone | 成功标准 |
| --- | --- | --- | --- | --- | --- |
| legacy tet dynamic | `tests/fixtures/legacy/tet/box/box.json` | penalty external contact | `tests/pypgo/test_volume_legacy_penalty.py::test_legacy_tet_one_step` | M6 | unified output frame 0 存在 |
| legacy cubic dynamic | `tests/fixtures/legacy/cubic/box/box.json` | penalty external contact | `tests/pypgo/test_volume_legacy_penalty.py::test_legacy_cubic_one_step` | M6 | unified output frame 0 存在 |
| legacy tet static drop without attachment | generated static legacy tet | expected failure | `tests/pypgo/test_static_sim.py::test_static_legacy_drop_without_attachment_fails` | M5 | 返回失败，不写 partial output |
| legacy tet static hang | generated fixed-vertices config | static penalty volume hang | `tests/pypgo/test_static_sim.py::test_static_legacy_tet_hang` | M5 | state/surface frame 0 存在 |
| legacy cubic static hang | generated fixed-vertices config | static penalty cubic hang | `tests/pypgo/test_static_sim.py::test_static_legacy_cubic_hang` | M5 | state/surface frame 0 存在 |

Python 主 `pypgo.sim` 路径只覆盖 volume legacy penalty。Legacy shell 不作为主 parity case；后续单独做 legacy sim Python 脚本时再建立独立 parity matrix。

## Restart、dump interval、日志和 profiling

| 场景 | 当前覆盖 | Python parity test | Milestone | 成功标准 |
| --- | --- | --- | --- | --- |
| default clears output | `DefaultRunClearsOutputAndDoesNotRestartFromDeformState` | `tests/pypgo/test_dynamic_sim.py::test_default_clears_output` | M6 | sentinel 被清除，日志包含清理信息 |
| restart-from-u keeps state | `RestartFromUTrueKeepsExistingDeformState` | `tests/pypgo/test_dynamic_sim.py::test_restart_from_u_keeps_existing_state` | M6 | 已有 state 保留，继续写下一帧 |
| dump interval default | `DeformStateDefaultsToDumpInterval` | `tests/pypgo/test_dynamic_sim.py::test_dump_interval_controls_state_output` | M6 | frame gap 外不写 state/surface |
| dump every frame | `DumpDeformEveryFrameWritesEveryTimestep` | `tests/pypgo/test_dynamic_sim.py::test_dump_deform_every_frame` | M6 | 每个 timestep 写 state，surface 仍按 frame gap |
| `--log` writes output log | `LogFlagWritesCliOutputIntoOutputDirectory` | `tests/pypgo/test_python_cli.py::test_log_flag_writes_output_log` | M7 | `runIPCSim.log` 存在且包含 CLI 输出 |
| profiling summary | `ProfilingConfigWritesSummaryToOutputLog` | `tests/pypgo/test_python_cli.py::test_profiling_summary_in_log` | M7 | log 包含 profiling summary |
| debug max-step summary | debug log tests | `tests/pypgo/test_python_cli.py::test_debug_log_contains_max_step_summary` | M7 | log verbosity 与 C++ 一致 |

## FEM/setup parity

| 场景 | 当前覆盖 | Python parity test | Milestone | 成功标准 |
| --- | --- | --- | --- | --- |
| tet embedding matrix | `TetEmbeddingMatrixMatchesBarycentricBaseline` | `tests/pypgo/test_embedding.py::test_tet_embedding_matrix_matches_baseline` | M4 | sparse pattern/value 与 C++ baseline 一致 |
| cubic embedding matrix | `CubicEmbeddingMatrixMatchesBarycentricBaseline` | `tests/pypgo/test_embedding.py::test_cubic_embedding_matrix_matches_baseline` | M4 | sparse pattern/value 与 C++ baseline 一致 |
| surface pressure projects to simulation DOFs | setup gtests | `tests/pypgo/test_surface_pressure.py::test_surface_pressure_projects_to_simulation_dofs` | M4/M6 | projected force shape/value 合理且与 C++ 一致 |
| floors array validation | setup gtests | `tests/pypgo/test_config.py::test_floors_array_validation` | M4 | empty floors accepted，missing fields rejected |
| multiple floors | setup gtests | `tests/pypgo/test_config.py::test_multiple_floors_create_models` | M4 | 创建多个 floor force models |

## Parity suite 分层

### 快速 smoke 层

每次 Python API 变更都应该运行：

```bash
python -m pytest -q tests/pypgo/test_package_scaffold.py
python -m pytest -q tests/pypgo/test_config.py
python -m pytest -q tests/pypgo/test_mesh.py tests/pypgo/test_sparse.py
```

### 仿真短跑层

实现 M5/M6 后运行：

```bash
python -m pytest -q tests/pypgo/test_static_sim.py
python -m pytest -q tests/pypgo/test_dynamic_sim.py -m "not slow"
python -m pytest -q tests/pypgo/test_volume_legacy_penalty.py
```

### CLI parity 层

实现 M7 后运行：

```bash
python -m pytest -q tests/pypgo/test_python_cli.py
```

## M0 结论

Python 迁移的 parity oracle 不应该只是一两个 smoke case。最小覆盖必须包括：

- shell/tet/cubic 三类模型。
- IPC 和 volume legacy penalty 两类 backend。
- static 和 dynamic 两类模式。
- output layout、restart、dump interval、logging、profiling。
- floor、moving floor、surface pressure、von Mises 等附加能力。

Legacy shell 明确不进入 Python 主 run-sim 路径；它会在后续独立 legacy sim 脚本设计中单独处理。
