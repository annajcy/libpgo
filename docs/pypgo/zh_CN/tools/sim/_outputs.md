# `pypgo/tools/sim/_outputs.py` — 输出写出助手（私有）

> 源文件：`pypgo/tools/sim/_outputs.py`（27 行）。模块架构见 [overview.md](overview.md)。
>
> 两个小函数，供 [_runners.md](_runners.md) 调用。`.u` 与应力 JSON 的写出不在此（runner 直接用 [`write_u_file`](../../animation/abc.md) 与 `json.dumps`）。

## func `write_summary(output_dir, payload) -> Path`

把 summary dict 写成 `{output_dir}/summary.json`（`indent=2` + 结尾换行），目录自动创建，返回路径。**每次运行必写**——是测试与 [批量器](batch.md) 判定结果的接口。

## func `write_surface(path, vertices, triangles) -> None`

把 `(m,3)` 顶点 + `(k,3)` 三角形包成 [`TriMeshData`](../../mesh/data.md) 写 OBJ，父目录自动创建。顶点来自 [`SceneBundle.surface_positions`](_scene.md)。

## 输出目录布局（汇总）

一次运行（`output.directory = out/` 为例，全部 flag 打开）：

```
out/
├── summary.json                  # 必写：收敛/逐帧状态
├── final_surface.obj             # static + write_surfaces
├── surface/surface%04d.obj       # dynamic + write_surfaces（按 dump_interval）
├── states/deform_final.u         # static + write_states
├── states/deform%04d.u           # dynamic + write_states
├── stress/von_mises_final.json   # static + write_stress
├── stress/von_mises%04d.json     # dynamic + write_stress
└── animation.abc                 # dynamic + write_abc
```

帧号是 stepper 的 `frame_index`（从 1 开始计步），只有 `frame_index % dump_interval == 0` 的帧落盘。

## 交叉链接

- 调用方与各文件的内容定义：[_runners.md](_runners.md)
- `states/` + `stress/` 的下游：[../../animation/stress_vdb.md](../../animation/stress_vdb.md)、[../../animation/stress_stats.md](../../animation/stress_stats.md)
