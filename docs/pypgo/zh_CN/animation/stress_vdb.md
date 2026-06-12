# `pypgo/animation/stress_vdb.py` — von Mises 应力 → OpenVDB 体积序列

> 源文件：`pypgo/animation/stress_vdb.py`（186 行）。模块架构见 [overview.md](overview.md)。
>
> Python 层是 `_core.PyStressFieldVDBExporter` 的薄门面 + 帧范围自动探测；splat 全部在 C++（`src/core/animationIO/stressFieldVDBExporter.cpp`）。需要 OpenVDB build（`PGO_ENABLE_OPENVDB=ON`，绑定层 `PYPGO_HAS_STRESS_VDB`）。

## 输入约定（标准仿真输出布局）

消费 [`tools/sim`](../tools/sim/_runners.md) 开启 `write_states` + `write_stress` 后的产物：

```
{sim_output}/states/deform{frame:04d}.u          # (3n, 1) 位移（.u 格式见 abc.md）
{sim_output}/stress/von_mises{frame:04d}.json    # {"frame", "time", "stress_type", "location", "values": [逐元素]}
```

`values` 长度必须等于 tet 数（C++ 校验）；逐元素 von Mises 由 [`DeformationEnergy.element_von_mises`](../fem/energy.md) 计算。

## func `has_stress_vdb_export()`

```python
has_stress_vdb_export() -> bool
```

本 build 是否可用（编译期 `PYPGO_HAS_STRESS_VDB`）。`StressFieldVDBExporter` 构造时检查，不可用抛 `RuntimeError`。

## class `StressFieldVDBExporter`

把逐 tet 应力 splat 进逐帧 OpenVDB fog volume。四步调用顺序固定：`load_tet_mesh` → `load_deformation_sequence` → `load_von_mises_sequence` → `export_animation_vdb`。

```python
StressFieldVDBExporter()
```

### `load_tet_mesh(veg_path)`

读静止构型 tet 网格（`.veg`）。后续两个序列的尺寸校验（$3n$ 行位移、`numElements` 个应力值）都以它为准。失败抛 `RuntimeError`。

### `load_deformation_sequence(folder, pattern, frame_start, frame_end)`

读逐帧位移 `.u`。`pattern` 是 `fmt` 风格（如 `"deform{:04d}.u"`），帧区间 $[\text{start},\text{end})$ **左闭右开**。as-implemented（C++）：缺帧只 warn 并保持零位移；行数 ≠ $3n$ 报错；多列文件只取第 0 列。

### `load_von_mises_sequence(folder, pattern, frame_start, frame_end)`

读逐帧应力 JSON（须含 `values` 数组，长度 = tet 数）。缺帧同样 warn + 置零。

### `export_animation_vdb(output_dir, prefix="vonMises", voxel_size=0.0)`

写 `{output_dir}/{prefix}{frame:04d}.vdb`（帧号沿用 `frame_start` 起的原始编号）。**as-implemented**（`stressFieldVDBExporter.cpp:185-291`）：

- `voxel_size <= 0` 时自动取 **静止网格平均棱长的一半**：$v=\tfrac12\,\overline{\ell}$（对全部 tet 的 6 条棱求平均）；
- 每帧建一个 `FloatGrid`（背景 0，名 `von_mises`，类别 **GRID_FOG_VOLUME**——是密度体不是 level-set）；
- 对每个 tet：取形变后四角点（rest + u），遍历其包围盒覆盖的体素，对体素中心做精确点-在-四面体判定（`Mesh::pointInTet`，谓词库），命中则写入该 tet 的应力值；
- 共享面/棱的体素可能被多个 tet 命中，取 **max**（保持面上连续观感）；
- 帧间相互独立，位移帧数与应力帧数必须相等。

### 属性 `num_frames`

已加载的帧数（int）。

---

## func `dump_stress_vdb()`

```python
dump_stress_vdb(veg_path, sim_output, output_dir, *,
                prefix="vonMises", voxel_size=0.0,
                frame_start=0, frame_end=-1) -> int
```

一步导出 + 帧范围自动探测。as-implemented（`stress_vdb.py:140-186`）：

1. 校验 `{sim_output}/states/` 与 `{sim_output}/stress/` 存在（否则 `FileNotFoundError`）；
2. `frame_end < 0` 时从 `frame_start` 起逐帧探测，直到 `deform{f:04d}.u` 与 `von_mises{f:04d}.json` **任一**缺失为止（文件名前缀固定为 `deform`/`von_mises`，与 tools/sim 输出一致）；
3. 探测结果为空区间抛 `ValueError`；
4. 走上面四步管线，返回写出的帧数。

CLI 封装：[`pypgo-stress-vdb`](../tools/animation/stress_vdb.md)。

## 用法示例

```python
import pypgo

if pypgo.animation.has_stress_vdb_export():
    n = pypgo.animation.dump_stress_vdb(
        "assets/bunny.veg",
        "out/bunny_drop",          # 含 states/ 与 stress/
        "out/bunny_drop/vdb",
        voxel_size=0.0,            # 自动 = 0.5 × 平均棱长
    )
    print(f"wrote {n} frames")
```

得到的 `.vdb` 序列在 Houdini/Blender 中作 volume 加载，按值域映射颜色即可做应力热图动画。

## 交叉链接

- 输入文件的生产者：[../tools/sim/_runners.md](../tools/sim/_runners.md)（`write_states`/`write_stress`）
- `.u` 格式：[abc.md](abc.md)
- 同一 JSON 序列的统计：[stress_stats.md](stress_stats.md)
- 几何 level-set 用途的 OpenVDB（与本文件的 fog volume 相区分）：[../implicit/extract.md](../implicit/extract.md)
- CLI：[../tools/animation/stress_vdb.md](../tools/animation/stress_vdb.md)
