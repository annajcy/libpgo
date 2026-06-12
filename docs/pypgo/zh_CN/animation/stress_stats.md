# `pypgo/animation/stress_stats.py` — 应力序列统计

> 源文件：`pypgo/animation/stress_stats.py`（166 行）。模块架构见 [overview.md](overview.md)。
>
> 纯 Python/NumPy，无编译依赖——不需要 OpenVDB/Alembic build 也可用。消费与 [stress_vdb.md](stress_vdb.md) 相同的逐帧应力 JSON 序列。

## class `FrameStressStats`（冻结 dataclass）

单帧统计。对该帧 JSON 的 `values` 数组 $v\in\mathbb R^{N}$（逐元素应力），**as-implemented**（`_compute_frame_stats`，`stress_stats.py:90-106`）：

| 字段 | 定义 | 实现 |
|---|---|---|
| `frame` | 帧号 | JSON `frame` 字段（缺省 0） |
| `time` | 物理时间 | JSON `time` 字段（缺省 0.0） |
| `count` | 元素数 $N$ | `len(v)` |
| `min` / `max` | $\min_i v_i$ / $\max_i v_i$ | `v.min()` / `v.max()` |
| `mean` | $\bar v=\frac1N\sum_i v_i$ | `v.mean()` |
| `stddev` | $\sqrt{\frac1N\sum_i(v_i-\bar v)^2}$ | `v.std()`——**总体标准差**（ddof=0，分母 $N$ 而非 $N-1$） |
| `median` | 0.50 分位 | `np.quantile(v, 0.50)` |
| `p99` | 0.99 分位 | `np.quantile(v, 0.99)`——NumPy 默认**线性插值**法（非最近样本） |

`values` 缺失或为空抛 `ValueError`。

## class `StressFieldStats`（冻结 dataclass）

整个序列的聚合：

| 字段 | 含义 |
|---|---|
| `stress_type` | 取**首帧** JSON 的 `stress_type`（tools/sim 写 `"von_mises"`） |
| `location` | 取首帧 JSON 的 `location`（tools/sim 写 `"element"`） |
| `source_dir` | 输入目录的绝对路径（`resolve()` 后） |
| `prefix` | 文件名前缀 |
| `frame_start` / `frame_end` | 帧区间 $[\text{start},\text{end})$ |
| `frames` | `list[FrameStressStats]`，按帧序 |

### 属性 `num_frames`

`len(frames)`。

### `to_dict()`

转嵌套 dict（顶层元数据 + `frames` 列表，每帧 9 个标量），即输出 JSON 的结构。

### `save(path)`

写 JSON（`indent=2`，结尾换行），父目录自动创建。

---

## func `compute_stress_field_stats()`

```python
compute_stress_field_stats(stress_dir, *, prefix="von_mises",
                           frame_start=0, frame_end=-1) -> StressFieldStats
```

读 `{stress_dir}/{prefix}{frame:04d}.json`（$\text{frame}\in[\text{start},\text{end})$，左闭右开）并逐帧统计。

| 参数 | 含义 |
|---|---|
| `stress_dir` | 应力 JSON 目录；不存在抛 `FileNotFoundError` |
| `prefix` | 文件名前缀，默认 `von_mises`（与 [tools/sim](../tools/sim/_runners.md) 输出一致） |
| `frame_start` | 起始帧（含） |
| `frame_end` | 结束帧（不含）；`-1` 自动探测——从 `frame_start` 起逐帧检查文件存在，到第一个缺失为止 |

探测结果为空区间抛 `ValueError`。`stress_type`/`location` 元数据取首帧。

与 [stress_vdb](stress_vdb.md) 不同，这里**不需要** tet 网格与位移序列——只看标量数组，所以也适用于任何按同一 JSON 约定写出的标量场序列。

## 用法示例

```python
import pypgo

stats = pypgo.animation.compute_stress_field_stats("out/bunny_drop/stress")
print(f"{stats.num_frames} frames, type={stats.stress_type}")

peak = max(s.max for s in stats.frames)
print(f"peak von Mises = {peak:.3e} Pa")

stats.save("out/bunny_drop/stress_stats.json")
```

## 交叉链接

- 输入 JSON 的生产者：[../tools/sim/_runners.md](../tools/sim/_runners.md)（`write_stress`）
- von Mises 的计算：[../fem/energy.md](../fem/energy.md)（`element_von_mises`）
- 同序列的体积可视化：[stress_vdb.md](stress_vdb.md)
