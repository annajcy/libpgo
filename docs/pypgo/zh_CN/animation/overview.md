# `pypgo.animation` — 动画 I/O 模块架构

> 包目录：`pypgo/animation/`（4 个文件）。上级架构见 [../overview.md](../overview.md)。

## 模块职责

把仿真产物（逐帧位移、逐元素应力）变成下游 DCC/渲染工具能消费的格式：

1. **Eigen 二进制位移 I/O**（`.u` 文件）——仿真状态的原生交换格式；
2. **Alembic 导出**（`.abc`）——逐帧曲面动画，进 Houdini/Maya/Blender；
3. **应力体积可视化**（`.vdb`）——逐四面体 von Mises 应力 splat 成 OpenVDB 体积序列；
4. **应力统计**——逐帧 min/mean/p99/max 汇总（纯 Python/NumPy，无编译依赖）。

两个能力探测函数对应两个**编译期可选依赖**：`has_animation_io()`（Alembic）与 `has_stress_vdb_export()`（OpenVDB）。统计部分永远可用。

## 理论流水线

```
仿真（tools/sim 或自写循环）
 ├── states/deform%04d.u          （write_u_file，Eigen 二进制位移）
 └── stress/von_mises%04d.json    （逐元素 von Mises，element_von_mises）
        │
        ├── abc.py        AbcWriter / AnimationLoader ──→ .abc（Alembic 曲面动画）
        ├── stress_vdb.py StressFieldVDBExporter      ──→ %04d.vdb（OpenVDB 体积序列）
        └── stress_stats.py compute_stress_field_stats ─→ 统计 JSON（min/mean/p99/max…）
```

标准文件布局由 [`pypgo.tools.sim`](../tools/sim/overview.md) 的输出层（[\_outputs/\_runners](../tools/sim/_runners.md)）生成：`{output}/states/deform{frame:04d}.u` + `{output}/stress/von_mises{frame:04d}.json`，本包的 `dump_stress_vdb` / `compute_stress_field_stats` 直接按此约定自动探测帧范围。

## 文件 ↔ 职责 主表

| 文件 | 职责 | 关键符号 | 文档 |
|---|---|---|---|
| `abc.py` | `.u` 二进制 I/O + Alembic 导出 | `read_u_file`、`write_u_file`、`AbcWriter`、`AnimationLoader`、`dump_animation`、`has_animation_io` | [abc.md](abc.md) |
| `stress_vdb.py` | von Mises → OpenVDB 体积序列 | `StressFieldVDBExporter`、`dump_stress_vdb`、`has_stress_vdb_export` | [stress_vdb.md](stress_vdb.md) |
| `stress_stats.py` | 应力序列统计 | `FrameStressStats`、`StressFieldStats`、`compute_stress_field_stats` | [stress_stats.md](stress_stats.md) |
| `__init__.py` | 公开面 | — | [\_\_init\_\_.md](__init__.md) |

## C++ 引擎对应

| Python | C++ | 位置 |
|---|---|---|
| `AbcWriter` → `_core.dump_abc` | `AnimationIO::dumpABC`（Alembic Ogawa 归档） | `src/core/animationIO/abcWriter.cpp` |
| `AnimationLoader` | `AnimationIO::AnimationLoader`（JSON 驱动） | `src/core/animationIO/animationLoader.cpp` |
| `StressFieldVDBExporter` | `AnimationIO::StressFieldVDBExporter` | `src/core/animationIO/stressFieldVDBExporter.cpp` |
| `.u` 格式 | `EigenSupport::readMatrix/writeMatrix` | `src/core/eigenSupport/EigenSupport.h:394-472` |
| `read_u_file`/`write_u_file` | （纯 Python `struct`+NumPy，与上行格式互通） | `pypgo/animation/abc.py` |
| 统计 | （纯 Python/NumPy） | `pypgo/animation/stress_stats.py` |

绑定层：`src/python/pypgo/animation/bindings.cpp` + `core.cpp`（全部调用释放 GIL）；Alembic 未编译时换链 `disabled/core.h` 的 stub（`has_animation_io` 返回 False）。

## 贯穿示例

```python
import pypgo

# 仿真循环内：逐帧写位移（tools/sim 的 write_states 即此格式）
pypgo.animation.write_u_file("out/states/deform0000.u", u.reshape(-1, 1))

# 收尾：曲面动画 → Alembic
if pypgo.animation.has_animation_io():
    with pypgo.animation.AbcWriter("out/anim.abc", "bunny",
                                   rest_positions=rest.ravel(),
                                   triangles=tris.ravel()) as w:
        for u in frames:
            w.add_frame(surface_disp(u))

# 应力体积可视化（需 OpenVDB build）
if pypgo.animation.has_stress_vdb_export():
    pypgo.animation.dump_stress_vdb("bunny.veg", "out/", "out/vdb/")

# 应力统计（永远可用）
stats = pypgo.animation.compute_stress_field_stats("out/stress")
stats.save("out/stress_stats.json")
```

## 交叉链接

- 标准输出布局的生产者：[../tools/sim/overview.md](../tools/sim/overview.md)（`write_states`/`write_stress`/`write_abc`）
- 对应的 CLI 封装：[../tools/animation/overview.md](../tools/animation/overview.md)（`pypgo-animation-convert`、`pypgo-stress-vdb`）
- von Mises 的来源：[../fem/energy.md](../fem/energy.md)（`DeformationEnergy.element_von_mises`）
