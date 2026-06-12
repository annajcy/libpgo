# `pypgo.tools.mesh` — 网格处理 CLI

> 包目录：`pypgo/tools/mesh/`（曲面、体两个子包）。工具层架构见 [../overview.md](../overview.md)；被封装的库见 [../../mesh/overview.md](../../mesh/overview.md)。

## 模块职责

[`pypgo.mesh`](../../mesh/overview.md) 处理算法的命令行封装，按输入/输出网格类型分两组：

- **surface/**：OBJ 进 OBJ 出（清理、质检、重网格化）——为体网格化准备干净输入；
- **volume/**：OBJ/MSH 进 `.veg` 出（tet/cubic 网格化、格式转换、信息摘要）——为 [tools/sim](../sim/overview.md) 准备 `mesh.volume`。

## 子包 ↔ 职责 主表

| 子包 | 命令 | 文档 |
|---|---|---|
| `surface/` | `pypgo-surface-cleanup` / `pypgo-surface-quality` / `pypgo-surface-remesh` | [surface/overview.md](surface/overview.md) |
| `volume/` | `pypgo-cubic-mesher` / `pypgo-tetgen-mesher` / `pypgo-ftetwild-mesher` / `pypgo-msh-converter` / `pypgo-volume-info` | [volume/overview.md](volume/overview.md) |
| `__init__.py` | — | [\_\_init\_\_.md](__init__.md) |

## 典型流水线

```
raw.obj
  │ pypgo-surface-quality（先看问题）
  │ pypgo-surface-cleanup --json report.json（保守修复）
  │ pypgo-surface-remesh -l 1.0（各向同性重网格化）
  ▼
clean.obj ── pypgo-tetgen-mesher / pypgo-ftetwild-mesher / pypgo-cubic-mesher ──▶ mesh.veg
                                                                                  │
                                              pypgo-volume-info mesh.veg ◀───────┘
                                              （再喂给 pypgo-sim-* 的 mesh.volume）
```

## 交叉链接

- 曲面处理库：[../../mesh/processing/surface.md](../../mesh/processing/surface.md)
- 体网格化库：[../../mesh/processing/volume.md](../../mesh/processing/volume.md)
- `.veg` 与材料：[../../mesh/volume/material.md](../../mesh/volume/material.md)
- 下游消费：[../sim/overview.md](../sim/overview.md)
