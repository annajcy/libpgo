# `pypgo.tools` — CLI 工具层架构

> 包目录：`pypgo/tools/`（3 个子包，29 个 `.py`）。上级架构见 [../overview.md](../overview.md)。

## 模块职责

把 pypgo 库 API 封装成**可执行命令行工具**，替代历史上的 libpgo C++ tools。每个工具是一个带 `main(argv=None) -> int` 的入口模块，经 `setup.py` 的 `console_scripts` 注册为 `pypgo-*` 命令；也可以 `python -m pypgo.tools.<模块路径>` 直接运行。

工具层**不含物理/几何逻辑**——只做参数解析、配置加载与库函数编排；全部计算在被调用的库模块（[mesh](../mesh/overview.md)、[fem](../fem/overview.md)、[contact](../contact/overview.md)、[animation](../animation/overview.md)…）。

## 子包 ↔ 职责 主表

| 子包 | 职责 | 命令 | 文档 |
|---|---|---|---|
| `tools/mesh/surface/` | 曲面网格处理（清理/质检/重网格化） | `pypgo-surface-{cleanup,quality,remesh}` | [mesh/surface/overview.md](mesh/surface/overview.md) |
| `tools/mesh/volume/` | 体网格生成与转换（tet/cubic/msh/info） | `pypgo-{cubic,tetgen,ftetwild}-mesher`、`pypgo-msh-converter`、`pypgo-volume-info` | [mesh/volume/overview.md](mesh/volume/overview.md) |
| `tools/sim/` | FEM 仿真套件（6 个入口 × JSON 场景配置 + 批量器） | `pypgo-sim-{tet,cubic,shell}-{static,dynamic}`、`pypgo-sim-batch` | [sim/overview.md](sim/overview.md) |
| `tools/animation/` | 动画后处理（Alembic 转换、应力 VDB） | `pypgo-animation-convert`、`pypgo-stress-vdb` | [animation/overview.md](animation/overview.md) |
| `__init__.py` | 命名空间声明 | — | [\_\_init\_\_.md](__init__.md) |

## 典型端到端流水线

```
OBJ 曲面
  │ pypgo-surface-cleanup / pypgo-surface-remesh     （tools/mesh/surface）
  ▼
干净曲面 ── pypgo-tetgen-mesher / pypgo-cubic-mesher ──▶ .veg 体网格（tools/mesh/volume）
  │                                                        │
  └──────────── mesh.surface ────────┐   ┌── mesh.volume ──┘
                                     ▼   ▼
              pypgo-sim-tet-dynamic --config scene.json     （tools/sim）
                                     │
            out/{summary.json, surface/, states/, stress/, animation.abc}
                                     │
       ┌─────────────────────────────┤
       ▼                             ▼
pypgo-stress-vdb（应力 .vdb）   pypgo-animation-convert（.u 序列 → .abc）
```

## 设计约定

- **入口模块极薄**：`tet_static.py` 等仅 12 行——一切共享逻辑在 `_` 前缀私有模块（`_cli.py`/`_config.py`/`_scene.py`/`_runners.py`/`_common.py`），既被多个入口复用又不进公共 API；
- **`main(argv=None) -> int`** 统一签名：测试与 [批量器](sim/batch.md) 直接以 Python 函数调用（不开子进程）；
- 退出码：0 成功；2 参数/配置错误（argparse 约定）；
- 报告输出偏 JSON（`summary.json`、cleanup 报告、quality 报告），方便脚本消费。

## 用法示例

```bash
# 已安装（console_scripts）
pypgo-volume-info assets/bunny.veg

# 等价的模块运行（开发树内）
python -m pypgo.tools.mesh.volume.volume_info assets/bunny.veg
python -m pypgo.tools.sim.tet_static --config examples/sim_configs/tet_static_dragon.json
```

## 交叉链接

- 仿真套件总表与配置 schema：[sim/overview.md](sim/overview.md)
- 示例场景库：`examples/sim_configs/`（[README](../../../../examples/sim_configs/README.md)）
