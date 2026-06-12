# `pypgo.implicit` — 惰性隐式曲面场模块架构

> 包目录：`pypgo/implicit/`（5 个文件）。上级架构见 [../overview.md](../overview.md)。

## 模块职责

把几何表示为标量场 $f:\mathbb R^3\to\mathbb R$ 的**隐式曲面**：曲面是零等值面

$$\mathcal S=\{\,\mathbf p\in\mathbb R^3 : f(\mathbf p)=0\,\},\qquad
f(\mathbf p)<0\ \text{内部},\quad f(\mathbf p)>0\ \text{外部}$$

本包提供：

1. **场的统一契约** `ImplicitField`（逐点求值 / 包围盒 / 网格采样 / CSG 组合）；
2. **形状原语**（球、盒、网格无符号距离场）与建立其上的 `thicken_mesh_surface` 加厚流水线；
3. **提取回三角网格**：marching cubes（必有）与 OpenVDB level-set 后端（可选编译依赖）。

典型用途：把开口/薄片三角网格加厚成有体积的闭合曲面，供 [`tet_mesher`/`cubic_mesher`](../mesh/processing/volume.md) 生成体网格；或用 CSG 组合解析形体作障碍物/裁剪几何。

## 理论流水线

```
解析场 SphereField/BoxField ┐
                            ├─ CSG（min/max 组合）/ offset ──┐
网格 → MeshUnsignedDistance ┘                                │
                                                             ▼
            GridSpec（包围盒 × 分辨率） ──→ sample_to_grid ──→ GridField（res³ 标量栅格）
                                                             │
                              ┌──────────────────────────────┤
                              ▼                              ▼
                  extract_marching_cubes          build_openvdb_from_grid_field
                  （isoOffset 等值面）             → smooth → extract_openvdb
                              │                              │
                              └────────→ TriMeshData ←───────┘
```

惰性（lazy）设计：`ImplicitField` 上的 CSG / offset 只组装求值树，**不**做任何采样；只有 `sample_to_grid` 才触发对每个栅格点的真实求值（C++ 并行）。`GridField` 自身也是 `ImplicitField`（三线性插值求值），所以采样结果可以继续参与 CSG。

## CSG 的数学语义（SDF min/max 组合）

实现在 `src/core/implicitSurface/operations/booleanOps.cpp:17-29`：

| Python 运算符 | 语义 | 公式 |
|---|---|---|
| `a \| b` | 并 | $f_{a\cup b}=\min(f_a,f_b)$ |
| `a & b` | 交 | $f_{a\cap b}=\max(f_a,f_b)$ |
| `a - b` | 差 | $f_{a\setminus b}=\max(f_a,-f_b)$ |
| `a.offset(t)` | 外扩 $t$ | $f(\mathbf p)-t$（`OffsetField.h`） |

注意：min/max 组合对**符号与零等值面**精确，但组合结果一般不再是精确欧氏 SDF（在曲面相交棱附近只是下界）。对零等值面提取（marching cubes / OpenVDB）这无影响。

## 文件 ↔ 职责 主表

| 文件 | 职责 | 关键符号 | 文档 |
|---|---|---|---|
| `base.py` | 场契约 + 栅格场 | `ImplicitField`、`GridField` | [base.md](base.md) |
| `grid.py` | 采样栅格规格 | `GridSpec`（`from_mesh` padding 规则） | [grid.md](grid.md) |
| `fields.py` | 形状原语 + 加厚流水线 | `SphereField`、`BoxField`、`MeshUnsignedDistanceField`、`thicken_mesh_surface` | [fields.md](fields.md) |
| `extract.py` | 提取回三角网格 | `extract_marching_cubes`、`OpenVDBOptions`、`extract_openvdb` 等 | [extract.md](extract.md) |
| `__init__.py` | 公开面 | — | [\_\_init\_\_.md](__init__.md) |

包 docstring 的演化约定：`ImplicitField` 是稳定契约；形状原语（`fields.py`）是预期增长的一侧——文件变重时升级为 `fields/` 子包（每个原语一个模块）。

## C++ 引擎对应

| Python | C++ | 位置 |
|---|---|---|
| `ImplicitField` | `ImplicitSurface::ImplicitField`（虚基类） | `src/core/implicitSurface/core/ImplicitField.h` |
| `GridField` | `ImplicitSurface::GridField` | `src/core/implicitSurface/fields/GridField.cpp` |
| `GridSpec` | `ImplicitSurface::GridSpec` | `src/core/implicitSurface/field/gridSpec.h` |
| `SphereField` / `BoxField` | 同名类 | `src/core/implicitSurface/fields/` |
| `MeshUnsignedDistanceField` | 同名类（BVH + libigl 距离场） | `fields/MeshUnsignedDistanceField.cpp` |
| CSG / offset | `BooleanField` / `OffsetField` | `operations/` |
| marching cubes | `extractMarchingCubes`（libigl 后端） | `extraction/marchingCubesExtractor.cpp` |
| OpenVDB 后端 | `buildOpenVDB*` / `extractOpenVDBLevelSet` | `extraction/openVDBExtractor.cpp` |

绑定层：`src/python/pypgo/implicit/bindings.cpp` + `core.cpp`（`PyImplicitField` 等包装、`implicit_union` 等自由函数；求值/采样均释放 GIL）。

## 贯穿示例

```python
import pypgo

mesh = pypgo.mesh.read_obj("thin_shell.obj")   # 开口薄片网格

# 一步加厚：UDF + offset + 采样 + marching cubes
thick = pypgo.implicit.thicken_mesh_surface(
    mesh, thickness=0.02, resolution=128, padding=0.1)

# 等价的手工流水线（可在中途插入 CSG）：
spec = pypgo.implicit.GridSpec.from_mesh(mesh, 128, padding=0.1)
field = pypgo.implicit.MeshUnsignedDistanceField(mesh).offset(0.01)
hole = pypgo.implicit.SphereField(center=[0, 0, 0], radius=0.05)
grid = (field - hole).sample_to_grid(spec)      # 加厚体上挖一个球洞
out = pypgo.implicit.extract_marching_cubes(grid)
pypgo.mesh.write_obj("thick.obj", out)
```

## 交叉链接

- 输入/输出网格类型：[../mesh/data.md](../mesh/data.md)（`TriMeshData`）
- 体网格化的下游消费：[../mesh/processing/volume.md](../mesh/processing/volume.md)（`tet_mesher` / `cubic_mesher`）
