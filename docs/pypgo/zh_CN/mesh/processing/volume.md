# `pypgo/mesh/processing/volume.py` — 体网格生成与几何汇总

> 源文件：`pypgo/mesh/processing/volume.py`（123 行）。子包架构见 [overview.md](overview.md)。
>
> 两个生成器把**闭合三角曲面**变为体网格：`tet_mesher`（TetGen / fTetWild 双后端）与 `cubic_mesher`（体素化）。重计算全部在 C++（`src/core/volumetricMeshMeshing/`，绑定 `src/python/pypgo/mesh/geo/core.cpp:328-411`）；`volume_mesh_info` 则是数据容器积分量（[../data.md](../data.md)）的纯 Python 汇总。

## 后端探测

### func `has_tetwild() -> bool`

fTetWild 是否编译进本构建（C++ 宏 `PGO_TET_MESHER_HAS_TET_WILD`）。

### func `has_cgal_remesher() -> bool`

CGAL 修复/重网格后端是否可用（宏 `PYPGO_HAS_CGAL`；被 [surface.md](surface.md) 的全部 CGAL 函数守门）。

---

## func `cubic_mesher(tri_data, *, resolution, E=1e6, nu=0.45, density=1000.0) -> CubicMeshData`

把闭合曲面**体素化**为六面体（立方体）网格。

| 参数 | 含义 | 校验（C++ 侧） |
|---|---|---|
| `resolution` | 包围盒**最短边**上的体素数 | $>0$ |
| `E`, `nu`, `density` | 写入中间 Vega 网格的材料参数 | $E>0$、$\nu\in(-1,0.5)$、$\rho>0$ |

算法（`triangleMeshVoxelizer.cpp`，已核对）：

1. **入口质检**：自交（BVH 精确）、流形性、闭合性三连检查，不过即抛错——先过 [`check_surface_quality`](surface.md)；
2. **网格化**：取包围盒最短边 $s_{\min}$，体素边长 $h=s_{\min}/\text{resolution}$，三方向格数 $n_x=\lceil s_x/h\rceil$ 等，栅格**居中对齐**包围盒；
3. **占据判定**：体素被保留当且仅当其**中心在曲面内部**（光线投射内外测试）**或**其盒与某三角形相交（保证薄壁特征不漏）；
4. 占据体素的角点去重 → 顶点表，输出 `CubicMeshData`（局部顶点序即 [../data.md](../data.md) 的 Vega 约定）。

体素数随 `resolution` 立方增长。**注意**：返回值是纯几何容器，`E/nu/density` 不随 `CubicMeshData` 带出——构造 [`VolumeMesh`](../volume/core.md) 时需重新给材料。产物是 [../../fem/formulations.md](../../fem/formulations.md) 中 `CubicLinear` / `CubicTricubicHermite` formulation 的标准输入。

---

## func `tet_mesher(tri_data, *, backend="tetgen", config=None) -> TetMeshData`

把闭合曲面四面体化。`config` 是后端专属参数字典（未给键用默认值）。

### backend `"tetgen"`

调 TetGen（`tetgenBackend.cpp`）；唯一参数：

| `config` 键 | 默认 | 含义 |
|---|---|---|
| `"command"` | `"pq1.414"` | TetGen 命令串：`p` 按输入分段曲面剖分（保形）、`q1.414` 质量约束（外接球半径/最短边比 $\le\sqrt2$） |

输出经 `tetMesh->orient()` 统一定向。共形保边，但对脏输入敏感。

### backend `"tetwild"`

调 fTetWild（`tetwildBackend.cpp`，需 `has_tetwild()`，否则 Python 层先抛 `RuntimeError`）。鲁棒（容忍自交/小缝隙，输出有 $\varepsilon$ 包络误差、**不严格保形**）：

| `config` 键 | 默认 | fTetWild 参数 | 含义 |
|---|---|---|---|
| `"lr"` | `0.05` | `ideal_edge_length_rel` | 目标边长（相对包围盒对角线） |
| `"la"` | — | `ideal_edge_length_abs` | 目标边长（绝对）；**给了 `la` 键就覆盖 `lr`**（66-72 行 `"la" in config` 判定） |
| `"epsr"` | `0.001` | `eps_rel` | 包络容差（相对） |
| `"stop_energy"` | `10.0` | `stop_energy` | AMIPS 能量阈值，质量优化到此停止 |
| `"max_threads"` | `0` | `num_threads` | 0 = 不限制 |

### 选择速查

| 需求 | 后端 |
|---|---|
| 干净曲面、要保形保边 | `tetgen` |
| 扫描件/脏输入、要鲁棒 | `tetwild`（先探测 `has_tetwild()`） |

两后端都经临时 OBJ 文件交换数据（绑定层 `TemporaryObjFile`，自动清理），返回纯几何 `TetMeshData`。

---

## class `VolumetricMeshInfo`（冻结 dataclass）

| 字段 | 含义 | 来源 |
|---|---|---|
| `num_vertices` / `num_elements` | 计数 | 容器属性 |
| `num_element_vertices` | 每元素顶点数（4 或 8） | `elements.shape[1]` |
| `total_volume` | $\sum_e V_e$（四面体 $\frac16\lvert\det\rvert$；六面体经 5-四面体分解，公式见 [../data.md](../data.md)） | `mesh.volume` |
| `center_of_mass` | 体积加权质心 | `mesh.center_of_mass` |

`__str__` 输出对齐的多行报告（数值 `%.17g` 全精度）。

## func `volume_mesh_info(mesh) -> VolumetricMeshInfo`

接受 `TetMeshData` / `CubicMeshData` / [`VolumeMesh`](../volume/core.md) / [`VegFile`](../volume/core.md)（后两者自动取 `mesh_data`）。生成后**核对体积与质心**是验证网格化没跑偏的最快手段。

## 用法示例

```python
import pypgo

box = pypgo.mesh.create_box(bmin=[0, 0, 0], bmax=[1, 1, 1])

# TetGen：默认质量约束
tets = pypgo.mesh.tet_mesher(box, backend="tetgen",
                             config={"command": "pq1.2a0.001"})  # 更严质量 + 体积上限

# fTetWild：绝对目标边长
if pypgo.mesh.has_tetwild():
    tets_tw = pypgo.mesh.tet_mesher(box, backend="tetwild",
                                    config={"la": 0.05, "epsr": 5e-4})

# 体素化六面体
hexes = pypgo.mesh.cubic_mesher(box, resolution=16)

info = pypgo.mesh.volume_mesh_info(tets)
print(info)            # 体积 ≈ 1.0，质心 ≈ (0.5, 0.5, 0.5)
```

## 交叉链接

- 入口质检与修复：[surface.md](surface.md)
- 体积/质心公式：[../data.md](../data.md)
- 产物去向：[../volume/core.md](../volume/core.md)（加材料）→ [../../fem/mesh.md](../../fem/mesh.md)（`SimulationMesh`）
- 六面体 formulation：[../../fem/formulations.md](../../fem/formulations.md)
