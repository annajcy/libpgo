# `pypgo/implicit/fields.py` — 形状原语与网格加厚流水线

> 源文件：`pypgo/implicit/fields.py`（68 行）。模块架构见 [overview.md](overview.md)。
>
> 每个原语是 [`ImplicitField`](base.md) 的具体子类，距离函数全部在 C++（`src/core/implicitSurface/fields/`）。包 docstring 约定：这是预期增长的一侧——文件变重时升级为 `fields/` 子包（每个原语一个模块）。

## class `SphereField(ImplicitField)`

```python
SphereField(center, radius: float)
```

| 参数 | 含义 |
|---|---|
| `center` | 球心，可转 float64 (3,) |
| `radius` | 半径，须为正且有限（C++ 构造校验，否则 `RuntimeError`） |

**精确欧氏 SDF**（`SphereField.cpp:19-22`）：

$$f(\mathbf p)=\|\mathbf p-\mathbf c\|_2-r$$

`bounds()` 返回 $[\mathbf c-r\mathbf 1,\ \mathbf c+r\mathbf 1]$。

### 类方法 `from_mesh_bbox(mesh)`

```python
SphereField.from_mesh_bbox(mesh: TriMeshData) -> SphereField
```

从一个"球状"网格反推解析球参数（`SphereField.cpp:fromMeshBBox`）：球心取网格 bbox 中心，半径取**所有顶点到该中心的最大距离**（外接半径，不是 bbox 半边长）。空网格或推得半径非正抛 `RuntimeError`。用途：把美术给的离散球网格还原成解析 SDF（如作 OpenVDB 球壳/裁剪球的输入）。

### 属性 `center` / `radius`

```python
sphere.center -> np.ndarray   # (3,) float64
sphere.radius -> float
```

回读构造参数。

---

## class `BoxField(ImplicitField)`

```python
BoxField(center, half_extent)
```

| 参数 | 含义 |
|---|---|
| `center` | 盒中心 |
| `half_extent` | 各轴半边长，每个分量须为正（C++ 校验） |

**不是精确欧氏 SDF**——是 Chebyshev（$L^\infty$ 型）盒距离（`BoxField.h:eval`）：

$$\mathbf q=|\mathbf p-\mathbf c|-\mathbf h,\qquad f(\mathbf p)=\max(q_x,\,q_y,\,q_z)$$

符号与零等值面精确（盒面即 $\{f=0\}$，内部为负），但在盒外角/棱区域数值小于真实欧氏距离（精确 SDF 还需 $\|\max(\mathbf q,0)\|_2$ 项）。对等值面提取与 CSG 的 min/max 组合（[overview.md](overview.md)）无影响；做 `offset(t)` 外扩时，角部会得到"尖角外推"而非圆角——与精确 SDF 的圆角外扩不同，使用时注意。

`bounds()` 返回 $[\mathbf c-\mathbf h,\ \mathbf c+\mathbf h]$。

### 类方法 `from_bbox(bmin, bmax)`

```python
BoxField.from_bbox(bmin, bmax) -> BoxField
```

由角点构造（C++ 侧换算 `center`/`halfExtent` 并校验盒有效性）。

---

## class `MeshUnsignedDistanceField(ImplicitField)`

```python
MeshUnsignedDistanceField(mesh: TriMeshData)
```

三角网格的**无符号**距离场（UDF，`MeshUnsignedDistanceField.cpp`）：

$$f(\mathbf p)=\min_{\mathbf q\in\mathcal M}\|\mathbf p-\mathbf q\|_2\ \ge 0$$

处处非负 ⟹ 零等值面退化为曲面本身（零测度），**必须配 `offset(t)`（$t>0$）才能得到有体积的等值面**——这正是 `thicken_mesh_surface` 的机制。无符号意味着对开口、非流形、自交网格都适用（不需要内外定向）。

实现要点（已核对 C++）：

- 逐点 `eval`：BVH 最近三角形查询（`TriMeshBVTree::closestTriangleQuery`），BVH 首次求值时惰性构建（`std::call_once`）；
- `sample_to_grid` 被 C++ 侧**重写**：不走逐点并行循环，而是调 libigl `igl::signed_distance` 的 `UNSIGNED` 模式一次算完整个栅格（`libiglInterface.cpp:computeDistanceField`，`sign=0`），此路径忽略 `num_threads`；
- `bounds()` 返回网格顶点的紧 bbox（注意：这是曲面的盒，不是 offset 后实体的盒——采样盒请用 [`GridSpec.from_mesh`](grid.md) 的 padding 外扩）。

`mesh` 必须是 [`TriMeshData`](../mesh/data.md)，否则 `TypeError`。

---

## func `thicken_mesh_surface()`

```python
thicken_mesh_surface(
    mesh: TriMeshData, *,
    thickness: float,
    resolution: int,
    padding: float = 0.1,
    iso_offset: float = 0.0,
) -> TriMeshData
```

把开口/薄片三角网格加厚成厚度约 `thickness` 的闭合实体壳，一步完成隐式壳 → 采样 → 提取。**as-implemented**（`fields.py:55-68`）就是四行流水线：

```
spec  = GridSpec.from_mesh(mesh, resolution, padding=padding)   # 采样盒（按轴外扩）
field = MeshUnsignedDistanceField(mesh).offset(0.5 * thickness) # f(p) = d(p, M) − t/2
grid  = field.sample_to_grid(spec)                              # libigl 距离场一次采样
return extract_marching_cubes(grid, iso_offset=iso_offset)      # 等值面 f = iso_offset
```

数学：提取的曲面是

$$\big\{\mathbf p:\ d(\mathbf p,\mathcal M)=\tfrac{\text{thickness}}{2}+\text{iso\_offset}\big\}$$

即曲面两侧各扩 `thickness/2` 的管状（offset surface）边界——故 `thickness` 是**总厚度**。`iso_offset` 在栅格上微调最终等值（正值再加厚）。

| 参数 | 含义 |
|---|---|
| `mesh` | 输入曲面（可开口/非流形——UDF 不需要定向） |
| `thickness` | 总厚度（两侧各一半） |
| `resolution` | 栅格分辨率 $r$（$r^3$ 个采样点）；须能解析 `thickness`，经验上栅格间距应明显小于 `thickness/2` |
| `padding` | 转交 [`GridSpec.from_mesh`](grid.md) 的相对外扩比例 |
| `iso_offset` | 转交 [`extract_marching_cubes`](extract.md) 的等值偏移 |

输出是 marching cubes 网格（分辨率受 $r$ 限制、有阶梯感）；要更光滑/自适应的结果走 OpenVDB 路径（[extract.md](extract.md) 的 `build_openvdb_shell_from_mesh`，本质相同但带 `smooth_steps`/`adaptivity`）。下游典型消费：[`tet_mesher`/`cubic_mesher`](../mesh/processing/volume.md) 体网格化。

## 用法示例

```python
import pypgo

mesh = pypgo.mesh.read_obj("cloth_patch.obj")        # 开口薄片

# 一步加厚
thick = pypgo.implicit.thicken_mesh_surface(
    mesh, thickness=0.02, resolution=128)

# 手工等价流水线，中途插入 CSG：加厚体上挖一个球
spec = pypgo.implicit.GridSpec.from_mesh(mesh, 128)
shell = pypgo.implicit.MeshUnsignedDistanceField(mesh).offset(0.01)
hole = pypgo.implicit.SphereField([0, 0, 0], 0.05)
grid = (shell - hole).sample_to_grid(spec)
out = pypgo.implicit.extract_marching_cubes(grid)
```

## 交叉链接

- 场契约、CSG 运算符与 `sample_to_grid`：[base.md](base.md)
- 采样盒与 padding 规则：[grid.md](grid.md)
- 提取（MC / OpenVDB 平滑版加厚）：[extract.md](extract.md)
- 输入/输出网格类型：[../mesh/data.md](../mesh/data.md)
