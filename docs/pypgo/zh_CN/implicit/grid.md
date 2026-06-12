# `pypgo/implicit/grid.py` — 采样栅格规格

> 源文件：`pypgo/implicit/grid.py`（45 行）。模块架构见 [overview.md](overview.md)。
>
> Python 层是 `_core.PyGridSpec` 的薄门面；校验与存储在 C++（`src/core/implicitSurface/field/gridSpec.cpp`）。本文件只有一个类。

## class `GridSpec`

定义 `sample_to_grid`（见 [base.md](base.md)）的采样格点：一个轴对齐盒 $[\mathbf b_{\min},\mathbf b_{\max}]$ 上、每轴 $r$ 个**含端点**的均匀节点，共 $r^3$ 个：

$$\mathbf p_{xyz}=\mathbf b_{\min}+\boldsymbol\delta\odot(x,y,z),\qquad
\boldsymbol\delta=\frac{\mathbf b_{\max}-\mathbf b_{\min}}{r-1},\qquad x,y,z\in\{0,\dots,r-1\}$$

C++ 线性索引约定 `index = z·r² + y·r + x`（`gridSpec.cpp:28-31`），对应 `GridField.values[z, y, x]`（见 [base.md](base.md)）。

```python
GridSpec(bmin, bmax, resolution: int)
```

| 参数 | 含义 |
|---|---|
| `bmin`, `bmax` | 盒角点，任意可转 float64 (3,) 的向量 |
| `resolution` | 每轴节点数 $r$ |

构造即在 C++ 侧校验（`validateGridSpec`，`gridSpec.cpp:8-19`）：`resolution >= 2`、两角点各分量有限、且每轴 `bmax > bmin`（严格大于——零厚度盒被拒绝），否则抛 `RuntimeError`。

`_from_core(core_obj)` 是绑定层回包用的类方法（绕过 `__init__`），如 `GridField.grid_spec` 取回时使用。

### 类方法 `from_mesh(mesh, resolution, padding=0.1)`

```python
GridSpec.from_mesh(mesh: TriMeshData, resolution: int, padding: float = 0.1) -> GridSpec
```

从网格包围盒派生采样盒，**按轴相对外扩**。as-implemented（`grid.py:23-30`）：设 $\mathbf e=\mathbf b_{\max}-\mathbf b_{\min}$（网格 bbox 尺寸），

$$\text{pad}_i=\begin{cases}
e_i\cdot\text{padding}, & e_i>0\\[2pt]
\max\big(\max_j e_j\cdot\text{padding},\ \text{padding},\ 10^{-6}\big), & e_i=0\ \text{（退化轴 fallback）}
\end{cases}$$

返回 `GridSpec(bmin - pad, bmax + pad, resolution)`。要点：

- `padding` 是**相对比例**（默认 0.1 即每轴外扩 10% 的该轴尺寸），不是绝对长度；
- 退化轴（平面片、线段使某轴 $e_i=0$）改用 fallback：取"最大轴尺寸 × padding"，并兜底到 `padding` 本身与 $10^{-6}$ 的较大者——保证 C++ 校验 `bmax > bmin` 不被零厚度轴触发；
- `mesh` 必须是 [`TriMeshData`](../mesh/data.md)，否则 `TypeError`。

外扩的意义：`thicken_mesh_surface`（[fields.md](fields.md)）把曲面沿 UDF 外推 `thickness/2`，采样盒必须把外推后的曲面整个包进去，否则等值面在盒边界被截断。

### 属性 `resolution` / `bmin` / `bmax`

```python
spec.resolution -> int
spec.bmin -> np.ndarray   # (3,) float64（拷贝）
spec.bmax -> np.ndarray   # (3,) float64（拷贝）
```

回读 C++ 侧存储。`repr(spec)` 打印三者，方便日志。

## 用法示例

```python
import pypgo

mesh = pypgo.mesh.read_obj("shell.obj")

# 自动盒：bbox 每轴外扩 10%
spec = pypgo.implicit.GridSpec.from_mesh(mesh, resolution=128, padding=0.1)

# 手动盒（如要和另一个场对齐）
spec2 = pypgo.implicit.GridSpec([-1, -1, -1], [1, 1, 1], 96)

grid = pypgo.implicit.MeshUnsignedDistanceField(mesh).sample_to_grid(spec)
```

## 交叉链接

- 消费方：[base.md](base.md)（`ImplicitField.sample_to_grid` / `GridField.grid_spec`）
- 加厚流水线如何用 `from_mesh`：[fields.md](fields.md)（`thicken_mesh_surface`）
- 网格 bbox 来源：[../mesh/data.md](../mesh/data.md)（`TriMeshData.bbox`）
