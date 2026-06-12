# `pypgo/implicit/base.py` — 隐式场契约与栅格场

> 源文件：`pypgo/implicit/base.py`（74 行）。模块架构见 [overview.md](overview.md)。
>
> Python 层是 `_core` 句柄的薄门面；求值/采样/CSG 的全部数学在 C++（`src/core/implicitSurface/`）。`GridField` 与 `ImplicitField.sample_to_grid` 耦合，因此与契约放在同一文件，而不与形状原语同住 `fields.py`。

## class `ImplicitField`

所有隐式场的稳定契约。语义：标量场 $f:\mathbb R^3\to\mathbb R$，零等值面 $\{f=0\}$ 是曲面，约定 $f<0$ 为内部。

```python
ImplicitField(core_obj)
```

| 构造参数 | 含义 |
|---|---|
| `core_obj` | C++ peer（`_core.PyImplicitField` 或派生），存入 `_handle` |

普通用户不直接构造它——通过 [fields.md](fields.md) 的原语类，或 CSG 运算符得到。`_from_core(core_obj)` 是绑定层回包用的类方法（绕过 `__init__`）。

### `eval(p)`

```python
field.eval(p) -> float
```

逐点求值 $f(\mathbf p)$。`p` 是任意可转 float64 (3,) 的向量。派发到 C++ 虚函数 `ImplicitField::eval`（`core/ImplicitField.h`），求值期间释放 GIL（`src/python/pypgo/implicit/core.cpp:84-89`）。适合诊断/抽查；密集采样请用 `sample_to_grid`（C++ 并行循环，避免逐点跨语言开销）。

### `bounds()`

```python
field.bounds() -> tuple[np.ndarray, np.ndarray] | None
```

返回保守包围盒 `(bmin, bmax)`（各 (3,) float64），即 $f<0$ 区域的外包。`None` 表示无界（C++ 用无效 `LightBoundingBox` 编码，`isUnbounded`，`core/ImplicitField.h`）。CSG 组合的包围盒规则（`booleanOps.cpp:31-55`）：并 = 两盒并（任一无界则无界）；交 = 两盒交（一方无界则取另一方）；差 = 被减方 `a` 的盒。

### `sample_to_grid(grid_spec, *, num_threads=None)`

```python
field.sample_to_grid(grid_spec: GridSpec, *, num_threads=None) -> GridField
```

把场采样到 $r^3$ 个栅格节点（$r$ = `grid_spec.resolution`）。**as-implemented**（`core/ImplicitField.cpp:19-33`）：

$$\boldsymbol\delta=\frac{\mathbf b_{\max}-\mathbf b_{\min}}{r-1},\qquad
g_{xyz}=f\big(\mathbf b_{\min}+\boldsymbol\delta\odot(x,y,z)\big),\quad x,y,z\in\{0,\dots,r-1\}$$

节点**含两端点**（所以间距分母是 $r-1$）。C++ 用 `parallelFor3D` 并行求值。

| 参数 | 含义 |
|---|---|
| `grid_spec` | 必须是 [`GridSpec`](grid.md)，否则 `TypeError` |
| `num_threads` | `None`→C++ 收 0（默认并行后端）；`1`→强制串行；`>1`→指定线程数；`<=0` 整数→`ValueError` |

例外：`MeshUnsignedDistanceField` 在 C++ 侧重写了 `sampleToGrid`，改走 libigl 距离场（见 [fields.md](fields.md)），此时 `num_threads` 被忽略。

### CSG 运算符 `|` `&` `-` 与 `offset()`

```python
a | b        # 并集   f = min(f_a, f_b)
a & b        # 交集   f = max(f_a, f_b)
a - b        # 差集   f = max(f_a, -f_b)
a.offset(t)  # 外扩 t  f(p) - t
```

**as-implemented**（`operations/booleanOps.cpp:17-29`、`operations/OffsetField.h`）。这是 SDF 的标准 min/max 组合：对 SDF 输入，结果的**符号和零等值面精确**，但数值上一般只是真实距离的界（如并集在两曲面外侧给出下界 $\min$），不再是精确 SDF——对等值面提取无影响。`offset(t)`：$f(\mathbf p)-t=0\iff f(\mathbf p)=t$，即把曲面沿距离场外推 $t$（$t>0$ 外扩、$t<0$ 内缩）；对 UDF 输入这是 `thicken_mesh_surface` 的加厚机制。操作数必须是 `ImplicitField`（模块私有 `_field_core` 校验），返回新的惰性组合场，不复制数据、不采样。

---

## class `GridField(ImplicitField)`

`sample_to_grid` 的结果：$r^3$ 标量栅格 + 其 `GridSpec`。它**本身仍是 `ImplicitField`**——可继续 CSG、再采样，或交给 [extract.md](extract.md) 提取。

栅格上的逐点求值（继承的 `eval`）是**三线性插值**，坐标先 clamp 到栅格范围（C++ `fields/GridField.cpp:20-50`）：

$$f(\mathbf p)=\sum_{i,j,k\in\{0,1\}} w_x^i w_y^j w_z^k\, g_{x_i y_j z_k},
\qquad w^0=1-t,\ w^1=t$$

### 属性 `values`

```python
grid.values -> np.ndarray   # 形状 (r, r, r)，float64
```

零拷贝视图（`np.asarray(self._handle)`，nanobind ndarray 包装 C++ 内部缓冲，`src/python/pypgo/implicit/core.cpp:111-115`）。**索引顺序注意**：C++ 线性布局是 `index = z·r² + y·r + x`（`field/gridSpec.cpp:28-31`），nanobind 按 C-order 包成 `(r, r, r)`，故 `values[z, y, x]` 对应空间点 $\mathbf b_{\min}+\boldsymbol\delta\odot(x,y,z)$——第一维是 z、最后一维是 x。

### 属性 `grid_spec`

```python
grid.grid_spec -> GridSpec
```

回取采样时的 [`GridSpec`](grid.md)（bmin/bmax/resolution）。

## 用法示例

```python
import pypgo

spec = pypgo.implicit.GridSpec([-1, -1, -1], [1, 1, 1], 96)
ball = pypgo.implicit.SphereField([0, 0, 0], 0.7)
box = pypgo.implicit.BoxField([0, 0, 0], [0.5, 0.5, 0.5])

field = (ball & box).offset(0.05)        # 惰性组合：球∩盒再外扩
field.eval([0.0, 0.0, 0.0])              # 单点抽查（应为负，内部）
grid = field.sample_to_grid(spec, num_threads=8)

grid.values.shape                        # (96, 96, 96)，索引 [z, y, x]
mesh = pypgo.implicit.extract_marching_cubes(grid)
```

## 交叉链接

- 栅格规格与 `from_mesh` padding：[grid.md](grid.md)
- 各原语的距离函数是否精确 SDF：[fields.md](fields.md)
- 提取（MC / OpenVDB）：[extract.md](extract.md)
