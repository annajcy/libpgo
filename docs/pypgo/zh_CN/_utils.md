# `pypgo/_utils.py` — FFI 输入校验器

> 源文件：`pypgo/_utils.py`（116 行，纯 Python，仅依赖 NumPy）。所属架构见 [overview.md](overview.md)。
>
> 整个包的输入卫生层。C++ 引擎期望**确定 dtype、确定形状、C 连续**的数组；本模块提供一组命名校验函数，把任意 array-like 输入规整成这种形式，校验失败时抛出带参数名的 `ValueError`/`TypeError`。所有门面模块（energy、fem、mesh…）在把数据交给 `_core` 之前都经过这里。这是私有模块（下划线前缀），不在公开 API 中。

## 共同约定

- **第一个参数总是 `name`**：失败消息形如 `"{name} must be 1-D, got shape ..."`，调用方不需要再包装异常即可定位是哪个形参出错。
- **返回值保证 C 连续**（`order="C"` / `np.ascontiguousarray`），nanobind 层可零拷贝接收。
- 浮点统一 `float64`、整数统一 `int64`——与 C++ 侧 `double` / `int64_t` 形参一一对应。

---

## func `float_vector(name, value)`

```python
float_vector(name: str, value) -> np.ndarray   # float64 (n,)，C 连续
```

把任意 array-like 转成一维 `float64` 数组；`ndim != 1` 抛 `ValueError`。是最常用的向量入口（目标位置、初值 `x0`、界向量等都经它）。

## func `sized_vector(name, value, n)`

```python
sized_vector(name: str, value, n: int) -> np.ndarray   # float64 (n,)
```

比 `float_vector` 更宽容也更严格：先 `ravel()` 接受任意形状（如 `(m, 3)` 的目标位置矩阵），再要求展平后长度**恰为** `n`，否则抛 `ValueError`。

## func `vec3(name, value)`

```python
vec3(name: str, value) -> np.ndarray   # float64 (3,)
```

平铺（`reshape(-1)`）后必须恰是 3 元，返回 C 连续 3-向量。

## func `vec3_finite(name, value)`

```python
vec3_finite(name: str, value) -> np.ndarray   # float64 (3,)，全有限
```

形状必须是 `(3,)`（不做平铺），并额外要求全部分量有限（`np.isfinite`，拒绝 NaN/Inf）。用于重力方向、平面法向这类"NaN 会静默毒化整个求解"的物理参数。

## func `vec3_list(name, value)`

```python
vec3_list(name: str, value) -> list[float]   # 3 元 Python list
```

同 `vec3` 的校验，但返回 Python `list`——部分 C++ 绑定形参签名是 `std::vector<double>` / 标量列表而非 ndarray。

## func `int_vector(name, value)`

```python
int_vector(name: str, value) -> np.ndarray   # int64 (n,)，C 连续
```

一维 `int64` 数组（顶点索引列表等）；`ndim != 1` 抛 `ValueError`。注意它**不**拒绝浮点输入（`np.asarray(..., dtype=np.int64)` 会截断）——需要严格整数类型检查时用 `index_matrix`。

## func `float_matrix(name, value, columns)`

```python
float_matrix(name: str, value, columns: int) -> np.ndarray   # float64 (m, columns)
```

二维 `float64` 矩阵，列数必须等于 `columns`。无法转 `float64` 时把底层异常包装成带 `name` 的 `ValueError`（链式 `from e`）。

## func `index_matrix(name, value, columns, *, num_vertices)`

```python
index_matrix(name: str, value, columns: int, *, num_vertices: int) -> np.ndarray   # int64 (m, columns)
```

索引矩阵（如四面体 `(m, 4)`、三角形 `(m, 3)` 的连接表）的严格校验，分两段是有意的：

1. **先用原 dtype 检查"必须是整数类型"**（`np.issubdtype(..., np.integer)`，抛 `TypeError`）——防止 `2.7` 被静默截断为 `2`；
2. 再转 `int64` 检查形状与范围：索引须落在 `[0, num_vertices)`，负数或越界抛 `ValueError`。

---

## func `finite_scalar(name, value)`

```python
finite_scalar(name: str, value: float) -> float
```

`float(value)` 后要求有限，否则抛 `ValueError`。是下面两个守卫的基础。

## func `positive_scalar(name, value)`

```python
positive_scalar(name: str, value: float) -> float
```

有限且 $> 0$（刚度系数、密度、时间步长等）。

## func `nonnegative_scalar(name, value)`

```python
nonnegative_scalar(name: str, value: float) -> float
```

有限且 $\ge 0$（允许取零的权重、容差等）。

---

## func `vertex_array(name, value)`

```python
vertex_array(name: str, value) -> np.ndarray   # float64 (n, 3)，C 连续
```

顶点坐标数组：必须是 `(n, 3)`。等价于 `float_matrix(name, value, 3)` 的命名特化（错误消息更贴合"顶点"语境）。

## func `triangle_array(name, value)`

```python
triangle_array(name: str, value) -> np.ndarray   # int64 (m, 3)，索引非负
```

三角形索引数组：`(m, 3)` 的 `int64`，且全部索引非负。与 `index_matrix` 不同，它**不**校验索引上界（构造时常常还不知道顶点数）也不拒绝浮点 dtype。

## 用法示例

```python
from pypgo._utils import vec3_finite, positive_scalar, index_matrix

def my_api(*, center, radius, tets, num_vertices):
    c = vec3_finite("center", center)        # NaN → ValueError("center must contain finite values")
    r = positive_scalar("radius", radius)
    t = index_matrix("tets", tets, 4, num_vertices=num_vertices)
    return _core.something(c, r, t)
```

## 交叉链接

- 典型调用方：[energy/attachment.md](energy/attachment.md)（`int_vector`/`float_vector`）、[solver/base.md](solver/base.md)（`x0` 经 `float_vector`）、[constraints/bounded.md](constraints/bounded.md)（界向量）
- 矩阵类输入的另一条规整通道：[sparse.md](sparse.md)（`as_sparse_matrix`/`as_coo`）
