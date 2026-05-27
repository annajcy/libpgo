# NumPy 数据契约

## 目标

`pypgo` 的第一层数据协议是 NumPy。公开 Python API 可以接受更宽松的 array-like 输入，但在进入 `_core` native binding 前必须统一成明确 shape、dtype、device、contiguity 和 ownership 的 NumPy 数据。

本文档定义 M1/M2 必须遵守的数据契约。

## 总原则

1. NumPy 是 required dependency。
2. SciPy 和 PyTorch 是 optional adapters，不能成为 `import pypgo` 的前置条件。
3. 公开 API 接受 array-like；内部 `_core` 接收已验证的 CPU contiguous NumPy arrays。
4. 第一版优先正确和清晰，零拷贝是优化目标，不是默认承诺。
5. C++ kernels 接收 `double` 和 validated `int`；Python 返回遵循 NumPy 习惯。

## Shape 契约

### Mesh arrays

| 名称 | Shape | dtype input | dtype normalized | 说明 |
| --- | --- | --- | --- | --- |
| triangle vertices | `(n, 3)` | `float32`, `float64` | `float64` | surface mesh 顶点 |
| triangle faces | `(m, 3)` | `int32`, `int64` | `int64` at Python layer, validated `int` for C++ | 三角面索引 |
| volume vertices | `(n, 3)` | `float32`, `float64` | `float64` | `.veg` volume mesh 顶点，适用于 tet/cubic |
| tet elements | `(m, 4)` | `int32`, `int64` | `int64` at Python layer, validated `int` for C++ | tet volume cell 索引 |
| cubic elements | `(m, 8)` | `int32`, `int64` | `int64` at Python layer, validated `int` for C++ | cubic volume cell 索引 |

### State arrays

| 名称 | Shape | dtype input | dtype normalized | 说明 |
| --- | --- | --- | --- | --- |
| positions | `(n, 3)` | `float32`, `float64` | `float64` | 几何位置 |
| rest positions flat | `(3 * n,)` | `float32`, `float64` | `float64` | solver/FEM DOF |
| displacement flat | `(3 * n,)` | `float32`, `float64` | `float64` | simulation state `u` |
| velocity flat | `(3 * n,)` | `float32`, `float64` | `float64` | simulation state `uvel` |
| acceleration flat | `(3 * n,)` | `float32`, `float64` | `float64` | simulation state `uacc` |
| frame state matrix | `(3 * n, 3)` | `float32`, `float64` | `float64` | `.u` 文件读写；列语义为 displacement/velocity/acceleration |

### Force 和参数 arrays

| 名称 | Shape | dtype input | dtype normalized | 说明 |
| --- | --- | --- | --- | --- |
| gravity | `(3,)` | `float32`, `float64` | `float64` | runtime config |
| initial velocity | `(3,)` | `float32`, `float64` | `float64` | runtime config |
| fixed vertex movement | `(3,)` | `float32`, `float64` | `float64` | attachment config |
| fixed vertex ids | `(k,)` | `int32`, `int64` | `int64` at Python layer, validated `int` for C++ | attachment vertices |
| element weights | `(num_elements,)` | `float32`, `float64` | `float64` | FEM setup |
| plastic parameters | `(num_elements * p,)` | `float32`, `float64` | `float64` | model-specific |
| elastic parameters | model-specific flat vector | `float32`, `float64` | `float64` | model-specific |

### Sparse arrays

| 名称 | Shape | dtype | 说明 |
| --- | --- | --- | --- |
| row indices | `(nnz,)` | `int64` | Python-facing COO rows |
| col indices | `(nnz,)` | `int64` | Python-facing COO cols |
| values | `(nnz,)` | `float64` | COO values |
| shape | `(2,)` or tuple | Python `tuple[int, int]` | matrix dimensions |

`SparseMatrix.to_coo()` 必须返回：

```python
rows: np.ndarray  # shape (nnz,), dtype int64
cols: np.ndarray  # shape (nnz,), dtype int64
values: np.ndarray  # shape (nnz,), dtype float64
```

## dtype 规则

### Floating inputs

- 接受 `np.float32` 和 `np.float64`。
- `_core` 层统一为 `float64` / C++ `double`。
- 如果输入是整数但语义要求 floating，公开 Python 层可以用 `np.asarray(x, dtype=np.float64)` 接受；但错误消息要指出目标语义是 floating array。

### Index inputs

- 接受 `np.int32` 和 `np.int64`。
- Python 层保留或转换为 `int64`。
- 进入 C++ 前必须验证所有 index 能安全转换到 C++ `int`：
  - `0 <= index <= std::numeric_limits<int>::max()`。
  - mesh element indices 必须小于对应 vertex count。
- 不接受 float indices，即使数值看起来是整数。

### Boolean inputs

- config flags 使用 Python `bool`。
- NumPy boolean arrays 只在确有数组语义时使用；M0 不需要。

## Contiguity 和 device

- `_core` 只接受 CPU arrays。
- `_core` 只接受 C-contiguous arrays，除非某个 binding 明确说明接受 Fortran-contiguous Eigen view。
- 公开 Python 层使用：

```python
np.ascontiguousarray(array, dtype=target_dtype)
```

- 非 contiguous 输入允许复制。
- device tensor 例如 CUDA tensor 不直接进入 `_core`；`pypgo.torch` adapter 必须显式 `.detach().cpu()` 或报错。

## Copy / view policy

### 构造对象

第一版 geometry 和 simulation 对象默认 copy 输入数据到 C++ owning object。几何层先进入 `pypgo.mesh_geo`：

```python
geo = pypgo.mesh_geo.VolumeMeshGeo.from_tets(vertices, tets)
tet_geo = pypgo.mesh_geo.TetMeshGeo(vertices, tets)
```

之后修改 `vertices` 或 `tets` 原数组，不应改变 `geo` / `tet_geo` 内部状态。Cubic geometry 使用 `VolumeMeshGeo.from_cubes(vertices, cubes)` 或 `CubicMeshGeo(vertices, cubes)`，遵循同样 copy 规则。

文件 I/O 不放在 `pypgo.mesh_geo` 或 `pypgo.mesh` 对象方法上。M1 geometry I/O 使用显式名称：`pypgo.io.read_veg_geo(path)` 返回 `(owning VolumeMeshGeo, MaterialSpec)`，因为 `.veg` 文件包含 material 信息，不能在读取 geometry 时静默丢弃；`pypgo.io.read_obj_geo(path)` 返回 owning `TriMeshGeo`。写出使用 `pypgo.io.write_veg_geo(path, volume_geo, material_spec=None)` 和 `pypgo.io.write_obj_geo(path, surface_geo)`。`pypgo.mesh_geo` 定义 geometry 内存对象和数组访问策略；`pypgo.mesh` 在 M1 只定义 `MaterialSpec`，后续再定义由 `mesh_geo + MaterialSpec` 构建的 simulation mesh。

### 返回数组

第一版优先返回 copy 或 read-only NumPy view：

- 如果 C++ lifetime 和 stride 能稳定保证，可以返回 read-only view。
- 如果存在 lifetime 风险，返回 copy。
- 文档必须明确每个 property 是 copy 还是 view。

建议默认：

| API | 返回策略 |
| --- | --- |
| `TriMeshGeo.vertices` | copy |
| `TriMeshGeo.faces` | copy |
| `VolumeMeshGeo.vertices` | copy |
| `VolumeMeshGeo.cells` | copy |
| `TetMeshGeo.tets` | copy |
| `CubicMeshGeo.cubes` | copy |
| `MaterialSpec` scalar fields | Python scalar copy |
| `SparseMatrix.to_coo()` | copy |
| `Frame.displacement` | copy or read-only view，按实现明确 |

### 后续零拷贝优化

只有在满足以下条件时才引入零拷贝：

- C++ 对象 lifetime 明确由 Python wrapper 持有。
- NumPy view 是 read-only 或 mutation semantics 明确。
- 测试覆盖 wrapper 析构后不悬空。
- 文档写明是否修改 view 会影响 C++ 对象。

## 错误消息风格

数组验证错误应该直接指出参数名、期望 shape/dtype、实际 shape/dtype。

示例：

```text
vertices must have shape (n, 3), got shape (12,)
tets must have integer dtype int32 or int64, got float64
x must be a flat vector with length 3 * num_vertices, got length 17 for num_vertices=6
faces contain vertex index 42, but vertices has only 10 rows
```

不要使用模糊错误：

```text
invalid array
bad input
shape mismatch
```

## Public API normalization examples

```python
def normalize_vertices(vertices: object) -> np.ndarray:
    arr = np.ascontiguousarray(vertices, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError(f"vertices must have shape (n, 3), got shape {arr.shape}")
    return arr


def normalize_tets(tets: object, num_vertices: int) -> np.ndarray:
    arr = np.ascontiguousarray(tets)
    if arr.dtype not in (np.dtype("int32"), np.dtype("int64")):
        raise TypeError(f"tets must have integer dtype int32 or int64, got {arr.dtype}")
    arr = arr.astype(np.int64, copy=False)
    if arr.ndim != 2 or arr.shape[1] != 4:
        raise ValueError(f"tets must have shape (m, 4), got shape {arr.shape}")
    if arr.size and (arr.min() < 0 or arr.max() >= num_vertices):
        raise ValueError(
            f"tets contain vertex index outside [0, {num_vertices})"
        )
    return np.ascontiguousarray(arr)
```

这些 helpers 可以先放在 Python 层；M1 之后再决定哪些验证下沉到 C++ `ndarray_utils.h`。

## SciPy adapter policy

SciPy 是 optional dependency。

```python
A.to_scipy_coo()
A.to_scipy_csr()
```

调用时才 import SciPy。如果没有安装 SciPy，抛出：

```text
scipy is required for SparseMatrix.to_scipy_csr(); install pypgo[scipy]
```

SciPy adapter 使用 `to_coo()` 的 NumPy arrays 构造，不需要额外 C++ binding。

## PyTorch adapter policy

PyTorch 是 optional dependency。

```python
A.to_torch_sparse_coo(device=None)
pypgo.torch.energy_value(energy, x)
```

规则：

- `import pypgo` 不 import torch。
- CPU tensor 可以在安全时通过 NumPy 共享内存；non-contiguous tensor 需要 contiguous copy。
- CUDA tensor 第一版不直接传入 `_core`；adapter 可以 `.detach().cpu()` 后计算，或明确报错。
- Energy-level autograd 先做；solver-level differentiable simulation 不属于 M0-M8 基线交付。

## C++ binding 输入边界

`_core` binding 入口不要接受随意 Python object。公开 Python 层负责 normalization，然后传入 `_core`：

```python
vertices64 = normalize_vertices(vertices)
cells64 = normalize_tets(tets, len(vertices64))
handle = _core.create_volume_mesh_geo_from_tets(vertices64, cells64)
```

C++ 仍需做防御式检查，因为 `_core` 是可 import 的 private module，但错误消息可以更偏开发者。

## M1/M2 测试要求

M1/M2 至少覆盖：

- `float32` vertices 输入会被接受并转换为 `float64`。
- `float64` vertices 输入不必要复制时可以保持数据值一致。
- `(3 * n,)` flat vertices 被拒绝，提示应使用 `(n, 3)`。
- `int32` 和 `int64` faces/tets/cubes 都被接受。
- float faces/tets/cubes 被拒绝。
- out-of-range element indices 被拒绝。
- sparse COO 返回 `int64, int64, float64`。
- SciPy/PyTorch 未安装时 adapter 错误是 lazy 且可理解的。

## M0 结论

NumPy 是 `pypgo` 的地基，不是附属功能。后续任何 mesh、FEM、solver、simulation API 设计都必须先写清楚 NumPy shape/dtype/copy 语义，再考虑 SciPy/PyTorch adapter。
