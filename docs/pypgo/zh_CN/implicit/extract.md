# `pypgo/implicit/extract.py` — 等值面提取（marching cubes 与 OpenVDB 后端）

> 源文件：`pypgo/implicit/extract.py`（49 行）。模块架构见 [overview.md](overview.md)。
>
> 两条提取路径：marching cubes（libigl 后端，**必有**）与 OpenVDB level-set（编译期可选，`-DPGO_ENABLE_OPENVDB=ON`）。C++ 实现在 `src/core/implicitSurface/extraction/`。

## func `extract_marching_cubes()`

```python
extract_marching_cubes(field: GridField, *, iso_offset: float = 0.0) -> TriMeshData
```

对采样栅格跑 marching cubes，提取等值面

$$\{\mathbf p:\ f(\mathbf p)=\text{iso\_offset}\}$$

**as-implemented**（`marchingCubesExtractor.cpp` → `libiglInterface.cpp:111-129`）：按 `GridSpec` 重建 $r^3$ 个格点坐标（与采样同一公式，间距 $\boldsymbol\delta=(\mathbf b_{\max}-\mathbf b_{\min})/(r-1)$），把 `iso_offset` 作为 `isovalue` 直接传给 `igl::marching_cubes`。

| 参数 | 含义 |
|---|---|
| `field` | 必须是 [`GridField`](base.md)（先 `sample_to_grid()`），否则 `TypeError` |
| `iso_offset` | 提取的等值；0 即零等值面，正值=向场值增大方向偏移（对 SDF 即外扩） |

输出顶点精度受栅格分辨率限制（单元内线性插值），无自适应/平滑——需要更光滑结果用下面的 OpenVDB 路径。

---

## OpenVDB level-set 后端（可选）

OpenVDB 路径把场转成稀疏窄带 level-set 栅格（`OpenVDBLevelSet`，内部是 `openvdb::FloatGrid`），可平滑、自适应抽取。是否可用由编译开关决定；不可用时三个 `build_*`/`extract_openvdb` 在 Python 层先行抛 `RuntimeError`（C++ 侧 stub 也会抛，`openVDBExtractor.cpp` 的 `#else` 分支）。

### func `has_openvdb()`

```python
has_openvdb() -> bool
```

能力探测：本 build 是否编入 OpenVDB（C++ `PGO_HAS_OPENVDB`）。调用任何 OpenVDB 函数前先查它分支。

### class `OpenVDBOptions`

```python
OpenVDBOptions(voxel_size: float, half_width: float = 3.0, adaptivity: float = 0.0, smooth_steps: int = 0)
```

OpenVDB 各阶段共用的参数包（C++ `OpenVDBOptions`，每次设值都过 `validateOpenVDBOptions` 校验，`openVDBExtractor.cpp:19-29`）：

| 参数 | 约束 | 含义 | 作用阶段 |
|---|---|---|---|
| `voxel_size` | $>0$，有限 | 体素边长（世界单位），决定 level-set 分辨率 | build |
| `half_width` | $>0$，有限 | 窄带半宽，**以体素数计**（默认 3 体素）；窄带外的值被截断为背景值 $\pm h_w\cdot v$ | build |
| `adaptivity` | $\ge 0$ | `volumeToMesh` 的自适应抽取强度，$[0,1]$：0=均匀全密度网格，越大平坦区合并越激进、三角形越少 | extract |
| `smooth_steps` | $\ge 0$ 整数 | 提取前的平均曲率流平滑迭代次数（`LevelSetFilter::meanCurvature`） | extract |

构造后四个参数都可经 `options._handle` 的同名属性读写（每次写都重新校验）。

### func `build_openvdb_shell_from_mesh()`

```python
build_openvdb_shell_from_mesh(mesh: TriMeshData, shell_thickness: float, options: OpenVDBOptions)
    -> _core.PyOpenVDBLevelSet
```

网格 → 厚度为 `shell_thickness` 的壳 level-set，OpenVDB 版"加厚"（对照 [`thicken_mesh_surface`](fields.md)）。**as-implemented**（`openVDBExtractor.cpp:buildOpenVDBShellFromMesh`）：

1. `meshToUnsignedDistanceField` 在窄带内建网格 UDF，窄带宽取 $h_w+\dfrac{t/2}{v}$ 体素（$t$=`shell_thickness`、$v$=`voxel_size`）——确保偏移后窄带仍覆盖零等值面；
2. 全栅格减去 $t/2$（`offsetOpenVDBGrid`，含背景值）：$f\leftarrow d(\mathbf p,\mathcal M)-t/2$，零等值面即两侧各扩 $t/2$ 的壳面；
3. 标记为 `GRID_LEVEL_SET`。

与 `thicken_mesh_surface` 同一数学（UDF − 半厚度），但栅格是稀疏窄带、分辨率由 `voxel_size` 而非全局 $r^3$ 决定，且提取时可平滑/自适应。

### func `build_openvdb_from_grid_field()`

```python
build_openvdb_from_grid_field(field: GridField, options: OpenVDBOptions) -> _core.PyOpenVDBLevelSet
```

把已采样的 [`GridField`](base.md)（含 CSG 组合结果）搬进 OpenVDB 栅格。**as-implemented**（`buildOpenVDBFromGridField`）：背景值取 $h_w\cdot v$；变换取线性缩放 $\delta_x$（GridField 的 **x 轴间距** `(bmax-bmin)/(r-1)` 的第 0 分量——OpenVDB 变换是各向同性的，非立方采样盒会被按 x 间距对待）再平移 `bmin`；逐体素拷贝 `field.at(x,y,z)`（稠密写入）。注意此函数**不使用** `options.voxel_size` 作间距，间距来自 GridField 自身；`voxel_size` 仍须为正以过校验。

典型用途：marching cubes 之外的第二条出口——同一份 `GridField` 既可 MC 快速预览，也可转 OpenVDB 平滑后出净版。

### func `extract_openvdb()`

```python
extract_openvdb(levelset, options: OpenVDBOptions) -> TriMeshData
```

level-set → 三角网格。**as-implemented**（`extractOpenVDBLevelSet`）：

1. 平滑：对栅格做 `smooth_steps` 次平均曲率流（**就地修改传入的 level-set**，重复调用会累积平滑）；
2. `volumeToMesh(isovalue=0, adaptivity, relaxDisorientedTriangles=true)` 抽取，quad 输出按 `(0,1,2)+(0,2,3)` 剖成三角形。

`levelset` 是 `build_openvdb_*` 返回的 `_core.PyOpenVDBLevelSet` 句柄（Python 层不再包装）。

## 用法示例

```python
import pypgo

mesh = pypgo.mesh.read_obj("patch.obj")

# 路径 1：marching cubes（必有）
spec = pypgo.implicit.GridSpec.from_mesh(mesh, 128)
grid = pypgo.implicit.MeshUnsignedDistanceField(mesh).offset(0.01).sample_to_grid(spec)
mc = pypgo.implicit.extract_marching_cubes(grid)

# 路径 2：OpenVDB（可选，更平滑 + 自适应）
if pypgo.implicit.has_openvdb():
    opts = pypgo.implicit.OpenVDBOptions(voxel_size=0.002, smooth_steps=5, adaptivity=0.3)
    ls = pypgo.implicit.build_openvdb_shell_from_mesh(mesh, shell_thickness=0.02, options=opts)
    smooth = pypgo.implicit.extract_openvdb(ls, opts)
```

## 交叉链接

- 输入栅格场：[base.md](base.md)（`GridField`）、[grid.md](grid.md)（`GridSpec`）
- 一步加厚封装：[fields.md](fields.md)（`thicken_mesh_surface`）
- 同一 OpenVDB 机制的动画应力可视化：[../animation/stress_vdb.md](../animation/stress_vdb.md)
- 输出类型：[../mesh/data.md](../mesh/data.md)（`TriMeshData`）
