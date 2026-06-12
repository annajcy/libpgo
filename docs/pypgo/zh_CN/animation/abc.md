# `pypgo/animation/abc.py` — Alembic 导出与 Eigen 二进制 `.u` I/O

> 源文件：`pypgo/animation/abc.py`（271 行）。模块架构见 [overview.md](overview.md)。
>
> `.u` 读写是纯 Python（`struct` + NumPy），与 C++ `EigenSupport::readMatrix/writeMatrix` 字节级互通；Alembic 导出经 `_core.dump_abc` / `_core.PyAnimationLoader` 走 C++（`src/core/animationIO/`），是编译期可选能力。

## func `has_animation_io()`

```python
has_animation_io() -> bool
```

本 build 是否编入 Alembic I/O。启用时绑定层链接 `core.cpp`（恒返回 True）；未启用时链接 `disabled/core.h` 的 stub（返回 False，且 `dump_abc` 等不可用）。`AbcWriter` / `AnimationLoader` 构造时先查它，不可用直接抛 `RuntimeError`。

---

## `.u` 文件格式（Eigen 二进制矩阵）

仿真位移的原生交换格式，**as-implemented** 与 C++ `EigenSupport::writeMatrix`（`src/core/eigenSupport/EigenSupport.h:444-472`）逐字节一致：

| 偏移 | 大小 | 内容 |
|---|---|---|
| 0 | 4 | `nrows`（int32，小端/本机序） |
| 4 | 4 | `ncols`（int32） |
| 8 | 4 | `entry_size`（int32，每元素字节数：8=float64，4=float32） |
| 12 | `nrows·ncols·entry_size` | 矩阵数据，**列主序**（Eigen 默认存储） |

逐帧位移按 $(3n,1)$ 列向量存（单列时行/列主序无差别，但多列文件如 `uAll` 缓存必须按列主序解读）。

### func `read_u_file()`

```python
read_u_file(path) -> np.ndarray   # (nrows, ncols)
```

读头部三个 int32，按 `entry_size` 选 dtype（8→float64，否则 float32），数据按 `order="F"` 重排成 `(nrows, ncols)` 再转 C 连续返回。

### func `write_u_file()`

```python
write_u_file(path, mat) -> None
```

固定写 float64（`entry_size=8`）、列主序。1-D 输入自动升成 $(n,1)$ 列。[`tools/sim`](../tools/sim/_runners.md) 的 `write_states` 即用它写 `states/deform{frame:04d}.u`。

---

## class `AbcWriter`

单网格的增量 Alembic 写出器：内存里攒位移帧，`write()` 时一次性写归档。支持上下文管理器（退出且有帧时自动 `write()`）。

```python
AbcWriter(path, name, *, rest_positions, triangles, fps=24.0)
```

| 参数 | 含义 |
|---|---|
| `path` | 输出 `.abc` 路径 |
| `name` | Alembic 对象名（节点名；shape 名为 `{name}Shape`） |
| `rest_positions` | 扁平 `(3·n_verts,)` float64 静止位置 |
| `triangles` | 扁平 `(3·n_tris,)` int 面索引 |
| `fps` | 写入归档的帧率，默认 24（见下方注意） |

构造即检查 `has_animation_io()`。

> **fps 注意（as-implemented）**：Python 侧存了 `self._fps`，但 `write()` 调用的 `_core.dump_abc` 绑定**没有 fps 参数**（`bindings.cpp:13-15`），C++ `dumpABC` 固定 `TimeSampling(1.0/24, 0.0)`（`abcWriter.cpp:23`）——当前归档时间采样恒为 24 fps，`fps` 参数实际不生效。
>
> **精度注意**：C++ 接口收 `std::vector<float>`，位置/位移在导出时降为 float32（Alembic 的 `V3f` 本来也是单精度）。

### `add_frame(displacement)`

```python
w.add_frame(displacement) -> None
```

追加一帧位移（扁平 `(3·n_verts,)`，可转 float64）。第 $i$ 帧的顶点位置 = `rest_positions + displacement_i`（C++ 侧逐分量相加，`abcWriter.cpp:51-60`）。

### `write()`

```python
w.write() -> None
```

把当前累积的所有帧一次写成 `.abc`（Ogawa 后端，覆盖既有文件）。可重复调用，每次用当前帧集重写。写失败抛 `RuntimeError`。

### 类方法 `dump(...)`

```python
AbcWriter.dump(path, name, *, rest_positions, triangles, displacements, fps=24.0) -> None
```

一步写出：等价于构造 + 逐个 `add_frame` + 退出时 `write()`。[`tools/sim`](../tools/sim/_runners.md) 的 `write_abc` 用它写 `animation.abc`。

---

## class `AnimationLoader`

JSON 配置驱动的批量管线（C++ `AnimationIO::AnimationLoader` 的门面）：读位移序列 + 驱动网格，按 mesh 各写一个 `.abc`。配置 schema（**as-implemented**，`animationLoader.cpp:26-44`）：

```json
{
    "save-cache": 1,
    "meshes": [
        {
            "name": "my_mesh",
            "driving-mesh": "rest.obj",
            "display-mesh": "render.obj",
            "sequence": "frame_{:04d}.u",
            "sequence-type": "u",
            "sequence-range": [0, 100],
            "gap": 1,
            "scale": "1,1,1"
        }
    ]
}
```

| 键 | 必填 | 含义 |
|---|---|---|
| `name` | 是 | 输出对象名/文件名前缀 |
| `driving-mesh` | 是 | 静止网格，`.obj` 或 `.veg`（tet 网格） |
| `display-mesh` | 否 | 渲染网格；给出时嵌入 driving mesh 插值位移（embedded rendering） |
| `sequence` | 是 | 帧文件模式（`fmt` 风格，如 `frame_{:04d}.u`） |
| `sequence-type` | 是 | `"u"`（逐帧 `.u`）/ `"uall"`（单个多列 `.u`）/ `"objmesh"`（逐帧 OBJ） |
| `sequence-range` | 是 | `[start, end]` 帧区间 |
| `gap` | 否 | 帧步距，默认 1 |
| `scale` | 否 | `"嵌入网格缩放,嵌入体缩放,位移缩放"` 三元串，默认 `"1,1,1"` |
| `save-cache`（根级） | 否 | 是否写 `{name}-uAll.u` 合并缓存加速重复运行 |

配置内相对路径由 C++ 按**配置文件所在目录**解析。

### `load(config_path)` / `save_abc(output_folder)`

```python
loader = AnimationLoader()      # 不可用时 RuntimeError
loader.load("anim.json")        # 解析配置 + 读全部序列；失败 RuntimeError
loader.save_abc("out/")         # 每个 mesh 一个 .abc；目录自动创建
```

---

## func `dump_animation()`

```python
dump_animation(config_path, output_folder=None) -> None
```

一步等价于 `AnimationLoader()` + `load` + `save_abc`。`output_folder=None` 时取配置 JSON 的 `output-folder` 字段（相对路径按配置目录解析），缺省退到配置文件所在目录。CLI 封装：[`pypgo-animation-convert`](../tools/animation/abc_convert.md)。

## 用法示例

```python
import numpy as np
import pypgo

rest = mesh.vertices.ravel()
tris = mesh.elements.ravel()

# 增量：边仿真边攒帧
with pypgo.animation.AbcWriter("out.abc", "bunny",
                               rest_positions=rest, triangles=tris) as w:
    for frame in run_simulation():
        w.add_frame(frame.surface_displacement)

# .u 往返
pypgo.animation.write_u_file("u0.u", np.zeros((3 * n, 1)))
u = pypgo.animation.read_u_file("u0.u")
```

## 交叉链接

- `.u`/`.abc` 的标准生产者：[../tools/sim/_runners.md](../tools/sim/_runners.md)（`write_states`/`write_abc`）
- CLI 封装：[../tools/animation/abc_convert.md](../tools/animation/abc_convert.md)
- 同布局的应力消费者：[stress_vdb.md](stress_vdb.md)（`load_deformation_sequence` 读同一批 `.u`）
