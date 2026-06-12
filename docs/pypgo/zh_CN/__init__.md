# `pypgo/__init__.py` — 包入口与惰性导入

> 源文件：`pypgo/__init__.py`（30 行，纯 Python）。所属架构见 [overview.md](overview.md)。

## 定位

pypgo 的顶层入口。不做任何实际计算，只声明包的公开子模块表，并用 PEP 562 模块级 `__getattr__` 实现**惰性导入**：子模块在第一次被访问时才真正 `import`。

## 机制

```python
__all__ = ["animation", "contact", "constraints", "fem", "energy",
           "implicit", "mesh", "parallel", "sim", "solver", "sparse", "tools"]

def __getattr__(name: str):
    if name == "_core" or name in __all__:
        module = import_module(f"{__name__}.{name}")
        globals()[name] = module      # 缓存，下次直接命中模块属性
        return module
    raise AttributeError(...)
```

两个要点：

1. **`import pypgo` 几乎零开销**——不触发 `_core.abi3.so` 的加载，也不触发可选依赖（PyTorch / PyVista / OpenVDB）的探测；只有 `pypgo.fem`、`pypgo.mesh.visualize` 等被实际访问时才付出代价。
2. **`_core` 也走同一通道**——`pypgo._core` 是 nanobind 原生模块（`_core.abi3.so`），属于私有 API；用户代码不应直接使用，各门面模块内部以 `import pypgo._core as _core` 引用它。

## 导出表

| 属性 | 内容 | 文档 |
|---|---|---|
| `pypgo.energy` | 通用势能 | [energy/overview.md](energy/overview.md) |
| `pypgo.constraints` | 硬约束 | [constraints/overview.md](constraints/overview.md) |
| `pypgo.solver` | 优化器 | [solver/overview.md](solver/overview.md) |
| `pypgo.fem` | FEM 形变能量 | [fem/overview.md](fem/overview.md) |
| `pypgo.contact` | 接触能量 | [contact/overview.md](contact/overview.md) |
| `pypgo.sim` | 动力学仿真 | [sim/overview.md](sim/overview.md) |
| `pypgo.mesh` | 网格与几何 | [mesh/overview.md](mesh/overview.md) |
| `pypgo.implicit` | 隐式场 | [implicit/overview.md](implicit/overview.md) |
| `pypgo.animation` | 动画 I/O | [animation/overview.md](animation/overview.md) |
| `pypgo.tools` | CLI 工具 | [tools/overview.md](tools/overview.md) |
| `pypgo.sparse` | 稀疏矩阵 | [sparse.md](sparse.md) |
| `pypgo.parallel` | 线程控制 | [parallel.md](parallel.md) |
| `pypgo._core` | nanobind 原生模块（私有） | — |

## 用法

```python
import pypgo                  # 快：什么都没加载
mesh = pypgo.mesh.read_obj("bunny.obj")   # 此刻才加载 pypgo.mesh 及 _core
```
