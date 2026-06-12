# pypgo 顶层架构

> 本篇讲 pypgo 包整体的系统设计；总导航与理论流水线见 [index.md](index.md)。

## 设计原则：Python 门面 + C++ 引擎

pypgo 的所有数值计算都发生在 C++ 引擎（`src/core/`）中，通过 nanobind 绑定层（`src/python/pypgo/` → `pypgo/_core.abi3.so`，stable ABI）暴露给 Python。Python 层（`pypgo/*.py`）只负责三件事：

1. **参数校验与数据整形**——把用户输入规整为 C++ 期望的 dtype / 形状 / 连续性（见 [_utils.md](_utils.md)）；
2. **API 设计**——把 C++ 的工厂函数和句柄组织成符合 Python 习惯的类层次；
3. **轻量纯 Python 逻辑**——少数不值得下沉到 C++ 的算法（如网格连通分量、`EmbeddedVertexAttachment` 的稀疏矩阵组装）。

## Handle peer 模式

绝大多数 pypgo 类是 C++ 对象的**只读句柄**：

```
Python 对象                          C++ peer（存于 self._handle）
─────────────────────────────────────────────────────────────
pypgo.energy.PotentialEnergy   ←→   _core.PyPotentialEnergy
pypgo.solver.Optimizer         ←→   _core.PyOptimizer
pypgo.sparse.SparseMatrix      ←→   _core.PySparseMatrix
pypgo.constraints.ConstraintFunction ←→ _core.PyConstraintFunctions
```

约定（见 `pypgo/energy/__init__.py` 模块 docstring）：

- 构造函数把参数交给 `_core` 工厂（如 `_core._create_vertex_attachment(...)`），返回的 peer 存入 `_handle`；
- 此后对象**不可变**——能量类重载了 `__setattr__` 直接抛 `AttributeError`，要换参数就重建对象；
- 求值方法（`value` / `gradient` / `hessian`）经 C++ 虚函数派发，Python 用户永远不接触 `hessianInPlace` / `hessianAlloc` 等底层接口；
- `isinstance(x, PotentialEnergy)` 对所有能量类型成立（FEM、接触、代数能量都继承同一基类）。

这个模式的意义：**C++ 侧持有全部状态和拓扑**（如 Hessian 稀疏模板在构造时分配、之后原位填值），Python 侧零拷贝传递 NumPy 数组，性能关键路径上没有 Python 解释器参与。

## 惰性子模块导入

`pypgo/__init__.py` 不直接 import 任何子包，而是用模块级 `__getattr__`（PEP 562）按需加载：

```python
import pypgo
pypgo.fem          # 第一次访问时才 import pypgo.fem
```

这使 `import pypgo` 本身极快，且可选依赖（PyTorch、PyVista、OpenVDB）只在用到对应子模块时才被检查。详见 [\_\_init\_\_.md](__init__.md)。

## 子包分层

```
                ┌──────────── tools/ （CLI，组合一切）────────────┐
                │                                                │
   sim/ ──────► solver/ ◄────── energy/ ◄── fem/  contact/      │
 （时间积分）   （Newton）      （势能抽象）   （形变能）（接触能）  │
                   │               ▲            ▲                │
                   │          constraints/      │                │
                   │          （硬约束→罚）   mesh/ ◄─────────────┘
                   ▼                        （几何/体网格/处理）
               sparse.py ◄──────────────────── implicit/  animation/
              （稀疏矩阵）                     （SDF）     （动画 I/O）
```

依赖方向自上而下：`sim` 用 `solver`，`solver` 的目标函数是 `energy.PotentialEnergy`，`fem` 和 `contact` 生产能量，`mesh` 为它们提供几何。`sparse` / `parallel` / `_utils` 是横向基础设施。

## 模块 ↔ 职责总表

| 模块 | 职责 | 类型 | 文档 |
|---|---|---|---|
| `__init__.py` | 惰性导入门面 | 纯 Python | [\_\_init\_\_.md](__init__.md) |
| `_utils.py` | FFI 输入校验器 | 纯 Python | [_utils.md](_utils.md) |
| `sparse.py` | COO 稀疏矩阵封装 | 薄封装 | [sparse.md](sparse.md) |
| `parallel.py` | 引擎线程数控制 | 薄封装 | [parallel.md](parallel.md) |
| `energy/` | 通用势能类型 | 薄封装+组装 | [energy/overview.md](energy/overview.md) |
| `constraints/` | 硬约束函数 | 薄封装 | [constraints/overview.md](constraints/overview.md) |
| `solver/` | 非线性优化 | 薄封装 | [solver/overview.md](solver/overview.md) |
| `fem/` | FEM 形变能量 | 薄封装+装配编排 | [fem/overview.md](fem/overview.md) |
| `contact/` | 接触能量 | 薄封装 | [contact/overview.md](contact/overview.md) |
| `sim/` | 动力学仿真 | 编排层 | [sim/overview.md](sim/overview.md) |
| `mesh/` | 网格与几何 | 混合 | [mesh/overview.md](mesh/overview.md) |
| `implicit/` | 隐式曲面场 | 薄封装 | [implicit/overview.md](implicit/overview.md) |
| `animation/` | 动画/应力 I/O | 混合 | [animation/overview.md](animation/overview.md) |
| `tools/` | CLI 入口 | 纯 Python | [tools/overview.md](tools/overview.md) |

## C++ 引擎对应关系

| pypgo 子包 | C++ 引擎目录（`src/core/`） | 数学内容 |
|---|---|---|
| `fem` | `solidDeformationModel/` | formulation、本构、装配、质量 |
| `energy` | `genericPotentialEnergies/`, `constraintPotentialEnergies/` | 二次/线性能量、顶点拉拽 |
| `constraints` | `nonlinearOptimization/constraints/` | 约束函数、罚能量 |
| `solver` | `nonlinearOptimization/solver/newton/` | Newton、线搜索、稀疏后端 |
| `contact` | `contact/`（含 `ipc/`） | IPC 屏障、CCD、罚接触 |
| `sim` | `simulation/` | 积分器、step-aware 能量 |
| `mesh` | `mesh/`, `volumetricMesh/`, `volumetricMeshMeshing/` | 几何查询、BVH、网格生成 |
| `implicit` | `implicitSurface/` | SDF、等值面 |
| `sparse` | `eigenSupport/` | Eigen 稀疏封装（MKL 加速） |
| `parallel` | `parallelism/` | TBB/OpenMP 线程控制 |

绑定层一一对应：`src/python/pypgo/<子模块>/bindings.cpp`（注册 `_core` 函数名）+ `core.cpp`（实现包装逻辑），模块入口为 `src/python/pypgo/module.cpp`。
