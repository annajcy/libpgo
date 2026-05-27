# Python API Future Work

本文记录不阻塞 `runIPCSim` Python-first 迁移主线、但值得在 API 设计中预留位置的后续方向。

## Python programming / plugin API

目标是让用户以后可以用 Python 定义研究型扩展，例如：

- custom energy
- custom force
- custom constraint
- custom material
- simulation callback
- output writer

推荐路线不是一开始就暴露所有 C++ base class，而是分阶段推进：

1. 先定义 Python 侧 protocol / adapter，让用户写出来的代码稳定。
2. 等 C++ 边界干净以后，再选择少量稳定 abstract base 通过 nanobind trampoline 暴露。
3. 如果现有 C++ base class 泄漏 ownership chain、raw pointer、setup order 或 filesystem side effect，则先重构出窄接口再绑定。

示意 API：

```python
class MyEnergy(pgo.energy.Energy):
    def value(self, x, context):
        ...

    def gradient(self, x, out, context):
        ...

    def hessian(self, x, out, context):
        ...
```

这类 API 可以借鉴 Mitsuba3 的 plugin/programming model，但 `pypgo` 的第一原则仍然是数值 kernel 边界清晰，而不是照搬 plugin 机制。

## nanobind trampoline

如果后续需要 C++ solver 通过 virtual dispatch 调用 Python-defined object，使用 nanobind trampoline。

适合用 trampoline 的边界应该是 coarse-grained 的，例如：

- `Energy.value(x)`
- `Energy.gradient(x, out)`
- `Energy.hessian(x, out)`
- `Callback.on_step(frame)`
- `OutputWriter.write(frame)`

不适合的边界：

- 每个 element 调一次 Python callback
- 每个 contact pair 调一次 Python callback
- 每个 quadrature point 调一次 Python callback
- solver inner loop 中大量细粒度 virtual call 跳回 Python

这些路径会被 Python 函数调用、GIL、跨语言 dispatch 和数组转换开销压垮。

## Python kernel performance policy

Python kernel 可以用于 prototype、公式验证和小规模测试，但默认不能成为高频 inner loop。

可接受的 Python 扩展方式：

- 一次 Python 调用处理整块 NumPy array 或 torch Tensor。
- Python 只做 orchestration，重计算落在 NumPy / PyTorch / C++ kernel。
- C++ long-running kernels 释放 GIL。
- Python callback 只在 coarse-grained 边界重新获取 GIL。

不推荐的方式：

```python
for element in elements:
    energy += python_kernel(element)
```

未来 API 设计要避免把这种 element-wise Python callback 变成默认用法。

## Compiled kernel / JIT 路线

短期不把 Dr.Jit-like tracing system 作为迁移目标。更现实的演进顺序是：

1. 复用现有 C++ kernels，通过 nanobind 暴露稳定 service。
2. 支持用户把热路径写成 C++ extension 或内部 compiled backend。
3. 在 PyTorch 路径中提供 `torch.autograd.Function` / custom op 适配。
4. 评估 `torch.compile`、Numba、JAX 等可选实验路径。
5. 只有当 Python programming API 已稳定、且确实需要大量用户自定义 kernel 时，再考虑 Dr.Jit-like tracing/JIT backend。

无论未来选择哪种 compiled backend，公共 API 都应该保持 batch array contract：输入和输出是明确 shape/dtype/copy 语义的数组，而不是 C++ 内部对象。

## Milestone 放置

- M3 只做 solver-facing energy / solver API，不要求完整 plugin framework。
- M8 做 PyTorch adapter 和 energy-level autograd，不做 solver-level differentiable simulation。
- M9 之后再设计完整 Python programming / plugin API。
- JIT / tracing / user-compiled kernel 属于 M9 之后的 research API，不阻塞 `runIPCSim` 主路径迁移。

## 开放问题

- 第一批支持 Python 扩展的类型应该是 `Energy`、`Constraint`、`Callback` 还是 `OutputWriter`？
- 是否需要 `pypgo.plugins` namespace，还是放在 `pypgo.energy` / `pypgo.sim` 下？
- Python-defined object 的 lifetime 应由 `Simulation` 强引用，还是由用户显式管理？
- compiled kernel 如何缓存、版本化和清理？
- 什么规模以上的 kernel 必须拒绝 Python element-wise callback，并提示用户改用 compiled backend？
