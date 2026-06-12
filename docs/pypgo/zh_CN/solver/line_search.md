# `pypgo/solver/line_search.py` — 线搜索策略

> 源文件：`pypgo/solver/line_search.py`（56 行，策略工厂）。模块架构见 [overview.md](overview.md)。

## 定位

Newton 方向上的一维步长选择。每个工厂函数构造一个携带自身参数的 C++ 策略对象（`_core.PyLineSearchPolicy` 子类），作为不可变句柄传给 [`NewtonOptimizer(line_search=...)`](optimizer.md)。算法实现在 `src/core/nonlinearOptimization/solver/newton/lineSearch.cpp`（策略对象层在 `newtonLineSearchPolicy.cpp`）。

## 数学背景

给定当前点 $\mathbf x$、下降方向 $\mathbf p$（Newton 步），定义一维函数

$$\phi(\alpha) = E(\mathbf x + \alpha\,\mathbf p),\qquad \alpha\in(0,\ \alpha_{\text{feas}}]$$

$\alpha_{\text{feas}}$ 是能量的可行步长上限（`max_step`，见 [../energy/base.md](../energy/base.md)）——所有策略都在该窗口内搜索；Golden/Brent 可能扩张括区间，但不会越过 $\alpha_{\text{feas}}$（`NewtonSolver.cpp` 主循环对此有注释与钳制）。

## 四种策略

### `Backtrack(armijo_c=1e-4, shrink=0.5, initial_alpha=1.0)` — 默认

Armijo 充分下降回溯：从 $\alpha=\alpha_0$ 起按 $\alpha\leftarrow\rho\,\alpha$ 收缩（$\rho$=`shrink`），直到

$$\phi(\alpha) \;\le\; \phi(0) + c\,\alpha\,\nabla E^\top\mathbf p,\qquad c=\texttt{armijo\_c}$$

参数域：$c\in(0,1)$、$\rho\in(0,1)$、$\alpha_0>0$（工厂内校验）。这是标准的全局化选择：满足充分下降、接近解时通常一步接受 $\alpha=1$ 保住 Newton 二阶收敛。

### `Simple(max_iterations=100, shrink=0.5)`

最朴素的收缩：只要能量下降（$\phi(\alpha)<\phi(0)$）就接受，否则 $\alpha\leftarrow\rho\,\alpha$，至多 `max_iterations` 次。无充分下降保证，但求值最便宜。

### `Golden()`

黄金分割搜索：在括住极小点的区间内按黄金比 $\varphi=\tfrac{\sqrt5-1}{2}$ 缩小区间，线性收敛、不需导数。适合 $\phi$ 在方向上明显非单调（如接触刚接触时）的场景，比回溯更接近一维极小点，但每步求值次数更多。

### `Brents()`

Brent 法：抛物线插值加速 + 黄金分割兜底，对光滑 $\phi$ 超线性收敛。求值次数与稳健性的折中优于纯黄金分割。

## 公式 ↔ 函数表

| 策略 | 接受条件 | Python 工厂 | C++ 对象 |
|---|---|---|---|
| Armijo 回溯 | $\phi(\alpha)\le\phi(0)+c\,\alpha\,g^\top p$ | `Backtrack(armijo_c, shrink, initial_alpha)` | `_core.PyBacktrackLineSearch` |
| 简单收缩 | $\phi(\alpha)<\phi(0)$ | `Simple(max_iterations, shrink)` | `_core.PySimpleLineSearch` |
| 黄金分割 | 区间极小化 | `Golden()` | `_core.PyGoldenLineSearch` |
| Brent | 区间极小化（插值加速） | `Brents()` | `_core.PyBrentsLineSearch` |

公共基类型 `LineSearch = _core.PyLineSearchPolicy`，可用于 `isinstance` 检查与类型标注。

## 用法

```python
import pypgo.solver as ps

opt = ps.NewtonOptimizer(line_search=ps.Backtrack(armijo_c=1e-4, shrink=0.5))
# 或对强非线性问题：
opt = ps.NewtonOptimizer(line_search=ps.Brents())
```

策略对象不可变；换参数就重新调用工厂。

## 选择建议

| 场景 | 推荐 |
|---|---|
| 一般弹性静力/动力 | `Backtrack()`（默认） |
| 接触多、$\phi$ 形状复杂 | `Brents()` / `Golden()` |
| 能量求值极贵、只求稳 | `Backtrack(initial_alpha=较小)` |

## 交叉链接

- 消费方：[optimizer.md](optimizer.md)；可行步长上限来源：[../energy/base.md](../energy/base.md)（`max_step`）
