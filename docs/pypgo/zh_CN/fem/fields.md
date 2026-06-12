# `pypgo/fem/fields.py` — 参数场描述符与视图

> 源文件：`pypgo/fem/fields.py`（78 行，纯 Python dataclass + 只读视图）。模块架构见 [overview.md](overview.md)。

## 共同数学框架

形变能量除了位移 $\mathbf u$ 还依赖材料参数：弹性参数 $\mathbf b$（如 Koiter 壳的 $E,\nu,h$ 通道）与塑性参数 $\mathbf a$（如主塑性拉伸）。参数以**场**的形式组织：

$$\theta\in\mathbb R^{n_{\text{rows}}\times c},\qquad
c=\text{每元素通道数},\quad
n_{\text{rows}}=\begin{cases}n_e & \text{elementwise（逐元素独立）}\\ 1 & \text{constant（全网格共享）}\end{cases}$$

本文件提供两类对象：**描述符**（构造能量时声明场的形态与初值）与**视图**（能量建好后只读访问 C++ 持有的实际状态）。

各模型的通道数 $c$：体本构（StVK/StableNeo/Linear/MooneyRivlin）$c=0$（参数烘焙进模型，见 [elastic.md](elastic.md)）；Koiter 壳 $c=5$；塑性 $c=$ [`PlasticModel.dofs`](plastic.md)（0/3/6 体、0/1 壳）。

---

## class `ElementwiseField`（冻结 dataclass）

```python
ElementwiseField(values=None)
```

逐元素参数场描述符。

| `values` | 行为 |
|---|---|
| `None`（默认） | 请 C++ 按网格/材料 payload 生成模型默认值（如 Koiter 从 `KoiterStVKShellMaterial` 的 $E,\nu,h$ 播种、塑性取恒等） |
| 平铺 `(n_e·c,)` 或 `(n_e, c)` | 显式初值；形状由 [`deformation_energy`](energy.md) 工厂按模型通道数校验（`energy.py` 的 `_field_values_array`） |

---

## class `ConstantField`（冻结 dataclass）

```python
ConstantField(values=None)
```

全网格共享的参数场：一组 $c$ 个参数被所有单元共用（优化变量从 $n_e\cdot c$ 缩到 $c$，适合"全局均匀材料"的反演）。`values` 为 `None`（C++ 播种）或 `(c,)` / `(1, c)`。

---

## class `ParameterField`

```python
ParameterField(core)   # core: _core.PyParameterField；用户不直接构造
```

C++ 持有状态的**只读视图**，经 [`DeformationEnergy.elastic_field` / `.plastic_field`](energy.md) 取得。

### 属性 `domain`

场所属域的字符串标识（如体/壳）。

### 属性 `model`

关联的材料模型名（与 [elastic.md](elastic.md) / [plastic.md](plastic.md) 的 `name` 一致）。

### 属性 `num_elements` / `num_value_rows` / `num_channels`

$n_e$、$n_{\text{rows}}$（elementwise 时 $=n_e$，constant 时 $=1$）、$c$。

### 属性 `values`

```python
field.values -> ndarray (num_value_rows, num_channels)
```

当前参数值的**拷贝**（修改返回数组不影响能量）。写入要走 [`DeformationEnergy.set_elastic_values` / `set_plastic_values`](energy.md)——这保证 C++ 侧缓存一致性，也是 [`ShellDensityElasticThickness`](mass.md) 能"实时看到"新厚度的机制。

---

## 用法示例

```python
import pypgo

deform = pypgo.fem.deformation_energy(
    shell_mesh, formulation=pypgo.fem.KoiterShell(),
    elastic=pypgo.fem.KoiterStVK(),
    elastic_field=pypgo.fem.ElementwiseField(),      # C++ 按壳材料播种 5 通道
    plastic=pypgo.fem.ShellPlasticity(dofs=0),
    plastic_field=pypgo.fem.ElementwiseField(),
)

f = deform.elastic_field          # ParameterField 视图
f.num_channels                    # 5（E_m, ν_m, E_b, ν_b, h）
vals = f.values                   # (n_e, 5) 拷贝
vals[:, 4] *= 2.0                 # 改厚度
deform.set_elastic_values(vals)   # 写回（经能量，不经视图）
```

## 交叉链接

- 工厂与写回：[energy.md](energy.md)；通道语义：[elastic.md](elastic.md)、[plastic.md](plastic.md)
- 参数→质量耦合：[mass.md](mass.md)；参数作为优化变量：[torch.md](torch.md)
