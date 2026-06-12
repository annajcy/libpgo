# `pypgo/fem/plastic.py` — 塑性参数化模型

> 源文件：`pypgo/fem/plastic.py`（37 行）。模块架构见 [overview.md](overview.md)。
>
> C++ 实现：`src/core/solidDeformationModel/material/plastic/`；绑定 `src/python/pypgo/fem/plastic/`。

## 共同数学框架

pypgo 的"塑性"是**乘法分解**意义上的静态参数化（不含屈服面/流动法则的演化——演化由外层优化或用户驱动）。每个单元携带塑性参数 $\mathbf a$，定义塑性形变矩阵 $\mathbf A(\mathbf a)$，弹性能量在弹性部分上求值：

$$\mathbf F_e=\mathbf F\,\mathbf A^{-1},\qquad
E_e=\int_{\Omega_e}\Psi\big(\mathbf F_e\big)\,\det\mathbf A\;dV$$

C++ 虚接口 `PlasticModel`（`plasticModel.h`）要求实现 $\mathbf A(\mathbf a)$、$\mathbf A^{-1}$、$\det\mathbf A$ 及其对 $\mathbf a$ 的一、二阶导——这正是 [`DeformationEnergy.plastic_gradient/plastic_jacobian`](energy.md) 与可微层（[torch.md](torch.md)）所需的链条。

参数 $\mathbf a$ 存放在塑性[参数场](fields.md)中，逐元素可异；`dofs` 是**每元素**的参数个数。

---

## class `PlasticModel`

抽象基类，持有 `_core.PyPlasticModel` peer。

### 属性 `name`

C++ 模型标识字符串，[`deformation_energy`](energy.md) 工厂派发用。

### 属性 `dofs`

```python
model.dofs -> int
```

每元素塑性参数个数（构造时选定）。`repr` 形如 `VolumetricPlasticity(dofs=6)`。

---

## class `VolumetricPlasticity`

```python
VolumetricPlasticity(dofs: int = 6)
```

体单元塑性。`dofs` 只接受 0 / 3 / 6，对应三种 C++ 参数化（`plasticModelFactory.cpp` 派发）：

| `dofs` | $\mathbf A(\mathbf a)$ | C++ 类 | 含义 |
|---|---|---|---|
| 0 | $\mathbf I$（常量） | `PlasticModel3DConstant` | 纯弹性，无塑性自由度——最常用的默认 |
| 3 | $R^\top\mathrm{diag}(a_1,a_2,a_3)\,R$ | `PlasticModel3D3DOF`（`plasticModel3D3DOF.cpp:32-36`） | 固定纤维系 $R$ 下的主塑性拉伸；默认 $\mathbf a=(1,1,1)$ |
| 6 | 对称矩阵（6 个独立分量） | `PlasticModel3D6DOF` | 完整对称塑性拉伸（无旋转部分） |

数值防护：$A^{-1}$ 在 $a_i$ 低于阈值时截断（`plasticModel3D3DOF.cpp:38-48`），避免奇异。

**自检**：$\mathbf a$ 取默认（恒等）时 $\mathbf F_e=\mathbf F$、$\det\mathbf A=1$，能量与纯弹性一致 ✓。

---

## class `ShellPlasticity`

```python
ShellPlasticity(dofs: int = 1)
```

壳单元塑性。`dofs` 只接受 0 / 1：

| `dofs` | 含义 | C++ 类 |
|---|---|---|
| 0 | 无塑性 | — |
| 1 | 各向同性面内"均匀拉伸"塑性（静止第一基本形式的缩放因子） | `PlasticModel2DFundamentalFormsUniformStretch` |

---

## 用法示例

```python
import pypgo

# 纯弹性（最常见）：
plastic = pypgo.fem.VolumetricPlasticity(dofs=0)

# 可优化的逐元素塑性拉伸（材料设计 / 逆问题）：
plastic6 = pypgo.fem.VolumetricPlasticity(dofs=6)
deform = pypgo.fem.deformation_energy(
    sim_mesh, formulation=pypgo.fem.TetLinear(),
    elastic=pypgo.fem.StableNeo(), elastic_field=pypgo.fem.ElementwiseField(),
    plastic=plastic6, plastic_field=pypgo.fem.ElementwiseField(),
)
deform.num_plastic_params      # 6（每元素）
deform.set_plastic_values(values)   # (num_elements, 6)
```

## 交叉链接

- 参数存储：[fields.md](fields.md)；参数导数面：[energy.md](energy.md)（`plastic_gradient` 等）
- 以塑性场为优化变量：[energy.md](energy.md)（`plastic_material_energy`）、[torch.md](torch.md)（`PlasticStaticEquilibriumLayer`）
