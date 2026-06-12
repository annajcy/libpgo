# `pypgo/mesh/volume/material.py` — 体网格材料载体

> 源文件：`pypgo/mesh/volume/material.py`（72 行，纯 Python dataclass + payload 转换）。子包架构见 [overview.md](overview.md)。
>
> 材料在 pypgo 中是**纯数据**：本模块的 dataclass 不做任何力学计算，只携带参数；力学语义在 C++ 本构构造期**烘焙**（见下文"参数走向"）。C++ 对应 `VolumetricMesh::ENuMaterial` / `MooneyRivlinMaterial`（Vega FEM，`src/core/volumetricMesh/`）。

## class `ENuMaterial`（dataclass）

```python
ENuMaterial(name="defaultMaterial", density=1000.0, E=1e9, nu=0.45)
```

各向同性线弹性材料参数。

| 字段 | 含义 | 默认 | 单位 |
|---|---|---|---|
| `name` | 材料名（VEG 内标识） | `"defaultMaterial"` | — |
| `density` | 密度 $\rho$ | `1000.0` | kg/m³ |
| `E` | 杨氏模量 | `1e9` | Pa |
| `nu` | 泊松比 $\nu$ | `0.45` | — |
| `type`（ClassVar） | 判别标签 `"enu"` | — | — |

### 属性 `lam`

第一 Lamé 常数（as-implemented，27-28 行）：

$$\lambda=\frac{E\,\nu}{(1+\nu)(1-2\nu)}$$

### 属性 `mu`

剪切模量（第二 Lamé 常数，31-32 行）：

$$\mu=\frac{E}{2(1+\nu)}$$

两式与 C++ `ENuMaterial::getLambda()/getMu()` 逐字符一致（`volumetricMeshENuMaterial.h:79-86`），并已数值验证（$E=10^6,\ \nu=0.45$：$\lambda=3.103448\times10^6$、$\mu=3.448276\times10^5$，Python 与手算误差为 0）。

**有效域**：$\nu\to0.5$ 时 $\lambda\to\infty$（不可压极限）、$\nu=0.5$ 除零；物理上要求 $\nu\in(-1,\,0.5)$（[`cubic_mesher`](../processing/volume.md) 的 C++ 校验即此区间）。

### 参数走向（与 ../../fem/elastic.md 的烘焙关系）

```
ENuMaterial(E, ν) ──write_veg/VolumeMesh──> C++ VolumetricMesh::ENuMaterial
        │ fem.SimulationMesh.create_volumetric（仅接受 ENu，见 core.md）
        ▼
fem.deformation_energy(elastic=StableNeo()/StVK()/...) 构造期：
        getMu()/getLambda() → (μ, λ) 烘焙进每个元素的本构
```

体本构**没有运行期弹性参数通道**（`getNumParameters()==0`，见 [../../fem/elastic.md](../../fem/elastic.md)）——改 `ENuMaterial` 字段只影响**之后**构建的 `VolumeMesh`/能量，已有能量不会变。`density` 则进入质量矩阵（[../../fem/mass.md](../../fem/mass.md) 的 `VolumeDensity`）。

---

## class `MooneyRivlinMaterial`（dataclass）

```python
MooneyRivlinMaterial(name="mooneyRivlinMaterial", density=1000.0,
                     mu01=0.0, mu10=0.0, v1=0.0)
```

Mooney–Rivlin 超弹材料参数（Vega 约定的三系数形式）。

| 字段 | 含义 | 默认 |
|---|---|---|
| `name` | 材料名 | `"mooneyRivlinMaterial"` |
| `density` | 密度 $\rho$ | `1000.0` |
| `mu01` | $\mu_{01}$ —— $(\bar I_2-3)$ 项系数 | `0.0` |
| `mu10` | $\mu_{10}$ —— $(\bar I_1-3)$ 项系数 | `0.0` |
| `v1` | $v_1$ —— 体积罚系数 | `0.0` |
| `type`（ClassVar） | `"mooney_rivlin"` | — |

三系数对应两项 Mooney–Rivlin 级数（$C_{10}=\mu_{10}$、$C_{01}=\mu_{01}$ 加体积项），是 [../../fem/elastic.md](../../fem/elastic.md) 中广义级数 $W=\sum_{p,q}C_{pq}(\bar I_1-3)^p(\bar I_2-3)^q+\text{体积项}$ 的 $N=1$ 特例载体；系数如何进入 `ElasticModel3DMooneyRivlin` 见该篇。

**当前限制**（as-implemented）：`fem.SimulationMesh.create_volumetric` 只接受全 ENu 材料的网格（`src/python/pypgo/mesh/volume/core.cpp:525-534` 逐元素检查并抛错）——`MooneyRivlinMaterial` 目前可经 `VegFile` 读/写/存（`.veg` 的 `MOONEYRIVLIN` 行），但还接不进 pypgo 的 FEM 装配路径。

---

## `MaterialLike`

```python
MaterialLike = ENuMaterial | MooneyRivlinMaterial
```

类型别名：所有接受"材料"的 API（`VegFile.materials`、`VolumeMesh` 的 regions）的元素类型。

## 内部 payload 转换（模块私有）

| 函数 | 方向 | 说明 |
|---|---|---|
| `_material_to_core_payload(m)` | Python → C++ | 派发到 `_core.create_enu_material_payload` / `create_mooney_rivlin_material_payload` |
| `_wrap_material_payload(m)` | C++ → Python | 按 peer 类型还原 dataclass |

> 绑定层还定义了正交各向异性 payload（`PyVegOrthotropicMaterialPayload`），但 Python 层未导出包装——读到含 orthotropic 材料的 `.veg` 时 `_wrap_material_payload` 抛 `RuntimeError`。

## 用法示例

```python
from pypgo.mesh.volume import ENuMaterial

rubber = ENuMaterial(name="rubber", E=5e5, nu=0.48, density=1100.0)
rubber.mu    # 168918.9... = E / (2(1+ν))
rubber.lam   # 4054054.0... = Eν / ((1+ν)(1-2ν))

# λ/μ 比衡量可压缩性：ν→0.5 时比值发散
rubber.lam / rubber.mu   # 24.0 = 2ν/(1-2ν)
```

## 交叉链接

- 消费方：[core.md](core.md)（`VegFile` / `VolumeMesh`）
- $(\mu,\lambda)$ 烘焙进本构：[../../fem/elastic.md](../../fem/elastic.md)
- $\rho$ 进质量矩阵：[../../fem/mass.md](../../fem/mass.md)
