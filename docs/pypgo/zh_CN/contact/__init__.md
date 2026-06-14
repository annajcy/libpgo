# `pypgo/contact/__init__.py` — 包公开面

> 源文件：`pypgo/contact/__init__.py`（36 行）。模块架构见 [overview.md](overview.md)。

纯再导出模块：把数据层（`surface`）、参数层（`params`）、状态混入（`base`）与能量实现（`energies`）拍平成一个公共命名空间。包 docstring 点明分工——`ContactSurface`/`ContactVertexEmbedding` 是稳定的数据层，`energies.py` 是预期增长的一侧。

## 导出表（`__all__`，10 个符号）

| 符号 | 来源 | 类型 | 文档 |
|---|---|---|---|
| `ContactSurface` | `surface.py` | 冻结 dataclass（表面 + 映射 $S$） | [surface.md](surface.md) |
| `ContactVertexEmbedding` | `surface.py` | 冻结 dataclass（顶点嵌入元数据） | [surface.md](surface.md) |
| `FloorEnergy` | `energies.py` | `PotentialEnergy` | [energies.md](energies.md) |
| `SampledPenaltyEnergy` | `energies.py` | `StatefulContactMixin + PotentialEnergy`；可选 `friction=FrictionParameters(...)` | [energies.md](energies.md) |
| `IPCEnergy` | `energies.py` | 同上（屏障型） | [energies.md](energies.md) |
| `FloorParameters` | `params.py` | 冻结 dataclass | [params.md](params.md) |
| `SampledPenaltyParameters` | `params.py` | 冻结 dataclass | [params.md](params.md) |
| `FrictionParameters` | `params.py` | 冻结 dataclass | [params.md](params.md) |
| `IPCParameters` | `params.py` | 冻结 dataclass | [params.md](params.md) |
| `ObstacleSpec` | `params.py` | 冻结 dataclass（IPC 障碍物） | [params.md](params.md) |

注意 `StatefulContactMixin`（[base.md](base.md)）**不在** `__all__` 中——它是实现细节，用户通过具体能量类的 `begin_step` / `is_step_dependent` 间接使用。

```python
import pypgo.contact as contact
contact.IPCEnergy, contact.IPCParameters, contact.ObstacleSpec  # 均可直接访问
```
