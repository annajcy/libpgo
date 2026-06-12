# `pypgo/tools/__init__.py` — 包公开面

> 源文件：`pypgo/tools/__init__.py`（3 行）。模块架构见 [overview.md](overview.md)。

纯命名空间声明，无逻辑、无 import（子包按需惰性导入，避免 CLI 启动拖入全部依赖）。docstring：替代历史 libpgo tools 的 CLI 入口模块。

## 导出表（`__all__`，3 个符号）

| 符号 | 类别 | 文档 |
|---|---|---|
| `animation` | 子包（动画后处理 CLI） | [animation/overview.md](animation/overview.md) |
| `mesh` | 子包（网格处理 CLI） | [mesh/overview.md](mesh/overview.md) |
| `sim` | 子包（仿真 CLI 套件） | [sim/overview.md](sim/overview.md) |

## 用法示例

```python
# 工具入口同时是普通 Python 函数，可编程调用：
from pypgo.tools.sim import tet_static
tet_static.main(["--config", "scene.json", "--output-dir", "/tmp/run"])
```
