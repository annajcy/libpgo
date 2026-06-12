# `pypgo/tools/sim/__init__.py` — 包公开面

> 源文件：`pypgo/tools/sim/__init__.py`（9 行）。模块架构见 [overview.md](overview.md)。

纯命名空间声明，无逻辑、无 import。`_` 前缀私有模块（`_cli`/`_config`/`_scene`/`_runners`/`_outputs`）不在 `__all__` 中。

## 导出表（`__all__`，7 个符号）

| 符号 | 命令 | 文档 |
|---|---|---|
| `tet_static` | `pypgo-sim-tet-static` | [tet_static.md](tet_static.md) |
| `tet_dynamic` | `pypgo-sim-tet-dynamic` | [tet_dynamic.md](tet_dynamic.md) |
| `cubic_static` | `pypgo-sim-cubic-static` | [cubic_static.md](cubic_static.md) |
| `cubic_dynamic` | `pypgo-sim-cubic-dynamic` | [cubic_dynamic.md](cubic_dynamic.md) |
| `shell_static` | `pypgo-sim-shell-static` | [shell_static.md](shell_static.md) |
| `shell_dynamic` | `pypgo-sim-shell-dynamic` | [shell_dynamic.md](shell_dynamic.md) |
| `batch` | `pypgo-sim-batch` | [batch.md](batch.md) |

## 用法示例

```python
from pypgo.tools.sim import tet_static
code = tet_static.main(["--config", "scene.json", "--output-dir", "/tmp/run"])
```
