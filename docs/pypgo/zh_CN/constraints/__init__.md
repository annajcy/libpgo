# `pypgo/constraints/__init__.py` — 公开面

> 源文件：`pypgo/constraints/__init__.py`（18 行，纯重导出）。模块架构见 [overview.md](overview.md)。

## 定位

重导出扁平公开面：`pypgo.constraints.Linear` 等。docstring 声明本包约定：`ConstraintFunction` 是稳定契约，具体约束在 `functions.py` 中增长，`Bounded` 提供逐元素上下界封装。

## 导出表

| 符号 | 来源文件 | 数学对象 | 文档 |
|---|---|---|---|
| `ConstraintFunction` | `base.py` | $C(x)\in\mathbb R^m$ 抽象 | [base.md](base.md) |
| `Linear` | `functions.py` | $C(x)=Ax+c$ | [functions.md](functions.md) |
| `ConstraintFunctionSet` | `functions.py` | 约束按行拼接 | [functions.md](functions.md) |
| `Bounded` | `bounded.py` | $\ell\le C(x)\le u$ | [bounded.md](bounded.md) |
