"""Small nbformat helpers for deterministic example notebook generation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import textwrap

import nbformat
from nbformat.v4 import new_code_cell, new_markdown_cell, new_notebook


@dataclass(frozen=True)
class Cell:
    kind: str
    source: str


def md(source: str) -> Cell:
    return Cell("markdown", source)


def code(source: str) -> Cell:
    return Cell("code", source)


def repo_root() -> Path:
    for candidate in Path(__file__).resolve().parents:
        if (candidate / ".git").exists():
            return candidate
    raise RuntimeError("Could not find repository root")


def clean_source(source: str) -> str:
    return textwrap.dedent(source).strip("\n") + "\n"


def write_notebook(path: Path, cells: list[Cell]) -> None:
    notebook_cells = []
    for index, cell in enumerate(cells, start=1):
        source = clean_source(cell.source)
        if cell.kind == "markdown":
            notebook_cell = new_markdown_cell(source)
        elif cell.kind == "code":
            notebook_cell = new_code_cell(source)
        else:
            raise ValueError(f"Unsupported notebook cell kind: {cell.kind}")
        notebook_cell["id"] = f"{path.stem.replace('_', '-')}-{index:02d}"
        notebook_cells.append(notebook_cell)

    notebook = new_notebook(
        cells=notebook_cells,
        metadata={
            "kernelspec": {
                "display_name": "libpgo",
                "language": "python",
                "name": "python3",
            },
            "language_info": {
                "name": "python",
                "pygments_lexer": "ipython3",
            },
        },
    )

    nbformat.validate(notebook)
    path.parent.mkdir(parents=True, exist_ok=True)
    nbformat.write(notebook, path)
