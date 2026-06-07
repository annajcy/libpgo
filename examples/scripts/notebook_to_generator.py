#!/usr/bin/env python3
"""Convert an existing .ipynb notebook back into a generator script.

Usage:

    conda run -n libpgo python examples/scripts/notebook_to_generator.py examples/mesh_api_demo.ipynb

This reads the notebook, extracts markdown / code cells, and writes a
``generate_<name>.py`` script next to the other generators.  The generated
script uses ``notebook_builder`` (``md``, ``code``, ``write_notebook``) and
follows the same conventions as the hand-written generators.
"""

from __future__ import annotations

import argparse
import textwrap
from pathlib import Path

import nbformat

# Same helpers the generators use — keep the import path relative to the repo.
try:
    from notebook_builder import repo_root, write_notebook
except ImportError:
    from examples.scripts.notebook_builder import repo_root, write_notebook


# ---------------------------------------------------------------------------
# Cell source → generator source helpers
# ---------------------------------------------------------------------------

_INDENT = "        "  # 8 spaces — inside a list literal inside a function body


def _escape_triple_quotes(source: str) -> str:
    """Escape triple-quote sequences inside the cell source so they don't
    prematurely close the generator's triple-quoted string."""
    return source.replace('"""', r"\"\"\"")


def _format_cell_source(source: str) -> str:
    """Wrap cell source into a triple-quoted string indented for the CELLS list."""
    dedented = textwrap.dedent(source).rstrip() + "\n"
    escaped = _escape_triple_quotes(dedented)
    wrapped = f'"""{escaped}"""'
    return textwrap.indent(wrapped, _INDENT)


def _cell_to_generator_source(cell) -> str:
    """Convert one nbformat cell to a ``md(...)`` or ``code(...)`` generator line."""
    source = "".join(cell["source"])
    if cell["cell_type"] == "markdown":
        return f"md(\n{_format_cell_source(source)}\n{_INDENT}),"
    elif cell["cell_type"] == "code":
        return f"code(\n{_format_cell_source(source)}\n{_INDENT}),"
    else:
        return f"# skipped unsupported cell_type={cell['cell_type']!r}"


# ---------------------------------------------------------------------------
# Notebook → generator script
# ---------------------------------------------------------------------------

_GENERATOR_TEMPLATE = '''#!/usr/bin/env python3
"""Generate examples/{notebook_name}.ipynb.

Run from the repository root:

    conda run -n libpgo python examples/scripts/{script_name}.py
"""

from __future__ import annotations

from examples.scripts.notebook_builder import code, md, repo_root, write_notebook


CELLS = [
{cells}
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "examples" / "{notebook_name}.ipynb", CELLS)


if __name__ == "__main__":
    main()
'''


def notebook_to_generator(notebook_path: Path, output_path: Path | None = None) -> Path:
    """Read *notebook_path* and write a generator script.

    Parameters
    ----------
    notebook_path : Path
        An existing ``.ipynb`` file.
    output_path : Path or None
        Where to write the generator.  Defaults to
        ``examples/scripts/generate_<stem>.py`` alongside the other generators.

    Returns
    -------
    Path
        The path of the written generator script.
    """
    notebook_path = notebook_path.resolve()
    if not notebook_path.suffix == ".ipynb":
        raise ValueError(f"notebook_path must be a .ipynb file, got {notebook_path}")

    stem = notebook_path.stem

    if output_path is None:
        root = repo_root()
        output_path = root / "examples" / "scripts" / f"generate_{stem}.py"

    # Read notebook
    nb = nbformat.read(str(notebook_path), as_version=4)

    # Convert every cell
    cell_lines: list[str] = []
    for cell in nb.cells:
        cell_lines.append(_cell_to_generator_source(cell))

    cells_block = "\n".join(cell_lines)

    # Fill template — use the *original* notebook stem so the generator
    # always targets the same .ipynb name regardless of -o.
    generator_source = _GENERATOR_TEMPLATE.format(
        notebook_name=stem,
        script_name=output_path.stem,
        cells=cells_block,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(generator_source)
    print(f"Wrote generator: {output_path}")
    return output_path


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert a .ipynb notebook back into a notebook_builder generator script."
    )
    parser.add_argument(
        "notebook",
        type=Path,
        help="Path to an existing .ipynb notebook.",
    )
    parser.add_argument(
        "-o", "--output",
        type=Path,
        default=None,
        help="Output path for the generator script (default: examples/scripts/generate_<stem>.py).",
    )
    args = parser.parse_args()
    notebook_to_generator(args.notebook, args.output)


if __name__ == "__main__":
    main()
