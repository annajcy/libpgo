#!/usr/bin/env python3
"""Reject direct first-party TBB scheduling calls outside pgo::parallel."""

from __future__ import annotations

import argparse
import re
import sys
import tempfile
from pathlib import Path


DENIED_PATTERNS = (
    (
        "direct TBB parallel scheduling API",
        re.compile(r"\b(?:tbb|oneapi::tbb)::parallel_[A-Za-z_][A-Za-z0-9_]*\b"),
    ),
    (
        "TBB task-group scheduling API",
        re.compile(r"\b(?:tbb|oneapi::tbb)::task_group(?:_context)?\b"),
    ),
    (
        "TBB flow-graph scheduling API",
        re.compile(r"\b(?:tbb|oneapi::tbb)::flow::[A-Za-z_][A-Za-z0-9_]*\b"),
    ),
    (
        "TBB task-arena scheduling API",
        re.compile(r"\b(?:tbb|oneapi::tbb)::task_arena\b"),
    ),
    (
        "TBB current-arena scheduling API",
        re.compile(
            r"\b(?:tbb|oneapi::tbb)::this_task_arena::(?:enqueue|isolate)\b"
        ),
    ),
    (
        "TBB alternate scheduling header",
        re.compile(
            r"#[ \t]*include[ \t]*[<\"](?:oneapi/)?tbb/"
            r"(?:parallel_[A-Za-z_][A-Za-z0-9_]*|task_group|flow_graph|task_arena)\.h[>\"]"
        ),
    ),
)
CPP_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"}
EXCLUDED_PREFIX = "src/core/parallel/"


def strip_comments_and_literals(source: str) -> str:
    output: list[str] = []
    index = 0
    state = "code"
    raw_delimiter = ""
    while index < len(source):
        char = source[index]
        if state == "code":
            if source.startswith("//", index):
                output.extend("  ")
                index += 2
                state = "line-comment"
            elif source.startswith("/*", index):
                output.extend("  ")
                index += 2
                state = "block-comment"
            elif source.startswith('R"', index):
                delimiter_end = source.find("(", index + 2)
                if delimiter_end == -1:
                    output.append(" ")
                    index += 1
                else:
                    raw_delimiter = source[index + 2 : delimiter_end]
                    output.extend(" " * (delimiter_end + 1 - index))
                    index = delimiter_end + 1
                    state = "raw-string"
            elif char == '"':
                output.append(" ")
                index += 1
                state = "string"
            elif char == "'":
                output.append(" ")
                index += 1
                state = "character"
            else:
                output.append(char)
                index += 1
        elif state == "line-comment":
            output.append("\n" if char == "\n" else " ")
            index += 1
            if char == "\n":
                state = "code"
        elif state == "block-comment":
            if source.startswith("*/", index):
                output.extend("  ")
                index += 2
                state = "code"
            else:
                output.append("\n" if char == "\n" else " ")
                index += 1
        elif state == "raw-string":
            terminator = ")" + raw_delimiter + '"'
            if source.startswith(terminator, index):
                output.extend(" " * len(terminator))
                index += len(terminator)
                state = "code"
            else:
                output.append("\n" if char == "\n" else " ")
                index += 1
        else:
            quote = '"' if state == "string" else "'"
            if char == "\\" and index + 1 < len(source):
                output.append(" ")
                output.append("\n" if source[index + 1] == "\n" else " ")
                index += 2
            elif char == quote:
                output.append(" ")
                index += 1
                state = "code"
            else:
                output.append("\n" if char == "\n" else " ")
                index += 1
    return "".join(output)


def scan_source(repo_root: Path) -> dict[str, list[tuple[int, str, str]]]:
    source_root = repo_root / "src"
    if not source_root.is_dir():
        raise ValueError(f"missing source directory: {source_root}")
    calls: dict[str, list[tuple[int, str, str]]] = {}
    for path in sorted(source_root.rglob("*")):
        if not path.is_file() or path.suffix.lower() not in CPP_SUFFIXES:
            continue
        relative = path.relative_to(repo_root).as_posix()
        if relative.startswith(EXCLUDED_PREFIX):
            continue
        filtered = strip_comments_and_literals(path.read_text(encoding="utf-8", errors="replace"))
        matches = []
        for description, pattern in DENIED_PATTERNS:
            matches.extend(
                (
                    filtered.count("\n", 0, match.start()) + 1,
                    match.group(0),
                    description,
                )
                for match in pattern.finditer(filtered)
            )
        matches.sort(key=lambda match: (match[0], match[1], match[2]))
        if matches:
            calls[relative] = matches
    return calls


def inventory_errors(repo_root: Path) -> list[str]:
    errors: list[str] = []
    for path, calls in scan_source(repo_root).items():
        for line, token, description in calls:
            errors.append(f"{path}:{line}: {description}: {token}")
    return errors


def run_self_test() -> list[str]:
    failures: list[str] = []
    with tempfile.TemporaryDirectory(prefix="pgo-tbb-inventory-") as directory:
        root = Path(directory)
        source = root / "src/core/example/example.cpp"
        backend = root / "src/core/parallel/backend.cpp"
        source.parent.mkdir(parents=True)
        backend.parent.mkdir(parents=True)
        source.write_text(
            "#include <tbb/enumerable_thread_specific.h>\n"
            "#include <tbb/combinable.h>\n"
            "#include <tbb/concurrent_vector.h>\n"
            "#include <tbb/cache_aligned_allocator.h>\n"
            "#include <tbb/spin_mutex.h>\n"
            "// tbb::parallel_for ignored\n"
            'const char *text = "tbb::parallel_reduce ignored";\n'
            'const char *raw = R"tag(#include <tbb/task_arena.h>)tag";\n'
            "tbb::enumerable_thread_specific<int> tls;\n"
            "tbb::combinable<int> combined;\n"
            "tbb::concurrent_vector<int> values;\n"
            "tbb::cache_aligned_allocator<int> allocator;\n"
            "tbb::spin_mutex mutex;\n",
            encoding="utf-8",
        )
        backend.write_text(
            "#include <tbb/task_arena.h>\n"
            "void backend() { tbb::parallel_pipeline(); tbb::task_arena arena; }\n",
            encoding="utf-8",
        )
        if inventory_errors(root):
            failures.append(
                "comments/literals, permitted utilities, or the implementation prefix "
                "produced a false positive"
            )
        injected = {
            "parallel_for": "void f() { tbb::parallel_for(0, 1, [](int) {}); }\n",
            "parallel_reduce": "void f() { tbb::parallel_reduce(0, 1, 0, f, g); }\n",
            "parallel_sort": "void f() { tbb::parallel_sort(begin, end); }\n",
            "parallel_pipeline": "void f() { tbb::parallel_pipeline(1, filters); }\n",
            "parallel_deterministic_reduce": (
                "void f() { oneapi::tbb::parallel_deterministic_reduce(range, body); }\n"
            ),
            "task_group": (
                "#include <tbb/task_group.h>\n"
                "void f() { tbb::task_group group; group.run([] {}); group.wait(); }\n"
            ),
            "flow_graph": (
                "#include <oneapi/tbb/flow_graph.h>\n"
                "void f() { oneapi::tbb::flow::graph graph; graph.wait_for_all(); }\n"
            ),
            "task_arena_execute": (
                "#include <tbb/task_arena.h>\n"
                "void f() { tbb::task_arena arena; arena.execute([] {}); }\n"
            ),
            "task_arena_enqueue": (
                "#include <oneapi/tbb/task_arena.h>\n"
                "void f() { oneapi::tbb::task_arena arena; arena.enqueue([] {}); }\n"
            ),
            "task_arena_header": "#include <tbb/task_arena.h>\nvoid f() {}\n",
            "current_arena_isolate": (
                "void f() { tbb::this_task_arena::isolate([] {}); }\n"
            ),
        }
        for name, fixture in injected.items():
            source.write_text(fixture, encoding="utf-8")
            errors = inventory_errors(root)
            if not errors:
                failures.append(f"injected {name} scheduling API was not rejected")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--repo-root", type=Path)
    args = parser.parse_args()
    try:
        errors = run_self_test() if args.self_test else inventory_errors(
            (args.repo_root or Path(__file__).resolve().parents[1]).resolve()
        )
    except ValueError as error:
        errors = [str(error)]
    if errors:
        print("TBB scheduling inventory guard failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1
    print("TBB scheduling inventory guard passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
