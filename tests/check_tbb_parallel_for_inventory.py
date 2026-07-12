#!/usr/bin/env python3
"""Guard the production tbb::parallel_for inventory.

The scanner is intentionally lexical: it ignores comments and string/character
literals, scans only C/C++ files below src/, and excludes the pgo::parallel
backend. The manifest line numbers are audit hints; per-file active-call counts
are the stable comparison key.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import tempfile
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


TOKEN = re.compile(r"\btbb::parallel_for\b")
CPP_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"}
EXCLUDED_PREFIX = "src/core/parallelism/"
REQUIRED_ENTRY_FIELDS = {
    "id",
    "path",
    "occurrence",
    "line",
    "index_type",
    "partitioner",
    "coupling",
    "classification",
    "reason",
}
CLASSIFICATIONS = {
    "migrate-v1",
    "typed-index",
    "chunk-scratch",
    "partitioner",
    "tls-coupled",
    "reduce",
    "non-index-range",
}


def strip_comments_and_literals(source: str) -> str:
    """Replace comments and literals with spaces while preserving newlines."""

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


def scan_source(repo_root: Path) -> dict[str, list[int]]:
    source_root = repo_root / "src"
    calls: dict[str, list[int]] = {}
    if not source_root.is_dir():
        raise ValueError(f"missing source directory: {source_root}")

    for path in sorted(source_root.rglob("*")):
        if not path.is_file() or path.suffix.lower() not in CPP_SUFFIXES:
            continue
        relative = path.relative_to(repo_root).as_posix()
        if relative.startswith(EXCLUDED_PREFIX):
            continue
        source = path.read_text(encoding="utf-8", errors="replace")
        filtered = strip_comments_and_literals(source)
        lines = [filtered.count("\n", 0, match.start()) + 1 for match in TOKEN.finditer(filtered)]
        if lines:
            calls[relative] = lines
    return calls


def load_manifest(path: Path) -> dict[str, Any]:
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read manifest {path}: {error}") from error
    if not isinstance(manifest, dict):
        raise ValueError("manifest root must be a JSON object")
    return manifest


def validate_manifest(manifest: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if manifest.get("schema_version") != 1:
        errors.append("schema_version must be 1")

    entries = manifest.get("calls")
    if not isinstance(entries, list):
        return errors + ["calls must be a JSON array"]

    seen_ids: set[str] = set()
    occurrences: dict[str, list[int]] = defaultdict(list)
    for position, entry in enumerate(entries, start=1):
        label = f"calls[{position - 1}]"
        if not isinstance(entry, dict):
            errors.append(f"{label} must be an object")
            continue
        missing = REQUIRED_ENTRY_FIELDS - entry.keys()
        if missing:
            errors.append(f"{label} missing fields: {', '.join(sorted(missing))}")
            continue
        entry_id = entry["id"]
        if not isinstance(entry_id, str) or not entry_id:
            errors.append(f"{label}.id must be a non-empty string")
        elif entry_id in seen_ids:
            errors.append(f"duplicate id: {entry_id}")
        else:
            seen_ids.add(entry_id)
        path = entry["path"]
        occurrence = entry["occurrence"]
        if not isinstance(path, str) or not path.startswith("src/") or path.startswith(EXCLUDED_PREFIX):
            errors.append(f"{label}.path is outside the production scan scope: {path!r}")
        if not isinstance(occurrence, int) or occurrence < 1:
            errors.append(f"{label}.occurrence must be a positive integer")
        elif isinstance(path, str):
            occurrences[path].append(occurrence)
        if not isinstance(entry["line"], int) or entry["line"] < 1:
            errors.append(f"{label}.line must be a positive informational line number")
        if entry["classification"] not in CLASSIFICATIONS:
            errors.append(f"{label}.classification is invalid: {entry['classification']!r}")
        for field in ("index_type", "partitioner", "coupling", "reason"):
            if not isinstance(entry[field], str) or not entry[field].strip():
                errors.append(f"{label}.{field} must be a non-empty string")

    for path, values in occurrences.items():
        expected = list(range(1, len(values) + 1))
        if sorted(values) != expected:
            errors.append(f"{path}: occurrences must be contiguous 1..{len(values)}, got {sorted(values)}")

    baseline = manifest.get("baseline")
    if not isinstance(baseline, dict):
        errors.append("baseline must be an object")
    elif baseline.get("active_calls") != len(entries):
        errors.append(
            f"baseline.active_calls={baseline.get('active_calls')!r} does not match {len(entries)} entries"
        )
    return errors


def compare_inventory(repo_root: Path, manifest: dict[str, Any]) -> list[str]:
    errors = validate_manifest(manifest)
    if errors:
        return errors

    actual = scan_source(repo_root)
    expected = Counter(entry["path"] for entry in manifest["calls"])
    actual_counts = {path: len(lines) for path, lines in actual.items()}
    all_paths = sorted(set(expected) | set(actual_counts))
    for path in all_paths:
        expected_count = expected.get(path, 0)
        actual_count = actual_counts.get(path, 0)
        if expected_count != actual_count:
            lines = actual.get(path, [])
            errors.append(
                f"{path}: expected {expected_count} active tbb::parallel_for call(s), "
                f"found {actual_count} at line(s) {lines}"
            )
    return errors


def run_self_test() -> list[str]:
    failures: list[str] = []
    with tempfile.TemporaryDirectory(prefix="pgo-tbb-inventory-") as directory:
        root = Path(directory)
        source = root / "src/core/example/example.cpp"
        backend = root / "src/core/parallelism/backend.cpp"
        source.parent.mkdir(parents=True)
        backend.parent.mkdir(parents=True)
        baseline_source = """// tbb::parallel_for(0, 1, ignored);
const char *token = "tbb::parallel_for";
void run() { tbb::parallel_for(0, 1, [](int) {}); }
"""
        source.write_text(baseline_source, encoding="utf-8")
        backend.write_text("void backend() { tbb::parallel_for(0, 1, [](int) {}); }\n", encoding="utf-8")
        manifest: dict[str, Any] = {
            "schema_version": 1,
            "baseline": {"active_calls": 1},
            "calls": [
                {
                    "id": "example#1",
                    "path": "src/core/example/example.cpp",
                    "occurrence": 1,
                    "line": 3,
                    "index_type": "int",
                    "partitioner": "default",
                    "coupling": "none",
                    "classification": "migrate-v1",
                    "reason": "fixture",
                }
            ],
        }

        failures.extend(f"baseline: {error}" for error in compare_inventory(root, manifest))
        source.write_text(
            baseline_source + "void added() { tbb::parallel_for(0, 1, [](int) {}); }\n",
            encoding="utf-8",
        )
        if not compare_inventory(root, manifest):
            failures.append("adding an active call did not fail the guard")
        source.write_text(baseline_source, encoding="utf-8")
        failures.extend(f"restored baseline: {error}" for error in compare_inventory(root, manifest))
        source.write_text("// no active calls\n", encoding="utf-8")
        if not compare_inventory(root, manifest):
            failures.append("removing an inventoried call did not fail the guard")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true", help="run hermetic scanner and guard tests")
    parser.add_argument("--repo-root", type=Path, help="repository root; defaults to the script's parent")
    parser.add_argument("--manifest", type=Path, help="inventory manifest path")
    args = parser.parse_args()

    if args.self_test:
        errors = run_self_test()
    else:
        repo_root = (args.repo_root or Path(__file__).resolve().parents[1]).resolve()
        manifest_path = args.manifest or (
            repo_root / "src/core/parallelism/PARALLEL_FOR_INVENTORY.json"
        )
        try:
            errors = compare_inventory(repo_root, load_manifest(manifest_path))
        except ValueError as error:
            errors = [str(error)]

    if errors:
        print("tbb::parallel_for inventory guard failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1
    print("tbb::parallel_for inventory guard passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
