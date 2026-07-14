#!/usr/bin/env python3
"""Create, materialize, and verify an exact tested-source identity.

The equality digest describes the byte-level working-tree inputs layered over a
separately recorded Git HEAD.  Porcelain and staging categories are evidence,
but deliberately do not participate in equality: a materialized checkout need
not reproduce whether a local change was staged.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import subprocess
import sys
import tempfile
from typing import Any, Iterable, Mapping, Sequence
import zipfile


FORMAT_VERSION = 1
FIXED_ZIP_TIME = (1980, 1, 1, 0, 0, 0)
EXCLUDED_UNTRACKED_COMPONENTS = frozenset(
    {
        ".cache",
        ".mypy_cache",
        ".pytest_cache",
        "__pycache__",
        "benchmark-results",
        "build",
        "cache",
        "node_modules",
        "results",
    }
)


class ManifestError(RuntimeError):
    """Raised when source state cannot be represented or safely applied."""


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _b64(data: bytes) -> str:
    return base64.b64encode(data).decode("ascii")


def _path_bytes(path: str) -> bytes:
    return os.fsencode(path)


def _path_text(path: bytes) -> str:
    return os.fsdecode(path)


def _run(
    command: Sequence[str],
    *,
    cwd: Path,
    input_bytes: bytes | None = None,
    check: bool = True,
) -> subprocess.CompletedProcess[bytes]:
    try:
        return subprocess.run(
            command,
            cwd=cwd,
            input=input_bytes,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=check,
        )
    except subprocess.CalledProcessError as exc:
        detail = exc.stderr.decode("utf-8", "replace").strip()
        raise ManifestError(f"command failed ({' '.join(command)}): {detail}") from exc


def _git(repo: Path, *arguments: str, input_bytes: bytes | None = None) -> bytes:
    return _run(["git", *arguments], cwd=repo, input_bytes=input_bytes).stdout


def _repo_root(path: Path) -> Path:
    root = path.resolve()
    if not root.is_dir():
        raise ManifestError(f"repository root is not a directory: {root}")
    actual = Path(_git(root, "rev-parse", "--show-toplevel").rstrip(b"\n").decode())
    if actual.resolve() != root:
        raise ManifestError(f"--repo-root must name the repository root: {root}")
    return root


def _parse_tree(raw: bytes) -> dict[bytes, tuple[str, str]]:
    result: dict[bytes, tuple[str, str]] = {}
    for record in raw.split(b"\0"):
        if not record:
            continue
        metadata, path = record.split(b"\t", 1)
        mode, _object_type, object_id = metadata.decode("ascii").split(" ")
        result[path] = (mode, object_id)
    return result


def _parse_index(raw: bytes) -> dict[bytes, tuple[str, str]]:
    result: dict[bytes, tuple[str, str]] = {}
    for record in raw.split(b"\0"):
        if not record:
            continue
        metadata, path = record.split(b"\t", 1)
        mode, object_id, stage = metadata.decode("ascii").split(" ")
        if stage != "0":
            raise ManifestError(f"unmerged index entry is not reproducible: {path!r}")
        result[path] = (mode, object_id)
    return result


def _split_nul_paths(raw: bytes) -> set[bytes]:
    return {record for record in raw.split(b"\0") if record}


def _is_excluded_untracked(path: bytes) -> bool:
    return any(
        part in EXCLUDED_UNTRACKED_COMPONENTS
        for part in PurePosixPath(_path_text(path)).parts
    )


def _validate_relative_path(path: str, *, label: str = "manifest path") -> bytes:
    raw = _path_bytes(path)
    if not raw or b"\0" in raw:
        raise ManifestError(f"{label} is empty or contains NUL")
    if raw.startswith(b"/"):
        raise ManifestError(f"{label} is absolute: {path!r}")
    parts = raw.split(b"/")
    if any(part in (b"", b".", b"..") for part in parts):
        raise ManifestError(f"{label} contains an unsafe component: {path!r}")
    return raw


def _validate_no_git_metadata_path(
    path: str, *, label: str = "manifest path"
) -> bytes:
    """Reject Git administrative aliases without consulting host filesystem semantics."""
    raw = _validate_relative_path(path, label=label)
    if any(component.lower() == b".git" for component in raw.split(b"/")):
        raise ManifestError(
            f"{label} contains a forbidden Git metadata component: {path!r}"
        )
    return raw


def _validate_explicit_inputs(declarations: Iterable[bytes]) -> None:
    for declaration in declarations:
        _validate_no_git_metadata_path(
            _path_text(declaration), label="explicit tested input"
        )


def _safe_source_leaf(root: Path, path: bytes) -> Path:
    """Return a lexical in-repository leaf without following parent symlinks."""
    _validate_no_git_metadata_path(_path_text(path), label="explicit tested input")
    current = root
    parts = path.split(b"/")
    for part in parts[:-1]:
        current = current / _path_text(part)
        if not os.path.lexists(current):
            raise ManifestError(
                f"explicit tested input parent does not exist: {_path_text(path)!r}"
            )
        mode = current.lstat().st_mode
        if stat.S_ISLNK(mode) or not stat.S_ISDIR(mode):
            raise ManifestError(
                f"explicit tested input parent is not a real directory: {current}"
            )
    return current / _path_text(parts[-1])


def _expand_explicit_inputs(repo: Path, declarations: set[bytes]) -> set[bytes]:
    expanded: set[bytes] = set()

    def visit(relative: bytes) -> None:
        absolute = _safe_source_leaf(repo, relative)
        if not os.path.lexists(absolute):
            raise ManifestError(
                f"explicit tested input does not exist: {_path_text(relative)!r}"
            )
        mode = absolute.lstat().st_mode
        if stat.S_ISLNK(mode) or stat.S_ISREG(mode):
            expanded.add(relative)
            return
        if not stat.S_ISDIR(mode):
            raise ManifestError(
                f"explicit tested input has unsupported type: {_path_text(relative)!r}"
            )
        children = sorted(os.scandir(absolute), key=lambda item: os.fsencode(item.name))
        if not children:
            raise ManifestError(
                f"empty explicit tested-input directory is not representable: {_path_text(relative)!r}"
            )
        for child in children:
            if child.name == ".git":
                raise ManifestError(
                    f"explicit tested input cannot include Git metadata: {_path_text(relative)!r}"
                )
            visit(relative + b"/" + os.fsencode(child.name))

    for declaration in sorted(declarations):
        visit(declaration)
    return expanded


def _relative_sidecar(base: Path, sidecar: Path) -> str:
    relative = os.path.relpath(sidecar, base.parent)
    _validate_relative_path(relative, label="sidecar path")
    return relative


def _resolve_sidecar(base: Path, relative: str) -> Path:
    _validate_relative_path(relative, label="sidecar path")
    candidate = base.parent.joinpath(relative)
    resolved_parent = candidate.parent.resolve()
    if os.path.commonpath((str(base.parent.resolve()), str(resolved_parent))) != str(
        base.parent.resolve()
    ):
        raise ManifestError(f"sidecar escapes its artifact directory: {relative!r}")
    return candidate


def _read_link_bytes(path: Path) -> bytes:
    return os.fsencode(os.readlink(path))


def _regular_mode(path: Path) -> str:
    return "100755" if path.stat(follow_symlinks=False).st_mode & stat.S_IXUSR else "100644"


def _gitlink_state(repo: Path, path: bytes) -> tuple[str | None, bytes]:
    nested = repo.joinpath(_path_text(path))
    if not os.path.lexists(nested):
        return None, b""
    nested_mode = nested.lstat().st_mode
    if not stat.S_ISDIR(nested_mode) or stat.S_ISLNK(nested_mode):
        return None, b""
    if not os.path.lexists(nested / ".git"):
        return None, b""
    top_level = _run(
        ["git", "rev-parse", "--show-toplevel"], cwd=nested, check=False
    )
    if top_level.returncode != 0:
        return None, b""
    reported_root = Path(top_level.stdout.rstrip(b"\n").decode()).resolve()
    if reported_root != nested.resolve():
        return None, b""
    head = _run(["git", "rev-parse", "HEAD"], cwd=nested, check=False)
    if head.returncode != 0:
        return None, b""
    porcelain = _git(
        nested,
        "status",
        "--porcelain=v2",
        "-z",
        "--untracked-files=all",
        "--ignore-submodules=none",
    )
    return head.stdout.strip().decode("ascii"), porcelain


def _partition_participating_paths(
    declarations: set[bytes], gitlink: bytes
) -> tuple[bool, set[bytes]]:
    prefix = gitlink + b"/"
    child = {path[len(prefix) :] for path in declarations if path.startswith(prefix)}
    return gitlink in declarations or bool(child), child


def _canonical_equality_entries(entries: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    canonical: list[dict[str, Any]] = []
    for entry in entries:
        item: dict[str, Any] = {
            "path_b64": _b64(_path_bytes(entry["path"])),
            "kind": entry["kind"],
        }
        kind = entry["kind"]
        if kind in ("regular", "symlink"):
            item.update(mode=entry["mode"], sha256=entry["sha256"])
        elif kind == "deleted":
            pass
        elif kind == "gitlink":
            item.update(mode="160000", index_commit=entry["index_commit"])
            if entry["participating"]:
                item["nested_tested_content_digest"] = entry[
                    "nested_tested_content_digest"
                ]
        else:
            raise ManifestError(f"unsupported entry kind: {kind!r}")
        canonical.append(item)
    canonical.sort(key=lambda item: base64.b64decode(item["path_b64"]))
    return canonical


def _tested_content_digest(entries: Sequence[Mapping[str, Any]]) -> str:
    encoded = json.dumps(
        _canonical_equality_entries(entries),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("ascii")
    return _sha256(encoded)


def _write_deterministic_archive(path: Path, objects: Mapping[str, bytes]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = json.dumps(
        {"format": "pgo-tested-source-objects", "version": FORMAT_VERSION},
        sort_keys=True,
        separators=(",", ":"),
    ).encode("ascii") + b"\n"
    temporary = path.with_name(path.name + f".tmp-{os.getpid()}")
    try:
        with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_STORED) as archive:
            for name, payload in [
                ("archive-format.json", descriptor),
                *((f"objects/{object_id}", objects[object_id]) for object_id in sorted(objects)),
            ]:
                info = zipfile.ZipInfo(name, FIXED_ZIP_TIME)
                info.compress_type = zipfile.ZIP_STORED
                info.create_system = 3
                info.external_attr = (stat.S_IFREG | 0o644) << 16
                archive.writestr(info, payload)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(
        value, sort_keys=True, indent=2, ensure_ascii=True
    ).encode("utf-8") + b"\n"
    temporary = path.with_name(path.name + f".tmp-{os.getpid()}")
    try:
        with temporary.open("wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _create_bundle(repo: Path, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f".tmp-{os.getpid()}")
    if temporary.exists():
        temporary.unlink()
    try:
        _git(repo, "bundle", "create", str(temporary), "HEAD")
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _collect_state(
    repo: Path,
    *,
    participating_gitlinks: set[bytes],
    explicit_includes: set[bytes],
    manifest_path: Path | None = None,
    archive_path: Path | None = None,
    create_sidecars: bool = False,
) -> tuple[dict[str, Any], dict[str, bytes]]:
    # Keep this check before every Git command and source-tree lookup. Besides the
    # public create entry point, _collect_state is also used directly by verification.
    _validate_explicit_inputs(explicit_includes)
    head = _git(repo, "rev-parse", "HEAD").strip().decode("ascii")
    porcelain = _git(
        repo,
        "status",
        "--porcelain=v2",
        "-z",
        "--untracked-files=all",
        "--ignore-submodules=none",
    )
    head_entries = _parse_tree(_git(repo, "ls-tree", "-rz", "--full-tree", "HEAD"))
    index_entries = _parse_index(_git(repo, "ls-files", "-s", "-z"))
    gitlink_paths = {
        path for path, (mode, _object_id) in index_entries.items() if mode == "160000"
    }
    for declaration in explicit_includes:
        containing_gitlink = next(
            (
                gitlink
                for gitlink in gitlink_paths
                if declaration == gitlink or declaration.startswith(gitlink + b"/")
            ),
            None,
        )
        if containing_gitlink is not None:
            raise ManifestError(
                "explicit tested input is inside a gitlink; declare the gitlink as "
                f"participating instead: {_path_text(declaration)!r}"
            )
    explicitly_included_paths = _expand_explicit_inputs(repo, explicit_includes)
    untracked_all = _split_nul_paths(
        _git(repo, "ls-files", "-z", "--others", "--exclude-standard")
    )
    excluded_untracked = sorted(
        (
            path
            for path in untracked_all
            if _is_excluded_untracked(path) and path not in explicitly_included_paths
        )
    )
    untracked = untracked_all.difference(excluded_untracked) | explicitly_included_paths
    candidates = set(head_entries) | set(index_entries) | untracked
    objects: dict[str, bytes] = {}
    entries: list[dict[str, Any]] = []
    seen_gitlinks: set[bytes] = set()

    for path in sorted(candidates):
        path_text = _path_text(path)
        _validate_relative_path(path_text)
        absolute = repo.joinpath(path_text)
        index = index_entries.get(path)
        prior = index or head_entries.get(path)
        if index and index[0] == "160000":
            seen_gitlinks.add(path)
            participating, child_declarations = _partition_participating_paths(
                participating_gitlinks, path
            )
            checked_out, nested_porcelain = _gitlink_state(repo, path)
            entry: dict[str, Any] = {
                "path": path_text,
                "kind": "gitlink",
                "mode": "160000",
                "index_commit": index[1],
                "checked_out_commit": checked_out,
                "dirty": bool(nested_porcelain),
                "dirty_status_b64": _b64(nested_porcelain),
                "participating": participating,
            }
            if participating:
                if checked_out is None:
                    raise ManifestError(
                        f"participating gitlink is not checked out: {path_text!r}"
                    )
                nested_repo = absolute
                if create_sidecars:
                    assert manifest_path is not None and archive_path is not None
                    token = _sha256(path)
                    nested_manifest = Path(str(manifest_path) + ".gitlinks") / f"{token}.json"
                    nested_archive = Path(str(archive_path) + ".gitlinks") / f"{token}.zip"
                    nested_bundle = Path(str(archive_path) + ".gitlinks") / f"{token}.bundle"
                    nested_value = create_manifest(
                        nested_repo,
                        nested_manifest,
                        nested_archive,
                        participating_gitlinks=child_declarations,
                        explicit_includes=set(),
                    )
                    _create_bundle(nested_repo, nested_bundle)
                    entry.update(
                        nested_manifest=_relative_sidecar(manifest_path, nested_manifest),
                        nested_manifest_sha256=_file_sha256(nested_manifest),
                        nested_archive=_relative_sidecar(archive_path, nested_archive),
                        nested_archive_sha256=_file_sha256(nested_archive),
                        nested_bundle=_relative_sidecar(archive_path, nested_bundle),
                        nested_bundle_sha256=_file_sha256(nested_bundle),
                        nested_head=nested_value["head"],
                        nested_tested_content_digest=nested_value[
                            "tested_content_digest"
                        ],
                    )
                else:
                    nested_value, _ = _collect_state(
                        nested_repo,
                        participating_gitlinks=child_declarations,
                        explicit_includes=set(),
                        create_sidecars=False,
                    )
                    entry.update(
                        nested_head=nested_value["head"],
                        nested_tested_content_digest=nested_value[
                            "tested_content_digest"
                        ],
                    )
            entries.append(entry)
            continue

        exists = os.path.lexists(absolute)
        if not exists:
            if prior is not None:
                entries.append(
                    {
                        "path": path_text,
                        "kind": "deleted",
                        "prior_mode": prior[0],
                    }
                )
            continue

        untracked_entry = path not in index_entries
        if absolute.is_symlink():
            payload = _read_link_bytes(absolute)
            object_id = _sha256(payload)
            objects[object_id] = payload
            entries.append(
                {
                    "path": path_text,
                    "kind": "symlink",
                    "mode": "120000",
                    "sha256": object_id,
                    "untracked": untracked_entry,
                    "explicitly_included": path in explicitly_included_paths,
                }
            )
        elif absolute.is_file():
            payload = absolute.read_bytes()
            object_id = _sha256(payload)
            objects[object_id] = payload
            entries.append(
                {
                    "path": path_text,
                    "kind": "regular",
                    "mode": _regular_mode(absolute),
                    "sha256": object_id,
                    "untracked": untracked_entry,
                    "explicitly_included": path in explicitly_included_paths,
                }
            )
        else:
            raise ManifestError(f"unsupported tested input type: {path_text!r}")

    undeclared = participating_gitlinks.difference(seen_gitlinks)
    undeclared = {
        path
        for path in undeclared
        if not any(path.startswith(gitlink + b"/") for gitlink in seen_gitlinks)
    }
    if undeclared:
        labels = ", ".join(repr(_path_text(path)) for path in sorted(undeclared))
        raise ManifestError(f"participating gitlink declarations not found: {labels}")

    entries.sort(key=lambda entry: _path_bytes(entry["path"]))
    value: dict[str, Any] = {
        "format": "pgo-tested-source-manifest",
        "version": FORMAT_VERSION,
        "head": head,
        "local_porcelain_b64": _b64(porcelain),
        "local_porcelain_sha256": _sha256(porcelain),
        "exclusion_policy": {
            "vcs_ignored_untracked": True,
            "untracked_path_components": sorted(EXCLUDED_UNTRACKED_COMPONENTS),
        },
        "explicit_includes": [_path_text(path) for path in sorted(explicit_includes)],
        "excluded_untracked_paths": [_path_text(path) for path in excluded_untracked],
        "entries": entries,
    }
    value["tested_content_digest"] = _tested_content_digest(entries)
    return value, objects


def create_manifest(
    repo_root: Path,
    manifest_path: Path,
    archive_path: Path,
    *,
    participating_gitlinks: set[bytes],
    explicit_includes: set[bytes],
) -> dict[str, Any]:
    # Explicit paths are untrusted CLI/API input. Reject Git metadata lexically
    # before resolving/scanning the repository or creating archive sidecars.
    _validate_explicit_inputs(explicit_includes)
    repo = _repo_root(repo_root)
    manifest = manifest_path.resolve()
    archive = archive_path.resolve()
    value, objects = _collect_state(
        repo,
        participating_gitlinks=participating_gitlinks,
        explicit_includes=explicit_includes,
        manifest_path=manifest,
        archive_path=archive,
        create_sidecars=True,
    )
    _write_deterministic_archive(archive, objects)
    value["archive_sha256"] = _file_sha256(archive)
    _write_json(manifest, value)
    return value


def _load_manifest(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ManifestError(f"cannot read manifest {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ManifestError("manifest root must be an object")
    if value.get("format") != "pgo-tested-source-manifest" or value.get("version") != FORMAT_VERSION:
        raise ManifestError("unsupported manifest format or version")
    head = value.get("head")
    if not isinstance(head, str) or len(head) not in (40, 64) or any(
        char not in "0123456789abcdef" for char in head
    ):
        raise ManifestError("manifest HEAD is not a full hexadecimal object id")
    try:
        porcelain = base64.b64decode(value.get("local_porcelain_b64", ""), validate=True)
    except (TypeError, ValueError) as exc:
        raise ManifestError("manifest local porcelain is not valid base64") from exc
    if value.get("local_porcelain_sha256") != _sha256(porcelain):
        raise ManifestError("manifest local porcelain digest does not match its bytes")
    explicit_values = value.get("explicit_includes")
    if not isinstance(explicit_values, list) or not all(
        isinstance(path, str) for path in explicit_values
    ):
        raise ManifestError("manifest explicit_includes must be an array of paths")
    explicit_paths = [
        _validate_no_git_metadata_path(path, label="explicit tested input")
        for path in explicit_values
    ]
    if explicit_paths != sorted(set(explicit_paths)):
        raise ManifestError("manifest explicit_includes must be unique and bytewise sorted")
    entries = value.get("entries")
    if not isinstance(entries, list):
        raise ManifestError("manifest entries must be an array")
    prior_path: bytes | None = None
    for entry in entries:
        if not isinstance(entry, dict) or not isinstance(entry.get("path"), str):
            raise ManifestError("each manifest entry needs a string path")
        # A manifest may be crafted independently of create(). No materialized
        # payload is ever allowed to address repository administrative data.
        path_bytes = _validate_no_git_metadata_path(entry["path"])
        if prior_path is not None and path_bytes <= prior_path:
            raise ManifestError("manifest paths must be unique and bytewise sorted")
        prior_path = path_bytes
        kind = entry.get("kind")
        if kind in ("regular", "symlink"):
            expected_mode = "120000" if kind == "symlink" else None
            if expected_mode and entry.get("mode") != expected_mode:
                raise ManifestError(f"invalid symlink mode for {entry['path']!r}")
            if kind == "regular" and entry.get("mode") not in ("100644", "100755"):
                raise ManifestError(f"invalid regular mode for {entry['path']!r}")
            if not isinstance(entry.get("untracked"), bool):
                raise ManifestError(f"missing untracked marker for {entry['path']!r}")
            if not isinstance(entry.get("explicitly_included"), bool):
                raise ManifestError(
                    f"missing explicit-include marker for {entry['path']!r}"
                )
            expected_explicit = any(
                path_bytes == declaration
                or path_bytes.startswith(declaration + b"/")
                for declaration in explicit_paths
            )
            if entry["explicitly_included"] != expected_explicit:
                raise ManifestError(
                    f"explicit-include marker mismatch for {entry['path']!r}"
                )
            _validate_hash(entry.get("sha256"), f"object for {entry['path']!r}")
        elif kind == "deleted":
            if not isinstance(entry.get("prior_mode"), str):
                raise ManifestError(f"deleted entry lacks prior mode: {entry['path']!r}")
        elif kind == "gitlink":
            if entry.get("mode") != "160000":
                raise ManifestError(f"invalid gitlink mode for {entry['path']!r}")
            _validate_hash(entry.get("index_commit"), f"gitlink {entry['path']!r}", git=True)
            if not isinstance(entry.get("dirty"), bool) or not isinstance(
                entry.get("dirty_status_b64"), str
            ):
                raise ManifestError(f"incomplete gitlink status: {entry['path']!r}")
            try:
                base64.b64decode(entry["dirty_status_b64"], validate=True)
            except ValueError as exc:
                raise ManifestError(f"invalid gitlink status encoding: {entry['path']!r}") from exc
            if not isinstance(entry.get("participating"), bool):
                raise ManifestError(f"gitlink lacks participation marker: {entry['path']!r}")
            checked_out = entry.get("checked_out_commit")
            if checked_out is not None:
                _validate_hash(
                    checked_out, f"checked-out gitlink {entry['path']!r}", git=True
                )
            if entry["participating"]:
                for key in ("nested_manifest", "nested_archive", "nested_bundle"):
                    if not isinstance(entry.get(key), str):
                        raise ManifestError(f"participating gitlink lacks {key}: {entry['path']!r}")
                    _validate_relative_path(entry[key], label=key)
                for key in (
                    "nested_manifest_sha256",
                    "nested_archive_sha256",
                    "nested_bundle_sha256",
                    "nested_tested_content_digest",
                ):
                    _validate_hash(entry.get(key), f"{key} for {entry['path']!r}")
                _validate_hash(
                    entry.get("nested_head"),
                    f"nested_head for {entry['path']!r}",
                    git=True,
                )
        else:
            raise ManifestError(f"unknown entry kind for {entry['path']!r}: {kind!r}")
    expected_digest = _tested_content_digest(entries)
    if value.get("tested_content_digest") != expected_digest:
        raise ManifestError("manifest tested-content digest does not match its entries")
    _validate_hash(value.get("archive_sha256"), "archive")
    return value


def _validate_hash(value: Any, label: str, *, git: bool = False) -> None:
    lengths = (40, 64) if git else (64,)
    if not isinstance(value, str) or len(value) not in lengths or any(
        char not in "0123456789abcdef" for char in value
    ):
        raise ManifestError(f"invalid {label} hash")


def _read_archive(path: Path, manifest: Mapping[str, Any]) -> dict[str, bytes]:
    if not path.is_file():
        raise ManifestError(f"archive does not exist: {path}")
    if _file_sha256(path) != manifest["archive_sha256"]:
        raise ManifestError("archive digest does not match the manifest")
    expected = {
        entry["sha256"]
        for entry in manifest["entries"]
        if entry["kind"] in ("regular", "symlink")
    }
    objects: dict[str, bytes] = {}
    try:
        with zipfile.ZipFile(path, "r") as archive:
            names = archive.namelist()
            if len(names) != len(set(names)):
                raise ManifestError("archive contains duplicate members")
            expected_names = {"archive-format.json"} | {
                f"objects/{object_id}" for object_id in expected
            }
            if set(names) != expected_names:
                raise ManifestError("archive object set does not match the manifest")
            descriptor = json.loads(archive.read("archive-format.json"))
            if descriptor != {
                "format": "pgo-tested-source-objects",
                "version": FORMAT_VERSION,
            }:
                raise ManifestError("unsupported object archive format")
            for object_id in expected:
                payload = archive.read(f"objects/{object_id}")
                if _sha256(payload) != object_id:
                    raise ManifestError(f"archive object is corrupt: {object_id}")
                objects[object_id] = payload
    except (OSError, zipfile.BadZipFile, json.JSONDecodeError, KeyError) as exc:
        raise ManifestError(f"cannot read object archive: {exc}") from exc
    return objects


def _safe_destination(root: Path, relative: str, *, create_parents: bool) -> Path:
    # Defense in depth if a caller ever bypasses _load_manifest().
    raw = _validate_no_git_metadata_path(relative)
    parts = [_path_text(part) for part in raw.split(b"/")]
    current = root
    for part in parts[:-1]:
        current = current / part
        if os.path.lexists(current):
            mode = current.lstat().st_mode
            if stat.S_ISLNK(mode) or not stat.S_ISDIR(mode):
                raise ManifestError(f"destination parent is not a real directory: {current}")
        elif create_parents:
            current.mkdir()
        else:
            break
    return root.joinpath(*parts)


def _remove_path(path: Path) -> None:
    if not os.path.lexists(path):
        return
    if path.is_symlink() or not path.is_dir():
        path.unlink()
    else:
        shutil.rmtree(path)


def _atomic_regular(path: Path, payload: bytes, mode: str) -> None:
    _remove_path(path)
    descriptor, temporary_name = tempfile.mkstemp(prefix=".pgo-source-", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.chmod(0o755 if mode == "100755" else 0o644)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _atomic_symlink(path: Path, target: bytes) -> None:
    _remove_path(path)
    temporary = path.parent / f".pgo-source-link-{os.getpid()}-{path.name}"
    if os.path.lexists(temporary):
        _remove_path(temporary)
    try:
        os.symlink(os.fsdecode(target), temporary)
        os.replace(temporary, path)
    finally:
        if os.path.lexists(temporary):
            _remove_path(temporary)


def _update_gitlink_index(repo: Path, entry: Mapping[str, Any]) -> None:
    record = (
        b"160000 "
        + entry["index_commit"].encode("ascii")
        + b"\t"
        + _path_bytes(entry["path"])
        + b"\0"
    )
    _git(repo, "update-index", "-z", "--index-info", input_bytes=record)


def _preflight_participating_gitlink(repo: Path, entry: Mapping[str, Any]) -> Path:
    destination = _safe_destination(repo, entry["path"], create_parents=False)
    components = _validate_relative_path(entry["path"]).split(b"/")
    descriptor = os.open(repo, os.O_RDONLY | os.O_DIRECTORY)
    try:
        for component in components:
            try:
                child_mode = os.stat(
                    component, dir_fd=descriptor, follow_symlinks=False
                ).st_mode
            except FileNotFoundError:
                break
            if stat.S_ISLNK(child_mode) or not stat.S_ISDIR(child_mode):
                raise ManifestError(
                    "participating gitlink path contains a symlink or non-directory "
                    f"component: {destination}"
                )
            child_descriptor = os.open(
                component,
                os.O_RDONLY | os.O_DIRECTORY | getattr(os, "O_NOFOLLOW", 0),
                dir_fd=descriptor,
            )
            os.close(descriptor)
            descriptor = child_descriptor
    finally:
        os.close(descriptor)
    if os.path.lexists(destination):
        mode = destination.lstat().st_mode
        if stat.S_ISLNK(mode):
            raise ManifestError(
                f"participating gitlink destination must not be a symlink: {destination}"
            )
        if not stat.S_ISDIR(mode):
            raise ManifestError(
                f"participating gitlink destination is not a real directory: {destination}"
            )
        if os.path.lexists(destination / ".git"):
            probe = _run(
                ["git", "rev-parse", "--show-toplevel"], cwd=destination, check=False
            )
            is_nested_repo = (
                probe.returncode == 0
                and Path(probe.stdout.rstrip(b"\n").decode()).resolve()
                == destination.resolve()
            )
            if not is_nested_repo:
                raise ManifestError(
                    f"participating gitlink repository is not rooted at its lexical destination: {destination}"
                )
        elif any(destination.iterdir()):
            raise ManifestError(
                f"participating gitlink destination is non-empty and not a repository: {destination}"
            )
    return destination


def _materialize_participating_gitlink(
    repo: Path,
    entry: Mapping[str, Any],
    manifest_path: Path,
    archive_path: Path,
    destination: Path,
) -> None:
    nested_manifest = _resolve_sidecar(manifest_path, entry["nested_manifest"])
    nested_archive = _resolve_sidecar(archive_path, entry["nested_archive"])
    nested_bundle = _resolve_sidecar(archive_path, entry["nested_bundle"])
    for path, key in (
        (nested_manifest, "nested_manifest_sha256"),
        (nested_archive, "nested_archive_sha256"),
        (nested_bundle, "nested_bundle_sha256"),
    ):
        if not path.is_file() or _file_sha256(path) != entry[key]:
            raise ManifestError(f"participating gitlink sidecar mismatch: {path}")
    if os.path.lexists(destination) and not os.path.lexists(destination / ".git"):
        destination.rmdir()
    if not os.path.lexists(destination):
        destination.parent.mkdir(parents=True, exist_ok=True)
        _run(["git", "clone", "--no-checkout", str(nested_bundle), str(destination)], cwd=repo)
    _git(destination, "checkout", "--detach", entry["nested_head"])
    materialize_manifest(destination, nested_manifest, nested_archive)


def _participating_declarations(manifest: Mapping[str, Any]) -> set[bytes]:
    return {
        _path_bytes(entry["path"])
        for entry in manifest["entries"]
        if entry["kind"] == "gitlink" and entry["participating"]
    }


def _explicit_include_declarations(manifest: Mapping[str, Any]) -> set[bytes]:
    return {_path_bytes(path) for path in manifest["explicit_includes"]}


def _identity_record(state: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "head": state["head"],
        "tested_content_digest": state["tested_content_digest"],
        "porcelain_b64": state["local_porcelain_b64"],
        "porcelain_sha256": state["local_porcelain_sha256"],
    }


def materialize_manifest(repo_root: Path, manifest_path: Path, archive_path: Path) -> dict[str, Any]:
    repo = _repo_root(repo_root)
    manifest_file = manifest_path.resolve()
    archive_file = archive_path.resolve()
    manifest = _load_manifest(manifest_file)
    current_head = _git(repo, "rev-parse", "HEAD").strip().decode("ascii")
    if current_head != manifest["head"]:
        raise ManifestError(
            f"base HEAD mismatch: expected {manifest['head']}, observed {current_head}"
        )
    objects = _read_archive(archive_file, manifest)
    participating_destinations = {
        entry["path"]: _preflight_participating_gitlink(repo, entry)
        for entry in manifest["entries"]
        if entry["kind"] == "gitlink" and entry["participating"]
    }

    for entry in sorted(
        (item for item in manifest["entries"] if item["kind"] == "deleted"),
        key=lambda item: (-len(_path_bytes(item["path"]).split(b"/")), _path_bytes(item["path"])),
    ):
        destination = _safe_destination(repo, entry["path"], create_parents=False)
        _remove_path(destination)

    for entry in manifest["entries"]:
        kind = entry["kind"]
        if kind == "deleted":
            continue
        if kind == "gitlink":
            _update_gitlink_index(repo, entry)
            if entry["participating"]:
                _materialize_participating_gitlink(
                    repo,
                    entry,
                    manifest_file,
                    archive_file,
                    participating_destinations[entry["path"]],
                )
            continue
        destination = _safe_destination(repo, entry["path"], create_parents=True)
        payload = objects[entry["sha256"]]
        if kind == "regular":
            _atomic_regular(destination, payload, entry["mode"])
        elif kind == "symlink":
            _atomic_symlink(destination, payload)

    observed, _ = _collect_state(
        repo,
        participating_gitlinks=_participating_declarations(manifest),
        explicit_includes=_explicit_include_declarations(manifest),
        create_sidecars=False,
    )
    if observed["head"] != manifest["head"]:
        raise ManifestError("materialization unexpectedly changed HEAD")
    if observed["tested_content_digest"] != manifest["tested_content_digest"]:
        raise ManifestError(
            "materialized tested-content digest mismatch: "
            f"expected {manifest['tested_content_digest']}, "
            f"observed {observed['tested_content_digest']}"
        )
    return _identity_record(observed)


def verify_manifest(repo_root: Path, manifest_path: Path) -> dict[str, Any]:
    repo = _repo_root(repo_root)
    manifest = _load_manifest(manifest_path.resolve())
    observed, _ = _collect_state(
        repo,
        participating_gitlinks=_participating_declarations(manifest),
        explicit_includes=_explicit_include_declarations(manifest),
        create_sidecars=False,
    )
    errors: list[str] = []
    if observed["head"] != manifest["head"]:
        errors.append(f"HEAD expected {manifest['head']}, observed {observed['head']}")
    if observed["tested_content_digest"] != manifest["tested_content_digest"]:
        errors.append(
            "tested content expected "
            f"{manifest['tested_content_digest']}, observed {observed['tested_content_digest']}"
        )
    if errors:
        raise ManifestError("source verification failed: " + "; ".join(errors))
    return _identity_record(observed)


def _configure_repo(repo: Path) -> None:
    _git(repo, "config", "user.name", "PGO Manifest Self Test")
    _git(repo, "config", "user.email", "manifest@example.invalid")


def _init_repo(path: Path) -> None:
    path.mkdir(parents=True)
    _run(["git", "init", "-q"], cwd=path)
    _configure_repo(path)


def _commit_all(repo: Path, message: str) -> None:
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", message)


def _make_submodule(parent: Path, source: Path, name: str) -> None:
    _run(
        [
            "git",
            "-c",
            "protocol.file.allow=always",
            "submodule",
            "add",
            "-q",
            str(source),
            name,
        ],
        cwd=parent,
    )


def _assert(condition: bool, message: str) -> None:
    if not condition:
        raise ManifestError(f"self-test failed: {message}")


def self_test() -> None:
    with tempfile.TemporaryDirectory(prefix="pgo-source-manifest-") as temporary:
        root = Path(temporary)
        submodule_sources: list[Path] = []
        for name in ("clean-source", "dirty-source", "participating-source"):
            source = root / name
            _init_repo(source)
            (source / "nested.txt").write_text(f"{name}\n", encoding="utf-8")
            _commit_all(source, "base")
            submodule_sources.append(source)

        source = root / "source"
        _init_repo(source)
        (source / ".gitignore").write_text("*.ignored\n", encoding="utf-8")
        (source / "modified.txt").write_text("before\n", encoding="utf-8")
        (source / "deleted.txt").write_text("delete me\n", encoding="utf-8")
        (source / "executable.sh").write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
        (source / "link").symlink_to("modified.txt")
        _make_submodule(source, submodule_sources[0], "clean-link")
        _make_submodule(source, submodule_sources[1], "dirty-link")
        _make_submodule(source, submodule_sources[2], "participating-link")
        _commit_all(source, "base source")
        base_head = _git(source, "rev-parse", "HEAD").strip().decode("ascii")

        (source / "modified.txt").write_text("after\n", encoding="utf-8")
        (source / "deleted.txt").unlink()
        (source / "executable.sh").chmod(0o755)
        (source / "link").unlink()
        (source / "link").symlink_to("executable.sh")
        (source / "added-and-staged.txt").write_text("staged addition\n", encoding="utf-8")
        _git(source, "add", "added-and-staged.txt")
        (source / "space name.txt").write_text("space\n", encoding="utf-8")
        (source / "line\nbreak.txt").write_text("newline\n", encoding="utf-8")
        (source / "dirty-link" / "nested.txt").write_text("dirty and excluded\n", encoding="utf-8")
        (source / "participating-link" / "nested.txt").write_text(
            "participating change\n", encoding="utf-8"
        )
        (source / "participating-link" / "nested extra.txt").write_text(
            "nested untracked\n", encoding="utf-8"
        )
        for directory in ("build", "cache", "results", "node_modules"):
            target = source / directory
            target.mkdir()
            (target / "excluded.txt").write_text("excluded\n", encoding="utf-8")
        (source / "also.ignored").write_text("ignored\n", encoding="utf-8")
        (source / "link.ignored").symlink_to("also.ignored")
        (source / "build" / "included.bin").write_bytes(b"generated tested input\0")

        explicit_includes = {
            b"also.ignored",
            b"build/included.bin",
            b"link.ignored",
        }

        artifacts = root / "artifacts"
        manifest = artifacts / "identity.json"
        archive = artifacts / "objects.zip"
        manifest_two = artifacts / "identity-two.json"
        archive_two = artifacts / "objects-two.zip"
        parsed_cli = parse_arguments(
            [
                "create",
                "--repo-root",
                str(source),
                "--manifest",
                str(manifest),
                "--archive",
                str(archive),
                "--include-tested-input",
                "also.ignored",
                "--include-tested-input",
                "build/included.bin",
            ]
        )
        _assert(
            parsed_cli.include_tested_input
            == ["also.ignored", "build/included.bin"],
            "repeatable explicit-include CLI did not preserve declarations",
        )
        created = create_manifest(
            source,
            manifest,
            archive,
            participating_gitlinks={b"participating-link"},
            explicit_includes=explicit_includes,
        )
        created_two = create_manifest(
            source,
            manifest_two,
            archive_two,
            participating_gitlinks={b"participating-link"},
            explicit_includes=explicit_includes,
        )
        _assert(archive.read_bytes() == archive_two.read_bytes(), "archive is not deterministic")
        _assert(
            created["tested_content_digest"] == created_two["tested_content_digest"],
            "repeated creation changed the tested-content digest",
        )
        entries = {entry["path"]: entry for entry in created["entries"]}
        entries_two = {entry["path"]: entry for entry in created_two["entries"]}
        first_nested_archive = _resolve_sidecar(
            archive, entries["participating-link"]["nested_archive"]
        )
        second_nested_archive = _resolve_sidecar(
            archive_two, entries_two["participating-link"]["nested_archive"]
        )
        _assert(
            first_nested_archive.read_bytes() == second_nested_archive.read_bytes(),
            "recursive gitlink archive is not deterministic",
        )
        _assert(entries["deleted.txt"]["kind"] == "deleted", "tracked deletion missing")
        _assert(entries["executable.sh"]["mode"] == "100755", "mode change missing")
        _assert(entries["link"]["kind"] == "symlink", "symlink missing")
        _assert(entries["space name.txt"]["untracked"], "space path was not untracked")
        _assert(entries["line\nbreak.txt"]["untracked"], "newline path was not preserved")
        _assert(not entries["clean-link"]["dirty"], "clean gitlink marked dirty")
        _assert(entries["dirty-link"]["dirty"], "dirty gitlink status missing")
        _assert(not entries["dirty-link"]["participating"], "dirty gitlink became participating")
        _assert(
            entries["participating-link"]["participating"],
            "participating gitlink marker missing",
        )
        for directory in ("build", "cache", "results", "node_modules"):
            _assert(
                f"{directory}/excluded.txt" not in entries,
                f"excluded directory entered manifest: {directory}",
            )
        _assert(
            entries["also.ignored"]["explicitly_included"]
            and entries["build/included.bin"]["explicitly_included"]
            and entries["link.ignored"]["explicitly_included"],
            "explicit ignored/generated tested inputs were not included",
        )
        _assert(
            entries["build/included.bin"]["sha256"]
            == _sha256(b"generated tested input\0"),
            "explicit generated input hash differs",
        )
        _assert(
            entries["link.ignored"]["kind"] == "symlink"
            and entries["link.ignored"]["sha256"] == _sha256(b"also.ignored"),
            "explicit ignored symlink type/target hash differs",
        )
        verify_manifest(source, manifest)

        materialized = root / "materialized"
        _run(["git", "clone", "-q", "--no-recurse-submodules", str(source), str(materialized)], cwd=root)
        _git(materialized, "checkout", "-q", "--detach", base_head)
        record = materialize_manifest(materialized, manifest, archive)
        _assert(
            record["tested_content_digest"] == created["tested_content_digest"],
            "materialized digest differs",
        )
        verify_manifest(materialized, manifest)
        materialized_state, _ = _collect_state(
            materialized,
            participating_gitlinks={b"participating-link"},
            explicit_includes=explicit_includes,
            create_sidecars=False,
        )
        materialized_entries = {
            entry["path"]: entry for entry in materialized_state["entries"]
        }
        for gitlink in ("clean-link", "dirty-link"):
            _assert(
                materialized_entries[gitlink]["checked_out_commit"] is None
                and not materialized_entries[gitlink]["dirty"]
                and materialized_entries[gitlink]["dirty_status_b64"] == "",
                f"uninitialized gitlink inherited superproject metadata: {gitlink}",
            )
        _assert((materialized / "modified.txt").read_bytes() == b"after\n", "modified file differs")
        _assert(not (materialized / "deleted.txt").exists(), "deletion was not applied")
        _assert(
            (materialized / "executable.sh").stat().st_mode & stat.S_IXUSR,
            "executable mode was not applied",
        )
        _assert(os.readlink(materialized / "link") == "executable.sh", "symlink target differs")
        _assert((materialized / "line\nbreak.txt").is_file(), "newline path was not materialized")
        _assert(
            (materialized / "also.ignored").read_bytes() == b"ignored\n"
            and (materialized / "build" / "included.bin").read_bytes()
            == b"generated tested input\0"
            and os.readlink(materialized / "link.ignored") == "also.ignored",
            "explicit ignored/generated input was not materialized",
        )
        _assert(
            (materialized / "participating-link" / "nested extra.txt").is_file(),
            "participating nested input was not materialized",
        )
        _assert(
            not (materialized / "dirty-link" / "nested.txt").exists()
            and not (materialized / "dirty-link" / ".git").exists(),
            "non-participating dirty gitlink contents were copied",
        )

        external = root / "external-attack-target"
        _init_repo(external)
        (external / "nested.txt").write_text("external base\n", encoding="utf-8")
        _commit_all(external, "external base")
        external_file_before = (external / "nested.txt").read_bytes()
        external_head_before = _git(external, "rev-parse", "HEAD")
        external_index_before = _git(external, "ls-files", "-s", "-z")
        external_status_before = _git(
            external, "status", "--porcelain=v2", "-z", "--untracked-files=all"
        )
        attacked = root / "attacked-materialization"
        _run(
            [
                "git",
                "clone",
                "-q",
                "--no-recurse-submodules",
                str(source),
                str(attacked),
            ],
            cwd=root,
        )
        _git(attacked, "checkout", "-q", "--detach", base_head)
        attacked_leaf = attacked / "participating-link"
        _remove_path(attacked_leaf)
        attacked_leaf.symlink_to(external, target_is_directory=True)
        try:
            materialize_manifest(attacked, manifest, archive)
        except ManifestError:
            pass
        else:
            raise ManifestError(
                "self-test failed: participating gitlink leaf symlink was accepted"
            )
        _assert(
            (external / "nested.txt").read_bytes() == external_file_before,
            "participating gitlink attack changed an external file",
        )
        _assert(
            _git(external, "rev-parse", "HEAD") == external_head_before,
            "participating gitlink attack changed external HEAD",
        )
        _assert(
            _git(external, "ls-files", "-s", "-z") == external_index_before,
            "participating gitlink attack changed external index",
        )
        _assert(
            _git(
                external,
                "status",
                "--porcelain=v2",
                "-z",
                "--untracked-files=all",
            )
            == external_status_before,
            "participating gitlink attack changed external status",
        )
        for unsafe_include in ("../outside.txt", "/absolute.txt"):
            try:
                _explicit_declarations([unsafe_include])
            except ManifestError:
                pass
            else:
                raise ManifestError(
                    f"self-test failed: unsafe explicit include was accepted: {unsafe_include!r}"
                )

        # Explicit tested inputs must never expose Git administrative data. Cover
        # the repository root, a direct child, a nested real directory, and a
        # nested symlink spelling; rejection must happen before any output exists.
        nested_metadata = source / "metadata-directory" / ".git"
        nested_metadata.mkdir(parents=True)
        (nested_metadata / "config").write_bytes(b"nested sentinel\n")
        symlink_metadata_parent = source / "metadata-symlink"
        symlink_metadata_parent.mkdir()
        (symlink_metadata_parent / ".git").symlink_to(
            source / ".git", target_is_directory=True
        )
        mixed_metadata = source / "metadata-mixed-directory" / ".GiT"
        mixed_metadata.mkdir(parents=True)
        (mixed_metadata / "config").write_bytes(b"mixed nested sentinel\n")
        mixed_symlink_metadata_parent = source / "metadata-mixed-symlink"
        mixed_symlink_metadata_parent.mkdir()
        (mixed_symlink_metadata_parent / ".gIt").symlink_to(
            source / ".git", target_is_directory=True
        )
        source_git_config_before = (source / ".git" / "config").read_bytes()
        source_head_before = _git(source, "rev-parse", "HEAD")
        source_index_before = _git(source, "ls-files", "-s", "-z")
        source_status_before = _git(
            source,
            "status",
            "--porcelain=v2",
            "-z",
            "--untracked-files=all",
            "--ignore-submodules=none",
        )
        archive_before_git_metadata_tests = archive.read_bytes()
        forbidden_git_inputs = (
            ".git",
            ".git/config",
            ".GIT/config",
            "metadata-directory/.git/config",
            "metadata-symlink/.git/config",
            "metadata-mixed-directory/.GiT/config",
            "metadata-mixed-symlink/.gIt/config",
        )
        for index, forbidden_input in enumerate(forbidden_git_inputs):
            try:
                _explicit_declarations([forbidden_input])
            except ManifestError:
                pass
            else:
                raise ManifestError(
                    "self-test failed: CLI accepted Git metadata explicit input: "
                    f"{forbidden_input!r}"
                )
            forbidden_manifest = artifacts / f"forbidden-git-{index}.json"
            forbidden_archive = artifacts / f"forbidden-git-{index}.zip"
            forbidden_outputs = (
                forbidden_manifest,
                forbidden_archive,
                Path(str(forbidden_manifest) + ".gitlinks"),
                Path(str(forbidden_archive) + ".gitlinks"),
            )
            try:
                create_manifest(
                    source,
                    forbidden_manifest,
                    forbidden_archive,
                    participating_gitlinks={b"participating-link"},
                    explicit_includes={_path_bytes(forbidden_input)},
                )
            except ManifestError:
                pass
            else:
                raise ManifestError(
                    "self-test failed: API accepted Git metadata explicit input: "
                    f"{forbidden_input!r}"
                )
            _assert(
                not any(os.path.lexists(path) for path in forbidden_outputs),
                f"forbidden Git input created an output artifact: {forbidden_input!r}",
            )
            cli_manifest = artifacts / f"forbidden-git-cli-{index}.json"
            cli_archive = artifacts / f"forbidden-git-cli-{index}.zip"
            cli_outputs = (
                cli_manifest,
                cli_archive,
                Path(str(cli_manifest) + ".gitlinks"),
                Path(str(cli_archive) + ".gitlinks"),
            )
            cli_result = _run(
                [
                    sys.executable,
                    str(Path(__file__).resolve()),
                    "create",
                    "--repo-root",
                    str(source),
                    "--manifest",
                    str(cli_manifest),
                    "--archive",
                    str(cli_archive),
                    "--participating-gitlink",
                    "participating-link",
                    "--include-tested-input",
                    forbidden_input,
                ],
                cwd=root,
                check=False,
            )
            _assert(
                cli_result.returncode != 0,
                f"real CLI accepted Git metadata explicit input: {forbidden_input!r}",
            )
            _assert(
                not any(os.path.lexists(path) for path in cli_outputs),
                f"forbidden CLI Git input created an output artifact: {forbidden_input!r}",
            )
        _assert(
            (source / ".git" / "config").read_bytes() == source_git_config_before
            and _git(source, "rev-parse", "HEAD") == source_head_before
            and _git(source, "ls-files", "-s", "-z") == source_index_before
            and _git(
                source,
                "status",
                "--porcelain=v2",
                "-z",
                "--untracked-files=all",
                "--ignore-submodules=none",
            )
            == source_status_before,
            "forbidden Git explicit inputs changed source repository state",
        )
        _assert(
            archive.read_bytes() == archive_before_git_metadata_tests,
            "forbidden Git explicit inputs changed an existing payload archive",
        )

        # Loading a hand-crafted manifest must apply the same rule even if the
        # malicious entry is not declared as an explicit include.
        destination_git_config_before = (
            materialized / ".git" / "config"
        ).read_bytes()
        destination_head_before = _git(materialized, "rev-parse", "HEAD")
        destination_index_before = _git(materialized, "ls-files", "-s", "-z")
        destination_status_before = _git(
            materialized,
            "status",
            "--porcelain=v2",
            "-z",
            "--untracked-files=all",
            "--ignore-submodules=none",
        )
        crafted_git_paths = (
            ".git/config",
            ".GIT/config",
            "payload/.GiT/config",
        )
        for index, crafted_git_path in enumerate(crafted_git_paths):
            malicious_git_manifest = json.loads(manifest.read_text(encoding="utf-8"))
            malicious_git_manifest["entries"].append(
                {
                    "path": crafted_git_path,
                    "kind": "deleted",
                    "prior_mode": "100644",
                }
            )
            malicious_git_manifest["entries"].sort(
                key=lambda entry: _path_bytes(entry["path"])
            )
            # Keep the fixture digest-valid so rejection is specifically due to
            # the Git-metadata component, not a later integrity mismatch.
            malicious_git_manifest["tested_content_digest"] = _tested_content_digest(
                malicious_git_manifest["entries"]
            )
            malicious_git_manifest_path = (
                artifacts / f"malicious-git-metadata-{index}.json"
            )
            _write_json(malicious_git_manifest_path, malicious_git_manifest)
            try:
                materialize_manifest(
                    materialized, malicious_git_manifest_path, archive
                )
            except ManifestError:
                pass
            else:
                raise ManifestError(
                    "self-test failed: crafted manifest addressed Git metadata: "
                    f"{crafted_git_path!r}"
                )
        _assert(
            (materialized / ".git" / "config").read_bytes()
            == destination_git_config_before
            and _git(materialized, "rev-parse", "HEAD") == destination_head_before
            and _git(materialized, "ls-files", "-s", "-z")
            == destination_index_before
            and _git(
                materialized,
                "status",
                "--porcelain=v2",
                "-z",
                "--untracked-files=all",
                "--ignore-submodules=none",
            )
            == destination_status_before,
            "crafted Git metadata manifest changed destination repository state",
        )
        _assert(
            archive.read_bytes() == archive_before_git_metadata_tests,
            "crafted Git metadata manifest changed the payload archive",
        )

        include_escape = materialized / "include-escape"
        include_escape.symlink_to(external, target_is_directory=True)
        try:
            create_manifest(
                materialized,
                artifacts / "unsafe-include.json",
                artifacts / "unsafe-include.zip",
                participating_gitlinks={b"participating-link"},
                explicit_includes={b"include-escape/nested.txt"},
            )
        except ManifestError:
            pass
        else:
            raise ManifestError(
                "self-test failed: explicit include followed an external parent symlink"
            )
        _assert(
            (external / "nested.txt").read_bytes() == external_file_before
            and _git(external, "rev-parse", "HEAD") == external_head_before
            and _git(external, "ls-files", "-s", "-z") == external_index_before,
            "unsafe explicit include changed the external repository",
        )

        outside = root / "outside.txt"
        outside.write_text("safe\n", encoding="utf-8")
        for index, unsafe_path in enumerate(("../outside.txt", "/absolute.txt")):
            malicious = json.loads(manifest.read_text(encoding="utf-8"))
            malicious["entries"].append(
                {"path": unsafe_path, "kind": "deleted", "prior_mode": "100644"}
            )
            malicious_path = artifacts / f"malicious-{index}.json"
            _write_json(malicious_path, malicious)
            try:
                materialize_manifest(materialized, malicious_path, archive)
            except ManifestError:
                pass
            else:
                raise ManifestError(
                    f"self-test failed: unsafe deletion was accepted: {unsafe_path!r}"
                )
        _assert(outside.read_text(encoding="utf-8") == "safe\n", "traversal deletion escaped root")


def _declarations(values: Iterable[str]) -> set[bytes]:
    result: set[bytes] = set()
    for value in values:
        result.add(_validate_relative_path(value, label="participating gitlink"))
    return result


def _explicit_declarations(values: Iterable[str]) -> set[bytes]:
    result: set[bytes] = set()
    for value in values:
        result.add(
            _validate_no_git_metadata_path(value, label="explicit tested input")
        )
    return result


def _print_record(value: Mapping[str, Any]) -> None:
    print(json.dumps(value, sort_keys=True, ensure_ascii=True))


def parse_arguments(arguments: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true", help="run the hermetic self-test")
    subparsers = parser.add_subparsers(dest="command")
    create_parser = subparsers.add_parser("create", help="create a manifest and object archive")
    create_parser.add_argument("--repo-root", type=Path, required=True)
    create_parser.add_argument("--manifest", type=Path, required=True)
    create_parser.add_argument("--archive", type=Path, required=True)
    create_parser.add_argument(
        "--participating-gitlink",
        action="append",
        default=[],
        metavar="PATH",
        help="recursively include this gitlink (repeatable)",
    )
    create_parser.add_argument(
        "--include-tested-input",
        action="append",
        default=[],
        metavar="PATH",
        help="include an ignored/generated regular file, symlink, or directory (repeatable)",
    )
    materialize_parser = subparsers.add_parser(
        "materialize", help="apply a manifest/archive to a base-HEAD checkout"
    )
    materialize_parser.add_argument("--repo-root", type=Path, required=True)
    materialize_parser.add_argument("--manifest", type=Path, required=True)
    materialize_parser.add_argument("--archive", type=Path, required=True)
    verify_parser = subparsers.add_parser("verify", help="verify current typed source state")
    verify_parser.add_argument("--repo-root", type=Path, required=True)
    verify_parser.add_argument("--manifest", type=Path, required=True)
    parsed = parser.parse_args(arguments)
    if parsed.self_test and parsed.command is not None:
        parser.error("--self-test cannot be combined with a command")
    if not parsed.self_test and parsed.command is None:
        parser.error("choose create, materialize, verify, or --self-test")
    return parsed


def main(arguments: Sequence[str] | None = None) -> int:
    options = parse_arguments(sys.argv[1:] if arguments is None else arguments)
    try:
        if options.self_test:
            self_test()
            print("create_tested_source_manifest self-test passed")
        elif options.command == "create":
            value = create_manifest(
                options.repo_root,
                options.manifest,
                options.archive,
                participating_gitlinks=_declarations(options.participating_gitlink),
                explicit_includes=_explicit_declarations(options.include_tested_input),
            )
            _print_record(
                {
                    "head": value["head"],
                    "tested_content_digest": value["tested_content_digest"],
                    "archive_sha256": value["archive_sha256"],
                    "local_porcelain_sha256": value["local_porcelain_sha256"],
                }
            )
        elif options.command == "materialize":
            _print_record(
                materialize_manifest(options.repo_root, options.manifest, options.archive)
            )
        elif options.command == "verify":
            _print_record(verify_manifest(options.repo_root, options.manifest))
    except ManifestError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
