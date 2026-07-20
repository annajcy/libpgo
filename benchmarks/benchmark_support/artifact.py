"""Atomic JSON artifacts with a uniform checkpoint and failure state."""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
import tempfile
from collections.abc import Callable, Iterator, Mapping
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def _timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def _checked_git(root: Path, *arguments: str) -> str | None:
    try:
        completed = subprocess.run(
            ["git", "-C", str(root), *arguments],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except OSError:
        return None
    return completed.stdout.strip() if completed.returncode == 0 else None


def runner_manifest(script: Path) -> dict[str, Any]:
    """Return common script, command, Python, and git provenance."""
    resolved = script.resolve()
    repository_text = _checked_git(resolved.parent, "rev-parse", "--show-toplevel")
    repository = Path(repository_text) if repository_text else None
    git = None
    if repository is not None:
        git = {
            "root": str(repository),
            "commit": _checked_git(repository, "rev-parse", "HEAD"),
            "branch": _checked_git(repository, "branch", "--show-current"),
            "dirty": bool(_checked_git(repository, "status", "--short")),
        }
    return {
        "created_at": _timestamp(),
        "script": str(resolved),
        "script_sha256": hashlib.sha256(resolved.read_bytes()).hexdigest(),
        "command": [sys.executable, *sys.argv],
        "python": {
            "executable": sys.executable,
            "version": sys.version,
        },
        "git": git,
    }


def atomic_write_json(
    path: Path,
    payload: Mapping[str, Any],
    *,
    sort_keys: bool = False,
    allow_nan: bool = True,
) -> None:
    """Atomically replace a JSON artifact in its destination directory."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_name = stream.name
            json.dump(
                payload,
                stream,
                indent=2,
                sort_keys=sort_keys,
                allow_nan=allow_nan,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        if temporary_name is not None:
            Path(temporary_name).unlink(missing_ok=True)
        raise


class JsonArtifact:
    """Track one runner's mechanical progress without owning experiment semantics."""

    def __init__(
        self,
        path: Path,
        *,
        scheduled_units: int | None = None,
        sort_keys: bool = False,
        allow_nan: bool = True,
    ) -> None:
        self.path = path
        self.sort_keys = sort_keys
        self.allow_nan = allow_nan
        self.state: dict[str, Any] = {
            "schema_version": 1,
            "state": "running",
            "complete": False,
            "created_at": _timestamp(),
            "updated_at": None,
            "scheduled_units": scheduled_units,
            "completed_units": 0,
            "active_unit": None,
            "failure": None,
        }

    def set_active(self, label: str | None) -> None:
        self.state["active_unit"] = label

    def document(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        document = dict(payload)
        document["artifact_state"] = dict(self.state)
        return document

    def checkpoint(
        self,
        payload: Mapping[str, Any],
        *,
        completed_units: int | None = None,
    ) -> None:
        if completed_units is not None:
            self.state["completed_units"] = completed_units
        self.state["updated_at"] = _timestamp()
        atomic_write_json(
            self.path,
            self.document(payload),
            sort_keys=self.sort_keys,
            allow_nan=self.allow_nan,
        )

    def complete(self, payload: Mapping[str, Any]) -> None:
        scheduled_units = self.state["scheduled_units"]
        self.state.update(
            {
                "state": "complete",
                "complete": True,
                "completed_units": (
                    scheduled_units
                    if scheduled_units is not None
                    else self.state["completed_units"]
                ),
                "active_unit": None,
                "failure": None,
            }
        )
        self.checkpoint(payload)

    def fail(self, payload: Mapping[str, Any], error: BaseException) -> None:
        self.state.update(
            {
                "state": "failed",
                "complete": False,
                "failure": {
                    "type": type(error).__name__,
                    "message": str(error),
                    "timestamp": _timestamp(),
                },
            }
        )
        self.checkpoint(payload)

    @contextmanager
    def capture_failures(
        self, payload: Callable[[], Mapping[str, Any]]
    ) -> Iterator["JsonArtifact"]:
        try:
            yield self
        except Exception as error:
            self.fail(payload(), error)
            raise
