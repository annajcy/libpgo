"""Small argument-validation primitives shared by benchmark CLIs."""

from __future__ import annotations


def require_positive(value: int, option: str) -> None:
    if value <= 0:
        raise ValueError(f"{option} must be positive.")


def require_nonnegative(value: int, option: str) -> None:
    if value < 0:
        raise ValueError(f"{option} must be nonnegative.")
