"""Helpers for reading the decoupled function-first config.

New code should read model endpoints from ``FUNCTIONS.<NAME>``.  A small
fallback to the old flat ``AGENT`` section is kept here so legacy configs do
not crash while the runtime moves to the function-first layout.
"""

from __future__ import annotations

from typing import Any, Mapping


def function_section(config: Mapping[str, Any], name: str) -> dict:
    return dict((config.get("FUNCTIONS", {}) or {}).get(name, {}) or {})


def agent_section(config: Mapping[str, Any]) -> dict:
    return dict(config.get("AGENT", {}) or {})


def first_value(*values: Any, default: Any = None) -> Any:
    for value in values:
        if value is None:
            continue
        if isinstance(value, str) and not value.strip():
            continue
        return value
    return default


def as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)
