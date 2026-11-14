from __future__ import annotations

from typing import Any, Callable


class ExtensionRegistry:
    """Explicit allow-list registry for future workflows and capabilities."""

    def __init__(self):
        self._factories: dict[str, Callable[..., Any]] = {}

    def register(self, name: str, factory: Callable[..., Any]) -> None:
        key = str(name).strip()
        if not key or key in self._factories:
            raise ValueError(f"invalid or duplicate extension name: {key!r}")
        self._factories[key] = factory

    def create(self, name: str, **kwargs: Any) -> Any:
        if name not in self._factories:
            raise KeyError(f"extension is not allow-listed: {name}")
        return self._factories[name](**kwargs)

