"""Base interface for visual language model backends."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Iterable, Mapping


class VLMModel(ABC):
    @abstractmethod
    def chat(self, messages: Iterable[Mapping[str, Any]], **kwargs) -> str:
        ...
