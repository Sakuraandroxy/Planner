"""Adapter-local messages prepared by an explicitly selected task prompt."""
from dataclasses import dataclass
from typing import Any, Protocol, TypeVar

RequestT = TypeVar("RequestT", contravariant=True)


@dataclass(frozen=True)
class CompletionMessages:
    system: str
    user: str
    images: tuple[tuple[Any, str], ...]


class CompletionPromptBuilder(Protocol[RequestT]):
    def prepare(self, request: RequestT) -> CompletionMessages: ...