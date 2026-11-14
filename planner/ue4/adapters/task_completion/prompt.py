"""Adapter-local messages prepared by an explicitly selected task prompt."""
from dataclasses import dataclass
from typing import Protocol, TypeVar
from PIL import Image

RequestT = TypeVar("RequestT", contravariant=True)


@dataclass(frozen=True)
class CompletionMessages:
    system: str
    user: str
    images: tuple[tuple[Image.Image, str], ...]


class CompletionPromptBuilder(Protocol[RequestT]):
    def prepare(self, request: RequestT) -> CompletionMessages: ...
