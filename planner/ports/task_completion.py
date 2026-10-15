"""Typed semantic completion dependency; each task owns its request contract."""
from typing import Protocol, TypeVar

RequestT = TypeVar("RequestT", contravariant=True)


class TaskCompletionChecker(Protocol[RequestT]):
    def check(self, request: RequestT, *, timeout_s: float) -> bool: ...
