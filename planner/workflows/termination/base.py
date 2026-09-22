from typing import Protocol, TypeVar

from planner.domain.progress import ExitDecision

ContextT = TypeVar("ContextT", contravariant=True)


class ExitPolicy(Protocol[ContextT]):
    """Task-specific lifecycle and exit rules behind a common contract."""

    def begin(self) -> None: ...

    def evaluate(self, context: ContextT) -> ExitDecision: ...
