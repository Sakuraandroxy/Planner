from typing import Protocol


class Capability(Protocol):
    @property
    def name(self) -> str: ...

