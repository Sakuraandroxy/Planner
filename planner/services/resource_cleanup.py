"""Attempt every owned cleanup once; report aggregate failures on Python 3.10."""
from typing import Callable

from planner.errors import ResourceCleanupError


def close_resources(callbacks: tuple[Callable[[], None], ...]) -> None:
    failures: list[BaseException] = []
    for close in callbacks:
        try:
            close()
        except BaseException as exc:
            failures.append(exc)
    if failures:
        interrupt = next((exc for exc in failures if isinstance(exc, (KeyboardInterrupt, SystemExit))), None)
        if interrupt is not None:
            raise interrupt
        raise ResourceCleanupError(tuple(failures))
