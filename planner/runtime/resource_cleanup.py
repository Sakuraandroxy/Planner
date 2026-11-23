"""Attempt every owned cleanup once; report aggregate failures on Python 3.10.

The thread helper is the one close path the runtime's background workers share. A
worker thread that is inside a model or transport call cannot be stopped from
outside, so ``stop_thread`` bounds the wait, names the straggler in the log, and
lets the caller keep its own closed flag set -- the thread stays a daemon and dies
with the process, but nothing pretends it exited.
"""
import logging
import threading
from collections.abc import Callable

from planner.errors import ResourceCleanupError

logger = logging.getLogger(__name__)

#: How long a background thread may take to leave its call before it is named.
CLOSE_TIMEOUT_S = 1.0


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


def stop_thread(
    thread: threading.Thread | None, name: str, timeout_s: float = CLOSE_TIMEOUT_S
) -> bool:
    """Join one background thread; return True when it is still running.

    ``False`` means the thread ended and whatever it held is released. ``True`` is
    the abandoned case: the caller has already refused new work, and the log line is
    the only signal that a thread outlived its owner.
    """
    if thread is None:
        return False
    thread.join(timeout=timeout_s)
    if not thread.is_alive():
        return False
    logger.warning(
        "%s did not stop within %gs; it keeps its call and its result is discarded",
        name,
        timeout_s,
    )
    return True
