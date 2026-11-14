"""Read console text while the calling thread continues its idle work."""
import queue
import threading
from typing import Callable


def read_with_idle(prompt: str, idle_tick: Callable[[], None], *, poll_interval_s: float = .05) -> str:
    result: queue.Queue[tuple[str | None, BaseException | None]] = queue.Queue(maxsize=1)

    def read() -> None:
        try:
            result.put((input(prompt), None))
        except BaseException as exc:
            result.put((None, exc))

    # Only terminal input runs here. Flight control stays on the calling thread.
    threading.Thread(target=read, name="planner-console-input", daemon=True).start()
    while True:
        idle_tick()
        try:
            text, error = result.get(timeout=poll_interval_s)
        except queue.Empty:
            continue
        if error is not None:
            raise error
        return text
