"""Runtime calibration of the streaming-view cycle.

``World.switch_streaming_view()`` walks the cameras flagged
``streaming-enabled`` in the robot config, but the server exposes neither the
current position nor a way to select a camera by id. The cycle order also
differs between builds, maps and even process restarts, so a hard-coded list
mixes up the four console views (front and chase of the rover and the drone).

This module measures the mapping at runtime instead:

1. fetch one frame from each capture-enabled streaming camera using the
   ``GetImages`` service,
2. walk one full ``switch_streaming_view`` cycle while taking an xpra
   screenshot at every position,
3. match every screenshot against the reference frames, and
4. identify the single camera that cannot be fetched (the rover chase camera
   has ``capture-enabled: false``) by elimination.

A normalized zero-mean grayscale signature at 160x90 is enough: the correct
camera scores above ~0.9 and every other camera below ~0.3. The world must be
advanced for the server to capture frames, so ``GetImages`` runs on a
service-only client while the caller keeps stepping the paused world through
its primary connection.
"""

from __future__ import annotations

import os
import subprocess
import tempfile
import threading
import time
from itertools import permutations
from typing import Callable, Mapping, Sequence

import cv2
import numpy as np

from projectairsim import ProjectAirSimClient
from projectairsim.utils import unpack_image

from ...domain.view import VIEW_MODES

SIGNATURE_SIZE = (160, 90)
# Matching thresholds measured against the live simulator: the right camera
# scores 0.92+, every other camera 0.28 or below.
MIN_MATCH_SCORE = 0.55
MIN_MATCH_MARGIN = 0.10
# The camera that cannot be fetched (rover chase) is identified by elimination.
# Its cycle position may still correlate with another camera's frame because
# both look down the same street (~0.75 in the live scene), so the leftover
# position is only accepted while it stays clearly below a true match.
MAX_UNMATCHED_SCORE = 0.90
# A frame needs a rendered tick after the view switch; the shadow client needs
# a moment before its screenshot reflects that tick. A cold scene can make the
# shadow lag by a whole switch interval, which used to return the previous
# camera's frame twice (two live positions scored identically). Wait for the
# screenshot to be stable across two captures *and* different from the
# previous position before matching it.
SETTLE_S = 0.9
SCREENSHOT_RETRY_S = 0.5
SCREENSHOT_RETRIES = 5
STABLE_SCORE = 0.9995
ADVANCE_S = 15.0 / 1000.0
FRAME_TIMEOUT_S = 8.0
SCREENSHOT_TIMEOUT_S = 20.0
# A single screenshot may anchor a cached cycle order only when it is a
# confident true match; the rover chase camera can leak ~0.75 against a front
# camera, so the normal calibration thresholds (0.55/0.10) are too loose here.
CACHE_ANCHOR_MIN_SCORE = 0.90
CACHE_ANCHOR_MIN_MARGIN = 0.15


def signature_bgr(bgr: np.ndarray) -> np.ndarray:
    """Return a zero-mean, unit-norm grayscale signature of a frame."""
    if bgr is None or not isinstance(bgr, np.ndarray) or bgr.size == 0:
        raise ValueError("cannot build a signature from an empty frame")
    if bgr.ndim == 2:
        gray = bgr
    else:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    small = cv2.resize(
        gray, SIGNATURE_SIZE, interpolation=cv2.INTER_AREA
    ).astype(np.float32)
    small -= float(small.mean())
    norm = float(np.linalg.norm(small))
    return small / norm if norm else small


def signature_score(left: np.ndarray, right: np.ndarray) -> float:
    """Normalized correlation of two signatures: 1 identical, ~0 unrelated."""
    if left.shape != right.shape:
        raise ValueError("signatures must have the same shape")
    return float(np.sum(left * right))


def xpra_screenshot(display: str) -> np.ndarray:
    """Capture the xpra shadow's window as a BGR frame."""
    handle = tempfile.NamedTemporaryFile(suffix=".png", delete=False)
    path = handle.name
    handle.close()
    try:
        subprocess.run(
            ["xpra", "screenshot", path, display],
            check=True,
            capture_output=True,
            timeout=SCREENSHOT_TIMEOUT_S,
        )
        bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass
    if bgr is None:
        raise RuntimeError(
            f"could not decode the xpra screenshot from display {display}"
        )
    return bgr


def service_frame_fetcher(
    *,
    address: str,
    port_topics: int,
    port_services: int,
    parent_topic: str,
    advance: Callable[[float], object],
    timeout_s: float = FRAME_TIMEOUT_S,
    advance_s: float = ADVANCE_S,
):
    """Build ``fetch_frame(robot, camera) -> ndarray | None``.

    The request blocks until the server captures a frame at or after the
    request time, so the paused world is advanced in this thread while a
    helper thread waits on a service-only client. The returned callable owns
    that client; call ``fetch_frame.close()`` when done.
    """
    state: dict[str, ProjectAirSimClient | None] = {"client": None}

    def _connect() -> ProjectAirSimClient:
        client = ProjectAirSimClient(address, port_topics, port_services)
        client.connect_services()
        client.socket_services.recv_timeout = int(max(1.0, timeout_s) * 1000)
        state["client"] = client
        return client

    def _drop(client: ProjectAirSimClient) -> None:
        if state["client"] is client:
            state["client"] = None
        try:
            client.disconnect()
        except Exception:
            pass

    def close() -> None:
        client, state["client"] = state["client"], None
        if client is not None:
            _drop(client)

    def fetch_frame(robot: str, camera: str) -> np.ndarray | None:
        client = state["client"] or _connect()
        result: dict[str, object] = {}

        def _request() -> None:
            try:
                result["data"] = client.request(
                    {
                        "method": (
                            f"{parent_topic}/robots/{robot}/sensors/"
                            f"{camera}/GetImages"
                        ),
                        "params": {"image_type_ids": [0]},
                        "version": 1.0,
                    }
                )
            except Exception as error:  # noqa: BLE001
                result["error"] = error

        worker = threading.Thread(
            target=_request, name=f"streaming-view-frame-{camera}", daemon=True
        )
        worker.start()
        deadline = time.monotonic() + timeout_s
        while worker.is_alive() and time.monotonic() < deadline:
            advance(advance_s)
        worker.join(timeout=timeout_s)
        if worker.is_alive() or "error" in result or "data" not in result:
            # A service timeout disconnects the client inside ``request``;
            # drop it so the next fetch opens a fresh link.
            _drop(client)
            return None
        data = result.get("data")
        if not isinstance(data, dict) or not data:
            _drop(client)
            return None
        message = data.get("0") or data.get(0)
        if not isinstance(message, dict):
            _drop(client)
            return None
        try:
            return unpack_image(message)
        except Exception:  # noqa: BLE001
            _drop(client)
            return None

    fetch_frame.close = close  # type: ignore[attr-defined]
    return fetch_frame


def _classify(
    shot_signature: np.ndarray,
    references: Mapping[str, np.ndarray],
) -> tuple[str | None, list[tuple[str, float]]]:
    scores = sorted(
        (
            (mode, signature_score(shot_signature, reference))
            for mode, reference in references.items()
        ),
        key=lambda item: item[1],
        reverse=True,
    )
    if not scores:
        return None, scores
    best_mode, best_score = scores[0]
    runner_up = scores[1][1] if len(scores) > 1 else None
    if best_score < MIN_MATCH_SCORE:
        return None, scores
    if runner_up is not None and best_score - runner_up < MIN_MATCH_MARGIN:
        return None, scores
    return best_mode, scores


def _solve_assignment(
    score_rows: Sequence[Mapping[str, float]],
    modes: Sequence[str],
    unfetchable: Sequence[str],
) -> tuple[tuple[str, ...] | None, str | None]:
    """Assign every cycle position to exactly one camera.

    Per-position classification is not enough: the non-capturing chase camera
    can correlate with a neighbouring front camera while the true front camera
    also appears elsewhere in the cycle, which used to abort calibration. The
    cycle is a permutation, so search every permutation and pick the one that
    maximises the sum of matched scores. The camera without a reference frame
    takes the leftover position by elimination.

    Two cameras can also look nearly identical at every position (the drone
    parked 1000 m above the map: its front and chase cameras see the same
    distant terrain). A position-level margin alone then rejects a mapping
    that the whole cycle identifies clearly: the runner-up camera has a
    stronger match at a different position, so that position must belong to
    it. The cycle-level margin at the end guards the remaining case where two
    permutations are genuinely equally good.
    """
    candidates: list[tuple[float, tuple[str, ...]]] = []
    for assignment in permutations(modes):
        total = 0.0
        consistent = True
        for position, mode in enumerate(assignment):
            if mode in unfetchable:
                continue
            score = score_rows[position].get(mode, 0.0)
            if score < MIN_MATCH_SCORE:
                consistent = False
                break
            total += score
        if consistent:
            candidates.append((total, assignment))
    if not candidates:
        return None, "no consistent position assignment"
    candidates.sort(key=lambda item: item[0], reverse=True)
    best = candidates[0][1]

    assigned_at = {mode: position for position, mode in enumerate(best)}
    for position, mode in enumerate(best):
        if mode in unfetchable:
            continue
        scores = score_rows[position]
        score = scores.get(mode, 0.0)
        competitor_name, competitor = max(
            (
                (name, value)
                for name, value in scores.items()
                if name != mode
            ),
            key=lambda item: item[1],
            default=(None, 0.0),
        )
        if score - competitor < MIN_MATCH_MARGIN:
            competitor_position = (
                assigned_at.get(competitor_name)
                if competitor_name is not None
                else None
            )
            confirmed_elsewhere = (
                competitor_position is not None
                and competitor_position != position
                and score_rows[competitor_position].get(
                    competitor_name, 0.0
                ) > competitor
            )
            if not confirmed_elsewhere:
                return None, (
                    f"ambiguous match at position {position}: "
                    f"{mode} vs another camera"
                )
        if score < max(
            (
                other_score
                for other_position, row in enumerate(score_rows)
                if other_position != position
                for name, other_score in row.items()
                if name == mode
            ),
            default=0.0,
        ):
            return None, f"camera {mode} matches a different position better"

    for position, mode in enumerate(best):
        if mode not in unfetchable:
            continue
        best_other = max(score_rows[position].values(), default=0.0)
        if best_other >= MAX_UNMATCHED_SCORE:
            return None, (
                f"position {position} assigned to {mode} but correlates "
                f"{best_other:.3f} with a capture-enabled camera"
            )

    # The winning mapping must beat every other permutation by a usable
    # margin; otherwise front and chase may be swapped while every position
    # looks plausible on its own.
    best_total = candidates[0][0]
    if len(candidates) > 1:
        runner_up_total = candidates[1][0]
        if best_total - runner_up_total < MIN_MATCH_MARGIN:
            return None, (
                "ambiguous cycle assignment: best total "
                f"{best_total:.3f} vs alternative {runner_up_total:.3f}"
            )
    return tuple(best), None


def _fetch_references(
    *,
    modes: Sequence[str],
    cameras: Mapping[str, tuple[str, str]],
    fetch_frame: Callable[[str, str], np.ndarray | None],
    uncapturable: Sequence[str],
    emit: Callable[[str, str], None],
) -> tuple[dict[str, np.ndarray], list[str]]:
    """Fetch one reference frame per capture-enabled streaming view.

    Views declared in ``uncapturable`` (``capture-enabled: false`` in the robot
    config) are never fetched: waiting for their frame only burns the full
    ``FRAME_TIMEOUT_S`` and the assignment solver locates them by elimination.
    """
    unavailable = [mode for mode in uncapturable if mode in cameras]
    references: dict[str, np.ndarray] = {}
    unfetchable = list(unavailable)
    for mode in modes:
        if mode in unavailable:
            continue
        robot, camera = cameras[mode]
        try:
            frame = fetch_frame(robot, camera)
        except Exception as error:  # noqa: BLE001
            emit(f"streaming-view reference {mode} failed: {error}", "WARNING")
            frame = None
        if frame is None:
            unfetchable.append(mode)
            continue
        try:
            references[mode] = signature_bgr(frame)
        except ValueError:
            unfetchable.append(mode)
    return references, unfetchable


def calibrate_streaming_views(
    *,
    cameras: Mapping[str, tuple[str, str]],
    screenshot: Callable[[], np.ndarray],
    fetch_frame: Callable[[str, str], np.ndarray | None],
    switch_view: Callable[[], bool],
    advance: Callable[[float], object],
    modes: Sequence[str] = VIEW_MODES,
    uncapturable: Sequence[str] = (),
    settle_s: float = SETTLE_S,
    advance_s: float = ADVANCE_S,
    sleep: Callable[[float], None] = time.sleep,
    log: Callable[[str, str], None] | None = None,
) -> tuple[str, ...] | None:
    """Measure the camera at every position of the streaming-view cycle.

    Returns a tuple where index ``i`` is the view mode at cycle position ``i``
    relative to the position the server was at on entry. The server is left at
    that same position. Returns ``None`` when the mapping cannot be identified;
    callers must then fall back to the built-in order.
    """
    emit = log or (lambda message, level="INFO": None)
    modes = tuple(modes)
    if not modes:
        raise ValueError("at least one streaming view mode is required")
    missing = [mode for mode in modes if mode not in cameras]
    if missing:
        emit(
            f"streaming-view calibration skipped: no camera for {missing}",
            "WARNING",
        )
        return None

    references, unfetchable = _fetch_references(
        modes=modes,
        cameras=cameras,
        fetch_frame=fetch_frame,
        uncapturable=uncapturable,
        emit=emit,
    )
    if len(unfetchable) > 1:
        emit(
            "streaming-view calibration is ambiguous: "
            f"could not fetch {unfetchable}",
            "WARNING",
        )
        return None
    if not references:
        emit("streaming-view calibration: no reference frames", "WARNING")
        return None

    steps = len(modes)
    performed = 0

    def _step() -> bool:
        nonlocal performed
        if not switch_view():
            return False
        performed += 1
        advance(advance_s)
        sleep(settle_s)
        return True

    observed: list[str | None] = []
    score_rows: list[dict[str, float]] = []
    failure: str | None = None

    def _settled_signature(previous: np.ndarray | None) -> np.ndarray:
        """Wait until the shadow shows a stable, changed frame.

        The xpra shadow can lag a cold scene by one switch interval, which
        returns the previous camera's frame for the new position. Two
        consecutive identical captures prove the shadow has caught up; the
        change against the previous position proves the switch was applied.
        """
        last = signature_bgr(screenshot())
        for _ in range(SCREENSHOT_RETRIES):
            sleep(SCREENSHOT_RETRY_S)
            current = signature_bgr(screenshot())
            stable = signature_score(last, current) >= STABLE_SCORE
            changed = (
                previous is None
                or signature_score(current, previous) < STABLE_SCORE
            )
            last = current
            if stable and changed:
                return current
        return last

    try:
        # Let the shadow window catch up with the position the server was
        # already showing when calibration began.
        sleep(settle_s)
        previous_signature: np.ndarray | None = None
        for position in range(steps):
            shot = _settled_signature(previous_signature)
            previous_signature = shot
            scores = {
                mode: signature_score(shot, reference)
                for mode, reference in references.items()
            }
            score_rows.append(scores)
            ranked = ", ".join(
                f"{name}={score:.3f}"
                for name, score in sorted(
                    scores.items(), key=lambda item: item[1], reverse=True
                )
            )
            best_mode = max(scores, key=scores.get) if scores else None
            observed.append(best_mode)
            emit(
                f"streaming-view position {position}: "
                f"{best_mode or 'unmatched'} ({ranked})"
            )
            if not _step():
                failure = (
                    "simulator refused to switch the streaming view during "
                    "calibration"
                )
                break
    except Exception as error:  # noqa: BLE001
        failure = f"{type(error).__name__}: {error}"
    finally:
        # Return to the position the cycle started at, even on failure.
        attempts = 0
        while performed % steps != 0 and attempts < steps:
            attempts += 1
            try:
                if not _step():
                    break
            except Exception:  # noqa: BLE001
                break

    if failure is not None:
        emit(f"streaming-view calibration failed: {failure}", "WARNING")
        return None
    if len(observed) != steps:
        emit("streaming-view calibration was incomplete", "WARNING")
        return None

    positions, reason = _solve_assignment(score_rows, modes, unfetchable)
    if positions is None:
        emit(
            "streaming-view calibration failed: "
            f"{reason or 'could not identify the cycle'}",
            "WARNING",
        )
        return None

    if sorted(positions) != sorted(modes):
        emit(
            "streaming-view calibration failed: cycle is not a permutation "
            "of the expected modes",
            "WARNING",
        )
        return None
    return tuple(positions)


def verify_streaming_views(
    *,
    cameras: Mapping[str, tuple[str, str]],
    screenshot: Callable[[], np.ndarray],
    fetch_frame: Callable[[str, str], np.ndarray | None],
    switch_view: Callable[[], bool],
    advance: Callable[[float], object],
    expected: Sequence[str],
    modes: Sequence[str] = VIEW_MODES,
    uncapturable: Sequence[str] = (),
    settle_s: float = SETTLE_S,
    advance_s: float = ADVANCE_S,
    sleep: Callable[[float], None] = time.sleep,
    log: Callable[[str, str], None] | None = None,
) -> str | None:
    """Identify the camera at the current cycle position, cheaply.

    Used to reuse a cached cycle order: one screenshot (plus at most one view
    switch when the position shows the uncapturable chase camera) anchors the
    cached tuple to the server's current position. Returns the identified mode
    or ``None`` when the screenshot is not a confident match, in which case the
    caller must run the full calibration.
    """
    emit = log or (lambda message, level="INFO": None)
    modes = tuple(modes)
    expected = tuple(expected)
    if not modes:
        raise ValueError("at least one streaming view mode is required")
    if sorted(expected) != sorted(modes):
        return None
    missing = [mode for mode in modes if mode not in cameras]
    if missing:
        return None
    references, unfetchable = _fetch_references(
        modes=modes,
        cameras=cameras,
        fetch_frame=fetch_frame,
        uncapturable=uncapturable,
        emit=emit,
    )
    # More than one unfetchable camera means the screenshot cannot anchor the
    # cycle: the elimination solver needs a single leftover position.
    if len(unfetchable) > 1 or not references:
        return None
    sleep(settle_s)
    for attempt in range(2):
        try:
            shot = signature_bgr(screenshot())
        except Exception as error:  # noqa: BLE001
            emit(f"streaming-view cache check failed: {error}", "WARNING")
            return None
        ranked = sorted(
            (
                (mode, signature_score(shot, reference))
                for mode, reference in references.items()
            ),
            key=lambda item: item[1],
            reverse=True,
        )
        best_mode, best_score = ranked[0]
        runner_up = ranked[1][1] if len(ranked) > 1 else 0.0
        if (
            best_score >= CACHE_ANCHOR_MIN_SCORE
            and best_score - runner_up >= CACHE_ANCHOR_MIN_MARGIN
        ):
            return best_mode
        if attempt > 0:
            break
        # The position probably shows the uncapturable chase camera; move one
        # step so the next screenshot is from a fetchable camera.
        try:
            if not switch_view():
                return None
        except Exception as error:  # noqa: BLE001
            emit(f"streaming-view cache check failed: {error}", "WARNING")
            return None
        advance(advance_s)
        sleep(settle_s)
    return None
