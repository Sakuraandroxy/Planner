"""Runtime streaming-view calibration without a live simulator."""

import numpy as np
import pytest

from planner.adapters.projectairsim.streaming_views import (
    MAX_UNMATCHED_SCORE,
    MIN_MATCH_SCORE,
    _solve_assignment,
    calibrate_streaming_views,
    signature_bgr,
    signature_score,
    verify_streaming_views,
)
from planner.domain.view import VIEW_MODES

MODES = VIEW_MODES
CAMERAS = {
    "rover_front": ("Rover1", "FrontCamera"),
    "rover_chase": ("Rover1", "Chase"),
    "drone_front": ("Drone1", "FrontCamera"),
    "drone_chase": ("Drone1", "Chase"),
}


def frame(seed, size=(90, 160)):
    rng = np.random.default_rng(seed)
    gray = rng.integers(0, 256, size=size, dtype=np.uint8)
    return np.dstack([gray, gray, gray])


FRAMES = {mode: frame(index) for index, mode in enumerate(MODES)}


class FakeServer:
    """A streaming-view cycle that starts at an arbitrary position."""

    def __init__(self, order, *, anchor=1, switch_ok=True):
        self.order = tuple(order)
        self.position = anchor
        self.anchor = anchor
        self.switch_ok = switch_ok
        self.switches = 0
        self.frames = FRAMES
        self.screenshots = 0
        self.fail_on_screenshot = None
        self.refuse_after = None

    def current(self):
        return self.order[self.position]

    def screenshot(self):
        self.screenshots += 1
        if self.fail_on_screenshot == self.screenshots:
            raise RuntimeError("screenshot failed")
        return self.frames[self.current()]

    def switch(self):
        if not self.switch_ok:
            return False
        if self.refuse_after is not None and self.switches >= self.refuse_after:
            return False
        self.position = (self.position + 1) % len(self.order)
        self.switches += 1
        return True


class LaggingShadowServer(FakeServer):
    """The shadow returns the previous camera for one capture after a switch."""

    def __init__(self, order, *, anchor=1):
        super().__init__(order, anchor=anchor)
        self._lag = False

    def switch(self):
        ok = super().switch()
        if ok:
            self._lag = True
        return ok

    def screenshot(self):
        if self._lag:
            self._lag = False
            self.screenshots += 1
            previous = (self.position - 1) % len(self.order)
            return self.frames[self.order[previous]]
        return super().screenshot()


def fetch(robots=None):
    def _fetch(robot, camera):
        for mode, (expected_robot, expected_camera) in CAMERAS.items():
            if (robot, camera) == (expected_robot, expected_camera):
                if mode == "rover_chase":
                    return None
                return FRAMES[mode]
        return None

    return _fetch


def run_calibration(server, **overrides):
    kwargs = dict(
        cameras=CAMERAS,
        screenshot=server.screenshot,
        fetch_frame=fetch(),
        switch_view=server.switch,
        advance=lambda seconds: None,
        sleep=lambda seconds: None,
    )
    kwargs.update(overrides)
    return calibrate_streaming_views(**kwargs)


def test_calibration_skips_declared_uncapturable_views():
    order = ("rover_chase", "rover_front", "drone_front", "drone_chase")
    server = FakeServer(order, anchor=1)
    calls = []

    def tracked_fetch(robot, camera):
        calls.append((robot, camera))
        return fetch()(robot, camera)

    positions = run_calibration(
        server,
        fetch_frame=tracked_fetch,
        uncapturable=("rover_chase",),
    )

    expected = tuple(
        order[(1 + step) % len(order)] for step in range(len(order))
    )
    assert positions == expected
    # The rover chase camera never gets a GetImages call (no 8 s timeout).
    assert ("Rover1", "Chase") not in calls
    assert len(calls) == len(MODES) - 1


def test_verify_identifies_the_current_position_without_switching():
    order = ("rover_chase", "rover_front", "drone_front", "drone_chase")
    server = FakeServer(order, anchor=2)

    mode = verify_streaming_views(
        cameras=CAMERAS,
        screenshot=server.screenshot,
        fetch_frame=fetch(),
        switch_view=server.switch,
        advance=lambda seconds: None,
        expected=order,
        uncapturable=("rover_chase",),
        sleep=lambda seconds: None,
    )

    assert mode == "drone_front"
    assert server.switches == 0


def test_verify_steps_once_when_the_position_is_uncapturable():
    order = ("rover_chase", "rover_front", "drone_front", "drone_chase")
    server = FakeServer(order, anchor=0)
    calls = []

    def tracked_fetch(robot, camera):
        calls.append((robot, camera))
        return fetch()(robot, camera)

    mode = verify_streaming_views(
        cameras=CAMERAS,
        screenshot=server.screenshot,
        fetch_frame=tracked_fetch,
        switch_view=server.switch,
        advance=lambda seconds: None,
        expected=order,
        uncapturable=("rover_chase",),
        sleep=lambda seconds: None,
    )

    assert mode == "rover_front"
    assert server.switches == 1
    assert ("Rover1", "Chase") not in calls


def test_verify_rejects_a_stale_cache_with_a_different_mode_set():
    server = FakeServer(MODES, anchor=0)
    assert (
        verify_streaming_views(
            cameras=CAMERAS,
            screenshot=server.screenshot,
            fetch_frame=fetch(),
            switch_view=server.switch,
            advance=lambda seconds: None,
            expected=("rover_front", "drone_front", "drone_chase"),
            uncapturable=("rover_chase",),
            sleep=lambda seconds: None,
        )
        is None
    )
    assert server.screenshots == 0


def test_verify_returns_none_when_the_screenshot_is_unrecognizable():
    order = ("rover_chase", "rover_front", "drone_front", "drone_chase")
    server = FakeServer(order, anchor=1)
    server.frames = {mode: frame(900 + index) for index, mode in enumerate(MODES)}

    assert (
        verify_streaming_views(
            cameras=CAMERAS,
            screenshot=server.screenshot,
            fetch_frame=fetch(),
            switch_view=server.switch,
            advance=lambda seconds: None,
            expected=order,
            uncapturable=("rover_chase",),
            sleep=lambda seconds: None,
        )
        is None
    )
    assert server.screenshots == 2


def test_signature_handles_bgr_and_grayscale():
    bgr = FRAMES["rover_front"]
    gray = bgr[:, :, 0]
    assert np.allclose(signature_bgr(bgr), signature_bgr(gray))
    assert signature_score(signature_bgr(bgr), signature_bgr(bgr)) == pytest.approx(1.0)
    other = signature_score(signature_bgr(bgr), signature_bgr(FRAMES["drone_front"]))
    assert abs(other) < MIN_MATCH_SCORE


def test_signature_rejects_empty_frames():
    with pytest.raises(ValueError, match="empty frame"):
        signature_bgr(np.zeros((0, 0, 3), dtype=np.uint8))


def test_calibration_measures_the_live_cycle_and_restores_the_anchor():
    order = ("rover_chase", "rover_front", "drone_front", "drone_chase")
    server = FakeServer(order, anchor=1)

    positions = run_calibration(server)

    expected = tuple(order[(1 + step) % len(order)] for step in range(len(order)))
    assert positions == expected
    # One capture to find the frame plus one confirming it is stable.
    assert server.screenshots == 2 * len(order)
    assert server.switches == len(order)
    # Exactly one full cycle: the server is back where calibration found it.
    assert server.position == server.anchor


def test_calibration_waits_out_a_stale_shadow_frame():
    order = ("drone_front", "drone_chase", "rover_chase", "rover_front")
    server = LaggingShadowServer(order, anchor=0)

    positions = run_calibration(server)

    expected = tuple(order[(0 + step) % len(order)] for step in range(len(order)))
    assert positions == expected
    assert server.position == server.anchor


def test_calibration_accepts_a_fully_capture_enabled_cycle():
    order = ("drone_front", "rover_front", "drone_chase", "rover_chase")
    server = FakeServer(order, anchor=3)

    def complete_fetch(robot, camera):
        for mode, (expected_robot, expected_camera) in CAMERAS.items():
            if (robot, camera) == (expected_robot, expected_camera):
                return FRAMES[mode]
        return None

    positions = run_calibration(server, fetch_frame=complete_fetch)
    expected = tuple(order[(3 + step) % len(order)] for step in range(len(order)))
    assert positions == expected
    assert server.position == server.anchor


def test_calibration_fails_when_two_cameras_are_unfetchable():
    order = ("rover_front", "rover_chase", "drone_front", "drone_chase")
    server = FakeServer(order)
    messages = []

    def partial_fetch(robot, camera):
        for mode, (expected_robot, expected_camera) in CAMERAS.items():
            if (robot, camera) == (expected_robot, expected_camera):
                if mode in ("rover_chase", "drone_chase"):
                    return None
                return FRAMES[mode]
        return None

    positions = run_calibration(
        server,
        fetch_frame=partial_fetch,
        log=lambda message, level="INFO": messages.append((message, level)),
    )
    assert positions is None
    assert any("ambiguous" in message for message, _ in messages)


def test_calibration_fails_when_the_screen_does_not_match_any_camera():
    order = ("rover_front", "rover_chase", "drone_front", "drone_chase")
    server = FakeServer(order)
    noise = np.dstack([np.full((90, 160), 128, dtype=np.uint8)] * 3)

    positions = run_calibration(server, screenshot=lambda: noise)
    assert positions is None
    assert server.position == server.anchor


def test_calibration_fails_when_a_mode_has_no_camera():
    order = ("rover_front", "rover_chase", "drone_front", "drone_chase")
    server = FakeServer(order)
    cameras = dict(CAMERAS)
    del cameras["drone_chase"]

    positions = run_calibration(server, cameras=cameras)
    assert positions is None
    assert server.screenshots == 0


def test_calibration_restores_the_anchor_when_a_screenshot_raises():
    order = ("rover_front", "rover_chase", "drone_front", "drone_chase")
    server = FakeServer(order)
    server.fail_on_screenshot = 2

    positions = run_calibration(server)
    assert positions is None
    assert server.position == server.anchor


def test_calibration_returns_none_when_the_switch_is_refused():
    order = ("rover_front", "rover_chase", "drone_front", "drone_chase")
    server = FakeServer(order, switch_ok=False)

    positions = run_calibration(server)
    assert positions is None
    assert server.position == server.anchor


# Real logged score matrices from the live simulator. In the Neighborhood
# scene with both vehicles on the same spawn point, the rover chase position
# correlates 0.75 with rover_front while the true rover_front position scores
# 0.998; per-position classification used to abort calibration here.
REAL_SCORE_ROWS = [
    {"drone_front": 0.998, "rover_front": 0.737, "drone_chase": 0.478},
    {"drone_chase": 0.999, "rover_front": 0.641, "drone_front": 0.492},
    {"drone_chase": 0.786, "rover_front": 0.754, "drone_front": 0.562},
    {"rover_front": 0.998, "drone_front": 0.746, "drone_chase": 0.629},
]


def test_assignment_survives_a_chase_camera_correlating_with_a_front_camera():
    positions, reason = _solve_assignment(
        REAL_SCORE_ROWS, VIEW_MODES, ["rover_chase"]
    )

    assert reason is None
    assert positions == (
        "drone_front",
        "drone_chase",
        "rover_chase",
        "rover_front",
    )


def test_assignment_rejects_a_leftover_position_that_truly_matches():
    rows = [dict(row) for row in REAL_SCORE_ROWS]
    # Make the elimination-only position look like a real match of a camera
    # that is already assigned elsewhere.
    rows[2]["drone_chase"] = MAX_UNMATCHED_SCORE + 0.01

    positions, reason = _solve_assignment(rows, VIEW_MODES, ["rover_chase"])

    assert positions is None
    assert reason is not None and "correlates" in reason


# Live Neighborhood scores where the parked drone made its two cameras look
# almost identical: position 1 differs from position 0 by 0.086 only. The
# runner-up (drone_front) matches position 0 at 1.000, which identifies the
# positions even though the single-position margin is small.
PARKED_DRONE_SCORE_ROWS = [
    {"drone_front": 1.000, "drone_chase": 0.893, "rover_front": 0.159},
    {"drone_chase": 0.996, "drone_front": 0.910, "rover_front": 0.171},
    {"rover_front": 0.642, "drone_front": 0.403, "drone_chase": 0.316},
    {"rover_front": 0.841, "drone_front": 0.267, "drone_chase": 0.237},
]


def test_assignment_uses_cycle_evidence_when_the_drone_pair_looks_alike():
    positions, reason = _solve_assignment(
        PARKED_DRONE_SCORE_ROWS, VIEW_MODES, ["rover_chase"]
    )

    assert reason is None
    assert positions == (
        "drone_front",
        "drone_chase",
        "rover_chase",
        "rover_front",
    )


def test_assignment_rejects_two_equally_good_permutations():
    # Both drone cameras match both positions equally well; front and chase
    # would be interchangeable, so the cycle must not be trusted.
    rows = [
        {"drone_front": 1.0, "drone_chase": 1.0, "rover_front": 0.1},
        {"drone_front": 1.0, "drone_chase": 1.0, "rover_front": 0.1},
        {"drone_front": 0.1, "drone_chase": 0.1, "rover_front": 0.8},
        {"drone_front": 0.1, "drone_chase": 0.1, "rover_front": 0.9},
    ]

    positions, reason = _solve_assignment(rows, VIEW_MODES, ["rover_chase"])

    assert positions is None
    assert reason is not None and "ambiguous" in reason
