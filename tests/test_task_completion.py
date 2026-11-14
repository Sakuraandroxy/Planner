import numpy as np
import pytest

from tests.support import ue_camera_matrix
from planner.adapters.task_completion import OpenAICompatibleTaskCompletionChecker
from planner.adapters.task_completion.prompts import VisualGoalCompletionPrompt
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import WorldPose
from planner.domain.visual_goal import (
    GoalGeometry,
    GoalObservation,
    GoalVisibility,
    VisualGoalNavigationParameters,
)
from planner.domain.visual_goal_completion import (
    VisualGoalCompletionInput,
    VisualGoalTriggerRules,
)
from planner.errors import ProtocolError


def _observation(pose=WorldPose(0.0, 0.0, 0.0, 0.0)) -> Observation:
    width, height = 8, 6
    return Observation(
        frame=1,
        timestamp_s=0.0,
        pose=pose,
        rgb=np.zeros((height, width, 3), dtype=np.uint8),
        depth_m=np.full((height, width), 5.0, dtype=np.float32),
        depth_image=np.zeros((height, width), dtype=np.uint8),
        intrinsics=CameraIntrinsics.from_horizontal_fov(width, height, 90.0),
        camera_matrix=ue_camera_matrix(pose.x, pose.y, -pose.z, pose.yaw_deg),
        timestamp_ns=10,
        captured_monotonic=1.0,
        capture_id="capture-1",
        stopped=True,
    )


def _request(**overrides) -> VisualGoalCompletionInput:
    values = dict(
        stage_id="stage-1",
        parameters=VisualGoalNavigationParameters(
            "drive to the red car", "the red car"
        ),
        observation=_observation(),
        initial_pose=WorldPose(0.0, 0.0, 0.0, 0.0),
        goal=GoalObservation(
            GoalVisibility.VISIBLE,
            (0.4, 0.4, 0.6, 0.6),
            None,
            "the red car",
            "visible",
            True,
            "model_surface_points",
            surface_points=(),
        ),
        geometry=GoalGeometry(True, "sampled", 5.0, (5.0, 0.0, 0.0)),
        rules=VisualGoalTriggerRules(radius_m=7.0),
        uncertainty_margin_m=0.0,
        height_tolerance_m=None,
        confirmation_count=2,
        required_confirmation_count=2,
        reference_description="the red car",
    )
    values.update(overrides)
    return VisualGoalCompletionInput(**values)


class _FakeTransport:
    def __init__(self, raw: str):
        self.raw = raw
        self.calls = []

    def request(self, system, text, images, *, timeout_s, operation, **kwargs):
        self.calls.append((system, text, images, timeout_s, operation))
        return self.raw


def test_completion_checker_accepts_exact_boolean_text():
    transport = _FakeTransport(" true \n")
    checker = OpenAICompatibleTaskCompletionChecker(
        "http://model/v1",
        "model",
        "",
        20.0,
        prompt_builder=VisualGoalCompletionPrompt(),
    )
    checker.transport = transport
    assert checker.check(_request(), timeout_s=3.0) is True
    system, text, images, timeout_s, operation = transport.calls[0]
    assert operation == "task completion"
    assert timeout_s == 3.0
    assert "视觉目标导航任务完成判断器" in system
    assert "drive to the red car" in text
    assert len(images) == 2
    assert images[0][1] == "JPEG"
    assert images[1][1] == "PNG"


def test_completion_checker_returns_false_and_rejects_other_text():
    transport = _FakeTransport("false")
    checker = OpenAICompatibleTaskCompletionChecker(
        "http://model/v1",
        "model",
        "",
        20.0,
        prompt_builder=VisualGoalCompletionPrompt(),
    )
    checker.transport = transport
    assert checker.check(_request(), timeout_s=3.0) is False
    transport.raw = "True"
    with pytest.raises(ProtocolError):
        checker.check(_request(), timeout_s=3.0)
    transport.raw = "yes"
    with pytest.raises(ProtocolError):
        checker.check(_request(), timeout_s=3.0)


def test_completion_prompt_adds_the_reference_crop_as_third_image():
    request = _request(
        reference_rgb=np.zeros((4, 4, 3), dtype=np.uint8)
    )
    messages = VisualGoalCompletionPrompt.prepare(request)
    assert len(messages.images) == 3
    assert messages.images[2][1] == "JPEG"
    assert "图片3：首次绑定的目标参考图" in messages.user


def test_completion_prompt_requires_local_completion_evidence():
    with pytest.raises(ProtocolError):
        VisualGoalCompletionPrompt.prepare(
            _request(confirmation_count=1)
        )


def test_completion_prompt_reports_local_measurements():
    messages = VisualGoalCompletionPrompt.prepare(_request())
    assert "完成距离上限：7.0米" in messages.user
    assert "当前车辆世界坐标" in messages.user
    assert "只输出true或false" in messages.user