"""Optional collision checks: no simulator/model connections in these tests."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from config import load_config
from config.visual_goal import visual_goal_config
from planner.adapters.airsim.observation_gate import ObservationGate
from planner.bootstrap import build_runtime
from planner.domain.mission import TaskKind
from planner.domain.motion import MotionLimits
from planner.domain.pose import WorldPose
from planner.errors import BudgetExceeded, ExecutionError
from planner.services.execution import ExecutionService
from tests.fakes.configuration import MAIN_CONFIG, visual_config_data, write_config
from tests.fakes.visual_goal import Clock


def test_collision_checks_default_off_in_types_and_main_config():
    assert not visual_goal_config({}).collision_check_enabled
    settings = load_config(MAIN_CONFIG)
    assert not hasattr(settings, "basic_motion")
    assert not settings.visual_goal.collision_check_enabled


@pytest.mark.parametrize("value", ["false", 0, None, {}])
def test_collision_flag_requires_yaml_boolean(value):
    with pytest.raises(ValueError, match="boolean"):
        visual_goal_config({"collision_check_enabled": value})


@pytest.mark.parametrize("visual_on", [False, True])
def test_only_visual_workflow_injects_optional_collision_source(tmp_path, monkeypatch, visual_on):
    raw = visual_config_data()
    raw['visual_goal_navigation'].update(collision_check_enabled=visual_on, geometry_verified=True)
    for role in ("task_parser", "target_detection", "task_completion"):
        monkeypatch.setenv(raw[role]["api_key_env"], "offline-" + role)
    connection = Mock()
    monkeypatch.setattr("planner.bootstrap.AirSimConnection", Mock(return_value=connection))
    post = Mock(side_effect=AssertionError("no HTTP during assembly"))
    monkeypatch.setattr("requests.post", post)
    runtime = build_runtime(load_config(write_config(tmp_path / "optional.yaml", raw)))
    try:
        workflows = runtime.application.runner.router._workflows
        basic = workflows[TaskKind.BASIC_MOTION]
        deferred = workflows[TaskKind.VISUAL_GOAL_NAVIGATION]
        deferred.prepare()
        visual = deferred.instance
        assert not hasattr(basic, "collision_check")
        assert (visual.collision_check is not None) is visual_on
        if visual.collision_check is not None:
            assert visual.collision_check.__self__ is basic.execution.vehicle
        connection.call.assert_not_called()
        post.assert_not_called()
    finally:
        runtime.close()


def test_enabled_event_is_latched_and_release_isolates_the_next_session():
    read = Mock(side_effect=[True, False])  # Models a consume-on-read source.
    execution = ExecutionService(SimpleNamespace())
    first, second = object(), object()
    execution.acquire_flight(first, collision_check=read)
    assert execution.collision_state()
    with pytest.raises(ExecutionError, match="collision"):
        execution.checkpoint()
    assert read.call_count == 1
    execution.release_flight(first)
    execution.acquire_flight(second)
    execution.checkpoint()
    assert not execution.collision_state()
    assert read.call_count == 1
    execution.release_flight(second)
    execution.acquire_flight(first, collision_check=read)
    assert not execution.collision_state()
    assert read.call_count == 2
    execution.release_flight(first)


@pytest.mark.parametrize("enabled", [False, True])
def test_observation_gate_obeys_active_workflow_policy(enabled):
    clock = Clock()
    read = Mock(return_value=True)
    vehicle = SimpleNamespace(hover=Mock(), current_pose=lambda: WorldPose(0, 0, -4, 0),
                              collision_state=read)
    execution = ExecutionService(vehicle, clock)
    owner = object()
    execution.acquire_flight(owner, collision_check=read if enabled else None)
    gate = ObservationGate(vehicle, MotionLimits(), 5, clock, clock.sleep,
                           check_safety=execution.check_safety)
    try:
        if enabled:
            with pytest.raises(ExecutionError, match="collision"):
                with gate.capture():
                    pytest.fail("a collision must block capture when enabled")
            assert read.call_count == 1
        else:
            with gate.capture():
                pass
            read.assert_not_called()
            vehicle.hover.assert_called_once()
    finally:
        execution.release_flight(owner)


def test_disabled_collision_checks_preserve_stop_and_deadline():
    clock = Clock()
    read = Mock(side_effect=AssertionError("disabled collision source was accessed"))
    execution = ExecutionService(SimpleNamespace(collision_state=read), clock)
    owner = object()
    execution.acquire_flight(owner)
    execution.request_stop()
    with pytest.raises(ExecutionError, match="stop requested"):
        execution.checkpoint()
    execution.set_deadline(clock() + 1)
    clock.sleep(2)
    with pytest.raises(BudgetExceeded):
        execution.checkpoint()
    execution.release_flight(owner)
    read.assert_not_called()


@pytest.mark.parametrize("source", [Mock(return_value="false"), Mock(side_effect=RuntimeError("source unavailable"))])
def test_enabled_source_fault_is_not_silently_treated_as_no_collision(source):
    execution = ExecutionService(SimpleNamespace())
    owner = object()
    execution.acquire_flight(owner, collision_check=source)
    try:
        with pytest.raises((ExecutionError, RuntimeError)):
            execution.checkpoint()
    finally:
        execution.release_flight(owner)
