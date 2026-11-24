"""Which visual-goal pipeline flies is one table, indexed by a config name.

The two names are the whole switch: a profile sets ``execution`` and the assembly
looks it up. An unknown name and a missing port both stop at construction time,
because the alternative is a task that flies something nobody selected.
"""

import pytest

from planner.application.mission_assembly import (
    VISUAL_PIPELINES,
    build_mission_common,
    build_vehicle_arm,
    require_pipeline_ports,
    visual_pipeline,
)
from planner.config import (
    BareNavigationSettings,
    default_config_path,
    load_config,
)
from planner.domain.motion import MotionLimits
from planner.errors import ProtocolError
from planner.simulator.ports import PIPELINE_REQUIREMENTS, SimulatorPorts


class _Vehicle:
    """Just enough vehicle for the staged arm: limits plus a collision reading."""

    def __init__(self):
        self.tracking_limits = MotionLimits()

    def collision_state(self):
        return False


class _PathVehicle(_Vehicle):
    """The same vehicle plus the engine path slot the bare pipeline needs."""

    def follow_path(self, points, speed_mps, *, timeout_s):
        raise AssertionError("assembly must not fly the path")


def _quiet(message, level="INFO"):
    return None


def test_the_table_and_the_slot_requirements_list_the_same_pipelines():
    assert set(VISUAL_PIPELINES) == set(PIPELINE_REQUIREMENTS)
    assert set(VISUAL_PIPELINES) == {"staged", "bare"}


def test_an_unknown_name_is_rejected_instead_of_falling_back():
    with pytest.raises(ProtocolError, match="unknown visual-goal execution"):
        visual_pipeline("segmented")


def test_the_bare_pipeline_requires_the_path_slot():
    ports = SimulatorPorts(vehicle=object(), observation=object())
    with pytest.raises(ProtocolError, match="path_follow"):
        require_pipeline_ports(ports, "bare")


def test_the_staged_pipeline_runs_without_the_path_slot():
    ports = SimulatorPorts(vehicle=object(), observation=object())
    require_pipeline_ports(ports, "staged")


def test_an_unknown_name_is_rejected_by_the_port_check_too():
    ports = SimulatorPorts(vehicle=object(), observation=object())
    with pytest.raises(ProtocolError, match="unknown visual-goal pipeline"):
        require_pipeline_ports(ports, "segmented")


def test_the_default_profile_still_requests_the_staged_pipeline():
    settings = load_config(default_config_path()).visual_goal_navigation
    assert settings.execution == "staged"
    assert settings.bare == BareNavigationSettings()


def test_a_bare_arm_without_a_path_port_fails_at_assembly():
    config = load_config(default_config_path())
    object.__setattr__(config.visual_goal_navigation, "execution", "bare")
    common = build_mission_common(
        config, planner=object(), detector=object(), log=_quiet
    )
    with pytest.raises(ProtocolError, match="path_follow"):
        build_vehicle_arm(
            config,
            common=common,
            vehicle=_Vehicle(),
            vehicle_id="drone",
            detector=object(),
            geometry=object(),
            telemetry=object(),
            log=_quiet,
        )


def test_a_bare_arm_with_a_path_port_builds_the_bare_workflow():
    config = load_config(default_config_path())
    object.__setattr__(config.visual_goal_navigation, "execution", "bare")
    common = build_mission_common(
        config, planner=object(), detector=object(), log=_quiet
    )
    vehicle = _PathVehicle()
    built = []

    def factory(moved):
        built.append(moved)
        return object()

    arm = build_vehicle_arm(
        config,
        common=common,
        vehicle=vehicle,
        vehicle_id="drone",
        detector=object(),
        geometry=object(),
        telemetry=object(),
        moving_source_factory=factory,
        path_follow=vehicle,
        log=_quiet,
    )
    kinds = {type(workflow).__name__ for workflow in arm.workflows}
    assert "BareVisualGoalNavigation" in kinds
    assert "VisualGoalNavigationWorkflow" not in kinds
    assert built == [], (
        "the bare arm passes moving_observation_source=None, so its factory must "
        "not run"
    )


def test_a_path_slot_holding_the_wrong_object_fails_at_assembly():
    """A filled slot is not a working port; the message has to name the slot."""
    config = load_config(default_config_path())
    object.__setattr__(config.visual_goal_navigation, "execution", "bare")
    common = build_mission_common(
        config, planner=object(), detector=object(), log=_quiet
    )
    with pytest.raises(ProtocolError, match="path_follow.*follow_path"):
        build_vehicle_arm(
            config,
            common=common,
            vehicle=_Vehicle(),
            vehicle_id="drone",
            detector=object(),
            geometry=object(),
            telemetry=object(),
            path_follow=object(),
            log=_quiet,
        )


def test_a_staged_arm_is_assembled_without_a_path_port():
    """The world-model arm keeps the workflow it flew before this switch existed."""
    config = load_config(default_config_path())
    common = build_mission_common(
        config, planner=object(), detector=object(), log=_quiet
    )
    arm = build_vehicle_arm(
        config,
        common=common,
        vehicle=_Vehicle(),
        vehicle_id="drone",
        detector=object(),
        geometry=object(),
        telemetry=object(),
        log=_quiet,
    )
    kinds = {type(workflow).__name__ for workflow in arm.workflows}
    assert "VisualGoalNavigationWorkflow" in kinds
    assert "BareVisualGoalNavigation" not in kinds
