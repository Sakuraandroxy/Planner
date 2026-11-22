"""Motion and collision contract types reject what the vehicle cannot execute."""

from dataclasses import FrozenInstanceError, replace

import pytest

from planner.domain import (
    CarCommand,
    CollisionContact,
    CollisionState,
    HeadingHoldSettings,
    HolonomicCommand,
    MotionCapability,
    MotionLimits,
    MotionModel,
    command_type_for,
    require_supported,
)

ACKERMANN = MotionCapability(
    MotionModel.ACKERMANN,
    max_lateral_speed_mps=0.0,
    min_turn_radius_m=3.2,
    allows_reverse=True,
)
HOLONOMIC = MotionCapability(
    MotionModel.HOLONOMIC,
    max_lateral_speed_mps=2.0,
    min_turn_radius_m=None,
    allows_reverse=True,
)


# -- MotionCapability ------------------------------------------------------
@pytest.mark.parametrize(
    "kwargs",
    [
        pytest.param(
            {
                "model": MotionModel.ACKERMANN,
                "max_lateral_speed_mps": 0.6,
                "min_turn_radius_m": 3.2,
                "allows_reverse": True,
            },
            id="ackermann-with-lateral-speed",
        ),
        pytest.param(
            {
                "model": MotionModel.ACKERMANN,
                "max_lateral_speed_mps": 0.0,
                "min_turn_radius_m": None,
                "allows_reverse": True,
            },
            id="ackermann-without-turn-radius",
        ),
        pytest.param(
            {
                "model": MotionModel.HOLONOMIC,
                "max_lateral_speed_mps": 2.0,
                "min_turn_radius_m": 3.2,
                "allows_reverse": True,
            },
            id="holonomic-with-turn-radius",
        ),
        pytest.param(
            {
                "model": MotionModel.HOLONOMIC,
                "max_lateral_speed_mps": 0.0,
                "min_turn_radius_m": None,
                "allows_reverse": True,
            },
            id="holonomic-without-lateral-speed",
        ),
        pytest.param(
            {
                "model": MotionModel.HOLONOMIC,
                "max_lateral_speed_mps": float("nan"),
                "min_turn_radius_m": None,
                "allows_reverse": True,
            },
            id="non-finite-lateral-speed",
        ),
        pytest.param(
            {
                "model": MotionModel.HOLONOMIC,
                "max_lateral_speed_mps": 2.0,
                "min_turn_radius_m": None,
                "allows_reverse": "yes",
            },
            id="non-boolean-allows-reverse",
        ),
    ],
)
def test_motion_capability_rejects_contradictions(kwargs):
    with pytest.raises(ValueError):
        MotionCapability(**kwargs)


# -- commands --------------------------------------------------------------
@pytest.mark.parametrize(
    "kwargs",
    [
        pytest.param(
            {"forward_speed_mps": 1.0, "yaw_rate_deg_s": 0.0, "reverse": True},
            id="forward-with-reverse",
        ),
        pytest.param(
            {"forward_speed_mps": 0.0, "yaw_rate_deg_s": 3.0, "reverse": True},
            id="standstill-with-reverse",
        ),
        pytest.param(
            {"forward_speed_mps": -1.0, "yaw_rate_deg_s": 0.0, "reverse": False},
            id="backward-without-reverse",
        ),
    ],
)
def test_car_command_rejects_a_reverse_flag_that_contradicts_the_sign(kwargs):
    with pytest.raises(ValueError):
        CarCommand(**kwargs)


def test_car_command_accepts_matching_sign_and_reverse():
    assert CarCommand(1.0, 0.0, reverse=False).reverse is False
    assert CarCommand(-1.0, 0.0, reverse=True).reverse is True
    assert CarCommand(0.0, 3.0, reverse=False).reverse is False


# -- command_type_for ------------------------------------------------------
def test_command_type_for_follows_the_motion_model():
    assert command_type_for(HOLONOMIC) is HolonomicCommand
    assert command_type_for(ACKERMANN) is CarCommand


# -- require_supported -----------------------------------------------------
def test_commands_that_match_the_capability_are_accepted():
    require_supported(CarCommand(1.0, 20.0, False), ACKERMANN)
    require_supported(HolonomicCommand(1.0, 0.3, 0.0, 0.0), HOLONOMIC)


@pytest.mark.parametrize(
    "command, capability",
    [
        pytest.param(
            HolonomicCommand(0.0, 0.3, 0.0, 0.0),
            ACKERMANN,
            id="lateral-command-to-ackermann",
        ),
        pytest.param(
            HolonomicCommand(1.0, 0.0, 0.0, 0.0),
            ACKERMANN,
            id="forward-only-holonomic-command-to-ackermann",
        ),
        pytest.param(
            CarCommand(1.0, 0.0, False),
            HOLONOMIC,
            id="car-command-to-holonomic",
        ),
        pytest.param(
            CarCommand(-1.0, 0.0, True),
            replace(ACKERMANN, allows_reverse=False),
            id="reverse-to-a-vehicle-without-reverse",
        ),
    ],
)
def test_require_supported_rejects_commands_the_vehicle_cannot_execute(
    command, capability
):
    with pytest.raises(ValueError):
        require_supported(command, capability)


# -- frozen ----------------------------------------------------------------
@pytest.mark.parametrize(
    "value, name, replacement",
    [
        (ACKERMANN, "allows_reverse", False),
        (HolonomicCommand(1.0, 0.0, 0.0, 5.0), "vx", 2.0),
        (CarCommand(1.0, 0.0, False), "yaw_rate_deg_s", 5.0),
        (CollisionContact("Building_3", 1.0), "counterpart", "Road_1"),
        (CollisionState(True, None), "in_contact", False),
    ],
)
def test_contract_types_are_frozen(value, name, replacement):
    with pytest.raises(FrozenInstanceError):
        setattr(value, name, replacement)


# -- heading hold nesting --------------------------------------------------
def test_motion_limits_nest_the_heading_hold_settings():
    limits = MotionLimits()
    assert isinstance(limits.heading, HeadingHoldSettings)
    assert limits.heading.heading_hold_gain == pytest.approx(0.4)
    assert limits.heading.heading_rate_damping == pytest.approx(0.0)


def test_heading_hold_settings_reject_a_non_positive_gain():
    with pytest.raises(ValueError):
        HeadingHoldSettings(heading_hold_gain=0.0)


def test_motion_limits_reject_a_heading_field_that_is_not_the_settings_object():
    with pytest.raises(ValueError):
        MotionLimits(heading=0.4)


# -- collision types -------------------------------------------------------
def test_collision_state_carries_the_last_contact_or_none():
    contact = CollisionContact("Building_3", 12.5)
    assert CollisionState(True, contact).last_contact is contact
    # A level-based source reports a contact without naming it.
    assert CollisionState(True, None).last_contact is None


@pytest.mark.parametrize(
    "factory",
    [
        pytest.param(lambda: CollisionContact("", 1.0), id="empty-counterpart"),
        pytest.param(
            lambda: CollisionContact("Building_3", float("inf")),
            id="non-finite-timestamp",
        ),
        pytest.param(lambda: CollisionState("yes", None), id="non-boolean-contact"),
        pytest.param(
            lambda: CollisionState(True, "Building_3"), id="contact-not-a-type",
        ),
    ],
)
def test_collision_types_reject_invalid_fields(factory):
    with pytest.raises(ValueError):
        factory()
