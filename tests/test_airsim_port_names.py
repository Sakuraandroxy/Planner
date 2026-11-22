"""Attribute names the UE4 adapters use must exist on the services they call.

Every one of these three came from reading the old branch's code instead of the
assembled machine's: the visual-goal stage died on
"'AirSimVehicle' object has no attribute 'capture'", then on
"'PositionHold' object has no attribute 'wait_for_observation'", then on
"'PositionHold' object has no attribute 'observation_ready'". Each cost a full simulator
boot to find. The names are checkable without an engine, so they are checked here.
"""

import ast
import re
from pathlib import Path

import pytest

from planner.runtime import PositionHold
from planner.simulators.airsim.observation_gate import ObservationGate

AIRSIM_PACKAGE = Path(__file__).resolve().parents[1] / "planner" / "simulators" / "airsim"


def _members(cls, source: Path) -> set[str]:
    """Class attributes plus the instance attributes its ``__init__`` assigns."""
    found = {name for name in dir(cls) if not name.startswith("__")}
    text = source.read_text(encoding="utf-8")
    found |= set(re.findall(r"self\.([a-z_]+)\s*[:=]", text))
    return found


HOLD_MEMBERS = _members(PositionHold, Path("planner/runtime/position_hold.py"))
GATE_MEMBERS = _members(ObservationGate, AIRSIM_PACKAGE / "observation_gate.py")


def _attribute_names(path: Path, receiver_pattern: str) -> set[tuple[str, int]]:
    """``name`` for every ``<receiver>.name`` in the file, with the first line seen."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    found: dict[str, int] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Attribute):
            continue
        target = node.value
        label = getattr(target, "id", None) or (
            getattr(getattr(target, "value", None), "id", None)
            if isinstance(target, ast.Attribute) else None
        )
        if label and re.fullmatch(receiver_pattern, label):
            found.setdefault(node.attr, node.lineno)
    return {(name, line) for name, line in found.items()}


@pytest.mark.parametrize("module", sorted(AIRSIM_PACKAGE.glob("*.py")), ids=lambda p: p.name)
def test_hold_attributes_exist(module):
    for name, line in _attribute_names(module, r"holding"):
        assert name in HOLD_MEMBERS, (
            f"{module.name}:{line} reads holding.{name}, which PositionHold does not "
            f"provide (its members include observable, contains_pose, wait, tick)"
        )


@pytest.mark.parametrize("module", sorted(AIRSIM_PACKAGE.glob("*.py")), ids=lambda p: p.name)
def test_gate_attributes_exist(module):
    for name, line in _attribute_names(module, r"(self_)?observation_gate"):
        assert name in GATE_MEMBERS, (
            f"{module.name}:{line} reads observation_gate.{name}, which "
            f"ObservationGate does not provide"
        )


def test_the_known_translations_are_the_short_ones():
    """Names the adapters settled on, recorded so a rename stays deliberate."""
    assert "observable" in HOLD_MEMBERS and "observation_ready" not in HOLD_MEMBERS
    assert "wait" in HOLD_MEMBERS and "wait_for_observation" not in HOLD_MEMBERS
