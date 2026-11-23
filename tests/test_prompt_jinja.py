"""Jinja2 prompt rendering: entry point, frozen baseline, template directory gate.

Three things are checked.

1. ``render_prompt`` fills variables, rejects a missing one, and takes its
   blank lines from the template rather than from the caller.
2. ``golden.json`` holds the text the current builders produce. The comparison
   reads that file back, so a template rewrite that changes wording fails.
3. Every ``.j2`` file under ``templates/`` is named by code that renders it.
"""

from __future__ import annotations

import ast
import json
from pathlib import Path

import pytest
from jinja2.exceptions import UndefinedError

from planner.adapters.prompts import (
    observation_values,
    pose_values,
    render_prompt,
    target_evidence_values,
    trajectory_values,
)
from planner.domain import RelativePoseDelta, RelativeTrajectory, WorldPose
from tests.prompts.scenarios import CASES

ROOT = Path(__file__).resolve().parents[1]
GOLDEN_PATH = Path(__file__).with_name("prompts") / "golden.json"
TEMPLATES_DIR = ROOT / "planner" / "adapters" / "prompts" / "templates"

#: Templates shipped before any code renders them. Empty on purpose.
UNREFERENCED_ALLOWED: frozenset[str] = frozenset()

#: Prompt families the baseline has to cover.
FAMILIES = (
    "task_parser",
    "target_detection",
    "completion",
    "base_",
    "free_navigation",
    "visual_goal",
)


def _render_selftest(**overrides: object) -> str:
    values: dict[str, object] = {
        "prompt_id": 7,
        "vehicle": "UAV",
        "points": [
            RelativePoseDelta(dx=1.5, dy=-0.25),
            RelativePoseDelta(dx=0.5, dy=2.0),
        ],
        "pose": pose_values(WorldPose(1.5, -2.25, 0.0, 15.0)),
        "tail": "done",
    }
    values.update(overrides)
    return render_prompt("selftest", **values)


def test_render_prompt_fills_every_variable():
    assert _render_selftest() == (
        "Prompt 7 for UAV.\n"
        "- forward 1.5, right -0.25\n"
        "- forward 0.5, right 2.0\n"
        "Pose: 1.5, -2.25, 0.0, 15.0\n"
        "Tail: done\n"
    )


def test_a_missing_variable_raises_instead_of_rendering_empty():
    with pytest.raises(UndefinedError, match="vehicle"):
        render_prompt("selftest", prompt_id=7)


def test_the_template_owns_leading_and_interior_blank_lines():
    text = _render_selftest()
    assert not text.startswith("\n")
    assert "\n\n" not in text
    assert text.splitlines() == [
        "Prompt 7 for UAV.",
        "- forward 1.5, right -0.25",
        "- forward 0.5, right 2.0",
        "Pose: 1.5, -2.25, 0.0, 15.0",
        "Tail: done",
    ]


def test_the_rendered_tail_is_exactly_one_newline():
    text = _render_selftest()
    assert text.endswith("\n")
    assert not text.endswith("\n\n")
    assert text == text.rstrip() + "\n"


def test_rendering_the_same_prompt_twice_gives_the_same_text():
    assert _render_selftest() == _render_selftest()


def test_values_module_returns_plain_data():
    from tests.prompts.scenarios import _observation

    observation = observation_values(_observation())
    assert observation["pose"] == {"x": 1.5, "y": -2.25, "z": 0.0, "yaw_deg": 15.0}
    assert observation["camera"]["camera_id"] == "front"
    assert isinstance(observation["camera"]["camera_position_world"], list)

    trajectory = trajectory_values(
        RelativeTrajectory((RelativePoseDelta(1.0, 2.0, 0.0, 5.0),))
    )
    assert trajectory == {
        "points": [{"dx": 1.0, "dy": 2.0, "dz": 0.0, "dyaw_deg": 5.0}],
        "count": 1,
    }

    from tests.prompts.scenarios import _target_evidence

    evidence = target_evidence_values(_target_evidence("jump exceeds the limit"))
    assert evidence["stage_id"] == "stage-1"
    assert evidence["position_update_rejection"] == "jump exceeds the limit"
    assert evidence["goal"]["status"] == "visible"


@pytest.fixture(scope="module")
def golden() -> dict[str, str]:
    return json.loads(GOLDEN_PATH.read_text(encoding="utf-8"))


def test_golden_covers_exactly_the_pinned_scenarios(golden):
    assert sorted(golden) == sorted(CASES)


def test_the_baseline_pins_at_least_thirty_scenarios():
    assert len(CASES) >= 30


def test_every_prompt_family_is_pinned():
    missing = [
        family for family in FAMILIES
        if not any(name.startswith(family) for name in CASES)
    ]
    assert not missing, f"families without a pinned prompt: {missing}"


def _first_difference(expected: str, actual: str) -> str:
    for index, (left, right) in enumerate(zip(expected, actual)):
        if left != right:
            return (
                f"first difference at character {index}: "
                f"expected {expected[index:index + 40]!r}, "
                f"got {actual[index:index + 40]!r}"
            )
    return f"lengths differ: expected {len(expected)}, got {len(actual)}"


@pytest.mark.parametrize("name", sorted(CASES))
def test_prompt_text_matches_the_frozen_baseline(name, golden):
    assert CASES[name] == golden[name], _first_difference(golden[name], CASES[name])


def _string_literals(root: Path) -> set[str]:
    names: set[str] = set()
    for path in sorted(root.rglob("*.py")):
        if "__pycache__" in path.parts:
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                names.add(node.value)
    return names


def test_a_template_nothing_renders_fails_the_gate():
    templates = sorted(TEMPLATES_DIR.glob("*.j2"))
    assert templates, f"no templates under {TEMPLATES_DIR}"
    referenced = _string_literals(ROOT / "planner") | _string_literals(ROOT / "tests")
    unreferenced = [
        path.name
        for path in templates
        if path.stem not in referenced and path.stem not in UNREFERENCED_ALLOWED
    ]
    assert not unreferenced, f"templates no code renders: {unreferenced}"
