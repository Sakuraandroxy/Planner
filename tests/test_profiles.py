"""Profiles must only differ where switching a profile can actually apply.

A profile switch rebuilds the mission worker and reloads the scene. The simulator
process, its display, its ports, the camera mounts inside the engine and the
vehicle appearance models keep running as they were, so a profile that disagrees
about one of those is either inert or wrong in a way nobody sees until the depth
numbers stop matching the picture.
"""

import pytest
import yaml

from planner import config as config_module
from planner.config import (
    available_profiles,
    profile_catalog,
    profile_drift,
)


def _write_profile(root, name, **sections):
    directory = root / name
    directory.mkdir(parents=True)
    raw = {"profile": {"id": name, "label": name}}
    raw.update(sections)
    (directory / "base.yaml").write_text(
        yaml.safe_dump(raw, sort_keys=False), encoding="utf-8"
    )
    return directory


@pytest.fixture
def profiles(tmp_path, monkeypatch):
    root = tmp_path / "core"
    monkeypatch.setattr(config_module, "profiles_root", lambda: root)
    return root


def test_a_deployment_section_that_differs_is_drift(profiles):
    _write_profile(
        profiles, "default", camera={"hfov_deg": 90.0}, motion={"max_speed_mps": 5.0}
    )
    _write_profile(
        profiles, "vlm", camera={"hfov_deg": 70.0}, motion={"max_speed_mps": 3.0}
    )
    assert profile_drift() == ("vlm: section 'camera' differs from 'default'",)


def test_a_section_a_switch_can_apply_is_not_drift(profiles):
    _write_profile(
        profiles,
        "default",
        motion={"max_speed_mps": 5.0},
        trajectory_planner={"model": "deepseek-flash", "max_step_m": 10.0},
    )
    _write_profile(
        profiles,
        "vlm",
        motion={"max_speed_mps": 3.0},
        trajectory_planner={"model": "deepseek-flash", "max_step_m": 6.0},
    )
    assert profile_drift() == ()


def test_a_section_only_one_profile_has_is_drift(profiles):
    _write_profile(profiles, "default", server={"port": 8000})
    _write_profile(profiles, "vlm")
    assert profile_drift() == ("vlm: section 'server' differs from 'default'",)


def test_a_profile_that_names_another_directory_is_refused(profiles):
    directory = profiles / "vlm"
    directory.mkdir(parents=True)
    (directory / "base.yaml").write_text(
        yaml.safe_dump({"profile": {"id": "default", "label": "x"}}), encoding="utf-8"
    )
    with pytest.raises(ValueError, match="does not match its directory"):
        profile_catalog()


def test_installed_profiles_introduce_themselves():
    """Every shipped profile is listed by directory and has something to show a person."""
    catalog = profile_catalog()
    assert tuple(info.id for info in catalog) == available_profiles()
    assert catalog, "config/core must carry at least one profile"
    for info in catalog:
        assert info.label, f"profile '{info.id}' has no label for the console"
        assert info.note, f"profile '{info.id}' has no note explaining itself"


def test_repository_profiles_do_not_drift():
    assert profile_drift() == ()
