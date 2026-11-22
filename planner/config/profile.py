"""Profile identity and drift: what a profile is, where it differs."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .paths import (
    available_profiles,
    default_profile,
    profile_config_path,
    profile_of,
)
from .sections import _section


@dataclass(frozen=True)
class ProfileInfo:
    """How one profile is introduced: the console shows ``label``, and ``id`` is the
    directory name and the value the switch endpoint takes.
    """

    id: str = ""
    label: str = ""
    note: str = ""
    #: The trajectory planner this profile calls, shown so a viewer can see what
    #: the switch actually changed. Read from the raw section, not validated here.
    model: str = ""


def _profile_info(raw: dict[str, Any], path: str | Path) -> ProfileInfo:
    section = _section(raw, "profile")
    name = profile_of(path)
    declared = str(section.get("id", "")).strip()
    if declared and name and declared != name:
        raise ValueError(
            f"profile id '{declared}' does not match its directory '{name}': "
            f"rename it or move the file, the directory is what the switch uses"
        )
    identifier = name or declared
    planner = raw.get("trajectory_planner")
    model = str(planner.get("model", "")).strip() if isinstance(planner, dict) else ""
    return ProfileInfo(
        id=identifier,
        label=str(section.get("label", "")).strip() or identifier,
        note=str(section.get("note", "")).strip(),
        model=model,
    )


#: Sections a profile is allowed to differ in. Everything else describes the
#: deployment, not the navigation: the simulator process, the engine's own camera
#: mount, the appearance models and the HTTP service are not rebuilt when a
#: profile switches, so a difference there is either inert or a quiet
#: disagreement with an engine that is already running.
PROFILE_SWITCHABLE_SECTIONS = frozenset({
    "profile",
    "drone",
    "depth",
    "motion",
    "sliding_window",
    "target_detection",
    "trajectory_planner",
    "task_completion",
    "task_parser",
    "visual_goal_navigation",
    "free_navigation",
})


def _profile_raw(profile: str) -> dict[str, Any]:
    raw = yaml.safe_load(
        profile_config_path(profile).read_text(encoding="utf-8")
    ) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"profile '{profile}' config root must be a mapping")
    return raw


def profile_drift(reference: str | None = None) -> tuple[str, ...]:
    """Where a profile differs from the reference outside ``PROFILE_SWITCHABLE_SECTIONS``.

    Empty means every profile only changes what switching can actually apply.
    ``scripts/check_profiles.py`` prints this and fails a deployment on it.
    """
    profiles = available_profiles()
    base = reference or default_profile()
    if base not in profiles:
        return (f"reference profile '{base}' is not installed",)
    reference_raw = _profile_raw(base)
    drift = []
    for name in profiles:
        if name == base:
            continue
        raw = _profile_raw(name)
        for section in sorted(
            (set(reference_raw) | set(raw)) - PROFILE_SWITCHABLE_SECTIONS
        ):
            if reference_raw.get(section) != raw.get(section):
                drift.append(
                    f"{name}: section '{section}' differs from '{base}'"
                )
    return tuple(drift)


def profile_catalog() -> tuple[ProfileInfo, ...]:
    """Every installed profile with the name it introduces itself by.

    Reads only the ``profile`` section of each base file, so a page can list the
    profiles without building simulators or validating the whole configuration.
    """
    entries = []
    for name in available_profiles():
        raw = _profile_raw(name)
        entries.append(_profile_info(raw, profile_config_path(name)))
    return tuple(entries)


