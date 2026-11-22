"""Repository locations: the config tree, the profile directories, shared data."""

from __future__ import annotations

from pathlib import Path


def _project_root() -> Path:
    return Path(__file__).resolve().parents[2]


#: Every directory under ``config/core`` is one complete runtime configuration:
#: a ``base.yaml`` plus the task rules it points at. Switching behaviour means
#: switching directory, not merging a patch onto a file.
def profiles_root() -> Path:
    return _project_root() / "config" / "core"


def _profiles_root() -> Path:
    """Read the profiles root through ``planner.config``.

    ``tests/test_profiles.py`` swaps ``planner.config.profiles_root`` for a
    temporary directory, so the package attribute is the one to call.
    """
    from . import profiles_root as resolve

    return resolve()


def default_profile() -> str:
    return "default"


def default_config_path() -> Path:
    return profile_config_path(default_profile())


def profile_of(path: str | Path) -> str:
    """The profile directory a config file belongs to, or '' when it is outside."""
    try:
        return Path(path).resolve().parent.relative_to(_profiles_root().resolve()).parts[0]
    except (IndexError, ValueError):
        return ""


def profile_config_path(profile: str) -> Path:
    name = str(profile).strip()
    if not name or "/" in name or "\\" in name or name in {".", ".."}:
        raise ValueError(f"invalid profile name '{profile}'")
    return _profiles_root() / name / "base.yaml"


def available_profiles() -> tuple[str, ...]:
    """Profile names, in directory order; a directory without a base is not one."""
    root = _profiles_root()
    if not root.is_dir():
        return ()
    return tuple(
        sorted(
            entry.name
            for entry in root.iterdir()
            if entry.is_dir() and (entry / "base.yaml").is_file()
        )
    )


def shared_or_profile(path: str | Path, rel: str | Path) -> Path:
    """Resolve a referenced file inside the profile directory, else the shared one.

    A profile carries only what differs per profile. ``maps.yaml``,
    ``simulators.yaml``, ``scenes/`` and the AirSim settings belong to the
    deployment and are written once under ``config/``. A profile that needs its
    own copy drops it next to its ``base.yaml`` and this finds that first.

    Only a file under ``config/core`` may borrow the deployment copy: a synthetic
    config written into a temp directory by a test must keep resolving against
    that directory, or the repository's own ``maps.yaml`` would leak into it.
    """
    parent = Path(path).parent
    candidate = parent / rel
    if candidate.is_file() or candidate.is_dir():
        return candidate
    root = _profiles_root()
    try:
        parent.resolve().relative_to(root.resolve())
    except ValueError:
        return candidate
    return _project_root() / "config" / rel
