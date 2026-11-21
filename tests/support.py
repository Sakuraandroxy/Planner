"""Shared helpers for tests that hand-build an observation or a config directory.

Not collected by pytest.
"""

from __future__ import annotations

import math
import shutil
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]


def write_config(tmp_path, data, *, app=None) -> Path:
    """Write a synthetic profile next to the shipped app and family catalogs.

    ``config/simulators.yaml`` is required rather than defaulted. It is the one
    place an xpra argv template is written down, and the old answer for a
    deployment without it was a Python copy of the same command line -- two homes
    for one command, which is how a driver ends up launching a stack it cannot
    stop. A test config directory therefore carries a catalog exactly like a real
    installation does.

    ``app`` overrides sections of ``config/app.yml`` (server, process). The
    shipped file is copied as-is otherwise, so a test that does not touch the
    program-level settings still reads the real defaults.
    """
    directory = Path(tmp_path)
    path = directory / "config.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    shutil.copyfile(REPO / "config/simulators.yaml", directory / "simulators.yaml")
    if app is None:
        shutil.copyfile(REPO / "config/app.yml", directory / "app.yml")
    else:
        base = yaml.safe_load((REPO / "config/app.yml").read_text(encoding="utf-8"))
        for section, value in app.items():
            base[section] = value
        (directory / "app.yml").write_text(yaml.safe_dump(base), encoding="utf-8")
    return path


def ue_camera_matrix(
    x_units: float, y_units: float, z_units: float, yaw_deg: float
) -> np.ndarray:
    """4x4 camera-to-UE-world matrix (row-major) for a level, yawed camera.

    UE yaw is right-positive, so the rotation maps forward (1,0,0) to
    (cos, sin, 0). Test observations pass ``(x, y, -z, yaw)`` so their default
    ``camera_position_world`` equals the planner pose.
    """
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [c, -s, 0.0, x_units],
            [s, c, 0.0, y_units],
            [0.0, 0.0, 1.0, z_units],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )