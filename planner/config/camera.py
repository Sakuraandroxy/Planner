"""Camera settings a simulator family declares (``camera`` in simulators.yaml)."""

from __future__ import annotations

from typing import Any

from .sections import CameraSettings, _triple


def build_camera_settings(raw: dict[str, Any]) -> CameraSettings:
    """Camera settings a simulator family declares (``camera`` in simulators.yaml).

    Resolution, field of view, mount geometry and depth encoding are properties
    of the simulator package that produces the images, so they sit with the
    family instead of the navigation core.
    """
    known = {
        "width",
        "height",
        "hfov_deg",
        "far_m",
        "depth_is_range",
        "offset_m",
        "mount_rpy_deg",
        "drone_offset_m",
        "drone_mount_rpy_deg",
    }
    unknown = set(raw) - known
    if unknown:
        raise ValueError(
            f"simulator camera has unknown settings {sorted(unknown)}; "
            f"expected any of {sorted(known)}"
        )
    camera = CameraSettings(
        width=int(raw.get("width", 1280)),
        height=int(raw.get("height", 720)),
        hfov_deg=float(raw.get("hfov_deg", 90.0)),
        far_m=float(raw.get("far_m", 1000.0)),
        depth_is_range=bool(raw.get("depth_is_range", True)),
        offset_m=_triple(raw.get("offset_m", (1.8, 0.0, -0.8)), where="camera.offset_m"),
        mount_rpy_deg=_triple(
            raw.get("mount_rpy_deg", (0.0, -3.0, 0.0)),
            where="camera.mount_rpy_deg",
        ),
    )
    if camera.width <= 0 or camera.height <= 0:
        raise ValueError("camera dimensions must be positive")
    if not 0.0 < camera.hfov_deg < 180.0:
        raise ValueError("camera.hfov_deg must be in (0, 180)")
    if camera.far_m <= 0:
        raise ValueError("camera.far_m must be positive")
    return camera
