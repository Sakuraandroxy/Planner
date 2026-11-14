"""Scene-configuration camera validation (pure functions, no simulator import).

ProjectAirSim has no camera-info query, so the robot JSONC is the only source
of the render size, horizontal FOV and mount pose. The planner config mirrors
those values; this module compares the two at startup so a scene edit cannot
silently invalidate depth back-projection.
"""

from __future__ import annotations

from ...domain.observation import CameraIntrinsics

_SCENE_CAMERA = 0
_DEPTH_CAMERA = 2


def _numbers(value: object, *, name: str) -> tuple[float, float, float]:
    if isinstance(value, str):
        parts = value.split()
    elif isinstance(value, (list, tuple)):
        parts = list(value)
    else:
        raise ValueError(f"{name} must be a whitespace string or a list")
    if len(parts) != 3:
        raise ValueError(f"{name} must contain three numbers")
    try:
        return tuple(float(part) for part in parts)  # type: ignore[return-value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain three numbers") from exc


def sensor_configuration(
    scene_configuration: dict, robot_name: str, camera_id: str
) -> dict:
    """Return the loaded robot-config sensor entry for ``camera_id``."""
    try:
        actors = scene_configuration["actors"]
        actor = next(
            entry for entry in actors if entry.get("name") == robot_name
        )
        robot_config = actor["robot-config"]
        sensors = robot_config.get("sensors", [])
    except (KeyError, StopIteration, TypeError) as exc:
        raise ValueError(
            f"scene configuration has no robot {robot_name!r}"
        ) from exc
    sensor = next(
        (entry for entry in sensors if entry.get("id") == camera_id), None
    )
    if sensor is None:
        raise ValueError(
            f"robot {robot_name!r} has no sensor {camera_id!r} in the scene"
        )
    if sensor.get("type") != "camera":
        raise ValueError(f"sensor {camera_id!r} is not a camera")
    return sensor


def capture_settings(sensor: dict) -> dict[int, dict]:
    """Capture settings keyed by image type."""
    settings = {}
    for setting in sensor.get("capture-settings", []):
        try:
            settings[int(setting["image-type"])] = setting
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("camera capture setting has no image type") from exc
    return settings


def mount_from_sensor(sensor: dict) -> tuple[
    tuple[float, float, float], tuple[float, float, float]
]:
    origin = sensor.get("origin") or {}
    return (
        _numbers(origin.get("xyz", "0 0 0"), name="camera origin xyz"),
        _numbers(origin.get("rpy-deg", "0 0 0"), name="camera origin rpy-deg"),
    )


def assert_camera_matches_scene(
    sensor: dict,
    *,
    width: int,
    height: int,
    hfov_deg: float,
    offset_m: tuple[float, float, float],
    mount_rpy_deg: tuple[float, float, float],
    tolerance: float = 1e-3,
) -> None:
    """Reject config/scene disagreement before any frame is decoded."""
    offset, rpy = mount_from_sensor(sensor)
    for label, configured, scene in (
        ("mount position", offset_m, offset),
        ("mount rpy", mount_rpy_deg, rpy),
    ):
        if max(abs(a - b) for a, b in zip(configured, scene)) > tolerance:
            raise ValueError(
                f"camera {label} {tuple(configured)} does not match the scene "
                f"{tuple(scene)}; update camera.offset_m/mount_rpy_deg"
            )
    settings = capture_settings(sensor)
    for image_type, label in (
        (_SCENE_CAMERA, "scene camera"),
        (_DEPTH_CAMERA, "depth camera"),
    ):
        setting = settings.get(image_type)
        if setting is None:
            raise ValueError(
                f"the camera has no image-type {image_type} ({label}) capture setting"
            )
        for key, expected in (
            ("width", width),
            ("height", height),
            ("fov-degrees", hfov_deg),
        ):
            actual = float(setting.get(key, -1))
            if abs(actual - float(expected)) > tolerance:
                raise ValueError(
                    f"camera {label} {key} {actual} does not match the configured "
                    f"{expected}; update the camera section"
                )


def intrinsics_from_camera(width: int, height: int, hfov_deg: float) -> CameraIntrinsics:
    return CameraIntrinsics.from_horizontal_fov(width, height, hfov_deg)