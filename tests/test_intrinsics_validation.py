"""内参构造校验：不能描述针孔相机的参数在构造期就拒掉。"""

from __future__ import annotations

import pytest

from planner.domain import CameraIntrinsics


def test_valid_intrinsics_construct():
    intr = CameraIntrinsics(640, 480, 320.0, 320.0, 319.5, 239.5, 90.0)
    assert intr.fx == pytest.approx(320.0)


@pytest.mark.parametrize(
    "kwargs, message",
    [
        ({"width": 0}, "dimensions must be positive"),
        ({"height": -1}, "dimensions must be positive"),
        ({"width": 640.0}, "must be integers"),
        ({"fx": 0.0}, "focal lengths must be positive"),
        ({"fy": -320.0}, "focal lengths must be positive"),
        ({"cx": 640.0}, "principal point"),
        ({"cy": -1.0}, "principal point"),
        ({"fx": float("nan")}, "must be finite"),
        ({"horizontal_fov_deg": 180.0}, "horizontal FOV"),
    ],
)
def test_invalid_intrinsics_rejected(kwargs, message):
    base = dict(
        width=640, height=480, fx=320.0, fy=320.0, cx=319.5, cy=239.5,
        horizontal_fov_deg=90.0,
    )
    base.update(kwargs)
    with pytest.raises(ValueError, match=message):
        CameraIntrinsics(**base)
