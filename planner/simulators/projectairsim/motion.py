"""配置包络转 domain 类型的唯一入口。

跟踪律、采样循环与停稳判定都在 ``planner.runtime`` 的 ``ControlLoop`` 里；
引擎只剩 ``stream_command`` 一条数据面动作。
"""

from __future__ import annotations

from planner.domain import MotionLimits


def tracking_limits(motion) -> MotionLimits:
    """Return the envelope as-is; configured and tracking limits are one type now.

    The wrapper stays because UE5's vehicles call it in three places and pass
    ``None`` when no envelope is configured.
    """
    return motion if motion is not None else MotionLimits()
