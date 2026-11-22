"""碰撞退避的被动能力：读接触事件、算后退距离、管预算。

``poll`` 每拍读一次载具的接触状态；``begin`` 按第几次连续恢复给出后退计划；``finish``
结束一次恢复；``note_progress`` 在累计前进超过 ``reset_after_progress_m`` 后把次数
归零。控制循环持有它并负责下发后退命令，任务层不读这里的状态。
"""

from __future__ import annotations

from dataclasses import dataclass

from planner.domain import (
    CollisionContact,
    CollisionRecoverySettings,
    CollisionState,
    WorldPose,
)
from planner.errors import ExecutionError


@dataclass(frozen=True)
class BackoffPlan:
    """一次恢复的后退目标；``distance_m`` 已按连续次数放大。"""

    contact: CollisionContact
    start: WorldPose
    distance_m: float
    attempt: int


class CollisionRecovery:
    def __init__(self, settings: CollisionRecoverySettings):
        if not isinstance(settings, CollisionRecoverySettings):
            raise ValueError("collision recovery requires CollisionRecoverySettings")
        self.settings = settings
        #: 连续恢复次数；超过 ``settings.maximum_recoveries`` 预算耗尽。
        self.attempts = 0
        #: 正在进行的恢复；非 ``None`` 时 ``poll`` 不再接受新接触。
        self.active: CollisionContact | None = None
        self._handled: CollisionContact | None = None
        self._baseline_m: float | None = None

    def poll(self, state: CollisionState) -> CollisionContact | None:
        """返回一个尚未处理的接触事件，没有就返回 ``None``。

        恢复进行中返回 ``None``：同一次物理接触在倒车过程中会反复上报，把它算成
        第二次恢复会让后退距离连乘。恢复结束后同一个事件也不再重复消费。
        """
        if not isinstance(state, CollisionState):
            raise ValueError("collision poll requires a CollisionState")
        if self.active is not None:
            return None
        contact = state.last_contact if state.in_contact else None
        if contact is None or contact == self._handled:
            return None
        self._handled = contact
        return contact

    def begin(self, contact: CollisionContact, pose: WorldPose) -> BackoffPlan:
        """记一次恢复并给出后退计划；超出预算抛 ``ExecutionError``。"""
        if not isinstance(contact, CollisionContact):
            raise ValueError("collision begin requires a CollisionContact")
        if not isinstance(pose, WorldPose):
            raise ValueError("collision begin requires a WorldPose")
        self.attempts += 1
        if self.attempts > self.settings.maximum_recoveries:
            raise ExecutionError("collision recovery budget exhausted")
        self.active = contact
        self._handled = contact
        self._baseline_m = None
        return BackoffPlan(
            contact, pose, self.settings.backoff_m * self.attempts, self.attempts
        )

    def finish(self) -> None:
        """结束当前恢复；恢复期间到达的接触事件不再触发新的后退。"""
        self.active = None

    def note_progress(self, path_length_m: float) -> None:
        """累计前进超过复位距离后把连续次数归零。

        基线取恢复结束后的第一拍累计路程，因此复位判据是"恢复之后又前进了多少"，
        不是任务开始以来的总路程。累计路程由执行层维护，这里只读。
        """
        if not self.attempts:
            return
        if self._baseline_m is None:
            self._baseline_m = float(path_length_m)
            return
        if path_length_m - self._baseline_m < self.settings.reset_after_progress_m:
            return
        self.attempts = 0
        self._baseline_m = None
