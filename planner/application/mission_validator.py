from planner.domain.mission import MissionPlan, TaskKind
from planner.domain.basic_motion import BasicMotionParameters
from planner.errors import ProtocolError
from planner.domain.visual_goal import VisualGoalNavigationParameters
from planner.domain.visual_goal_limits import VisualGoalLimits


class MissionValidator:
    def __init__(self, supported_kinds: set[TaskKind], visual_goal_limits: VisualGoalLimits | None = None,
                 maximum_depth_m: float = 200.0, *, maximum_step_m: float = 30.0,
                 maximum_yaw_step_deg: float = 180.0, basic_motion_ready: bool = True):
        self.supported_kinds = supported_kinds
        self.visual_goal_limits = visual_goal_limits
        self.maximum_depth_m = maximum_depth_m
        self.maximum_step_m = maximum_step_m
        self.maximum_yaw_step_deg = maximum_yaw_step_deg
        self.basic_motion_ready = basic_motion_ready

    def validate(self, plan: MissionPlan) -> None:
        if plan.protocol_version != "mission_plan_v1":
            raise ProtocolError(f"unsupported mission protocol: {plan.protocol_version}")
        if not plan.stages:
            raise ProtocolError("mission contains no stages")
        seen: set[str] = set()
        for stage in plan.stages:
            if not stage.stage_id or stage.stage_id in seen:
                raise ProtocolError("mission stage IDs must be non-empty and unique")
            seen.add(stage.stage_id)
            if stage.kind not in self.supported_kinds:
                raise ProtocolError(f"unsupported task kind: {stage.kind.value}")
            if stage.kind is TaskKind.BASIC_MOTION:
                if not isinstance(stage.parameters, BasicMotionParameters):
                    raise ProtocolError("basic motion stage has invalid parameters")
                if not self.basic_motion_ready:
                    raise ProtocolError("confirm basic_motion.free_space before executing this mission")
                try:
                    stage.parameters.validate_limits(self.maximum_step_m, self.maximum_yaw_step_deg)
                except ValueError as exc:
                    raise ProtocolError(str(exc)) from exc
            elif stage.kind is TaskKind.VISUAL_GOAL_NAVIGATION:
                if not isinstance(stage.parameters, VisualGoalNavigationParameters) or self.visual_goal_limits is None:
                    raise ProtocolError("visual goal parameters or required capabilities unavailable")
                try:
                    self.visual_goal_limits.validate_standoff(stage.parameters.standoff_m, self.maximum_depth_m)
                except ValueError as exc:
                    raise ProtocolError(str(exc)) from exc

