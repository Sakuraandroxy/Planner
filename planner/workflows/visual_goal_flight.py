"""Task-owned pause, semantic verification and retained-flight recovery phase."""
from dataclasses import replace

from planner.domain.execution import ExecutionReceipt
from planner.domain.trajectory import RelativeTrajectory, MotionTrajectory
from planner.domain.visual_goal_completion import VisualGoalCompletionInput
from planner.domain.visual_goal import VisualGoalStatus
from planner.errors import ExecutionError
from planner.capabilities.completion.visual_goal_retained_motion import retained_world_trajectory
from .termination.visual_goal_navigation import VisualGoalFailure


class VisualGoalFlightPhase:
    def __init__(self, execution, motion, validator, limits, trigger_rules,
                 verification, clock, event):
        self.execution, self.motion, self.validator = execution, motion, validator
        self.limits, self.trigger_rules, self.verification = limits, trigger_rules, verification
        self.clock, self._event = clock, event

    def verify(self, parameters, observation, goal, session, trace, *, target_memory=None):
        if session.completion_checks >= self.trigger_rules.maximum_completion_checks:
            raise VisualGoalFailure(VisualGoalStatus.TIMEOUT, "completion check budget exhausted")
        session.completion_checks += 1
        request = VisualGoalCompletionInput(session.stage_id, parameters, observation, session.initial, goal,
            session.geometry, self.trigger_rules,
            self.limits.distance_uncertainty_margin_m, self.limits.maximum_vertical_offset_m,
            session.confirmations, self.limits.confirmation_count, session.reference.description,
            reference_rgb=session.reference.crop,
            target_memory=target_memory if target_memory is not None else session.target_memory)

        def checkpoint():
            session.require_time(self.clock)
            if self.execution.collision_state():
                raise ExecutionError("collision during completion wait")

        verdict = self.verification.check(request, session.budget, self.limits.model_request_timeout_s,
            self.limits.model_retry_count, checkpoint, self.clock)
        checkpoint()
        if not session.policy.anchor_valid(observation, self.execution.current_pose(), self.clock()):
            self._event(trace, "completion_discarded", reason="observation anchor changed")
            return None
        self._event(trace, "completion_check", completed=verdict, check_count=session.completion_checks,
                    retained_points=len(session.pending_motion.segments), radius_m=self.trigger_rules.radius_m,
                    pause_radius_m=self.trigger_rules.effective_pause_radius_m,
                    measured_distance_m=session.geometry.distance_m,
                    distance_source='distance_to_stage_memory' if request.target_memory else 'current_depth')
        return verdict

    def resume(self, observation, parameters, session, trace, *, prefetch=None):
        world = retained_world_trajectory(session.pending_motion, observation.vehicle_pose)
        self.validator.validate_world(world)
        if not session.policy.anchor_valid(observation, self.execution.current_pose(), self.clock()):
            return
        self._event(trace, "resume_retained", retained_points=len(session.pending_motion.segments),
                    execution_policy="direct_model_trajectory")
        self.execute(self.motion.create_motion(world), session, trace,
                     enable_completion_pause=True, prefetch=prefetch)

    def execute(self, motion, session, trace, *, enable_completion_pause: bool, retained_tail=(), prefetch=None):
        round_budget = session.budget
        before = self.execution.current_pose()
        pause_when = (session.trigger.should_pause if enable_completion_pause and
                      session.trigger is not None and session.geometry is not None else None)
        window = session.window if enable_completion_pause else None
        all_pending = MotionTrajectory((*motion.segments, *retained_tail))
        session.pending_motion = all_pending
        if window is not None:
            window.load(all_pending)
            receipt = self._execute_window(motion, retained_tail, session, round_budget, pause_when, prefetch)
        else:
            receipt = self.execution.execute_with_receipt(motion, budget=round_budget, pause_when=pause_when)
        session.pending_motion = receipt.remaining
        if window is not None:
            session.previous = window.record.actions
            session.previous_start = window.record.start
        if receipt.measured_actions and window is None:
            session.previous = RelativeTrajectory(receipt.measured_actions)
            session.previous_start = before
        session.policy.anchor_failures = 0
        if not receipt.paused:
            session.policy.record_motion(before, receipt.final_pose)
        session.path_length_m += receipt.measured_chord_length_m
        self._event(trace, "executed", receipt=receipt, motion=motion)
        if receipt.paused:
            session.paused_for_completion = True
            self._event(trace, "radius_pause", reason=session.trigger.last_reason,
                        remaining_points=len(session.pending_motion.segments), pose=receipt.final_pose)

    def _execute_window(self, motion, retained_tail, session, budget, pause_when, prefetch):
        began = self.clock()
        actions, completed, chords = [], 0, 0.0
        window = session.window
        for index, segment in enumerate(motion.segments):
            if window.should_refill(session.pending_motion, lambda: session.require_time(self.clock)) and prefetch is not None:
                prefetch()
            before = self.execution.current_pose()
            tail = (*motion.segments[index+1:], *retained_tail)
            try:
                receipt = self.execution.execute_with_receipt(MotionTrajectory((segment,)),
                    budget=budget, pause_when=pause_when)
            except (ExecutionError, KeyboardInterrupt) as exc:
                receipt = getattr(exc, "receipt", None)
                if receipt is not None:
                    remaining = MotionTrajectory((*receipt.remaining.segments, *tail))
                    session.pending_motion = remaining
                    exc.receipt = replace(receipt, completed_segments=completed+receipt.completed_segments,
                        elapsed_s=self.clock()-began, measured_chord_length_m=chords+receipt.measured_chord_length_m,
                        remaining=remaining, measured_actions=(*actions, *receipt.measured_actions))
                raise
            remaining = MotionTrajectory((*receipt.remaining.segments, *tail))
            session.pending_motion = remaining
            window.executed(before, receipt, remaining)
            actions.extend(receipt.measured_actions)
            chords += receipt.measured_chord_length_m
            completed += receipt.completed_segments
            if receipt.paused:
                return ExecutionReceipt(completed, receipt.final_pose, self.clock()-began, chords,
                                        True, remaining, tuple(actions))
        return ExecutionReceipt(completed, self.execution.current_pose(), self.clock()-began, chords,
                                False, session.pending_motion, tuple(actions))

