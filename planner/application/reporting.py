"""The same result text is displayed by the CLI and the HTTP log window."""
from planner.domain.result import MissionResult
from planner.domain.server import WorkerEvent, WorkerEventKind


def _stage_payload(stage, stage_result) -> dict:
    """JSON-safe summary of one stage, for the console's eval snapshot.

    Only goal-bearing stages (``visual_goal_navigation``) are saved by the
    console, but every stage is described here so the console can decide.
    """
    parameters = stage.parameters
    goal = stage.kind.value == "visual_goal_navigation"
    item = {
        "stage_id": stage.stage_id,
        "kind": stage.kind.value,
        "goal": goal,
        "instruction": getattr(parameters, "instruction", "") or "",
        "success": bool(stage_result.success),
    }
    if goal:
        item["target_description"] = getattr(parameters, "target_description", "") or ""
    value = stage_result.value
    for name in ("final_distance_m", "elapsed_s", "path_length_m", "planning_calls"):
        number = getattr(value, name, None)
        if isinstance(number, (int, float)) and not isinstance(number, bool):
            item[name] = number
    summary = stage_result.summary
    if summary is not None:
        item["status"] = summary.status
        item["completion_confirmed"] = bool(summary.completion_confirmed)
        for measurement in summary.measurements:
            if measurement.name not in item and isinstance(
                measurement.value, (int, float)
            ) and not isinstance(measurement.value, bool):
                item[measurement.name] = measurement.value
    return item


def _result_payload(result: MissionResult) -> dict | None:
    """Per-stage summaries plus the plan, for the eval save button."""
    plan = getattr(result, "plan", None)
    if plan is None:
        return None
    by_id = {stage.stage_id: stage for stage in result.stages}
    stages = []
    for stage in plan.stages:
        stage_result = by_id.get(stage.stage_id)
        if stage_result is not None:
            stages.append(_stage_payload(stage, stage_result))
    return {"stages": stages, "success": bool(result.success),
            "error": result.error}


def mission_report(result: MissionResult) -> tuple[tuple[str, ...], int]:

    lines = []
    for stage in result.stages:
        summary = stage.summary
        if summary is None:
            lines.append(
                f"Stage {stage.stage_id}: completion not verified; "
                f"reason={stage.error or 'no summary'}"
            )
            continue
        measurements = "; ".join(
            f"{item.name}={item.value}{item.unit}"
            for item in summary.measurements
        )
        lines.append(
            f"Stage {stage.stage_id}: {summary.status}; "
            f"basis={summary.completion_basis.value}; "
            f"reason={summary.reason}"
            + (f"; {measurements}" if measurements else "")
        )
        if summary.diagnostics_path:
            lines.append(f"Stage diagnostics: {summary.diagnostics_path}")
    if result.success:
        verified = bool(result.stages) and all(
            stage.success
            and stage.summary is not None
            and stage.summary.completion_confirmed
            for stage in result.stages
        )
        lines.append(
            "Mission completion confirmed by task-specific evidence: "
            f"{len(result.stages)} stage(s)"
            if verified
            else f"Stages executed: {len(result.stages)}; "
            "task completion not verified"
        )
        return tuple(lines), 0
    lines.append(f"Mission failed: {result.error}")
    return tuple(lines), 1


def task_finish_events(result, task_id: str):
    """Render a finished mission into its log lines and the one FINISHED event.

    Both worker styles report through here on purpose: the console has to read the
    same completion text whether the session ran on a thread inside the HTTP process
    or in a child that holds the AirSim client. The cancellation line is part of that
    contract -- the controller waits for it as the stop confirmation.
    """
    lines, code = mission_report(result)
    lines = tuple(lines)
    if (result.error or "").startswith("cancelled"):
        # The kinematic car always holds the last applied setpoint; the FINISHED
        # event is the stop confirmation the controller waits for.
        lines = lines + (
            f"Stop confirmed for {task_id}: the vehicle holds its last setpoint.",
        )
    message = lines[-1] if lines else "Task finished."
    return lines, WorkerEvent(
        WorkerEventKind.FINISHED,
        task_id=task_id,
        message=message,
        success=code == 0,
        payload=_result_payload(result),
    )
