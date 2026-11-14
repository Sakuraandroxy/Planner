"""The same result text is displayed by the CLI and the HTTP log window."""
from planner.domain.result import MissionResult


def mission_report(result: MissionResult) -> tuple[tuple[str, ...], int]:
    lines = []
    for stage in result.stages:
        summary = stage.summary
        if summary is None:
            lines.append(f"Stage {stage.stage_id}: completion not verified; reason={stage.error or 'no summary'}")
            continue
        measurements = "; ".join(f"{item.name}={item.value}{item.unit}" for item in summary.measurements)
        lines.append(f"Stage {stage.stage_id}: {summary.status}; basis={summary.completion_basis.value}; "
                     f"reason={summary.reason}" + (f"; {measurements}" if measurements else ""))
        if summary.diagnostics_path:
            lines.append(f"Stage diagnostics: {summary.diagnostics_path}")
    if result.success:
        verified = bool(result.stages) and all(stage.success and stage.summary is not None and
                                              stage.summary.completion_confirmed for stage in result.stages)
        lines.append(f"Mission completion confirmed by task-specific evidence: {len(result.stages)} stage(s)"
                     if verified else f"Stages executed: {len(result.stages)}; task completion not verified")
        return tuple(lines), 0
    lines.append(f"Mission failed: {result.error}")
    return tuple(lines), 1
