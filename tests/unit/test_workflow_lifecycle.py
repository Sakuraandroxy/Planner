"""Bounded lifecycle and generic completion summaries without task-specific CLI branches."""
from dataclasses import replace
from unittest.mock import Mock

import pytest

from planner.bootstrap import Runtime
from planner.domain.result import CompletionBasis, MissionResult, StageResult, StageSummary
from planner.errors import ExecutionError, ResourceCleanupError
from planner.workflows.configured import ConfiguredWorkflow
from run_airsim_cli import _report


def test_unused_or_closed_deferred_workflow_never_constructs_resources():
    factory = Mock()
    deferred = ConfiguredWorkflow(factory)
    deferred.close()
    deferred.close()
    with pytest.raises(ExecutionError, match="closed"):
        deferred.prepare()
    factory.assert_not_called()


def test_deferred_lifecycle_is_explicit_idempotent_and_rejects_run_only_objects():
    instance = Mock()
    factory = Mock(return_value=instance)
    deferred = ConfiguredWorkflow(factory)
    deferred.prepare()
    deferred.prepare()
    deferred.run(object())
    assert deferred.instance is instance
    factory.assert_called_once()
    assert instance.prepare.call_count == 3
    deferred.close()
    deferred.close()
    instance.close.assert_called_once()

    class RunOnly:
        def run(self, stage):
            pass
    invalid = ConfiguredWorkflow(RunOnly)
    with pytest.raises(ExecutionError, match="lifecycle|prepare, run, and close"):
        invalid.prepare()
    invalid.close()


def test_failed_prepare_still_closes_created_resources():
    instance = Mock()
    instance.prepare.side_effect = ValueError("unavailable")
    deferred = ConfiguredWorkflow(Mock(return_value=instance))
    with pytest.raises(ValueError):
        deferred.prepare()
    deferred.close()
    instance.close.assert_called_once()


def test_cleanup_failure_attempts_all_callbacks_and_shutdown_exactly_once():
    calls = []
    def first():
        calls.append("first")
        raise ValueError("private error detail")
    def second():
        calls.append("second")
        raise RuntimeError("second failure")
    connection = Mock()
    connection.shutdown.side_effect = lambda: calls.append("shutdown")
    runtime = Runtime(None, connection, (first, second))
    with pytest.raises(ResourceCleanupError) as error:
        runtime.close()
    assert calls == ["first", "second", "shutdown"] and len(error.value.failures) == 2
    assert "private error detail" not in str(error.value)
    runtime.close()
    connection.shutdown.assert_called_once()


def test_cleanup_preserves_interrupt_after_attempting_remaining_callbacks():
    calls = []
    def interrupt():
        calls.append(1)
        raise KeyboardInterrupt()
    connection = Mock()
    connection.shutdown.side_effect = lambda: calls.append(3)
    runtime = Runtime(None, connection, (interrupt, lambda: calls.append(2)))
    with pytest.raises(KeyboardInterrupt):
        runtime.close()
    assert calls == [1, 2, 3]


def test_generic_cli_reports_any_typed_task_summary_and_marks_missing_evidence(capsys):
    summary = StageSummary("inspected", "required views captured",
        CompletionBasis.MODEL_WITH_LOCAL_CHECKS, True, "/test/diagnostics")
    result = MissionResult(True, (StageResult("inspection", True, object(), summary=summary),))
    assert _report(result) == 0
    output = capsys.readouterr().out
    assert "inspected" in output and "model_assessment" in output and "/test/diagnostics" in output
    assert "Mission completion confirmed" in output
    assert _report(MissionResult(True, (StageResult("execution-only", True),))) == 0
    assert "task completion not verified" in capsys.readouterr().out
    with pytest.raises(ValueError):
        replace(summary, completion_basis=CompletionBasis.NOT_VERIFIED)
    with pytest.raises(ValueError):
        StageResult("bad", False, summary=summary)


def test_cached_assembly_rechecks_readiness_before_each_mission():
    instance = Mock()
    instance.prepare.side_effect = [None, ExecutionError("previous request still running")]
    factory = Mock(return_value=instance)
    deferred = ConfiguredWorkflow(factory)
    deferred.prepare()
    with pytest.raises(ExecutionError, match="previous request"):
        deferred.prepare()
    factory.assert_called_once()
    deferred.close()
