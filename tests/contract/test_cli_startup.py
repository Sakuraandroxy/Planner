"""Offline lifecycle checks: startup flight precedes all task interaction."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import run_airsim_cli as cli


def install_runtime(monkeypatch, *, instruction=None, no_takeoff=False):
    events = []
    connection = Mock()
    for name in ("connect", "prepare_vehicle", "takeoff"):
        getattr(connection, name).side_effect = lambda name=name: events.append(name)
    application = Mock()
    application.execute.side_effect = lambda text: events.append(("task", text)) or object()
    runtime = SimpleNamespace(connection=connection, application=application, close=Mock())
    monkeypatch.setattr(cli, "parse_args", lambda: SimpleNamespace(
        config="offline.yaml", instruction=instruction, no_takeoff=no_takeoff,
        record_camera_api=False))
    monkeypatch.setattr(cli, "load_config", lambda path: object())
    monkeypatch.setattr(cli, "build_runtime", lambda config: runtime)
    monkeypatch.setattr(cli, "_report", lambda result: 0)
    return runtime, events


def test_interactive_input_follows_hover_and_connection_is_reused(monkeypatch):
    runtime, events = install_runtime(monkeypatch)
    replies = iter(("上升2米", "向前飞1米", "q"))

    def read_task(prompt):
        assert events[:3] == ["connect", "prepare_vehicle", "takeoff"]
        events.append("input")
        return next(replies)

    monkeypatch.setattr("builtins.input", read_task)
    assert cli.main() == 0
    assert events == ["connect", "prepare_vehicle", "takeoff", "input",
        ("task", "上升2米"), "input", ("task", "向前飞1米"), "input"]
    runtime.connection.connect.assert_called_once_with()
    runtime.connection.takeoff.assert_called_once_with()
    runtime.close.assert_called_once_with()


@pytest.mark.parametrize("no_takeoff", [False, True])
def test_single_instruction_waits_for_startup_and_respects_skip_flag(monkeypatch, no_takeoff):
    runtime, events = install_runtime(monkeypatch, instruction="上升2米", no_takeoff=no_takeoff)
    read_task = Mock(side_effect=AssertionError("single instruction must not prompt"))
    monkeypatch.setattr("builtins.input", read_task)
    assert cli.main() == 0
    expected = ["connect", "prepare_vehicle"]
    if not no_takeoff:
        expected.append("takeoff")
    assert events == expected + [("task", "上升2米")]
    read_task.assert_not_called()
    runtime.close.assert_called_once_with()


def test_interactive_task_rejection_keeps_session_for_next_instruction(monkeypatch):
    runtime, _ = install_runtime(monkeypatch)
    runtime.application.execute.side_effect = [ValueError("unavailable task"), object()]
    replies = iter(("invalid", "上升2米", "q"))
    monkeypatch.setattr("builtins.input", lambda prompt: next(replies))
    assert cli.main() == 0
    assert runtime.application.execute.call_count == 2
    runtime.connection.takeoff.assert_called_once_with()
    runtime.close.assert_called_once_with()


@pytest.mark.parametrize("step", ["connect", "prepare_vehicle", "takeoff"])
def test_startup_failure_prevents_task_input_and_still_cleans_up(monkeypatch, step):
    runtime, _ = install_runtime(monkeypatch)
    getattr(runtime.connection, step).side_effect = RuntimeError("startup failed")
    read_task = Mock(side_effect=AssertionError("startup did not finish"))
    monkeypatch.setattr("builtins.input", read_task)
    with pytest.raises(RuntimeError, match="startup failed"):
        cli.main()
    read_task.assert_not_called()
    runtime.application.execute.assert_not_called()
    runtime.close.assert_called_once_with()


def test_interrupt_while_waiting_after_takeoff_closes_runtime(monkeypatch):
    runtime, _ = install_runtime(monkeypatch)
    monkeypatch.setattr("builtins.input", Mock(side_effect=KeyboardInterrupt()))
    assert cli.main() == 130
    runtime.connection.takeoff.assert_called_once_with()
    runtime.close.assert_called_once_with()
