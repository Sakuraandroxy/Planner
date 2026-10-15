from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import threading
import pytest
import yaml
from config import load_config
from planner.domain.pose import WorldPose, RelativePoseDelta
from planner.errors import ConfigurationError, ExecutionError
from planner.services.execution import ExecutionService
from tests.fakes.components import FakeVehicle


def test_flight_handover_waits_for_old_request():
    service = ExecutionService(FakeVehicle())
    first, second = object(), object()
    pending = threading.Event()
    pending.set()
    service.acquire_flight(first)
    service.release_flight(first, pending.is_set)
    with pytest.raises(ExecutionError, match="previous request"):
        service.acquire_flight(second)
    pending.clear()
    service.acquire_flight(second)
    service.release_flight(second)


def test_control_owner_and_stop_request():
    vehicle = FakeVehicle()
    service = ExecutionService(vehicle)
    with ThreadPoolExecutor(1) as pool:
        with pytest.raises(ExecutionError, match="owner"):
            pool.submit(service.stop).result()
        pool.submit(service.request_stop).result()
    assert not vehicle.cancelled
    with pytest.raises(ExecutionError, match="stop requested"):
        service.checkpoint()
    service.stop()
    assert vehicle.cancelled and vehicle.hovered


@pytest.mark.parametrize("bad", [True, "1", float("nan")])
def test_pose_refuses_coercion(bad):
    with pytest.raises(ValueError):
        WorldPose(bad, 0, 0, 0)
    with pytest.raises(ValueError):
        RelativePoseDelta(bad, 0, 0, 0)


@pytest.mark.parametrize("section,key,bad", [
    ("airsim", "speed_mps", True), ("airsim", "port", "41451"),
    ("depth", "max_m", "200"), ("trajectory", "point_count", True),
    ("motion", "control_hz", True), ("airsim", "speeed_mps", 1)])
def test_strict_shared_config(tmp_path, section, key, bad):
    raw = yaml.safe_load(Path("config/base.yaml").read_text(encoding="utf-8"))
    raw[section][key] = bad
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(ConfigurationError):
        load_config(path)


def test_cli_invalid_task_after_startup_does_not_execute_task_motion(monkeypatch):
    import run_airsim_cli as cli
    application = Mock()
    application.prepare.side_effect = ValueError("invalid mission")
    from planner.application.application import PlannerApplication
    application.execute.side_effect = lambda text: PlannerApplication.execute(application, text)
    runtime = SimpleNamespace(application=application, connection=Mock(), close=Mock())
    monkeypatch.setattr(cli, "parse_args", lambda: SimpleNamespace(
        config="config/base.yaml", instruction="invalid", no_takeoff=False, record_camera_api=False))
    monkeypatch.setattr(cli, "build_runtime", lambda config: runtime)
    assert cli.main() == 1
    runtime.connection.connect.assert_called_once_with()
    runtime.connection.prepare_vehicle.assert_called_once_with()
    runtime.connection.takeoff.assert_called_once_with()
    application.execute_plan.assert_not_called()
    runtime.close.assert_called_once()
