import socket
import sys
import threading
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from planner.adapters.http import server_api


port_open = server_api._port_open


class FakeProcess:
    def __init__(self, pid):
        self.pid = pid
        self.returncode = None

    def poll(self):
        return self.returncode


@pytest.fixture
def eval_start(monkeypatch, tmp_path):
    eval_dir = tmp_path / "eval"
    eval_dir.mkdir()
    interpreter = tmp_path / "python"
    interpreter.touch()
    cfg = SimpleNamespace(dashboard_port=18010, dashboard_host="0.0.0.0", config_path=tmp_path / "eval.yml")
    monkeypatch.setattr(server_api, "_EVAL_DIR", eval_dir)
    monkeypatch.setattr(server_api, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(server_api, "_eval_config", lambda: cfg)
    monkeypatch.setattr(server_api, "_EVAL_PROC", None)
    monkeypatch.setattr(server_api, "_EVAL_LOCK", threading.Lock())
    monkeypatch.setenv("AVLN_EVAL_PYTHON", str(interpreter))

    processes = []

    def popen(*args, **kwargs):
        proc = FakeProcess(len(processes) + 1)
        processes.append((proc, args, kwargs))
        return proc

    monkeypatch.setattr(server_api.subprocess, "Popen", popen)
    monkeypatch.setattr(
        server_api, "_port_open",
        lambda *args, **kwargs: any(proc.poll() is None for proc, _, _ in processes),
    )
    assert not hasattr(server_api, "_pids_on_port")
    assert not hasattr(server_api, "_kill_eval_service")
    controller = SimpleNamespace(start=lambda: None, close=lambda: None)
    settings = SimpleNamespace(host="127.0.0.1", port=8000, allowed_origins=[])
    app = server_api.create_api(controller, settings, eval_runtime_python="")
    return TestClient(app), processes


def test_eval_start_lazily_starts_then_reuses_dashboard(eval_start):
    client, processes = eval_start

    first = client.post("/api/eval/start")
    second = client.post("/api/eval/start")

    assert first.status_code == second.status_code == 200
    assert first.json() == {"started": True, "port": 18010}
    assert second.json() == {"started": False, "port": 18010}
    assert len(processes) == 1
    assert processes[0][2]["start_new_session"] is True


def test_eval_start_restarts_dashboard_after_child_exits(eval_start):
    client, processes = eval_start

    assert client.post("/api/eval/start").status_code == 200
    processes[0][0].returncode = 1
    response = client.post("/api/eval/start")

    assert response.status_code == 200
    assert response.json() == {"started": True, "port": 18010}
    assert len(processes) == 2


def test_eval_start_returns_conflict_for_untracked_listener(eval_start, monkeypatch):
    client, processes = eval_start
    cfg = server_api._eval_config()
    monkeypatch.setattr(server_api, "_port_open", port_open)

    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        cfg.dashboard_port = listener.getsockname()[1]
        response = client.post("/api/eval/start")

        assert response.status_code == 409
        assert "已被占用" in response.json()["detail"]["message"]
        assert processes == []
        assert port_open(cfg.dashboard_port)


def test_concurrent_eval_start_requests_share_one_child(eval_start):
    client, processes = eval_start
    barrier = threading.Barrier(3)
    results = []

    def request_start():
        barrier.wait()
        results.append(client.post("/api/eval/start"))

    threads = [threading.Thread(target=request_start) for _ in range(2)]
    for thread in threads:
        thread.start()
    barrier.wait()
    for thread in threads:
        thread.join()

    assert [response.status_code for response in results] == [200, 200]
    assert len(processes) == 1


def test_planner_shutdown_closes_dashboard_child(monkeypatch, tmp_path):
    eval_dir = server_api.REPO_ROOT / "eval"
    for name in ("run.py", "evalcore"):
        (tmp_path / name).symlink_to(eval_dir / name)
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    monkeypatch.setattr(server_api, "_EVAL_DIR", tmp_path)
    monkeypatch.setattr(server_api, "_EVAL_PROC", None)
    monkeypatch.setenv("AVLN_EVAL_PORT", str(port))
    monkeypatch.setenv("AVLN_EVAL_PYTHON", sys.executable)
    config_path = tmp_path / "eval.yml"
    config_path.write_text("dashboard_host: 127.0.0.1\n", encoding="utf-8")
    monkeypatch.setenv("AVLN_EVAL_CONFIG", str(config_path))
    controller = SimpleNamespace(start=lambda: None, close=lambda: None)
    settings = SimpleNamespace(host="127.0.0.1", port=8000, allowed_origins=[])
    app = server_api.create_api(controller, settings)

    with TestClient(app) as client:
        response = client.post("/api/eval/start")
        assert response.json() == {"started": True, "port": port}, response.text
        proc, _, _ = server_api._EVAL_PROC
        assert proc.poll() is None
        assert port_open(port)

    assert proc.poll() == 0
    assert not port_open(port)
    assert server_api._EVAL_PROC is None
