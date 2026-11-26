import json
import threading
from http.server import ThreadingHTTPServer
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import urlopen

from eval.evalcore import dashboard


def _serve_report(tmp_path):
    output_dir = tmp_path / "runs"
    run_dir = output_dir / "run_20250101_120000"
    run_dir.mkdir(parents=True)
    (run_dir / "metrics.json").write_text(
        json.dumps({"episodes": [{"uuid": "ep-1", "final_ne": float("nan")}], "metrics": {"sr": 100}}),
        encoding="utf-8",
    )
    dashboard._Handler.cfg = SimpleNamespace(output_dir=str(output_dir))
    server = ThreadingHTTPServer(("127.0.0.1", 0), dashboard._Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def test_report_endpoint_returns_run_metadata_and_episode_metrics(tmp_path):
    server, thread = _serve_report(tmp_path)
    try:
        url = f"http://127.0.0.1:{server.server_port}/api/report?run=run_20250101_120000"
        with urlopen(url) as response:
            report = json.load(response)
        assert report["run"] == "run_20250101_120000"
        assert report["episodes"] == [{"uuid": "ep-1", "final_ne": None}]
        assert report["progress"] == {}
    finally:
        server.shutdown()
        thread.join(timeout=2)
        server.server_close()


def test_report_endpoint_returns_not_found_when_report_is_missing(tmp_path):
    output_dir = tmp_path / "runs"
    (output_dir / "run_20250101_120000").mkdir(parents=True)
    dashboard._Handler.cfg = SimpleNamespace(output_dir=str(output_dir))
    server = ThreadingHTTPServer(("127.0.0.1", 0), dashboard._Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        url = f"http://127.0.0.1:{server.server_port}/api/report?run=run_20250101_120000"
        try:
            urlopen(url)
            raise AssertionError("expected HTTP 404")
        except HTTPError as error:
            assert error.code == 404
    finally:
        server.shutdown()
        thread.join(timeout=2)
        server.server_close()
