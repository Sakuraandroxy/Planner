"""Focused transport tests for the GroundingDINO HTTP client."""

from __future__ import annotations

from PIL import Image

from agent.models.detection import groundingdino_detector as detector_module
from agent.models.detection.groundingdino_detector import GroundingDINODetector


def test_groundingdino_uses_short_connect_and_long_read_timeouts(monkeypatch):
    captured = {}

    class Response:
        @staticmethod
        def json():
            return {"success": True, "detections": []}

    def fake_post(url, *, json, timeout):
        captured.update(url=url, payload=json, timeout=timeout)
        return Response()

    monkeypatch.setattr(detector_module.requests, "post", fake_post)
    detector = GroundingDINODetector.__new__(GroundingDINODetector)
    detector.url = "http://detector.test/detect"
    detector.connect_timeout = 5.0
    detector.timeout = 120.0
    detector.box_threshold = 0.4
    detector.text_threshold = 0.3

    detections = detector.detect_all(Image.new("RGB", (16, 12)), "building")

    assert detections == []
    assert captured["timeout"] == (5.0, 120.0)
    assert captured["url"] == detector.url
