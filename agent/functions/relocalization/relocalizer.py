"""Bounded, validator-gated target relocalization for compact targets."""

from __future__ import annotations

import contextlib
import io
import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional, Sequence

from agent.functions.common import web_runtime_helpers as web_helpers
from agent.functions.common.detection_policy import (
    detection_caption_for_stage,
    detection_reliability,
    is_large_structure_stage,
)
from agent.models.detection.base import DetectionResult
from agent.functions.perception.camera_geometry import (
    attach_detection_camera_context,
    project_world_to_pixel,
)


@dataclass
class RelocalizationResult:
    found: bool = False
    detection: Optional[DetectionResult] = None
    front_image: Any = None
    down_image: Any = None
    direction_index: int = 0
    yaw_delta_deg: float = 0.0
    total_rotation_deg: float = 0.0
    searched_yaws_deg: list[float] = field(default_factory=list)
    front_depth: Any = None
    down_depth: Any = None
    front_detections: list[DetectionResult] = field(default_factory=list)
    down_detections: list[DetectionResult] = field(default_factory=list)
    camera_images: dict[str, Any] = field(default_factory=dict)
    camera_depths: dict[str, Any] = field(default_factory=dict)
    observer_world: Optional[list[float]] = None
    observer_yaw_deg: Optional[float] = None
    capture_timestamp_s: float = 0.0
    session_id: str = ""
    elapsed: float = 0.0
    reason: str = ""

class TargetRelocalizer:
    """Search a compact target inside an explicitly bounded yaw set."""

    def __init__(self, cfg: dict, detector=None):
        rcfg = cfg.get("RELOCALIZATION", {})
        self.enabled = bool(rcfg.get("ENABLED", True))
        self.turn_deg = float(rcfg.get("TURN_DEG", 90.0))
        self.max_turns = int(rcfg.get("MAX_TURNS", 4))
        self.capture_profile = str(rcfg.get("CAPTURE_PROFILE", "front_down_both_depth"))
        self.accept_down_view = bool(rcfg.get("ACCEPT_DOWN_VIEW", False))
        self.center_on_target = bool(rcfg.get("CENTER_ON_TARGET", True))
        self.camera_hfov_deg = float(rcfg.get("CAMERA_HFOV_DEG", 90.0))
        self.min_center_offset_deg = float(rcfg.get("MIN_CENTER_OFFSET_DEG", 3.0))
        self.min_bbox_area_ratio = float(rcfg.get("MIN_BBOX_AREA_RATIO", 0.002))
        self.max_bbox_span = float(rcfg.get("MAX_BBOX_SPAN", 0.90))
        self.detector = detector
        ag = cfg.get("AGENT", {})
        perception = cfg.get("FUNCTIONS", {}).get("PERCEPTION", {}) or {}
        self.min_confidence = float(rcfg.get(
            "MIN_CONFIDENCE",
            perception.get(
                "MIN_CONFIDENCE",
                ag.get("DETECTOR_MIN_CONFIDENCE", ag.get("DETECTOR_BOX_THRESHOLD", 0.0)),
            ),
        ))

    def search(
        self,
        client,
        stage: Any,
        front_image=None,
        down_image=None,
        capture_mode: str = "parallel",
        skip_initial_frame: bool = False,
        validator: Optional[
            Callable[[Any, Any, Any, DetectionResult, DetectionResult, DetectionResult], Optional[DetectionResult]]
        ] = None,
        yaw_offsets_deg: Optional[Sequence[float]] = None,
        base_yaw_deg: Optional[float] = None,
        max_total_rotation_deg: Optional[float] = None,
        session_id: str = "",
        expected_target_world: Optional[Sequence[float]] = None,
    ) -> RelocalizationResult:
        started = time.perf_counter()
        if not self.enabled:
            return RelocalizationResult(reason="relocalization disabled", session_id=session_id)
        if self.detector is None:
            return RelocalizationResult(reason="detector unavailable", session_id=session_id)
        if is_large_structure_stage(stage):
            return RelocalizationResult(
                reason="large structure yaw search forbidden; use locked geometry and down-view roof acquisition",
                session_id=session_id,
            )

        target = detection_caption_for_stage(stage)
        if not target:
            return RelocalizationResult(reason="empty target", session_id=session_id)

        _start_pos, start_yaw = client.get_pose()
        start_yaw = float(start_yaw)
        searched_yaws: list[float] = []
        total_rotation = 0.0
        last_commanded_yaw = start_yaw
        offsets = list(yaw_offsets_deg) if yaw_offsets_deg is not None else None
        view_count = len(offsets) if offsets is not None else max(1, self.max_turns)

        for idx in range(view_count):
            if idx == 0 and front_image is not None and not skip_initial_frame:
                frame, down = front_image, down_image
                front_depth = down_depth = None
            else:
                if offsets is not None:
                    desired_yaw = (
                        float(base_yaw_deg if base_yaw_deg is not None else start_yaw)
                        + float(offsets[idx])
                    ) % 360.0
                    turn_amount = abs(self._signed_yaw_delta_deg(desired_yaw, last_commanded_yaw))
                    restore_after_view = abs(self._signed_yaw_delta_deg(start_yaw, desired_yaw))
                    if (
                        max_total_rotation_deg is not None
                        and total_rotation + turn_amount + restore_after_view
                        > float(max_total_rotation_deg)
                    ):
                        break
                    self._rotate_to_yaw(client, desired_yaw, last_commanded_yaw)
                    total_rotation += turn_amount
                    last_commanded_yaw = desired_yaw
                elif skip_initial_frame or idx > 0:
                    client.rotate_yaw(self.turn_deg)
                    total_rotation += abs(self.turn_deg)
                    last_commanded_yaw = (last_commanded_yaw + self.turn_deg) % 360.0
                frame, down, front_depth, down_depth, _timing = client.capture_views(
                    profile=self.capture_profile,
                    mode=capture_mode,
                    verbose=False,
                )

            observer_world, observer_yaw = client.get_pose()
            capture_timestamp = time.perf_counter()
            searched_yaws.append(float(observer_yaw))
            predicted_views = self._predicted_visible_views(
                (frame, down),
                expected_target_world,
            )
            if predicted_views and not any(predicted_views):
                continue
            (
                front_det,
                down_det,
                detection,
                front_detections,
                down_detections,
            ) = self._detect_views(
                stage,
                frame,
                down,
                target,
                front_depth=front_depth,
                down_depth=down_depth,
                allowed_views=predicted_views,
            )

            accepted = None
            if validator is not None:
                for candidate in self._candidate_order(
                    stage,
                    front_detections,
                    down_detections,
                    frame,
                    down,
                ):
                    try:
                        accepted = validator(stage, frame, down, front_det, down_det, candidate)
                    except Exception:
                        accepted = None
                    if accepted is not None and accepted.visible:
                        break
            elif detection.visible:
                accepted = detection

            if accepted is not None and accepted.visible:
                center_offset = 0.0
                if self.center_on_target:
                    accepted_image = self._image_for_detection(accepted, frame, down)
                    center_offset = self._camera_center_offset_deg(accepted, accepted_image)
                    if abs(center_offset) >= self.min_center_offset_deg:
                        client.rotate_yaw(center_offset)
                camera_images = self._camera_value_map(frame, down)
                camera_depths = self._camera_value_map(front_depth, down_depth)
                return RelocalizationResult(
                    found=True,
                    detection=accepted,
                    front_image=frame,
                    down_image=down,
                    front_depth=front_depth,
                    down_depth=down_depth,
                    front_detections=front_detections,
                    down_detections=down_detections,
                    camera_images=camera_images,
                    camera_depths=camera_depths,
                    observer_world=[float(v) for v in observer_world[:3]],
                    observer_yaw_deg=float(observer_yaw),
                    capture_timestamp_s=capture_timestamp,
                    session_id=str(session_id or ""),
                    direction_index=idx,
                    yaw_delta_deg=self._signed_yaw_delta_deg(float(observer_yaw) + center_offset, start_yaw),
                    total_rotation_deg=total_rotation + abs(center_offset),
                    searched_yaws_deg=searched_yaws,
                    elapsed=time.perf_counter() - started,
                    reason=(
                        "validated target detected in "
                        f"{getattr(accepted, 'camera_id', '') or accepted.camera} view"
                    ),
                )

        restore_turn = abs(self._signed_yaw_delta_deg(start_yaw, last_commanded_yaw))
        self._rotate_to_yaw(client, start_yaw, last_commanded_yaw)
        return RelocalizationResult(
            found=False,
            total_rotation_deg=total_rotation + restore_turn,
            searched_yaws_deg=searched_yaws,
            session_id=str(session_id or ""),
            elapsed=time.perf_counter() - started,
            reason=f"'{target}' identity not validated after {len(searched_yaws)} bounded views",
        )

    def _detect_views(
        self,
        stage: Any,
        front_image,
        down_image,
        target: str,
        *,
        front_depth=None,
        down_depth=None,
        allowed_views: Optional[Sequence[bool]] = None,
    ):
        def detect_all(image, camera_name: str, depth, allowed: bool = True):
            if image is None:
                return []
            if not allowed:
                return []
            camera_frame = getattr(image, "camera_frame", None)
            if hasattr(self.detector, "detect_all"):
                results = list(self.detector.detect_all(
                    image,
                    target,
                    depth_meters=depth,
                    camera_name=camera_name,
                ) or [])
            else:
                detected = self.detector.detect(
                    image,
                    target,
                    depth_meters=depth,
                    camera_name=camera_name,
                )
                results = [detected] if detected is not None and detected.visible else []
            for index, result in enumerate(results):
                attach_detection_camera_context(result, image, camera_frame)
                if depth is not None:
                    web_helpers.target_depth_text(
                        f"relocalize_{camera_name}#{index}",
                        result,
                        image,
                        depth,
                    )
            return results

        with contextlib.redirect_stdout(io.StringIO()):
            front_allowed = bool(allowed_views[0]) if allowed_views and len(allowed_views) > 0 else True
            down_allowed = bool(allowed_views[1]) if allowed_views and len(allowed_views) > 1 else True
            front_all = detect_all(front_image, "front", front_depth, front_allowed)
            down_all = detect_all(down_image, "down", down_depth, down_allowed)
        front_det = self._best_detection(stage, front_all, front_image, "front")
        down_det = self._best_detection(stage, down_all, down_image, "down")
        candidates = (
            (front_det, down_det)
            if self._secondary_camera_allowed(front_image, down_image)
            else (front_det,)
        )
        visible = [detection for detection in candidates if detection.visible]
        best = (
            max(visible, key=lambda detection: float(detection.score or 0.0))
            if visible
            else DetectionResult(visible=False, camera="none")
        )
        return front_det, down_det, best, front_all, down_all

    def _best_detection(self, stage: Any, detections, image, camera_name: str) -> DetectionResult:
        reliable = [
            detection
            for detection in list(detections or [])
            if detection is not None
            and detection.visible
            and float(detection.score or 0.0) >= self.min_confidence
            and self._bbox_is_reliable(stage, detection, image)
        ]
        if reliable:
            return max(reliable, key=lambda detection: float(detection.score or 0.0))
        return DetectionResult(visible=False, camera=camera_name)

    def _candidate_order(
        self,
        stage,
        front_detections,
        down_detections,
        front_image,
        down_image,
    ) -> list[DetectionResult]:
        candidates = [
            (candidate, front_image)
            for candidate in list(front_detections or [])
        ]
        if self._secondary_camera_allowed(front_image, down_image):
            candidates.extend((candidate, down_image) for candidate in list(down_detections or []))
        candidates = [
            candidate
            for candidate, image in candidates
            if candidate is not None
            and candidate.visible
            and float(candidate.score or 0.0) >= self.min_confidence
            and self._bbox_is_reliable(stage, candidate, image)
        ]
        return sorted(candidates, key=lambda candidate: float(candidate.score or 0.0), reverse=True)

    def _bbox_is_reliable(self, stage: Any, detection: DetectionResult, image) -> bool:
        if not detection.bbox or image is None or not hasattr(image, "size"):
            return False
        width, height = float(image.size[0]), float(image.size[1])
        if width <= 1.0 or height <= 1.0:
            return False
        x1, y1, x2, y2 = [float(v) for v in detection.bbox[:4]]
        box_w = max(0.0, min(width, x2) - max(0.0, x1))
        box_h = max(0.0, min(height, y2) - max(0.0, y1))
        area_ratio = box_w * box_h / max(width * height, 1.0)
        return bool(
            area_ratio >= self.min_bbox_area_ratio
            and detection_reliability(stage, detection, image, max_span=self.max_bbox_span) > 0.0
        )

    def _front_center_offset_deg(self, detection: DetectionResult, image) -> float:
        if not detection.bbox or image is None or not hasattr(image, "width") or image.width <= 1:
            return 0.0
        center_x = (float(detection.bbox[0]) + float(detection.bbox[2])) / 2.0
        normalized_x = center_x / float(image.width) - 0.5
        return normalized_x * self.camera_hfov_deg

    def _camera_center_offset_deg(self, detection: DetectionResult, image) -> float:
        """Return vehicle yaw correction from exact world rays when possible."""
        world_ray = getattr(detection, "world_ray", None)
        camera_frame = getattr(detection, "camera_frame", None) or getattr(image, "camera_frame", None)
        if world_ray is not None and camera_frame is not None:
            ray = world_ray.direction_world
            optical = camera_frame.optical_axis_world
            ray_horizontal = math.hypot(float(ray[0]), float(ray[1]))
            optical_horizontal = math.hypot(float(optical[0]), float(optical[1]))
            if ray_horizontal > 1e-6 and optical_horizontal > 0.15:
                ray_bearing = math.degrees(math.atan2(float(ray[1]), float(ray[0])))
                optical_bearing = math.degrees(math.atan2(float(optical[1]), float(optical[0])))
                return self._signed_yaw_delta_deg(ray_bearing, optical_bearing)
            return 0.0
        if str(getattr(detection, "camera", "") or "").lower() == "front":
            return self._front_center_offset_deg(detection, image)
        return 0.0

    def _predicted_visible_views(
        self,
        images: Sequence[Any],
        expected_target_world: Optional[Sequence[float]],
    ) -> list[bool]:
        """Gate detector calls using the locked target projected into each image.

        An empty return means exact camera geometry is unavailable and the
        legacy detector behavior should be retained.
        """
        target = list(expected_target_world or [])
        if len(target) < 3:
            return []
        decisions = []
        exact_count = 0
        margin_ratio = 0.20
        for image in images:
            camera_frame = getattr(image, "camera_frame", None) if image is not None else None
            if camera_frame is None or camera_frame.rgb_intrinsics is None:
                decisions.append(True)
                continue
            exact_count += 1
            projected = project_world_to_pixel(camera_frame, target, require_in_frame=False)
            if projected is None:
                decisions.append(False)
                continue
            u, v, _forward = projected
            width = float(camera_frame.rgb_intrinsics.width)
            height = float(camera_frame.rgb_intrinsics.height)
            decisions.append(bool(
                -margin_ratio * width <= u < (1.0 + margin_ratio) * width
                and -margin_ratio * height <= v < (1.0 + margin_ratio) * height
            ))
        return decisions if exact_count > 0 else []

    def _secondary_camera_allowed(self, primary_image, secondary_image) -> bool:
        if secondary_image is None:
            return False
        # Any Camera API frame has an exact pose and is therefore a valid
        # relocalization sensor. ACCEPT_DOWN_VIEW remains a legacy-only switch.
        if getattr(secondary_image, "camera_frame", None) is not None:
            return True
        return self.accept_down_view

    @staticmethod
    def _image_for_detection(detection, primary_image, secondary_image):
        camera_id = str(getattr(detection, "camera_id", "") or "")
        for image in (primary_image, secondary_image):
            frame = getattr(image, "camera_frame", None) if image is not None else None
            if frame is not None and camera_id and str(frame.camera_id) == camera_id:
                return image
        return secondary_image if str(getattr(detection, "camera", "")).lower() == "down" else primary_image

    @staticmethod
    def _camera_value_map(primary, secondary) -> dict[str, Any]:
        values = {}
        for index, value in enumerate((primary, secondary)):
            if value is None:
                continue
            camera_frame = getattr(value, "camera_frame", None)
            camera_id = str(getattr(camera_frame, "camera_id", "") or f"view_{index}")
            values[camera_id] = value
        return values

    @staticmethod
    def _signed_yaw_delta_deg(target_yaw_deg: float, current_yaw_deg: float) -> float:
        return (float(target_yaw_deg) - float(current_yaw_deg) + 180.0) % 360.0 - 180.0

    def _rotate_to_yaw(self, client, desired_yaw: float, current_yaw: float) -> None:
        rotate_to = getattr(client, "rotate_to_yaw", None)
        if callable(rotate_to):
            rotate_to(float(desired_yaw))
            return
        client.rotate_yaw(self._signed_yaw_delta_deg(desired_yaw, current_yaw))
