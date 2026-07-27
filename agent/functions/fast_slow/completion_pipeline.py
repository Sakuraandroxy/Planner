"""Asynchronous detector/depth observation pipeline for the fast-slow runtime.

In the current distance-only completion mode this pipeline refreshes the
cached target world pose. It never advances stages itself.
"""

from __future__ import annotations

import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Any, Optional

from agent.functions.common import web_runtime_helpers as web_helpers
from agent.models.detection.base import DetectionResult


@dataclass
class DetectionDepthBundle:
    best_detection: Optional[DetectionResult]
    front_detection: Optional[DetectionResult]
    down_detection: Optional[DetectionResult]
    front_image: Any = None
    down_image: Any = None
    front_depth: Any = None
    down_depth: Any = None
    detect_elapsed: float = 0.0
    depth_elapsed: float = 0.0
    distance_m: Optional[float] = None
    distance_view: str = "none"
    distance_reason: str = ""
    front_reliability: float = 0.0
    down_reliability: float = 0.0
    observer_world: Optional[list[float]] = None
    observer_yaw_deg: Optional[float] = None


@dataclass
class CompletionPipelineEvent:
    kind: str
    stage_key: tuple
    bundle: Optional[DetectionDepthBundle] = None
    completion: Any = None
    elapsed: float = 0.0


@dataclass
class CompletionPipelineJob:
    stage_key: tuple
    stage: Any
    task_text: str
    frame: Any
    down_frame: Any
    submitted_at: float = field(default_factory=time.perf_counter)
    detect_future: Optional[Future] = None
    depth_future: Optional[Future] = None
    vlm_future: Optional[Future] = None
    bundle: Optional[DetectionDepthBundle] = None
    phase: str = "detect_depth"
    near_stop_emitted: bool = False


class CompletionPipeline:
    """Chain detection/depth/VLM in the background without blocking execution."""

    def __init__(
        self,
        *,
        detector: Any,
        checker: Any,
        client: Any,
        capture_mode: str,
        detect_executor: ThreadPoolExecutor,
        slow_executor: ThreadPoolExecutor,
        stop_radius_m: float,
        slow_radius_m: float,
        distance_view_policy: str = "relation_aware",
        debug_logs: bool = False,
    ):
        self.detector = detector
        self.checker = checker
        self.client = client
        self.capture_mode = capture_mode
        self.detect_executor = detect_executor
        self.slow_executor = slow_executor
        self.stop_radius_m = float(stop_radius_m)
        self.slow_radius_m = float(slow_radius_m)
        self.distance_view_policy = str(distance_view_policy or "relation_aware").strip().lower()
        self.debug_logs = bool(debug_logs)
        self._job: Optional[CompletionPipelineJob] = None

    @property
    def active(self) -> bool:
        return self._job is not None

    def clear(self) -> None:
        self._job = None

    def submit(self, stage: Any, task_text: str, frame: Any, down_frame: Any) -> bool:
        if self._job is not None or not self.checker.should_check_stage(stage):
            return False
        stage_key = self.stage_key(stage)
        job = CompletionPipelineJob(
            stage_key=stage_key,
            stage=stage,
            task_text=task_text,
            frame=frame,
            down_frame=down_frame,
        )
        if getattr(self.checker, "uses_detector", False) and self.checker.is_detector_enabled():
            job.detect_future = self.slow_executor.submit(self._capture_detect_depth_bundle, stage, task_text)
            job.phase = "detect_depth"
        else:
            # Distance completion requires detector/depth observations. There
            # is deliberately no VLM-only fallback.
            return False
        self._job = job
        return True

    def poll(self) -> Optional[CompletionPipelineEvent]:
        job = self._job
        if job is None:
            return None

        if job.phase == "detect_depth":
            if not (job.detect_future and job.detect_future.done()):
                return None
            bundle = job.detect_future.result()
            job.bundle = bundle
            job.frame = bundle.front_image
            job.down_frame = bundle.down_image
            if bundle.best_detection is None or not getattr(bundle.best_detection, "visible", False):
                self._job = None
                return CompletionPipelineEvent(
                    "target_lost",
                    job.stage_key,
                    bundle=bundle,
                    elapsed=time.perf_counter() - job.submitted_at,
                )
            bundle.distance_m = self._distance_for_stage(job.stage, bundle, job.frame, job.down_frame)
            depth_text = "none" if bundle.distance_m is None else f"{bundle.distance_m:.1f}m"
            if self.debug_logs:
                print(
                    "  [CompletionDistance] "
                    f"policy={self.distance_view_policy} selected={bundle.distance_view} "
                    f"depth={depth_text} reason={bundle.distance_reason} "
                    f"front_rel={bundle.front_reliability:.3f} down_rel={bundle.down_reliability:.3f}"
                )
            if bundle.distance_m is not None and bundle.distance_m <= self.stop_radius_m:
                job.near_stop_emitted = True
                elapsed = time.perf_counter() - job.submitted_at
                self._job = None
                return CompletionPipelineEvent("near_stop", job.stage_key, bundle=bundle, elapsed=elapsed)

            # Far from the target, geometry is sufficient. Release this job so
            # the next detector snapshot can refresh the cached world pose;
            # the expensive VLM judge is reserved for fresh near-target confirmation.
            self._job = None
            return CompletionPipelineEvent("observation", job.stage_key, bundle=bundle,
                                           elapsed=time.perf_counter() - job.submitted_at)

        if job.phase == "vlm":
            if not job.vlm_future or not job.vlm_future.done():
                return None
            completion = job.vlm_future.result()
            elapsed = time.perf_counter() - job.submitted_at
            self._job = None
            if getattr(completion, "done", False):
                return CompletionPipelineEvent(
                    "done_candidate",
                    job.stage_key,
                    bundle=job.bundle,
                    completion=completion,
                    elapsed=elapsed,
                )
            return CompletionPipelineEvent(
                "not_done",
                job.stage_key,
                bundle=job.bundle,
                completion=completion,
                elapsed=elapsed,
            )

        return None

    @staticmethod
    def stage_key(stage: Any) -> tuple:
        return (
            getattr(stage, "index", None),
            getattr(stage, "instruction", None),
            getattr(stage, "mode", None),
        )

    def _detect_dual_view(self, stage: Any, task_text: str, frame: Any, down_frame: Any):
        caption = (
            getattr(stage, "target_query", None)
            or getattr(stage, "target", None)
            or getattr(stage, "instruction", None)
            or task_text
            or ""
        )

        def detect_one(image, camera_name: str):
            if image is None:
                return DetectionResult(visible=False, camera=camera_name)
            return self.detector.detect(image, caption, depth_meters=None, camera_name=camera_name)

        t0 = time.perf_counter()
        front_future = self.detect_executor.submit(detect_one, frame, "front")
        down_future = self.detect_executor.submit(detect_one, down_frame, "down")
        front_det = front_future.result()
        down_det = down_future.result()
        self._suppress_giant_bbox(front_det, frame)
        self._suppress_giant_bbox(down_det, down_frame)
        return front_det, down_det, time.perf_counter() - t0

    def _capture_detect_depth_bundle(self, stage: Any, task_text: str) -> DetectionDepthBundle:
        profile = "front_down"
        t0 = time.perf_counter()
        (
            frame,
            down_frame,
            _front_depth,
            _down_depth,
            timing,
            observer_world,
            observer_yaw_deg,
        ) = web_helpers.capture_profile_isolated_with_pose(self.client, profile)
        capture_elapsed = time.perf_counter() - t0
        timing = dict(timing or {})
        timing.setdefault("total_s", capture_elapsed)
        if self.debug_logs:
            print(
                f"  [CompletionSnapshot] profile={profile} time={timing.get('total_s', capture_elapsed):.2f}s  "
                f"front={web_helpers.shape_text(frame)} down={web_helpers.shape_text(down_frame)}"
            )

        front_det, down_det, detect_elapsed = self._detect_dual_view(stage, task_text, frame, down_frame)
        bundle = self._make_bundle(job=None, front_det=front_det, down_det=down_det,
                                   front_depth=None, down_depth=None,
                                   detect_elapsed=detect_elapsed, depth_elapsed=0.0,
                                   frame=frame, down_frame=down_frame, stage=stage)
        bundle.observer_world = list(observer_world)
        bundle.observer_yaw_deg = float(observer_yaw_deg)
        if bundle.best_detection is None or not getattr(bundle.best_detection, "visible", False):
            bundle.distance_view = "none"
            bundle.distance_reason = "target_not_detected_in_rgb"
            return bundle

        depth_profile = web_helpers.depth_only_profile("front_down_both_depth")
        depth_t0 = time.perf_counter()
        _front_rgb, _down_rgb, front_depth, down_depth, depth_timing = web_helpers.capture_profile_isolated(
            self.client,
            depth_profile,
        )
        depth_elapsed = time.perf_counter() - depth_t0
        depth_timing = dict(depth_timing or {})
        depth_timing.setdefault("total_s", depth_elapsed)
        if self.debug_logs:
            print(
                f"  [CompletionDepth] profile={depth_profile} "
                f"time={depth_timing.get('total_s', depth_elapsed):.2f}s  "
                f"front_depth={web_helpers.shape_text(front_depth)} "
                f"down_depth={web_helpers.shape_text(down_depth)}"
            )

        # Depth attachment is functional state, not logging. target_depth_text
        # populates depth_median/depth_bbox while also returning debug text.
        front_depth_text = web_helpers.target_depth_text("front", front_det, frame, front_depth)
        down_depth_text = web_helpers.target_depth_text("down", down_det, down_frame, down_depth)
        if self.debug_logs:
            print(f"  [TargetDepth] {front_depth_text}  {down_depth_text}")
        bundle = self._make_bundle(job=None, front_det=front_det, down_det=down_det,
                                   front_depth=front_depth, down_depth=down_depth,
                                   detect_elapsed=detect_elapsed, depth_elapsed=depth_elapsed,
                                   frame=frame, down_frame=down_frame, stage=stage)
        bundle.observer_world = list(observer_world)
        bundle.observer_yaw_deg = float(observer_yaw_deg)
        return bundle

    def _make_bundle(self, job, front_det, down_det, front_depth, down_depth,
                     detect_elapsed, depth_elapsed, frame=None, down_frame=None, stage=None):
        frame = frame if frame is not None else getattr(job, "frame", None)
        down_frame = down_frame if down_frame is not None else getattr(job, "down_frame", None)
        stage = stage if stage is not None else getattr(job, "stage", None)
        visible = [d for d in (front_det, down_det) if d and d.visible]
        best = self._select_reliable_detection(
            stage,
            front_det,
            down_det,
            frame,
            down_frame,
            require_depth=False,
        ) if visible else None
        return DetectionDepthBundle(
            best_detection=best,
            front_detection=front_det,
            down_detection=down_det,
            front_image=frame,
            down_image=down_frame,
            front_depth=front_depth,
            down_depth=down_depth,
            detect_elapsed=detect_elapsed,
            depth_elapsed=depth_elapsed,
        )

    def _evaluate_with_bundle(self, job: CompletionPipelineJob, bundle: DetectionDepthBundle):
        completion = self.checker.evaluate_with_detection(
            job.stage,
            job.task_text,
            bundle.front_image,
            bundle.down_image,
            bundle.best_detection,
            front_detection=bundle.front_detection,
            down_detection=bundle.down_detection,
            front_depth_meters=bundle.front_depth,
            down_depth_meters=bundle.down_depth,
        )
        return completion

    def _evaluate_freshless(self, stage: Any, task_text: str):
        profile = "front_down_both_depth"
        frame, down_frame, front_depth, down_depth, _timing = web_helpers.capture_profile_isolated(self.client, profile)
        return self.checker.evaluate(
            stage,
            task_text,
            frame,
            down_frame,
            front_depth_meters=front_depth,
            down_depth_meters=down_depth,
        )

    def _distance_for_stage(
        self,
        stage: Any,
        bundle: DetectionDepthBundle,
        front_image: Any = None,
        down_image: Any = None,
    ) -> Optional[float]:
        detection = self._select_distance_detection(
            stage,
            bundle.front_detection,
            bundle.down_detection,
            front_image,
            down_image,
            bundle,
        )
        if detection is None or detection.depth_median is None:
            return None
        return float(detection.depth_median)

    def _select_distance_detection(
        self,
        stage: Any,
        front_detection: Optional[DetectionResult],
        down_detection: Optional[DetectionResult],
        front_image: Any = None,
        down_image: Any = None,
        bundle: Optional[DetectionDepthBundle] = None,
    ) -> Optional[DetectionResult]:
        """Pick the view whose depth drives stop/confirm timing.

        This is intentionally configurable and logged, because completion bugs
        are otherwise very hard to diagnose from terminal output.
        """
        front_rel = self._detection_reliability(stage, front_detection, front_image) if front_detection else 0.0
        down_rel = self._detection_reliability(stage, down_detection, down_image) if down_detection else 0.0
        if bundle is not None:
            bundle.front_reliability = front_rel
            bundle.down_reliability = down_rel

        options = {
            "front": (front_detection, front_image, front_rel),
            "down": (down_detection, down_image, down_rel),
        }
        policy = self.distance_view_policy
        if policy == "front_only":
            ordered_names = ["front"]
            reason = "configured_front_only"
        elif policy == "down_only":
            ordered_names = ["down"]
            reason = "configured_down_only"
        elif policy == "best_reliable":
            ordered_names = ["front", "down"] if front_rel >= down_rel else ["down", "front"]
            reason = "configured_best_reliable"
        else:
            above = self._is_above_stage(stage)
            ordered_names = ["down", "front"] if above else ["front", "down"]
            reason = "relation_above_prefers_down" if above else "relation_non_above_prefers_front"

        for name in ordered_names:
            detection, _image, reliability = options[name]
            if not detection or not detection.visible:
                continue
            if detection.depth_median is None:
                continue
            if reliability > 0.0:
                if bundle is not None:
                    bundle.distance_view = name
                    bundle.distance_reason = reason
                return detection
        if bundle is not None:
            bundle.distance_view = "none"
            bundle.distance_reason = f"{reason}_no_reliable_depth"
        return None

    def _select_reliable_detection(
        self,
        stage: Any,
        front_detection: Optional[DetectionResult],
        down_detection: Optional[DetectionResult],
        front_image: Any = None,
        down_image: Any = None,
        require_depth: bool = True,
    ) -> Optional[DetectionResult]:
        candidates = []
        for detection, image in ((front_detection, front_image), (down_detection, down_image)):
            if not detection or not detection.visible:
                continue
            if require_depth and detection.depth_median is None:
                continue
            reliability = self._detection_reliability(stage, detection, image)
            if reliability > 0.0:
                candidates.append((reliability, detection))
        if not candidates:
            return None
        return max(candidates, key=lambda item: item[0])[1]

    def _detection_reliability(self, stage: Any, detection: DetectionResult, image: Any = None) -> float:
        score = max(0.0, min(1.0, float(getattr(detection, "score", 0.0) or 0.0)))
        if image is None or not getattr(detection, "bbox", None) or not hasattr(image, "size"):
            return score

        width, height = float(image.size[0]), float(image.size[1])
        if width <= 1.0 or height <= 1.0:
            return score

        x1, y1, x2, y2 = [float(v) for v in detection.bbox[:4]]
        box_w = max(0.0, min(width, x2) - max(0.0, x1))
        box_h = max(0.0, min(height, y2) - max(0.0, y1))
        area_ratio = (box_w * box_h) / max(width * height, 1.0)
        span_x = box_w / width
        span_y = box_h / height
        touches_border = x1 <= 2.0 or y1 <= 2.0 or x2 >= width - 2.0 or y2 >= height - 2.0

        quality = 1.0
        if area_ratio <= 0.0002:
            quality *= 0.35
        elif area_ratio <= 0.001:
            quality *= 0.65
        if span_x >= 0.90 or span_y >= 0.90:
            return 0.0
        if area_ratio >= 0.55:
            quality *= 0.03
        elif area_ratio >= 0.30 or span_x >= 0.75 or span_y >= 0.75:
            quality *= 0.20
        elif area_ratio >= 0.18:
            quality *= 0.45
        if touches_border:
            quality *= 0.45 if self._is_above_stage(stage) else 0.35
        return score * quality

    @staticmethod
    def _suppress_giant_bbox(
        detection: Optional[DetectionResult],
        image,
        max_span: float = 0.90,
    ) -> None:
        """Treat full-image detector boxes as no target evidence."""
        if detection is None or not detection.bbox or image is None or not hasattr(image, "size"):
            return
        width, height = float(image.size[0]), float(image.size[1])
        if width <= 1.0 or height <= 1.0:
            return
        x1, y1, x2, y2 = [float(v) for v in detection.bbox[:4]]
        box_w = max(0.0, min(width, x2) - max(0.0, x1))
        box_h = max(0.0, min(height, y2) - max(0.0, y1))
        if box_w / width >= max_span or box_h / height >= max_span:
            detection.score = 0.0
            detection.visible = False

    @staticmethod
    def _is_above_stage(stage: Any) -> bool:
        relation = str(getattr(stage, "relation", "") or "").strip().lower()
        instruction = str(getattr(stage, "instruction", "") or "").strip().lower()
        return (
            relation in {"above", "over", "on top", "on top of"}
            or "above" in instruction
            or "on top" in instruction
        )
