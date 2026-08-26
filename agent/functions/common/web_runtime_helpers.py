"""Shared helpers for AirSim web runtimes."""

from __future__ import annotations

import contextlib
import io
import math
import socket
import time

from config import cfg
from sim.airsim_client import AirSimClient


def shape_text(value):
    if value is None:
        return "N/A"
    if hasattr(value, "shape"):
        return str(value.shape)
    if hasattr(value, "size") and not isinstance(value, (list, tuple)):
        return str(value.size)
    return str(value)


def target_depth_text(name, detection, image, depth_meters) -> str:
    if detection is None or not getattr(detection, "visible", False) or not getattr(detection, "bbox", None):
        return f"{name}:not_visible"
    if image is None or depth_meters is None:
        return (
            f"{name}:bbox={getattr(detection, 'bbox', None)} "
            f"score={float(getattr(detection, 'score', 0.0) or 0.0):.2f} depth=N/A"
        )
    try:
        import numpy as np

        h, w = depth_meters.shape
        sw = w / float(image.width)
        sh = h / float(image.height)
        bbox = list(getattr(detection, "bbox", []) or [])
        db = [
            max(0, min(w - 1, int(bbox[0] * sw))),
            max(0, min(h - 1, int(bbox[1] * sh))),
            max(0, min(w - 1, int(bbox[2] * sw))),
            max(0, min(h - 1, int(bbox[3] * sh))),
        ]
        if db[2] < db[0]:
            db[0], db[2] = db[2], db[0]
        if db[3] < db[1]:
            db[1], db[3] = db[3], db[1]
        region = depth_meters[db[1]:db[3] + 1, db[0]:db[2] + 1]
        valid = region[np.isfinite(region)]
        valid = valid[valid > 0]
        median_depth = float(np.median(valid)) if valid.size else float("nan")
        region_h, region_w = region.shape
        crop_h = max(1, region_h // 2)
        crop_w = max(1, region_w // 2)
        crop_y = max(0, (region_h - crop_h) // 2)
        crop_x = max(0, (region_w - crop_w) // 2)
        center_region = region[crop_y:crop_y + crop_h, crop_x:crop_x + crop_w]
        center_valid = center_region[np.isfinite(center_region)]
        center_valid = center_valid[center_valid > 0]
        robust_depth = float(np.median(center_valid)) if center_valid.size else median_depth
        valid_count = int(center_valid.size)
        center_count = max(1, int(center_region.size))
        depth_mad = (
            float(np.median(np.abs(center_valid - robust_depth)))
            if center_valid.size and np.isfinite(robust_depth)
            else float("nan")
        )
        depth_p10 = (
            float(np.percentile(center_valid, 10.0))
            if center_valid.size
            else float("nan")
        )
        depth_p90 = (
            float(np.percentile(center_valid, 90.0))
            if center_valid.size
            else float("nan")
        )
        detection.depth_median = robust_depth if np.isfinite(robust_depth) else None
        detection.depth_bbox_median = median_depth if np.isfinite(median_depth) else None
        detection.depth_p10_m = depth_p10 if np.isfinite(depth_p10) else None
        detection.depth_p90_m = depth_p90 if np.isfinite(depth_p90) else None
        detection.depth_bbox = db
        detection.depth_valid_ratio = float(valid_count) / float(center_count)
        detection.depth_mad_m = depth_mad if np.isfinite(depth_mad) else None
        detection.depth_sample_count = valid_count
        detection.surface_depth_samples = _surface_depth_samples(
            depth_meters,
            db,
            robust_depth,
            max_samples=36,
        )
        median_text = f"{median_depth:.1f}m" if np.isfinite(median_depth) else "N/A"
        robust_text = f"{robust_depth:.1f}m" if np.isfinite(robust_depth) else "N/A"
        return (
            f"{name}:bbox={bbox} score={float(getattr(detection, 'score', 0.0) or 0.0):.2f} "
            f"depth={robust_text} bbox_median={median_text} depth_bbox={db} "
            f"valid={detection.depth_valid_ratio:.2f} mad="
            f"{('N/A' if detection.depth_mad_m is None else f'{detection.depth_mad_m:.2f}m')}"
        )
    except Exception as exc:
        return (
            f"{name}:bbox={getattr(detection, 'bbox', None)} "
            f"score={float(getattr(detection, 'score', 0.0) or 0.0):.2f} depth=ERR({exc})"
        )


def _surface_depth_samples(depth_meters, depth_bbox, anchor_depth, *, max_samples: int = 36):
    """Return a bounded foreground-like sample set from one detected bbox.

    Bounding boxes often include sky, road, or objects behind the target.  A
    small regular grid is therefore filtered around the robust central depth.
    Coordinates are normalized to the depth image so the memory projection is
    independent of RGB/depth resolution differences.
    """
    import numpy as np

    if depth_meters is None or depth_bbox is None or len(depth_bbox) < 4:
        return []
    if anchor_depth is None or not np.isfinite(anchor_depth) or float(anchor_depth) <= 0.0:
        return []
    h, w = depth_meters.shape
    x1, y1, x2, y2 = [int(v) for v in depth_bbox[:4]]
    x1, x2 = sorted((max(0, min(w - 1, x1)), max(0, min(w - 1, x2))))
    y1, y2 = sorted((max(0, min(h - 1, y1)), max(0, min(h - 1, y2))))
    if x2 < x1 or y2 < y1:
        return []

    # Stay slightly inside the box to reduce background leakage at detector
    # edges, while retaining enough facade extent for large structures.
    margin_x = int((x2 - x1) * 0.06)
    margin_y = int((y2 - y1) * 0.06)
    sx1, sx2 = min(x2, x1 + margin_x), max(x1, x2 - margin_x)
    sy1, sy2 = min(y2, y1 + margin_y), max(y1, y2 - margin_y)
    side = max(2, int(math.ceil(math.sqrt(max(1, int(max_samples))))))
    xs = np.unique(np.rint(np.linspace(sx1, sx2, side)).astype(int))
    ys = np.unique(np.rint(np.linspace(sy1, sy2, side)).astype(int))

    candidates = []
    anchor = float(anchor_depth)
    # Keep the surface patch close to the central identity depth. A wider
    # tolerance can silently combine a foreground facade and a rear building
    # when one GroundingDINO box spans both.
    tolerance = max(2.0, 0.08 * anchor)
    for py in ys:
        for px in xs:
            depth = float(depth_meters[int(py), int(px)])
            if not np.isfinite(depth) or depth <= 0.0:
                continue
            candidates.append((abs(depth - anchor), int(px), int(py), depth))
    if not candidates:
        return []

    selected = [item for item in candidates if item[0] <= tolerance]
    # Thin or oblique targets can have a broad depth range.  Keep the samples
    # nearest to the robust central depth rather than returning no geometry.
    if len(selected) < min(4, len(candidates)):
        selected = sorted(candidates, key=lambda item: item[0])[: min(max_samples, len(candidates))]
    else:
        selected = selected[:max_samples]
    return [
        [
            (float(px) + 0.5) / max(float(w), 1.0),
            (float(py) + 0.5) / max(float(h), 1.0),
            float(depth),
        ]
        for _delta, px, py, depth in selected
    ]


def push_pil_png_to_frontend(state, frame, *, view: str = "front"):
    if state is None or frame is None:
        return
    buf = io.BytesIO()
    frame.save(buf, format="PNG")
    if view == "down":
        state.set_down_frame(buf.getvalue())
    else:
        state.set_frame(buf.getvalue())


def depth_only_profile(profile: str) -> str:
    normalized = (profile or "").strip().lower()
    mapping = {
        "front_depth": "front_depth_only",
        "front_down_front_depth": "front_depth_only",
        "front_down_both_depth": "front_down_depth_only",
    }
    return mapping.get(normalized, normalized)


def capture_profile_isolated(client: AirSimClient, profile: str):
    aux = AirSimClient(ip=getattr(client, "_ip", ""), port=getattr(client, "_port", 41451), use_config_ip=False)
    return aux.capture_views(profile=profile, mode="batch", verbose=False)


def capture_profile_isolated_with_pose(client: AirSimClient, profile: str):
    """Capture with a pose sampled immediately before simGetImages on the same RPC client."""
    aux = AirSimClient(ip=getattr(client, "_ip", ""), port=getattr(client, "_port", 41451), use_config_ip=False)
    observer_world, observer_yaw_deg = aux.get_pose()
    captured = aux.capture_views(profile=profile, mode="batch", verbose=False)
    return (*captured, observer_world, observer_yaw_deg)


@contextlib.contextmanager
def pause_background_capture(capturer):
    enabled = bool(cfg.get("SIM", {}).get("PAUSE_BACKGROUND_CAPTURE_DURING_STEP", True))
    if not enabled or capturer is None or not hasattr(capturer, "pause"):
        yield
        return
    capturer.pause(wait=True, timeout=2.0)
    try:
        yield
    finally:
        capturer.resume()


def should_takeoff(client: AirSimClient) -> tuple[bool, list, float, str]:
    pos, yaw = client.get_pose()
    landed_text = "unknown"
    should = pos[2] > -0.5
    try:
        state = client.get_multirotor_state()
        landed_state = getattr(state, "landed_state", None)
        landed_text = "unknown" if landed_state is None else str(landed_state)
        try:
            import airsim  # type: ignore

            if landed_state == airsim.LandedState.Landed:
                should = True
            elif landed_state == airsim.LandedState.Flying:
                should = False
        except Exception:
            normalized = landed_text.strip().lower()
            if normalized in {"0", "landed", "landedstate.landed"}:
                should = True
            elif normalized in {"1", "flying", "landedstate.flying"}:
                should = False
    except Exception:
        pass
    return should, pos, yaw, landed_text


def connect_web_airsim_client():
    import logging

    def error_text(exc: Exception) -> str:
        msg = str(exc).strip()
        return msg if msg else exc.__class__.__name__

    sim_cfg = cfg.get("SIM", {})
    wait_enabled = bool(sim_cfg.get("WAIT_FOR_AIRSIM_ON_WEB_START", True))
    retry_interval = float(sim_cfg.get("WAIT_FOR_AIRSIM_INTERVAL", 2.0))
    rpc_timeout = float(sim_cfg.get("AIRSIM_CONNECT_TIMEOUT", 3.0))
    host = str(sim_cfg.get("AIRSIM_IP", "") or "127.0.0.1").strip() or "127.0.0.1"
    port = int(sim_cfg.get("AIRSIM_PORT", 41451))

    logging.getLogger("tornado.general").setLevel(logging.CRITICAL)
    while True:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.settimeout(1.0)
            if sock.connect_ex((host, port)) != 0:
                raise ConnectionError(f"tcp {host}:{port} not ready")
            client_ip = "" if host in {"127.0.0.1", "localhost"} else host
            probe_client = AirSimClient(
                ip=client_ip,
                port=port,
                use_config_ip=False,
                timeout_value=rpc_timeout,
            )
            try:
                probe_client.connect(False)
            except Exception as exc:
                raise TimeoutError(
                    f"AirSim RPC handshake timeout/error after {rpc_timeout:.1f}s: {error_text(exc)}"
                ) from exc
            return AirSimClient(ip=client_ip, port=port, use_config_ip=False)
        except Exception as exc:
            if not wait_enabled:
                raise
            print(f"[AirSim] waiting for {host}:{port} ... ({error_text(exc)})", flush=True)
            time.sleep(max(0.5, retry_interval))
        finally:
            try:
                sock.close()
            except Exception:
                pass
