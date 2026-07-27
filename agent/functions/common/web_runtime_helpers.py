"""Shared helpers for AirSim web runtimes."""

from __future__ import annotations

import contextlib
import io
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
        detection.depth_median = robust_depth if np.isfinite(robust_depth) else None
        detection.depth_bbox = db
        median_text = f"{median_depth:.1f}m" if np.isfinite(median_depth) else "N/A"
        robust_text = f"{robust_depth:.1f}m" if np.isfinite(robust_depth) else "N/A"
        return (
            f"{name}:bbox={bbox} score={float(getattr(detection, 'score', 0.0) or 0.0):.2f} "
            f"depth={robust_text} bbox_median={median_text} depth_bbox={db}"
        )
    except Exception as exc:
        return (
            f"{name}:bbox={getattr(detection, 'bbox', None)} "
            f"score={float(getattr(detection, 'score', 0.0) or 0.0):.2f} depth=ERR({exc})"
        )


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
