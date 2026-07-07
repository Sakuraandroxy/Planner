"""抓帧 + 编码 + 推送前端。

从 run_airsim_web.py 迁移出来，保持入口文件精简。
"""

import base64
import time
from io import BytesIO


def capture_scene_depth(client, state=None, use_dual=False):
    """从 AirSim 抓取 RGB + 深度图，编码并推送前端。

    Returns dict with frame, depth_meters, encoded base64, timing.
    """
    if use_dual:
        started = time.perf_counter()
        front_rgb, down_rgb, front_depth = client.get_dual_view()
        capture_time = time.perf_counter() - started
        frame = front_rgb
        depth_meters = front_depth
    else:
        started = time.perf_counter()
        frame, depth_meters = client.get_scene_and_depth_meters()
        capture_time = time.perf_counter() - started
        down_rgb = None

    # 编码 PNG（给前端）
    encode_started = time.perf_counter()
    rgb_png_bytes = None
    rgb_base64 = None
    down_png_bytes = None
    down_base64 = None
    if frame is not None:
        rgb_buffer = BytesIO()
        frame.save(rgb_buffer, format="PNG")
        rgb_png_bytes = rgb_buffer.getvalue()
        rgb_base64 = base64.b64encode(rgb_png_bytes).decode("utf-8")
    if down_rgb is not None:
        down_buffer = BytesIO()
        down_rgb.save(down_buffer, format="PNG")
        down_png_bytes = down_buffer.getvalue()
        down_base64 = base64.b64encode(down_png_bytes).decode("utf-8")
    encode_time = time.perf_counter() - encode_started

    # 深度预览
    preview_started = time.perf_counter()
    depth_frame_display = client.depth_meters_to_image(depth_meters) if depth_meters is not None else None
    preview_time = time.perf_counter() - preview_started

    # 深度统计
    stats_started = time.perf_counter()
    depth_data = client.depth_meters_to_stats(depth_meters) if depth_meters is not None else None
    stats_time = time.perf_counter() - stats_started

    # 推送前端
    frontend_time = 0.0
    if state is not None:
        frontend_started = time.perf_counter()
        _push_frame_to_frontend(state, frame, depth_frame_display,
                                rgb_png_bytes=rgb_png_bytes, down_frame=down_rgb)
        frontend_time = time.perf_counter() - frontend_started

    return {
        "frame": frame,
        "down_frame": down_rgb,
        "depth_meters": depth_meters,
        "depth_frame_display": depth_frame_display,
        "depth_data": depth_data,
        "rgb_base64": rgb_base64,
        "down_base64": down_base64,
        "captured_at": time.monotonic(),
        "capture_time": capture_time,
        "rgb_encode_time": encode_time,
        "preview_time": preview_time,
        "stats_time": stats_time,
        "frontend_time": frontend_time,
    }


def _push_frame_to_frontend(state, frame, depth_frame_display,
                            rgb_png_bytes=None, down_frame=None):
    """推送 PNG 帧到前端 SharedState。"""
    if frame is not None:
        if rgb_png_bytes is None:
            buf = BytesIO()
            frame.save(buf, format="PNG")
            rgb_png_bytes = buf.getvalue()
        state.set_frame(rgb_png_bytes)
    if down_frame is not None:
        bd = BytesIO()
        down_frame.save(bd, format="PNG")
        state.set_down_frame(bd.getvalue())
    if depth_frame_display is not None:
        bd = BytesIO()
        depth_frame_display.save(bd, format="PNG")
        state.set_depth_frame(bd.getvalue())
