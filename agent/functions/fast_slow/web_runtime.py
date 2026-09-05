"""Web dashboard bootstrap for the fast-slow AirSim runtime."""

from __future__ import annotations

import os
import sys
import threading
import time

from config import cfg, get_cfg

from agent.functions.common import web_runtime_helpers as web_helpers
from agent.functions.common.warmup import warmup_from_config
from agent.functions.debug import TargetSnapshotRecorder
from sim.camera_api_recorder import CameraApiRecorder
from sim.camera_frame_hub import CameraFrameHub
from sim.camera_recording_options import CameraRecordingOptions
from sim.frame_capturer import FrameCapturer
from web.app import create_app
from web.shared_state import SharedState


def run_fast_slow_web(
    *,
    target_snapshot_recorder: TargetSnapshotRecorder | None = None,
    camera_recording_options: CameraRecordingOptions | None = None,
) -> None:
    """Start the dashboard and execute submitted tasks against one AirSim client."""
    # Local import avoids a module cycle while runtime.py re-exports this entrypoint.
    from agent.functions.fast_slow.runtime import (
        print_last_navigation_summary,
        run_fast_slow_loop,
    )

    script_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    sys.path.insert(0, script_dir)
    get_cfg(os.path.join(script_dir, "config", "default.yaml"))

    state = SharedState()
    frame_hub = CameraFrameHub()
    recording_options = camera_recording_options or CameraRecordingOptions()
    camera_recorder = CameraApiRecorder(frame_hub, recording_options, state=state)
    app = create_app(state, camera_recorder=camera_recorder)
    web_port = cfg.get("WEB", {}).get("PORT", 5000)

    def run_web():
        import logging

        logging.getLogger("werkzeug").setLevel(logging.ERROR)
        cli = sys.modules.get("flask.cli")
        if cli:
            cli.show_server_banner = lambda *_, **__: None
        app.run(host="0.0.0.0", port=web_port, debug=False, use_reloader=False)

    threading.Thread(target=run_web, daemon=True).start()
    print("=" * 50)
    print(f"  Dashboard: http://localhost:{web_port}")
    print("  Config: config/default.yaml")
    print("  Runtime: fast_slow")
    print("=" * 50)

    if bool(cfg.get("SIM", {}).get("APPLY_SETTINGS_ON_WEB_START", True)):
        from sim.airsim_settings import write_local_airsim_settings

        settings_path = write_local_airsim_settings(make_backup=True)
        print(f"[AirSimSettings] wrote: {settings_path}")
        print("[AirSimSettings] start/restart AirSim now so the camera settings take effect")

    print("[AirSim] connecting...", flush=True)
    client = web_helpers.connect_web_airsim_client()
    if hasattr(client, "attach_frame_hub"):
        client.attach_frame_hub(frame_hub)
    client.warmup_capture()
    client.enable_api_control(True)
    client.arm(True)
    should_takeoff, start_pos, start_yaw, landed_text = web_helpers.should_takeoff(client)
    print(
        f"[AirSim] startup pose=({start_pos[0]:.1f}, {start_pos[1]:.1f}, {start_pos[2]:.1f}) "
        f"yaw={start_yaw:.1f}deg landed_state={landed_text}"
    )
    if should_takeoff:
        print("[AirSim] takeoff...")
        client.takeoff()
    else:
        print("[AirSim] skip takeoff: vehicle already airborne")

    capturer = FrameCapturer(
        state,
        interval=0.1,
        frame_hub=frame_hub,
        camera_ids=None,
    )
    capturer.start()
    print("[FrameCapturer] background capture started")
    if recording_options.enabled:
        recording_status = camera_recorder.start()
        print(
            f"[CameraApiRecorder] enabled mode={recording_status['mode']} "
            "output=created lazily on first frame"
        )
    else:
        state.set_camera_recording_status(camera_recorder.status())
        print("[CameraApiRecorder] disabled (CLI/Web can enable it explicitly)")

    warmup_from_config()

    print("[AirSim] waiting for first frame...")
    while True:
        rgb, depth = capturer.get_latest_frame()
        if rgb is not None:
            depth_text = getattr(depth, "shape", None)
            print(f"[Ready] first frame ready (depth shape={depth_text or 'unavailable'})")
            break
        time.sleep(0.1)

    current_task = ""
    while not current_task.strip():
        time.sleep(1)
        current_task = state.get_state().get("task", "").strip()

    try:
        while True:
            print(f"\n{'=' * 50}")
            print(f"  New task: {current_task}")
            print(f"{'=' * 50}")
            state.update(status="running", task_done=False)
            run_fast_slow_loop(
                state,
                initial_task=current_task,
                max_steps=int(cfg.get("EVAL", {}).get("MAX_STEPS", 100)),
                client=client,
                capturer=capturer,
                target_snapshot_recorder=target_snapshot_recorder,
            )
            state.update(status="waiting_task", task="", task_done=True)
            print("\n[TASK] task complete, waiting for next task...")
            current_task = ""
            while not current_task.strip():
                time.sleep(1)
                current_task = state.get_state().get("task", "").strip()
    except KeyboardInterrupt:
        print("\n[Exit] shutting down...")
        print_last_navigation_summary()
    finally:
        camera_recorder.stop(reason="web_exit")
        capturer.stop()
        try:
            client.cleanup()
        except Exception:
            pass
