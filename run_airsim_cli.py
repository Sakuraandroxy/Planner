#!/usr/bin/env python3
"""Terminal-entrypoint for the fast-slow AirSim closed loop.
Presents a nice input prompt and runs tasks until Ctrl+C.
"""
import argparse
import os
import sys
import time

_script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _script_dir)

from config import cfg, get_cfg
get_cfg(os.path.join(_script_dir, "config", "default.yaml"))

from sim.frame_capturer import FrameCapturer
from sim.camera_api_recorder import CameraApiRecorder
from sim.camera_frame_hub import CameraFrameHub
from sim.camera_recording_options import (
    add_camera_recording_arguments,
    resolve_camera_recording_options,
)
from agent.functions.common.warmup import warmup_from_config
from agent.functions.debug import TargetSnapshotRecorder
from agent.functions.fast_slow.runtime import run_fast_slow_loop, print_last_navigation_summary
from agent.functions.common import web_runtime_helpers as web_helpers


BOX_TOP    = "╔" + "═" * 58 + "╗"
BOX_MID    = "║" + " " * 58 + "║"
BOX_BOT    = "╚" + "═" * 58 + "╝"


def box_line(text: str) -> str:
    padded = "  " + text
    return "║" + padded + " " * (58 - len(padded)) + "║"


def print_box(lines: list[str]):
    print()
    print(BOX_TOP)
    for line in lines:
        print(box_line(line))
    print(BOX_BOT)
    print()


def input_task() -> str | None:
    print_box([
        "✈ Uni-LaViRA — 快慢双系统闭环",
        "",
        "输入自然语言任务",
        "例如: 飞到红色车旁边",
        "调试快照: /target-snapshot on 或 off",
        "相机录制: /camera-record on [frames|video|events] 或 off",
        "空行或 Ctrl+C 退出",
    ])
    try:
        raw = input("  ▸ ").strip()
        return raw if raw else None
    except (EOFError, KeyboardInterrupt):
        return None


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Uni-LaViRA AirSim terminal runtime")
    parser.add_argument(
        "--save-target-snapshots",
        action="store_true",
        help="保存每个实际锁定目标的首次检测框图（默认关闭）",
    )
    add_camera_recording_arguments(parser)
    return parser.parse_args(argv)


def _handle_snapshot_command(task: str, recorder: TargetSnapshotRecorder) -> bool:
    parts = task.strip().lower().split()
    if not parts or parts[0] not in {"/target-snapshot", "/target-snapshots"}:
        return False
    if len(parts) != 2 or parts[1] not in {"on", "off"}:
        print("[TargetSnapshot] 用法: /target-snapshot on 或 /target-snapshot off")
        return True
    enabled = parts[1] == "on"
    run_directory = recorder.set_enabled(enabled)
    if enabled:
        print(f"[TargetSnapshot] enabled output={run_directory}")
    else:
        print("[TargetSnapshot] disabled (已保存文件会保留)")
    return True


def _handle_camera_record_command(task: str, recorder: CameraApiRecorder) -> bool:
    parts = task.strip().lower().split()
    if not parts or parts[0] not in {"/camera-record", "/camera-recording"}:
        return False
    if len(parts) >= 2 and parts[1] == "off":
        status = recorder.stop()
        print(f"[CameraApiRecorder] disabled frames={status['written_frames']}")
        return True
    if len(parts) >= 2 and parts[1] == "on":
        mode = parts[2] if len(parts) >= 3 else "video"
        if mode not in {"frames", "video", "events"}:
            print("[CameraApiRecorder] 用法: /camera-record on [frames|video|events] 或 /camera-record off")
            return True
        status = recorder.start(mode=mode)
        print(f"[CameraApiRecorder] enabled mode={status['mode']} output=延迟到首帧创建")
        return True
    if len(parts) >= 2 and parts[1] == "event":
        name = parts[2] if len(parts) >= 3 else "cli"
        status = recorder.trigger_event(name)
        print(f"[CameraApiRecorder] event={status.get('event_name', '')}")
        return True
    print("[CameraApiRecorder] 用法: /camera-record on [frames|video|events]、off 或 event [name]")
    return True


def _run_task_prompt_loop(state, client, capturer, target_snapshot_recorder, camera_recorder):
    warmup_from_config()

    print("[AirSim] waiting for first frame...")
    while True:
        rgb, depth = capturer.get_latest_frame()
        if rgb is not None:
            print("[Ready] first frame ready")
            break
        time.sleep(0.1)

    while True:
        task = input_task()
        if task is None:
            break
        if _handle_snapshot_command(task, target_snapshot_recorder):
            continue
        if _handle_camera_record_command(task, camera_recorder):
            continue

        print_box(["Task: " + task, "", "运行中... Ctrl+C 中断"])
        state.update(status="running", task=task, task_done=False)

        try:
            run_fast_slow_loop(
                state,
                initial_task=task,
                max_steps=int(cfg.get("EVAL", {}).get("MAX_STEPS", 100)),
                client=client,
                capturer=capturer,
                isolated_planning_capture=True,
                target_snapshot_recorder=target_snapshot_recorder,
            )
        except KeyboardInterrupt:
            print("\n[Interrupted]")
            print_last_navigation_summary()
            break

        print("\n[Task complete]\n")


def main(argv=None):
    args = _parse_args(argv)
    recording_options = resolve_camera_recording_options(args, cfg)
    target_snapshot_recorder = TargetSnapshotRecorder(enabled=args.save_target_snapshots)
    print_box([
        "✈ Uni-LaViRA 闭环 — 终端模式",
        "",
        "正在启动 AirSim 连接...",
    ])
    if target_snapshot_recorder.enabled:
        print(f"[TargetSnapshot] enabled output={target_snapshot_recorder.run_directory}")
    else:
        print("[TargetSnapshot] disabled (使用 --save-target-snapshots 或 /target-snapshot on 开启)")

    # Connect AirSim
    print("[AirSim] connecting...")
    client = web_helpers.connect_web_airsim_client()
    client.warmup_capture()
    client.enable_api_control(True)
    client.arm(True)
    should_takeoff, start_pos, start_yaw, landed_text = web_helpers.should_takeoff(client)
    print(f"[AirSim] pose=({start_pos[0]:.1f}, {start_pos[1]:.1f}, {start_pos[2]:.1f}) yaw={start_yaw:.1f}deg")
    if should_takeoff:
        print("[AirSim] takeoff...")
        client.takeoff()
    else:
        print("[AirSim] skip takeoff: already airborne")

    from web.shared_state import SharedState
    state = SharedState()

    frame_hub = CameraFrameHub()
    if hasattr(client, "attach_frame_hub"):
        client.attach_frame_hub(frame_hub)
    camera_recorder = CameraApiRecorder(frame_hub, recording_options, state=state)
    capturer = FrameCapturer(
        state,
        interval=0.1,
        frame_hub=frame_hub,
        camera_ids=None,
    )
    capturer.start()
    print("[FrameCapturer] started")
    if recording_options.enabled:
        status = camera_recorder.start()
        print(f"[CameraApiRecorder] enabled mode={status['mode']} output=延迟到首帧创建")
    else:
        state.set_camera_recording_status(camera_recorder.status())
        print("[CameraApiRecorder] disabled (使用 --record-camera-api 或 /camera-record on 开启)")

    try:
        _run_task_prompt_loop(
            state,
            client,
            capturer,
            target_snapshot_recorder,
            camera_recorder,
        )
    finally:
        print_box(["✈ 退出"])
        camera_recorder.stop(reason="cli_exit")
        capturer.stop()
        try:
            client.cleanup()
        except Exception:
            pass


if __name__ == "__main__":
    main()
