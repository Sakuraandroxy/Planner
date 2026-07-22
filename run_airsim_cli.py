#!/usr/bin/env python3
"""Terminal-entrypoint for the fast-slow AirSim closed loop.
Presents a nice input prompt and runs tasks until Ctrl+C.
"""
import os
import sys
import time

_script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _script_dir)

from config import cfg, get_cfg
get_cfg(os.path.join(_script_dir, "config", "default.yaml"))

from sim.frame_capturer import FrameCapturer
from agent.functions.common.warmup import warmup_from_config
from agent.functions.fast_slow.runtime import run_fast_slow_loop
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
        "空行或 Ctrl+C 退出",
    ])
    try:
        raw = input("  ▸ ").strip()
        return raw if raw else None
    except (EOFError, KeyboardInterrupt):
        return None


def main():
    print_box([
        "✈ Uni-LaViRA 闭环 — 终端模式",
        "",
        "正在启动 AirSim 连接...",
    ])

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

    capturer = FrameCapturer(state, interval=0.1)
    capturer.start()
    print("[FrameCapturer] started")

    warmup_from_config()

    print("[AirSim] waiting for first frame...")
    while True:
        rgb, depth = capturer.get_latest_frame()
        if rgb is not None and depth is not None:
            print("[Ready] first frame ready")
            break
        time.sleep(0.1)

    while True:
        task = input_task()
        if task is None:
            break

        print_box(["Task: " + task, "", "运行中... Ctrl+C 中断"])
        state.update(status="running", task=task, task_done=False)

        try:
            run_fast_slow_loop(
                state,
                initial_task=task,
                max_steps=int(cfg.get("EVAL", {}).get("MAX_STEPS", 100)),
                client=client,
                capturer=capturer,
            )
        except KeyboardInterrupt:
            print("\n[Interrupted]")
            break

        print("\n[Task complete]\n")

    print_box(["✈ 退出"])
    try:
        client.cleanup()
    except Exception:
        pass


if __name__ == "__main__":
    main()
