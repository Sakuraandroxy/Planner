#!/usr/bin/env python3
"""run_airsim_minimal.py — AirSim closed-loop with VLMPlanner, no torch needed."""

import os, sys, math, time, re, json
import numpy as np
from PIL import Image
import airsim


def load_dotenv(path):
    """手写 .env 解析器，不依赖第三方库"""
    if not os.path.isfile(path):
        return
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if '=' not in line:
                continue
            key, _, val = line.partition('=')
            key = key.strip()
            val = val.strip().strip('"').strip("'")
            os.environ.setdefault(key, val)


_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

if not load_dotenv(os.path.join(_script_dir, '.env')):
    load_dotenv(os.path.join(_script_dir, 'planner', '.env'))

from planner import VLMPlanner, PlannerConfig
from typing import List, Dict


def _fmt_seconds(value):
    return f"{float(value or 0.0):.2f}s"


def _usage_summary(usage):
    if not isinstance(usage, dict) or not usage:
        return "tokens=n/a"
    parts = []
    for key in ("prompt_tokens", "completion_tokens", "total_tokens"):
        if key in usage and usage[key] is not None:
            parts.append(f"{key}={usage[key]}")
    details = usage.get("completion_tokens_details")
    if isinstance(details, dict) and details.get("reasoning_tokens") is not None:
        parts.append(f"reasoning_tokens={details['reasoning_tokens']}")
    return " ".join(parts) if parts else "tokens=n/a"


def print_step_timing(step, timing, planner_timing):
    planner_timing = planner_timing or {}
    reasoning_tokens = 0
    thinking_type = "unknown"
    reasoning_effort = "unknown"
    for call in planner_timing.get("vlm_calls", []):
        thinking_type = call.get("thinking_type", thinking_type)
        reasoning_effort = call.get("reasoning_effort", reasoning_effort)
        usage = call.get("usage")
        details = usage.get("completion_tokens_details") if isinstance(usage, dict) else None
        if isinstance(details, dict):
            reasoning_tokens += int(details.get("reasoning_tokens") or 0)
    print(
        f"[TIMING Step {step}] "
        f"total={_fmt_seconds(timing.get('total'))} "
        f"capture={_fmt_seconds(timing.get('capture'))} "
        f"bbox_api={_fmt_seconds(planner_timing.get('target_bbox_api'))} "
        f"planning_api={_fmt_seconds(planner_timing.get('planning_api'))} "
        f"vlm_total={_fmt_seconds(timing.get('vlm_total'))} "
        f"execute={_fmt_seconds(timing.get('execute'))} "
        f"thinking={thinking_type} "
        f"effort={reasoning_effort} "
        f"reasoning_tokens={reasoning_tokens}"
    )


# ========== 工具函数 ==========

def actions_to_waypoints(actions, start_pos, start_yaw_deg, config):
    x, y, z = start_pos
    yaw = start_yaw_deg
    waypoints = []
    for action in actions:
        if action == "forward":
            rad = math.radians(yaw)
            x += config.horizontal_step * math.cos(rad)
            y += config.horizontal_step * math.sin(rad)
        elif action == "left":
            yaw -= config.yaw_step_deg
        elif action == "right":
            yaw += config.yaw_step_deg
        elif action == "up":
            z -= config.vertical_step
        elif action == "down":
            z += config.vertical_step
        waypoints.append([round(x, 3), round(y, 3), round(z, 3)])
    return waypoints


def get_drone_image(client):
    responses = client.simGetImages([
        airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)
    ])
    if not responses:
        return None
    resp = responses[0]
    img_bytes = resp.image_data_uint8
    if not img_bytes:
        return None
    img_np = np.frombuffer(img_bytes, dtype=np.uint8).reshape(resp.height, resp.width, 3)
    return Image.fromarray(img_np, "RGB")


def get_vehicle_pose(client):
    pose = client.simGetVehiclePose()
    pos = [pose.position.x_val, pose.position.y_val, pose.position.z_val]
    q = pose.orientation
    siny_cosp = 2.0 * (q.w_val * q.z_val + q.x_val * q.y_val)
    cosy_cosp = 1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val)
    yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return pos, yaw_deg


def check_collision(client):
    """AirSim 原生碰撞检测"""
    info = client.simGetCollisionInfo()
    return info.has_collided


# ========== 历史上下文管理 ==========

class ConversationHistory:
    """管理 VLM 对话历史，自动裁剪防止超长"""

    def __init__(self, max_steps=5):
        self.messages: List[Dict] = []  # [{role, content}...] 纯文本
        self.max_steps = max_steps

    def add_step(self, step_num: int, actions: List[str],
                 old_pos, new_pos, yaw_deg: float, collided: bool):
        """记录一步执行结果"""
        summary = (
            f"[Step {step_num}] 动作: {actions}. "
            f"位置 ({old_pos[0]:.2f}, {old_pos[1]:.2f}, {old_pos[2]:.2f}) -> "
            f"({new_pos[0]:.2f}, {new_pos[1]:.2f}, {new_pos[2]:.2f}), "
            f"偏航 {yaw_deg:.1f} deg."
        )
        if collided:
            summary += " 发生碰撞！"

        self.messages.append({"role": "user", "content": summary})
        self.messages.append({
            "role": "assistant",
            "content": f"已执行 {actions}，继续下一步。"
        })

        # 只保留最近 N 步
        max_pairs = self.max_steps * 2
        if len(self.messages) > max_pairs:
            self.messages = self.messages[-max_pairs:]

    def get_messages(self) -> List[Dict]:
        return self.messages


# ========== 主循环 ==========

def main():
    task = os.environ.get("TASK", os.environ.get("task", "fly forward and explore"))
    max_steps = int(os.environ.get("MAX_STEPS", os.environ.get("max_steps", "20")))

    cfg = PlannerConfig(
        base_url=os.environ.get("PLANNER_BASE_URL", "http://localhost:8000/v1"),
        api_key=os.environ.get("PLANNER_API_KEY", "no-key"),
        model_name=os.environ.get("PLANNER_MODEL_NAME", "gpt-4o"),
        candidate_count=int(os.environ.get("PLANNER_CANDIDATE_COUNT", "3")),
        temperature=float(os.environ.get("PLANNER_TEMPERATURE", "0.8")),
        thinking_mode=os.environ.get("PLANNER_THINKING_MODE", "disabled").lower(),
        reasoning_effort=os.environ.get("PLANNER_REASONING_EFFORT", "default").lower(),
        enable_thinking=os.environ.get("PLANNER_ENABLE_THINKING", "default").lower(),
    )
    planner = VLMPlanner(cfg)
    history = ConversationHistory(max_steps=5)

    print(f"[Planner] model: {cfg.model_name}")
    print(f"[Planner] URL: {cfg.base_url}")
    print(f"[VLM API] {planner.vlm.describe_api_mode()}")
    print(f"[VLM API] thinking_mode={cfg.thinking_mode}")
    print(f"[VLM API] reasoning_effort={cfg.reasoning_effort}")
    print(f"[VLM API] enable_thinking={cfg.enable_thinking}")
    print(f"[Task] {task}")

    print("[AirSim] connecting...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("[AirSim] connected")

    client.enableApiControl(True)
    client.armDisarm(True)
    print("[AirSim] takeoff...")
    client.takeoffAsync().join()
    print("[AirSim] airborne")

    step = 0
    while step < max_steps:
        step += 1
        print(f"\n{'='*60}")
        print(f"  Step {step}/{max_steps}")
        print(f"{'='*60}")
        step_started = time.perf_counter()
        timing = {}

        # 获取当前帧
        capture_started = time.perf_counter()
        frame = get_drone_image(client)
        timing["capture"] = time.perf_counter() - capture_started
        if frame is None:
            print("[ERROR] no image, skip")
            timing["total"] = time.perf_counter() - step_started
            print_step_timing(step, timing, planner.last_timing)
            time.sleep(1)
            continue

        # 碰撞检测
        collision_started = time.perf_counter()
        collided = check_collision(client)
        timing["collision"] = time.perf_counter() - collision_started
        if collided:
            print("[!] 碰撞检测到了！")

        # 调用 VLM（传入上下文历史）
        print("[VLM] 思考中...")
        vlm_started = time.perf_counter()
        trajectories, scene_analysis, raw_json, reasoning, reasoning_summary, task_done, selected_idx, raw_candidates = planner.generate_candidates(
            frame, task, k=cfg.candidate_count,
            conversation_history=history.get_messages()
        )
        timing["vlm_total"] = time.perf_counter() - vlm_started

        # 打印思考过程
        if reasoning:
            print(f"\n{'─'*50}")
            print("  VLM 思考过程")
            print(f"{'─'*50}")
            print(reasoning.strip())
            print(f"{'─'*50}")

        print(f"[VLM] 场景分析: {scene_analysis}")
        print(f"[VLM] 候选轨迹: {len(trajectories)} 条")
        if reasoning_summary:
            print(f"[VLM] 推理摘要: {reasoning_summary}")

        if not trajectories:
            print("[WARN] no valid trajectory, hover")
            timing["total"] = time.perf_counter() - step_started
            print_step_timing(step, timing, planner.last_timing)
            time.sleep(2)
            continue

        # 选第一条（也可展示所有候选项）
        best = trajectories[0]
        actions = best.actions

        # 打印所有候选和理由
        print(f"\n候选轨迹:")
        for i, t in enumerate(trajectories):
            print(f"  {i+1}. {t.actions}  (delta: dx={t.clean_delta.dx:.2f}, dy={t.clean_delta.dy:.2f})")

        print(f"[Plan] 选择: {actions}")

        # 记录执行前位置
        old_pos, _ = get_vehicle_pose(client)

        # 计算并执行航点
        pos, yaw = get_vehicle_pose(client)
        waypoints = actions_to_waypoints(actions, pos, yaw, cfg)
        print(f"[Waypoints] {waypoints}")

        execute_started = time.perf_counter()
        for wp in waypoints:
            client.moveToPositionAsync(wp[0], wp[1], wp[2], velocity=0.5).join()
            time.sleep(0.5)
        timing["execute"] = time.perf_counter() - execute_started

        # 记录执行后位置
        new_pos, new_yaw = get_vehicle_pose(client)
        col_after = check_collision(client)
        print(f"[New pose] ({new_pos[0]:.2f}, {new_pos[1]:.2f}, {new_pos[2]:.2f}) yaw={new_yaw:.1f}")
        if col_after:
            print("[!] 执行后检测到碰撞")

        # 写入上下文历史
        history.add_step(step, actions, old_pos, new_pos, new_yaw, col_after)
        timing["total"] = time.perf_counter() - step_started
        print_step_timing(step, timing, planner.last_timing)

        time.sleep(1.0)

    print(f"\n{'='*60}")
    print("  Closed loop done")
    print(f"{'='*60}")
    client.armDisarm(False)
    client.enableApiControl(False)
    print(f"executed {step} steps")


if __name__ == "__main__":
    main()
