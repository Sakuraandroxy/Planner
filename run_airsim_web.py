#!/usr/bin/env python3
"""run_airsim_web.py — Thin entry point. Orchestrates all modules."""
# AirSim settings example: notepad "C:\Users\86136\Documents\AirSim\settings.json"
import os, sys, math, time, threading, io
from io import BytesIO

_script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _script_dir)

def load_dotenv(path):
    """Load key=value pairs from a local .env file without extra dependencies."""
    if not os.path.isfile(path):
        return False
    with open(path, encoding="utf-8-sig") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            value = value.split("#")[0].strip().strip("\"'")
            os.environ.setdefault(key.strip(), value)
    return True


# Load root .env first. Keep planner/.env as legacy fallback only.
if not load_dotenv(os.path.join(_script_dir, ".env")):
    load_dotenv(os.path.join(_script_dir, "planner", ".env"))

from sim.airsim_client import AirSimClient
from sim.frame_capturer import FrameCapturer
from sim.action_executor import ActionExecutor, actions_to_waypoints
from agent.planner import Planner
from web.shared_state import SharedState
from web.app import create_app
from planner.config import PlannerConfig


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
    if isinstance(details, dict):
        reasoning_tokens = details.get("reasoning_tokens")
        if reasoning_tokens is not None:
            parts.append(f"reasoning_tokens={reasoning_tokens}")
    return " ".join(parts) if parts else "tokens=n/a"


def print_step_timing(step, max_steps, timing, planner_timing):
    planner_timing = planner_timing or {}
    total = timing.get("step_total", 0.0)
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
        f"[TIMING Step {step}/{max_steps}] "
        f"total={_fmt_seconds(total)} "
        f"capture={_fmt_seconds(timing.get('capture_rpc'))} "
        f"bbox_api={_fmt_seconds(planner_timing.get('target_bbox_api'))} "
        f"planning_api={_fmt_seconds(planner_timing.get('planning_api'))} "
        f"vlm_total={_fmt_seconds(timing.get('vlm_total'))} "
        f"execute={_fmt_seconds(timing.get('execute'))} "
        f"thinking={thinking_type} "
        f"effort={reasoning_effort} "
        f"reasoning_tokens={reasoning_tokens}"
    )


def warmup_vlm_api(planner: Planner):
    """Send one tiny request before the first real planning call."""
    started = time.perf_counter()
    try:
        text, _ = planner.vlm.call(
            [{"role": "user", "content": "只回答 OK"}],
            max_tokens=8,
        )
        elapsed = time.perf_counter() - started
        answer = (text or "").strip().replace("\n", " ")[:30]
        print(f"[Warmup] VLM API ready in {elapsed:.2f}s answer={answer!r}")
    except Exception as exc:
        elapsed = time.perf_counter() - started
        print(f"[Warmup] VLM API skipped after {elapsed:.2f}s: {exc}")


def main_loop(state: SharedState, planner: Planner, initial_task: str,
               max_steps: int, cfg: PlannerConfig):
    """Main closed-loop: connect AirSim, plan, execute."""
    try:
        # --- Connect ---
        state.update(status="connecting")
        print("[AirSim] connecting...")
        client = AirSimClient()
        client.connect()
        print("[AirSim] connected")
        state.update(status="connected")
        client.enable_api_control(True)
        client.arm(True)
        print("[AirSim] API control enabled")

        gs = client.get_multirotor_state()
        if gs.landed_state != "Flying":
            client.takeoff()
        state.update(status="airborne")

        # Flush collision
        client.check_collision()
        state.update(collided=False)

        # Start background frame stream
        capturer = FrameCapturer(
            cfg.capture_interval,
            include_depth=True,
            depth_interval=float(os.environ.get("PLANNER_DEPTH_PREVIEW_INTERVAL", "2.0")),
        )
        capturer.start(state)
        print("[Ready]")

        executor = ActionExecutor(client, cfg)
        step = 0
        active_steps = 0

        while True:
            cur_task = state.task or initial_task
            if not cur_task or not cur_task.strip():
                state.update(status="waiting_task")
                time.sleep(0.5)
                continue

            step += 1
            active_steps += 1
            if active_steps > max_steps:
                state.update(status="done")
                break

            state.update(step=step, status="planning", error="")
            step_started = time.perf_counter()
            step_timing = {}

            # Capture fresh RGB + depth for VLM with one AirSim RPC
            capture_started = time.perf_counter()
            frame, depth_meters = client.get_scene_and_depth_meters()
            step_timing["capture_rpc"] = time.perf_counter() - capture_started

            depth_preview_started = time.perf_counter()
            depth_frame_display = client.depth_meters_to_image(depth_meters)
            step_timing["depth_preview"] = time.perf_counter() - depth_preview_started

            depth_stats_started = time.perf_counter()
            depth_data = client.depth_meters_to_stats(depth_meters)
            step_timing["depth_stats"] = time.perf_counter() - depth_stats_started

            # Push to frontend
            frontend_started = time.perf_counter()
            if frame is not None:
                buf = BytesIO()
                frame.save(buf, format="PNG")
                state.set_frame(buf.getvalue())
            if depth_frame_display is not None:
                bd = BytesIO()
                depth_frame_display.save(bd, format="PNG")
                state.set_depth_frame(bd.getvalue())
            step_timing["frontend_push"] = time.perf_counter() - frontend_started

            # Check collision
            pose_started = time.perf_counter()
            pos, yaw = client.get_pose()
            collided = client.check_collision()
            state.update(pose=pos, yaw=yaw, collided=collided)
            step_timing["pose_collision"] = time.perf_counter() - pose_started

            # --- VLM planning ---
            print(f"[Step {step}/{max_steps}] VLM...")
            state.update(status="thinking")
            try:
                vlm_started = time.perf_counter()
                trajectories, scene_analysis, _, reasoning, reasoning_summary, task_done, selected_idx, raw_candidates = \
                    planner.generate_candidates(
                        frame,
                        cur_task,
                        k=cfg.candidate_count,
                        depth_frame=None,
                        center_depth=depth_data,
                        depth_meters=depth_meters,
                        step_num=step,
                        pose=pos,
                        yaw_deg=yaw,
                    )
                step_timing["vlm_total"] = time.perf_counter() - vlm_started
            except Exception as e:
                step_timing["vlm_total"] = time.perf_counter() - vlm_started if "vlm_started" in locals() else 0.0
                step_timing["step_total"] = time.perf_counter() - step_started
                print(f"[VLM ERROR] {e}")
                print_step_timing(step, max_steps, step_timing, planner.last_timing)
                state.update(status="planning", error=str(e))
                time.sleep(2)
                continue

            # Build candidate data for frontend
            cand_list = []
            for i, t in enumerate(trajectories):
                raw_cand = raw_candidates[i] if isinstance(raw_candidates, list) and i < len(raw_candidates) and isinstance(raw_candidates[i], dict) else {}
                cand_list.append({
                    "actions": t.actions,
                    "reason": raw_cand.get("reason", ""),
                    "delta": {"dx": t.clean_delta.dx, "dy": t.clean_delta.dy,
                              "dz": t.clean_delta.dz, "dphi": t.clean_delta.dphi}
                })
            selected_reason = ""
            if 0 <= selected_idx < len(cand_list):
                selected_reason = cand_list[selected_idx].get("reason", "")
            if cand_list:
                candidate_summary = ["候选轨迹:"]
                for i, cand in enumerate(cand_list):
                    marker = " ← 最终选择" if i == selected_idx else ""
                    reason_text = cand.get("reason") or "未提供原因"
                    candidate_summary.append(f"- 轨迹{i + 1}{marker}: {cand['actions']}；原因：{reason_text}")
                if selected_reason:
                    candidate_summary.append(f"最终选择原因：{selected_reason}")
                reasoning_summary = (reasoning_summary + "\n\n" if reasoning_summary else "") + "\n".join(candidate_summary)
            state.update(
                reasoning=reasoning or "",
                reasoning_summary=reasoning_summary or "",
                scene_analysis=scene_analysis,
                candidates=cand_list,
            )

            # Case 1: done + no actions — immediate stop
            if task_done and not trajectories:
                print(f"[DONE] Task completed at step {step} — no movement needed")
                step_timing["step_total"] = time.perf_counter() - step_started
                print_step_timing(step, max_steps, step_timing, planner.last_timing)
                state.update(status="done", task_done=True, step=0)
                state.update(task="")
                planner.clear_context()
                time.sleep(2)
                continue

            if not trajectories:
                step_timing["step_total"] = time.perf_counter() - step_started
                print_step_timing(step, max_steps, step_timing, planner.last_timing)
                time.sleep(2)
                continue

            best_idx = selected_idx if 0 <= selected_idx < len(trajectories) else 0
            best = trajectories[best_idx]
            actions = best.actions
            state.update(selected_actions=actions, status="executing")

            # Record pre-execution pose for context
            pos_before, yaw_before = pos, yaw

            # --- Execute via ActionExecutor ---
            waypoint_started = time.perf_counter()
            pos, yaw = client.get_pose()
            waypoints = actions_to_waypoints(actions, pos, yaw, cfg, scale=best.scale)
            step_timing["waypoint"] = time.perf_counter() - waypoint_started
            print(f"  actions={actions}  wps={waypoints}")

            execute_started = time.perf_counter()
            pos_final, yaw_final, col = executor.execute(actions, scale=best.scale)
            step_timing["execute"] = time.perf_counter() - execute_started
            state.update(pose=pos_final, yaw=yaw_final, collided=col)

            # Record this step into the planner's context manager
            record_started = time.perf_counter()
            planner.record_step(step, actions, pos_before, pos_final, yaw_final, col)
            step_timing["record"] = time.perf_counter() - record_started

            # Case 2: done after executing actions
            if task_done:
                print(f"[DONE] Actions executed, task complete at step {step}")
                state.update(status="done", task_done=True, step=0)
                state.update(task="")
                planner.clear_context()

            # Collision recovery
            recovery_started = time.perf_counter()
            if col:
                print("[COLLISION] Backing up...")
                executor.back_up(pos_final, yaw_final)
                state.update(collided=False)
            step_timing["recovery"] = time.perf_counter() - recovery_started
            step_timing["step_total"] = time.perf_counter() - step_started
            print_step_timing(step, max_steps, step_timing, planner.last_timing)

            time.sleep(cfg.capture_interval)

        print("[Done] Closed loop finished.")
    except Exception as e:
        state.update(status="error", error=str(e))
        import traceback
        traceback.print_exc()
    finally:
        try:
            client.cleanup()
        except Exception:
            pass


def main():
    initial_task = ""  # start empty, user sets via frontend
    max_steps = int(os.environ.get("MAX_STEPS", os.environ.get("max_steps", "20")))

    cfg = PlannerConfig(
        base_url=os.environ.get("PLANNER_BASE_URL", "http://localhost:8000/v1"),
        api_key=os.environ.get("PLANNER_API_KEY", "no-key"),
        model_name=os.environ.get("PLANNER_MODEL_NAME", "gpt-4o"),
        candidate_count=int(os.environ.get("PLANNER_CANDIDATE_COUNT", "5")),
        max_trajectory_length=int(os.environ.get("PLANNER_MAX_TRAJECTORY_LENGTH", "5")),
        max_forward_step=float(os.environ.get("PLANNER_MAX_FORWARD_STEP", "10.0")),
        max_tracking_yaw_step_deg=float(os.environ.get("PLANNER_MAX_TRACKING_YAW_STEP_DEG", "10.0")),
        velocity=float(os.environ.get("PLANNER_VELOCITY", "0.5")),
        capture_interval=float(os.environ.get("PLANNER_CAPTURE_INTERVAL", "0.1")),
        temperature=float(os.environ.get("PLANNER_TEMPERATURE", "0.8")),
        planner_max_tokens=int(os.environ.get("PLANNER_MAX_TOKENS", "8192")),
        thinking_mode=os.environ.get("PLANNER_THINKING_MODE", "disabled").lower(),
        reasoning_effort=os.environ.get("PLANNER_REASONING_EFFORT", "default").lower(),
        enable_thinking=os.environ.get("PLANNER_ENABLE_THINKING", "default").lower(),
        action_mode=os.environ.get("PLANNER_ACTION_MODE", "atomic"),
        target_depth_enabled=os.environ.get("PLANNER_TARGET_DEPTH_ENABLED", "1").lower() not in ("0", "false", "no"),
        context_enabled=os.environ.get("PLANNER_CONTEXT_ENABLED", "1").lower() not in ("0", "false", "no"),
        context_reuse_bbox_enabled=os.environ.get("PLANNER_CONTEXT_REUSE_BBOX_ENABLED", "1").lower() not in ("0", "false", "no"),
        context_local_forward_enabled=os.environ.get("PLANNER_CONTEXT_LOCAL_FORWARD_ENABLED", "1").lower() not in ("0", "false", "no"),
        context_recovery_enabled=os.environ.get("PLANNER_CONTEXT_RECOVERY_ENABLED", "1").lower() not in ("0", "false", "no"),
        context_max_steps=int(os.environ.get("PLANNER_CONTEXT_MAX_STEPS", "5")),
        context_bbox_reuse_max_steps=int(os.environ.get("PLANNER_CONTEXT_BBOX_REUSE_MAX_STEPS", "1")),
        context_bbox_reuse_max_forward=float(os.environ.get("PLANNER_CONTEXT_BBOX_REUSE_MAX_FORWARD", "2.0")),
        context_depth_jump_ratio=float(os.environ.get("PLANNER_CONTEXT_DEPTH_JUMP_RATIO", "1.5")),
        context_switch_depth_ratio=float(os.environ.get("PLANNER_CONTEXT_SWITCH_DEPTH_RATIO", "0.6")),
        context_switch_guard_steps=int(os.environ.get("PLANNER_CONTEXT_SWITCH_GUARD_STEPS", "3")),
        context_max_lost_before_redetect=int(os.environ.get("PLANNER_CONTEXT_MAX_LOST_BEFORE_REDETECT", "2")),
        context_local_forward_min_depth=float(os.environ.get("PLANNER_CONTEXT_LOCAL_FORWARD_MIN_DEPTH", "20.0")),
        context_local_forward_min_action=float(os.environ.get("PLANNER_CONTEXT_LOCAL_FORWARD_MIN_ACTION", "2.0")),
        context_goal_safety_margin=float(os.environ.get("PLANNER_CONTEXT_GOAL_SAFETY_MARGIN", "2.0")),
        context_obstacle_min_safe_depth=float(os.environ.get("PLANNER_CONTEXT_OBSTACLE_MIN_SAFE_DEPTH", "4.0")),
        context_obstacle_safety_margin=float(os.environ.get("PLANNER_CONTEXT_OBSTACLE_SAFETY_MARGIN", "2.0")),
        context_search_yaw_step_deg=float(os.environ.get("PLANNER_CONTEXT_SEARCH_YAW_STEP_DEG", "15.0")),
        context_arrival_depth=float(os.environ.get("PLANNER_CONTEXT_ARRIVAL_DEPTH", "5.0")),
        context_front_corridor_half_width=float(os.environ.get("PLANNER_CONTEXT_FRONT_CORRIDOR_HALF_WIDTH", "0.03")),
        context_front_corridor_y_min=float(os.environ.get("PLANNER_CONTEXT_FRONT_CORRIDOR_Y_MIN", "0.45")),
        context_front_corridor_y_max=float(os.environ.get("PLANNER_CONTEXT_FRONT_CORRIDOR_Y_MAX", "0.60")),
        context_target_corridor_half_width=float(os.environ.get("PLANNER_CONTEXT_TARGET_CORRIDOR_HALF_WIDTH", "0.05")),
        context_target_corridor_y_min=float(os.environ.get("PLANNER_CONTEXT_TARGET_CORRIDOR_Y_MIN", "0.45")),
        context_target_corridor_y_max=float(os.environ.get("PLANNER_CONTEXT_TARGET_CORRIDOR_Y_MAX", "0.65")),
        context_camera_fov_deg=float(os.environ.get("PLANNER_CONTEXT_CAMERA_FOV_DEG", "90.0")),
        context_expected_depth_abs_tolerance=float(os.environ.get("PLANNER_CONTEXT_EXPECTED_DEPTH_ABS_TOLERANCE", "5.0")),
        context_expected_depth_ratio_tolerance=float(os.environ.get("PLANNER_CONTEXT_EXPECTED_DEPTH_RATIO_TOLERANCE", "0.25")),
        context_expected_depth_soft_multiplier=float(os.environ.get("PLANNER_CONTEXT_EXPECTED_DEPTH_SOFT_MULTIPLIER", "1.8")),
        context_expected_depth_hard_multiplier=float(os.environ.get("PLANNER_CONTEXT_EXPECTED_DEPTH_HARD_MULTIPLIER", "2.5")),
        context_direction_tolerance_deg=float(os.environ.get("PLANNER_CONTEXT_DIRECTION_TOLERANCE_DEG", "35.0")),
        context_candidate_anchor_min_score=float(os.environ.get("PLANNER_CONTEXT_CANDIDATE_ANCHOR_MIN_SCORE", "0.55")),
    )

    planner = Planner(cfg)
    print(f"[VLM API] {planner.vlm.describe_api_mode()}")
    print(f"[VLM API] thinking_mode={cfg.thinking_mode}")
    print(f"[VLM API] reasoning_effort={cfg.reasoning_effort}")
    print(f"[VLM API] enable_thinking={cfg.enable_thinking}")
    print(f"[Context] enabled={cfg.context_enabled} reuse_bbox={cfg.context_reuse_bbox_enabled} local_forward={cfg.context_local_forward_enabled} recovery={cfg.context_recovery_enabled}")
    if os.environ.get("PLANNER_WARMUP_ENABLED", "1").lower() not in ("0", "false", "no"):
        warmup_vlm_api(planner)
    state = SharedState()
    state.update(model_name=cfg.model_name, action_mode=cfg.action_mode)
    # Start main loop in background thread
    loop_thread = threading.Thread(
        target=main_loop, args=(state, planner, initial_task, max_steps, cfg),
        daemon=True
    )
    loop_thread.start()

    # Start Flask (main thread)
    app = create_app(state)
    port = int(os.environ.get("WEB_PORT", "5000"))
    print(f"\n{'='*50}")
    print(f"  Dashboard: http://localhost:{port}")
    print(f"  Task: {initial_task}")
    print(f"  Model: {cfg.model_name}")
    print(f"{'='*50}\n")
    import logging
    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    app.run(host="0.0.0.0", port=port, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()

