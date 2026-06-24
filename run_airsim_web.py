#!/usr/bin/env python3
"""run_airsim_web.py — Thin entry point. Orchestrates all modules."""
"""
notepad "C:\Users\86136\Documents\AirSim\settings.json"
"""
import os, sys, math, time, threading, io
from io import BytesIO

_script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _script_dir)

# Load .env
_env_path = os.path.join(_script_dir, "planner", ".env")
if os.path.isfile(_env_path):
    with open(_env_path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, _, v = line.partition("=")
            v = v.split("#")[0].strip().strip("\"'")
            os.environ.setdefault(k.strip(), v)

from sim.airsim_client import AirSimClient
from sim.frame_capturer import FrameCapturer
from sim.action_executor import ActionExecutor, actions_to_waypoints
from agent.planner import Planner
from web.shared_state import SharedState
from web.app import create_app
from planner.config import PlannerConfig


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
        capturer = FrameCapturer(cfg.capture_interval)
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

            # Capture fresh frame for VLM
            frame = client.get_image()
            depth_frame_display = client.get_depth_image()
            depth_frame_vlm = client.get_labeled_depth_heatmap()
            depth_meters = client.get_depth_meters()
            depth_data = client.get_center_depth_meters()

            # Push to frontend
            if frame is not None:
                buf = BytesIO()
                frame.save(buf, format="PNG")
                state.set_frame(buf.getvalue())
            if depth_frame_display is not None:
                bd = BytesIO()
                depth_frame_display.save(bd, format="PNG")
                state.set_depth_frame(bd.getvalue())

            # Check collision
            pos, yaw = client.get_pose()
            collided = client.check_collision()
            state.update(pose=pos, yaw=yaw, collided=collided)

            # --- VLM planning ---
            print(f"[Step {step}/{max_steps}] VLM...")
            state.update(status="thinking")
            try:
                trajectories, scene_analysis, _, reasoning, reasoning_summary, task_done, selected_idx, raw_candidates = \
                    planner.generate_candidates(frame, cur_task, k=cfg.candidate_count, depth_frame=depth_frame_vlm, center_depth=depth_data, depth_meters=depth_meters)
            except Exception as e:
                print(f"[VLM ERROR] {e}")
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
                state.update(status="done", task_done=True, step=0)
                state.update(task="")
                planner.clear_context()
                time.sleep(2)
                continue

            if not trajectories:
                time.sleep(2)
                continue

            best_idx = selected_idx if 0 <= selected_idx < len(trajectories) else 0
            best = trajectories[best_idx]
            actions = best.actions
            state.update(selected_actions=actions, status="executing")

            # Record pre-execution pose for context
            pos_before, yaw_before = pos, yaw

            # --- Execute via ActionExecutor ---
            pos, yaw = client.get_pose()
            waypoints = actions_to_waypoints(actions, pos, yaw, cfg, scale=best.scale)
            print(f"  actions={actions}  wps={waypoints}")

            pos_final, yaw_final, col = executor.execute(actions, scale=best.scale)
            state.update(pose=pos_final, yaw=yaw_final, collided=col)

            # Record this step into the planner's context manager
            planner.record_step(step, actions, pos_before, pos_final, yaw_final, col)

            # Case 2: done after executing actions
            if task_done:
                print(f"[DONE] Actions executed, task complete at step {step}")
                state.update(status="done", task_done=True, step=0)
                state.update(task="")
                planner.clear_context()

            # Collision recovery
            if col:
                print("[COLLISION] Backing up...")
                executor.back_up(pos_final, yaw_final)
                state.update(collided=False)

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
    max_steps = int(os.environ.get("max_steps", "20"))

    cfg = PlannerConfig(
        base_url=os.environ.get("PLANNER_BASE_URL", "http://localhost:8000/v1"),
        api_key=os.environ.get("PLANNER_API_KEY", "no-key"),
        model_name=os.environ.get("PLANNER_MODEL_NAME", "gpt-4o"),
        candidate_count=int(os.environ.get("PLANNER_CANDIDATE_COUNT", "5")),
        max_forward_step=float(os.environ.get("PLANNER_MAX_FORWARD_STEP", "10.0")),
        max_tracking_yaw_step_deg=float(os.environ.get("PLANNER_MAX_TRACKING_YAW_STEP_DEG", "10.0")),
        velocity=float(os.environ.get("PLANNER_VELOCITY", "0.5")),
        capture_interval=float(os.environ.get("PLANNER_CAPTURE_INTERVAL", "0.1")),
        temperature=float(os.environ.get("PLANNER_TEMPERATURE", "0.8")),
        action_mode=os.environ.get("PLANNER_ACTION_MODE", "atomic"),
        target_depth_enabled=os.environ.get("PLANNER_TARGET_DEPTH_ENABLED", "1").lower() not in ("0", "false", "no"),
    )

    planner = Planner(cfg)
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
