#!/usr/bin/env python3
"""run_airsim_web.py — Thin entry point. Orchestrates all modules."""
# AirSim settings example: notepad "C:\Users\86136\Documents\AirSim\settings.json"
import os, sys, math, time, threading, io, base64
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
from sim.action_executor import ActionExecutor, actions_to_waypoints
from agent.planner import Planner
from agent.relocalizer import Relocalizer
from agent.task_parser import TaskParser
from agent.task_manager import TaskManager
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
        f"cache_age={_fmt_seconds(timing.get('capture_cache_age'))} "
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


def capture_scene_depth(client: AirSimClient, state: SharedState = None):
    """Capture RGB + depth once, optionally push preview frames to the frontend."""
    started = time.perf_counter()
    frame, depth_meters = client.get_scene_and_depth_meters()
    capture_time = time.perf_counter() - started

    encode_started = time.perf_counter()
    rgb_png_bytes = None
    rgb_base64 = None
    if frame is not None:
        rgb_buffer = BytesIO()
        frame.save(rgb_buffer, format="PNG")
        rgb_png_bytes = rgb_buffer.getvalue()
        rgb_base64 = base64.b64encode(rgb_png_bytes).decode("utf-8")
    encode_time = time.perf_counter() - encode_started

    preview_started = time.perf_counter()
    depth_frame_display = client.depth_meters_to_image(depth_meters)
    preview_time = time.perf_counter() - preview_started

    stats_started = time.perf_counter()
    depth_data = client.depth_meters_to_stats(depth_meters)
    stats_time = time.perf_counter() - stats_started

    frontend_time = 0.0
    if state is not None:
        frontend_started = time.perf_counter()
        push_frame_to_frontend(state, frame, depth_frame_display, rgb_png_bytes=rgb_png_bytes)
        frontend_time = time.perf_counter() - frontend_started

    return {
        "frame": frame,
        "depth_meters": depth_meters,
        "depth_frame_display": depth_frame_display,
        "depth_data": depth_data,
        "rgb_base64": rgb_base64,
        "captured_at": time.monotonic(),
        "capture_time": capture_time,
        "rgb_encode_time": encode_time,
        "preview_time": preview_time,
        "stats_time": stats_time,
        "frontend_time": frontend_time,
    }


def push_frame_to_frontend(state: SharedState, frame, depth_frame_display, rgb_png_bytes=None):
    if frame is not None:
        if rgb_png_bytes is None:
            buf = BytesIO()
            frame.save(buf, format="PNG")
            rgb_png_bytes = buf.getvalue()
        state.set_frame(rgb_png_bytes)
    if depth_frame_display is not None:
        bd = BytesIO()
        depth_frame_display.save(bd, format="PNG")
        state.set_depth_frame(bd.getvalue())


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

        frame_cache = None
        if os.environ.get("PLANNER_STARTUP_CAPTURE_ENABLED", "1").lower() not in ("0", "false", "no"):
            print("[AirSim] initial frame capture...")
            try:
                frame_cache = capture_scene_depth(client, state)
                print(f"[AirSim] initial frame ready in {frame_cache['capture_time']:.2f}s")
            except Exception as exc:
                print(f"[AirSim] initial frame skipped: {exc}")
        print("[Ready]")

        executor = ActionExecutor(client, cfg)
        relocalizer = Relocalizer(client, planner.vlm, planner.prompter, cfg)
        task_manager = TaskManager(enabled=getattr(cfg, "task_manager_enabled", True))
        task_parser = TaskParser(planner.vlm, max_tokens=getattr(cfg, "task_parser_max_tokens", 2048))
        step = 0
        active_steps = 0
        last_task = ""
        last_stage_index = -1
        last_stage_target_key = ""

        while True:
            cur_task = state.task or initial_task
            if not cur_task or not cur_task.strip():
                last_task = ""
                state.update(status="waiting_task")
                time.sleep(0.5)
                continue
            cur_task = cur_task.strip()
            if cur_task != last_task:
                step = 0
                active_steps = 0
                last_stage_index = -1
                last_stage_target_key = ""
                if getattr(cfg, "task_manager_enabled", True):
                    try:
                        print("[TASK PARSER] parsing task with VLM...")
                        parsed_stages = task_parser.parse(cur_task)
                        task_manager.start_with_stages(cur_task, parsed_stages)
                        parse_elapsed = task_parser.last_timing.get("elapsed", 0.0)
                        print(f"[TASK PARSER] parsed {len(parsed_stages)} stages in {parse_elapsed:.2f}s")
                    except Exception as exc:
                        print(f"[TASK PARSER ERROR] {exc}")
                        state.update(status="planning", error=f"task parser failed: {exc}")
                        time.sleep(2)
                        continue
                else:
                    task_manager.reset()
                last_task = cur_task
                planner.clear_context()
                print(f"[TASK] {task_manager.summary()}")
                state.update(
                    step=0,
                    task_done=False,
                    reasoning="",
                    reasoning_summary="",
                    scene_analysis="",
                    candidates=[],
                    selected_actions=[],
                    error="",
                )

            step += 1
            active_steps += 1
            if active_steps > max_steps:
                state.update(status="done")
                state.update(task="")
                last_task = ""
                task_manager.reset()
                step = 0
                active_steps = 0
                continue

            state.update(step=step, status="planning", error="")
            step_started = time.perf_counter()
            step_timing = {}
            current_stage = task_manager.current_stage()
            stage_task = task_manager.current_prompt() if current_stage else cur_task
            stage_requires_target = current_stage.requires_target if current_stage else True
            stage_allow_relocalize = current_stage.allow_relocalize if current_stage else True
            stage_target_key = current_stage.target_query if current_stage else ""

            if current_stage and current_stage.index != last_stage_index:
                print(f"[TASK] stage {current_stage.index + 1}/{len(task_manager.stages)} "
                      f"mode={current_stage.mode}: {current_stage.instruction}")
                if (
                    current_stage.requires_target
                    and stage_target_key
                    and last_stage_target_key
                    and stage_target_key != last_stage_target_key
                ):
                    planner.identity.clear()
                if current_stage.requires_target and stage_target_key:
                    last_stage_target_key = stage_target_key
                last_stage_index = current_stage.index

            if current_stage and current_stage.is_direct_action:
                actions = list(current_stage.actions)
                state.update(
                    status="executing",
                    reasoning_summary=task_manager.summary() + f"\n\n直接动作阶段：执行 {actions}",
                    scene_analysis="当前阶段是固定动作，不调用VLM和bbox。",
                    candidates=[{"actions": actions, "reason": "固定动作阶段", "delta": {}}],
                    selected_actions=actions,
                )
                pos_before, yaw_before = client.get_pose()
                collided = client.check_collision()
                state.update(pose=pos_before, yaw=yaw_before, collided=collided)
                print(f"[Step {step}/{max_steps}] DIRECT actions={actions}")
                waypoint_started = time.perf_counter()
                waypoints = actions_to_waypoints(actions, pos_before, yaw_before, cfg)
                step_timing["waypoint"] = time.perf_counter() - waypoint_started
                print(f"  actions={actions}  wps={waypoints}")
                execute_started = time.perf_counter()
                pos_final, yaw_final, col = executor.execute(actions)
                step_timing["execute"] = time.perf_counter() - execute_started
                state.update(pose=pos_final, yaw=yaw_final, collided=col)
                planner.record_step(step, actions, pos_before, pos_final, yaw_final, col)
                task_manager.complete_current("direct action executed")
                print(f"[TASK] completed stage; {task_manager.summary()}")
                if task_manager.is_done():
                    state.update(status="done", task_done=True, task="", step=0)
                    last_task = ""
                    task_manager.reset()
                    planner.clear_context()
                else:
                    state.update(status="planning", task_done=False)
                step_timing["step_total"] = time.perf_counter() - step_started
                print_step_timing(step, max_steps, step_timing, planner.last_timing)
                time.sleep(cfg.capture_interval)
                continue

            # Capture RGB + depth. Step 1 may reuse the startup/idle cache; later
            # steps always capture fresh frames because the drone has moved.
            cache_max_age = float(os.environ.get("PLANNER_FRAME_CACHE_MAX_AGE", "10.0"))
            can_use_cache = (
                step == 1
                and frame_cache is not None
                and (time.monotonic() - frame_cache.get("captured_at", 0.0)) <= cache_max_age
            )
            if can_use_cache:
                capture = frame_cache
                step_timing["capture_rpc"] = 0.0
                step_timing["capture_cache_age"] = time.monotonic() - frame_cache.get("captured_at", 0.0)
                print(f"[Capture] using cached startup frame age={step_timing['capture_cache_age']:.2f}s")
            else:
                capture = capture_scene_depth(client, state)
                frame_cache = capture
                step_timing["capture_rpc"] = capture["capture_time"]
            frame = capture["frame"]
            depth_meters = capture["depth_meters"]
            depth_data = capture["depth_data"]
            rgb_base64 = capture.get("rgb_base64")
            step_timing["depth_preview"] = capture["preview_time"]
            step_timing["depth_stats"] = capture["stats_time"]
            step_timing["frontend_push"] = capture["frontend_time"]

            if frame is None or not rgb_base64:
                print("[Capture] missing RGB/base64 before VLM; recapturing from AirSim...")
                capture = capture_scene_depth(client, state)
                frame_cache = capture
                step_timing["capture_rpc"] = step_timing.get("capture_rpc", 0.0) + capture["capture_time"]
                step_timing["capture_cache_age"] = 0.0
                frame = capture["frame"]
                depth_meters = capture["depth_meters"]
                depth_data = capture["depth_data"]
                rgb_base64 = capture.get("rgb_base64")
                step_timing["depth_preview"] = capture["preview_time"]
                step_timing["depth_stats"] = capture["stats_time"]
                step_timing["frontend_push"] = capture["frontend_time"]
                if frame is None or not rgb_base64:
                    raise RuntimeError("AirSim capture did not provide an RGB frame/base64 image")

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
                        stage_task,
                        k=cfg.candidate_count,
                        depth_frame=None,
                        center_depth=depth_data,
                        depth_meters=depth_meters,
                        step_num=step,
                        pose=pos,
                        yaw_deg=yaw,
                        task_key=cur_task,
                        target_depth_enabled=stage_requires_target,
                        allow_relocalize=stage_allow_relocalize,
                        encoded_rgb=rgb_base64,
                        detect_only=bool(current_stage and current_stage.mode == "detect"),
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
                reasoning_summary=((task_manager.summary() + "\n\n") if current_stage else "") + (reasoning_summary or ""),
                scene_analysis=scene_analysis,
                candidates=cand_list,
            )

            # Case 1: done + no actions — immediate stop
            if task_done and not trajectories:
                step_timing["step_total"] = time.perf_counter() - step_started
                print_step_timing(step, max_steps, step_timing, planner.last_timing)
                if current_stage:
                    task_manager.complete_current("VLM reported done")
                    print(f"[TASK] completed stage; {task_manager.summary()}")
                if current_stage and not task_manager.is_done():
                    state.update(
                        status="planning",
                        task_done=False,
                        selected_actions=[],
                        reasoning_summary=task_manager.summary() + "\n\n当前阶段完成，进入下一阶段。",
                    )
                else:
                    print(f"[DONE] Task completed at step {step} — no movement needed")
                    try:
                        final_capture = capture_scene_depth(client, state)
                        frame_cache = final_capture
                    except Exception as exc:
                        print(f"[Capture] final frame skipped: {exc}")
                    state.update(status="done", task_done=True, step=0)
                    state.update(task="")
                    last_task = ""
                    task_manager.reset()
                    planner.clear_context()
                time.sleep(2)
                continue

            if not trajectories:
                if (
                    stage_allow_relocalize
                    and getattr(planner, "last_target_missing", False)
                    and getattr(cfg, "relocalizer_enabled", True)
                ):
                    locked_distance = planner.identity.distance_to_locked_target(pos)
                    last_target_depth = getattr(planner, "last_target_depth", None)
                    identity_rejected = bool(
                        isinstance(last_target_depth, dict)
                        and last_target_depth.get("target_identity_rejected")
                    )
                    if planner.identity.is_near_locked_target(pos) and not identity_rejected:
                        print(f"[DONE] Near locked target distance={locked_distance:.2f}m — stop current task")
                        step_timing["step_total"] = time.perf_counter() - step_started
                        print_step_timing(step, max_steps, step_timing, planner.last_timing)
                        state.update(
                            status="done",
                            task_done=True,
                            task="",
                            step=0,
                            reasoning_summary=(
                                (reasoning_summary + "\n\n" if reasoning_summary else "")
                                + f"当前无人机已进入锁定目标到达半径（约 {locked_distance:.2f}m），"
                                  "当前帧目标不可见时不再触发重定位，直接完成任务。"
                            ),
                            scene_analysis="已到达锁定目标附近。",
                            selected_actions=[],
                        )
                        last_task = ""
                        task_manager.reset()
                        step = 0
                        active_steps = 0
                        planner.clear_context()
                        time.sleep(2)
                        continue
                    if identity_rejected:
                        print(
                            "[ARRIVAL] skipped near-lock completion because current frame "
                            "contains a rejected target candidate"
                        )
                    relocalize_started = time.perf_counter()
                    print("[RELOCALIZE] target missing; scanning 4 views...")
                    state.update(status="relocalizing")
                    try:
                        locked_world_pos = planner.identity.lock.world_pos
                        result = relocalizer.run(stage_task, locked_world_pos=locked_world_pos)
                        step_timing["relocalize"] = time.perf_counter() - relocalize_started
                        if result.found and result.target_yaw is not None:
                            candidate_count = len(result.candidates or [])
                            selected_pos = result.selected_world_pos
                            selected_depth = result.selected_depth
                            print(
                                f"[RELOCALIZE] found view={result.view_index} "
                                f"yaw={result.target_yaw:.1f} conf={result.confidence:.2f} "
                                f"depth={selected_depth} world_pos={selected_pos} candidates={candidate_count}"
                            )
                            client.rotate_to_yaw(result.target_yaw)
                            time.sleep(float(getattr(cfg, "relocalizer_settle_seconds", 0.3)))
                            frame_cache = capture_scene_depth(client, state)
                            state.update(
                                yaw=result.target_yaw,
                                reasoning_summary=(
                                    (reasoning_summary + "\n\n" if reasoning_summary else "")
                                    + f"四向环视重定位：共得到 {candidate_count} 个可疑目标；"
                                      f"程序根据深度/世界坐标选择第 {result.view_index} 张图，"
                                      f"深度 {selected_depth}m，置信度 {result.confidence:.2f}，已转向该方向。"
                                ),
                                scene_analysis="四向环视已根据候选目标世界坐标重新定位方向。",
                                selected_actions=[f"rotate_to_yaw {result.target_yaw:.1f}"],
                            )
                        else:
                            print(f"[RELOCALIZE] target not found conf={result.confidence:.2f} reason={result.reason}")
                            state.update(
                                status="done",
                                task_done=True,
                                task="",
                                step=0,
                                reasoning_summary=(
                                    (reasoning_summary + "\n\n" if reasoning_summary else "")
                                    + "四向环视重定位：四张图都未可靠发现任务目标，停止当前任务。"
                                ),
                                scene_analysis="四向环视未发现任务目标。",
                                selected_actions=[],
                            )
                            last_task = ""
                            task_manager.reset()
                            step = 0
                            active_steps = 0
                            planner.clear_context()
                    except Exception as exc:
                        step_timing["relocalize"] = time.perf_counter() - relocalize_started
                        print(f"[RELOCALIZE ERROR] {exc}")
                        state.update(error=f"relocalize failed: {exc}")
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
                if current_stage:
                    task_manager.complete_current("actions executed and VLM reported done")
                    print(f"[TASK] completed stage; {task_manager.summary()}")
                if current_stage and not task_manager.is_done():
                    state.update(
                        status="planning",
                        task_done=False,
                        reasoning_summary=task_manager.summary() + "\n\n当前阶段动作执行完成，进入下一阶段。",
                    )
                else:
                    print(f"[DONE] Actions executed, task complete at step {step}")
                    try:
                        final_capture = capture_scene_depth(client, state)
                        frame_cache = final_capture
                    except Exception as exc:
                        print(f"[Capture] final frame skipped: {exc}")
                    state.update(status="done", task_done=True, step=0)
                    state.update(task="")
                    last_task = ""
                    task_manager.reset()
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
        max_forward_step=float(os.environ.get("PLANNER_MAX_FORWARD_STEP", "inf")),
        max_tracking_yaw_step_deg=float(os.environ.get("PLANNER_MAX_TRACKING_YAW_STEP_DEG", "inf")),
        velocity=float(os.environ.get("PLANNER_VELOCITY", "0.5")),
        capture_interval=float(os.environ.get("PLANNER_CAPTURE_INTERVAL", "0.1")),
        temperature=float(os.environ.get("PLANNER_TEMPERATURE", "0.8")),
        planner_max_tokens=int(os.environ.get("PLANNER_MAX_TOKENS", "8192")),
        task_parser_max_tokens=int(os.environ.get("PLANNER_TASK_PARSER_MAX_TOKENS", "2048")),
        thinking_mode=os.environ.get("PLANNER_THINKING_MODE", "disabled").lower(),
        reasoning_effort=os.environ.get("PLANNER_REASONING_EFFORT", "default").lower(),
        enable_thinking=os.environ.get("PLANNER_ENABLE_THINKING", "default").lower(),
        action_mode=os.environ.get("PLANNER_ACTION_MODE", "atomic"),
        task_manager_enabled=os.environ.get("PLANNER_TASK_MANAGER_ENABLED", "1").lower() not in ("0", "false", "no"),
        target_depth_enabled=os.environ.get("PLANNER_TARGET_DEPTH_ENABLED", "1").lower() not in ("0", "false", "no"),
        scene_obstacle_planning_enabled=os.environ.get("PLANNER_SCENE_OBSTACLE_PLANNING_ENABLED", "1").lower() not in ("0", "false", "no"),
        context_enabled=os.environ.get("PLANNER_CONTEXT_ENABLED", "0").lower() not in ("0", "false", "no"),
        context_max_steps=int(os.environ.get("PLANNER_CONTEXT_MAX_STEPS", "5")),
        context_arrival_depth=float(os.environ.get("PLANNER_CONTEXT_ARRIVAL_DEPTH", "5.0")),
        approach_stop_margin=float(os.environ.get("PLANNER_APPROACH_STOP_MARGIN", "1.0")),
        context_camera_fov_deg=float(os.environ.get("PLANNER_CONTEXT_CAMERA_FOV_DEG", "90.0")),
        context_world_pos_update_alpha=float(os.environ.get("PLANNER_CONTEXT_WORLD_POS_UPDATE_ALPHA", "0.35")),
        target_identity_enabled=os.environ.get("PLANNER_TARGET_IDENTITY_ENABLED", "1").lower() not in ("0", "false", "no"),
        target_identity_world_tolerance_abs=float(os.environ.get("PLANNER_TARGET_IDENTITY_WORLD_TOLERANCE_ABS", "15.0")),
        target_identity_world_tolerance_ratio=float(os.environ.get("PLANNER_TARGET_IDENTITY_WORLD_TOLERANCE_RATIO", "0.35")),
        target_identity_arrival_radius=float(os.environ.get("PLANNER_TARGET_IDENTITY_ARRIVAL_RADIUS", "6.0")),
        target_identity_update_alpha=float(os.environ.get("PLANNER_TARGET_IDENTITY_UPDATE_ALPHA", "0.35")),
        relocalizer_enabled=os.environ.get("PLANNER_RELOCALIZER_ENABLED", "1").lower() not in ("0", "false", "no"),
        relocalizer_view_count=int(os.environ.get("PLANNER_RELOCALIZER_VIEW_COUNT", "4")),
        relocalizer_yaw_step_deg=float(os.environ.get("PLANNER_RELOCALIZER_YAW_STEP_DEG", "90.0")),
        relocalizer_settle_seconds=float(os.environ.get("PLANNER_RELOCALIZER_SETTLE_SECONDS", "0.3")),
        relocalizer_confidence_threshold=float(os.environ.get("PLANNER_RELOCALIZER_CONFIDENCE_THRESHOLD", "0.5")),
        relocalizer_max_tokens=int(os.environ.get("PLANNER_RELOCALIZER_MAX_TOKENS", "1024")),
    )

    planner = Planner(cfg)
    print(f"[VLM API] {planner.vlm.describe_api_mode()}")
    print(f"[VLM API] thinking_mode={cfg.thinking_mode}")
    print(f"[VLM API] reasoning_effort={cfg.reasoning_effort}")
    print(f"[VLM API] enable_thinking={cfg.enable_thinking}")
    print(f"[TaskManager] enabled={cfg.task_manager_enabled}")
    print(f"[Context] enabled={cfg.context_enabled} memory=target+state")
    print(f"[Planner] approach_stop_margin={cfg.approach_stop_margin}")
    print(f"[TargetIdentity] enabled={cfg.target_identity_enabled} tolerance_abs={cfg.target_identity_world_tolerance_abs} tolerance_ratio={cfg.target_identity_world_tolerance_ratio}")
    print(f"[Relocalizer] enabled={cfg.relocalizer_enabled} views={cfg.relocalizer_view_count} yaw_step={cfg.relocalizer_yaw_step_deg}")
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

