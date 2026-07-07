"""API 原子动作规划器 —— 调用 VLM API，输出 K 条候选轨迹。

与旧 Planner 项目使用相同的 prompt 格式和输出结构：
  - VLM 输出 K 条候选轨迹（candidates），每条含 actions + reason
  - 程序解析所有候选，计算每条 delta（供世界模型打分）
  - selected_index 决定当前执行哪条
"""

import base64
import io
import json
import math
import re
import time
from typing import List, Optional, Tuple

import numpy as np
from PIL import Image
from openai import OpenAI

from agent.planner.base import BasePlanner, TrajectoryResult
from agent.planner import register_planner

# ══════════════════════════════════════════════════════════════
#  Prompt — 与旧项目 Planner/planner/prompt_templates.py 一致
# ══════════════════════════════════════════════════════════════

PLANNER_SYSTEM_PROMPT = """你是无人机路径规划器。

输入包括：当前前视图、下视图、任务目标、程序计算的目标/障碍物位置与深度。
你的唯一任务：根据当前帧和深度信息，输出可执行路径规划。

可用动作：
- forward X：前进X米
- backward X：后退X米
- left X / right X：左/右转X度
- up X / down X：上升/下降X米

任务：{task_description}
{depth_info}

输出 {k} 条候选轨迹，每条最多 {max_trajectory_length} 个动作，只输出合法JSON。
下面示例只说明格式和决策风格，动作必须是字符串数组，禁止输出对象动作，动作数值必须根据当前目标深度和障碍物深度重新计算：
{{
  "selected_index": 0,
  "done": false,
  "scene_analysis": "一句话描述当前场景",
  "reasoning_summary": "简短说明目标、障碍物和选择原因",
  "candidates": [
    {{"actions": ["forward 30"], "reason": "目标在前方较远且正前方通道可行，使用单个较长前进动作减少步数", "scale": 1.0}},
    {{"actions": ["left 10", "forward 28"], "reason": "目标略偏左或右侧有障碍，先小角度修正再长距离接近", "scale": 1.0}},
    {{"actions": ["right 10", "forward 24"], "reason": "左侧有障碍或目标略偏右，绕开后用较少动作接近", "scale": 1.0}}
  ]
}}

规则：
1. 主要依据深度提示中的目标、障碍物、位置和深度规划，不要凭空估计深度。
2. 是否完成必须根据任务语义判断：如果任务是"飞到目标旁边/附近/接近/靠近目标"，目标深度进入约 {arrival_depth} 米可视为完成；如果任务包含"上方/上面/顶部/左侧/右侧/后方/绕过/穿过"等关系，不能只因深度近就完成，必须规划到对应相对位置。
3. 到达半径只用于判断 done=true/false，禁止把到达半径从目标深度中减去来生成 forward 距离；如果当前目标深度大于到达半径，说明任务尚未完成，应规划尽可能少步数接近目标。
4. 动作距离应根据目标深度、障碍物深度和接近停止余量 {approach_stop_margin}m 决定，而不是 target_depth - arrival_radius。例如：target_depth=41m, arrival_radius=5m, approach_stop_margin={approach_stop_margin}m 时，不能因为半径为5m就输出 forward 36；若路径安全，应接近输出 forward 40。
5. 如果当前还未完成，应在不碰撞、不越过目标、不切换目标实例的前提下，用尽可能少的动作完成任务；安全可通行时优先使用单个较大的 forward，而不是多次小步前进。
6. 只有当深度提示明确写着"当前RGB图中未可靠发现目标"时，才禁止 forward/backward/up/down；此时只能原地 left/right 旋转搜索。
7. 如果当前帧有目标bbox和目标深度，就基于当前目标位置、深度和障碍物规划；不要因为历史记忆而忽略当前可见目标。
8. 相邻动作必须是不同类型；连续同类动作必须合并。
9. 需要绕障时可以先转向再前进；动作数值必须由当前目标深度和障碍物深度决定，不能照抄示例数值。
10. 所有动作必须写在 candidates[].actions 中，reasoning_summary 只写文字解释。"""


# ══════════════════════════════════════════════════════════════
#  Planner 实现
# ══════════════════════════════════════════════════════════════

@register_planner("api_atomic_planner")
class ApiAtomicPlanner(BasePlanner):
    """OpenAI 格式 VLM 规划器，输出 K 条候选轨迹 → 供世界模型打分。"""

    def __init__(self):
        from config import cfg
        ag = cfg["AGENT"]
        self.client = OpenAI(
            base_url=ag["PLANNER_URL"],
            api_key=ag.get("PLANNER_API_KEY", "no-key"),
        )
        self.model = ag.get("PLANNER_MODEL", "")
        self.max_tokens = int(ag.get("PLANNER_MAX_TOKENS", 2048))
        self.stop_threshold = float(cfg["AGENT"]["STOP_DEPTH_THRESHOLD"])
        self.candidate_count = int(ag.get("PLANNER_CANDIDATE_COUNT", 3))
        self.max_trajectory_length = int(ag.get("PLANNER_MAX_TRAJECTORY_LENGTH", 5))
        self.approach_stop_margin = float(ag.get("PLANNER_APPROACH_STOP_MARGIN", 1.0))

    def plan(self, front_img, down_img, instruction: str,
             direction: str = "", detected_bbox=None,
             depth_meters=None) -> TrajectoryResult:
        """调用 VLM API 获取 K 条候选轨迹，计算每条 delta 供世界模型使用。"""
        t_total_start = time.time()
        k = self.candidate_count

        # ─── 编码图片（优先复用 ImageEncoder 缓存） ───
        from agent.common.image_encoder import get_cached_front_b64, get_cached_down_b64

        def _b64(img, cache_getter=None):
            if img is None:
                return None
            if cache_getter:
                cached = cache_getter()
                if cached:
                    return cached
            if img.mode == "RGBA":
                img = img.convert("RGB")
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=85)
            return base64.b64encode(buf.getvalue()).decode()

        # ─── 构建深度信息文本（与旧项目 _format_depth_info 等价） ───
        depth_info = _build_depth_info(depth_meters, detected_bbox, front_img)

        # ─── 方向提示 ───
        direction_hint = f"\n方向提示：{direction}" if direction else ""

        # ─── 构建 system prompt ───
        system = PLANNER_SYSTEM_PROMPT.format(
            task_description=instruction,
            depth_info=depth_info + direction_hint,
            k=k,
            max_trajectory_length=self.max_trajectory_length,
            arrival_depth=self.stop_threshold,
            approach_stop_margin=self.approach_stop_margin,
        )

        # ─── 构建 user messages ───
        content = []
        # 前视图在前（主要参考），下视图在后
        for label, img, getter in [("前视图", front_img, get_cached_front_b64),
                                     ("下视图", down_img, get_cached_down_b64)]:
            b64 = _b64(img, getter)
            if b64:
                content.append({
                    "type": "image_url",
                    "image_url": {"url": f"data:image/jpeg;base64,{b64}"},
                })

        # 用户文本：任务 + 深度信息
        user_text = (
            f"任务：{instruction}。{depth_info}。"
            f"根据当前帧、目标/障碍物位置和深度生成 {k} 条候选轨迹；"
            "是否完成由任务语义决定：旁边/附近/接近/靠近类可按到达半径判断，上方/顶部/侧方/绕行/穿过等关系必须到对应位置；"
            "到达半径只用于判断done=true/false，禁止用target_depth-arrival_radius生成forward距离；"
            "如果目标深度大于到达半径，任务尚未完成，应根据目标深度、障碍物深度和接近停止余量用尽可能少步数接近目标；"
            "未完成时在安全可通行、不越过目标、不切换目标实例的前提下，用尽可能少的动作完成任务，安全时优先单个较大的forward；"
            "只有目标真正不可见时才只允许原地旋转搜索；"
            "相邻动作必须不同类型，连续 forward 必须合并；只输出JSON。"
        )
        content.append({"type": "text", "text": user_text})

        messages = [
            {"role": "system", "content": system},
            {"role": "user", "content": content},
        ]

        # ─── 调用 API ───
        t_api_start = time.time()
        try:
            resp = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                max_tokens=self.max_tokens,
                temperature=0.0,
                extra_body={"thinking": {"type": "disabled"}},
            )
            raw = resp.choices[0].message.content or ""
        except Exception as exc:
            print(f"  [ApiAtomicPlanner] API error: {exc}")
            return TrajectoryResult(
                waypoints=[[0.0, 0.0, 0.0]] * 5,
                done=False,
                reasoning=f"API error: {exc}",
            )
        t_api_elapsed = time.time() - t_api_start

        # ─── 解析响应：提取 candidates + selected_index ───
        parsed = _parse_candidate_response(raw)

        actions = parsed["selected_actions"]
        done = parsed["done"]
        reasoning = parsed.get("reasoning_summary", "") or parsed.get("reasoning", "")
        scene_analysis = parsed.get("scene_analysis", "")
        all_candidates = parsed["candidates"]  # list of {actions, reason, delta}

        # ─── 计算 waypoints（兼容占位） ───
        body_waypoints = _actions_to_body_waypoints(actions)
        K = 5
        body_waypoints = body_waypoints[:K]
        while len(body_waypoints) < K:
            body_waypoints.append([0.0, 0.0, 0.0, 0.0])

        t_total = time.time() - t_total_start
        sel_act = actions[0] if actions else "none"
        sel_idx = parsed.get("selected_index", 0) if isinstance(parsed, dict) else 0
        print(
            f"  [ApiAtomicPlanner] {t_api_elapsed:.2f}s api, {t_total:.2f}s total → "
            f"[{sel_idx}] {sel_act}, done={done}, "
            f"{len(all_candidates)} candidates"
        )

        return TrajectoryResult(
            waypoints=body_waypoints,
            done=done,
            reasoning=reasoning,
            actions=actions,
            candidates=all_candidates,
        )


# ══════════════════════════════════════════════════════════════
#  深度信息构建（从 depth_meters + bbox 计算，与旧项目等价）
# ══════════════════════════════════════════════════════════════

def _build_depth_info(
    depth_meters: Optional[np.ndarray],
    detected_bbox: Optional[List[int]],
    front_img,
) -> str:
    """从原始深度矩阵构建 VLM 可读的深度提示文字。

    旧项目用 estimate_depth_in_bbox() 取目标区域中位深度，
    此处简化：取 bbox 中心点 + 画面统计。
    """
    if depth_meters is None:
        return "深度提示：未获取到有效深度数据，请保守行动并优先小步观察。"

    h, w = depth_meters.shape
    all_valid = depth_meters[(depth_meters > 0.1) & (depth_meters < 1000.0)]

    parts = []
    if len(all_valid) > 0:
        scene_min = float(np.min(all_valid))
        parts.append(f"全画面最近深度={scene_min:.1f}m")

    # 画面中心区域深度
    cy, cx = h // 2, w // 2
    half_h, half_w = max(1, h // 10), max(1, w // 10)
    center_region = depth_meters[cy - half_h:cy + half_h, cx - half_w:cx + half_w]
    center_valid = center_region[(center_region > 0.1) & (center_region < 1000.0)]
    if len(center_valid) > 0:
        center_min = float(np.min(center_valid))
        center_avg = float(np.mean(center_valid))
        parts.append(f"画面中心最近深度={center_min:.1f}m")
        parts.append(f"画面中心平均深度={center_avg:.1f}m")

    # 目标 bbox 深度
    if detected_bbox is not None and front_img is not None:
        try:
            x1, y1, x2, y2 = detected_bbox
            cx_bbox = (x1 + x2) // 2
            cy_bbox = (y1 + y2) // 2
            sw = w / front_img.width
            sh = h / front_img.height
            dcx = int(cx_bbox * sw)
            dcy = int(cy_bbox * sh)
            if 0 <= dcy < h and 0 <= dcx < w:
                target_depth = float(depth_meters[dcy, dcx])
                if target_depth > 0.1 and target_depth < 1000.0:
                    parts.insert(0, f"目标bbox中心深度={target_depth:.1f}m")
                    # 也在 bbox 区域内采样几个点取中位数
                    region_x1 = max(0, int(x1 * sw))
                    region_y1 = max(0, int(y1 * sh))
                    region_x2 = min(w, int(x2 * sw))
                    region_y2 = min(h, int(y2 * sh))
                    if region_x2 > region_x1 and region_y2 > region_y1:
                        region = depth_meters[region_y1:region_y2, region_x1:region_x2]
                        region_valid = region[(region > 0.1) & (region < 1000.0)]
                        if len(region_valid) > 0:
                            parts.insert(1, f"目标区域中位深度={float(np.median(region_valid)):.1f}m")
                            parts.insert(2, f"目标区域最近深度={float(np.min(region_valid)):.1f}m")
        except (IndexError, ValueError, TypeError):
            pass

    if parts:
        return "深度提示：" + "，".join(parts) + "。规划必须优先使用这些深度数值。"
    return "深度提示：未获取到有效深度统计，请保守行动并优先小步观察。"


# ══════════════════════════════════════════════════════════════
#  响应解析（兼容新旧两种 JSON 格式）
# ══════════════════════════════════════════════════════════════

def _parse_candidate_response(text: str) -> dict:
    """解析 VLM 返回的 JSON，提取所有候选 + 选中轨迹。

    Returns:
        {
            "selected_index": int,
            "selected_actions": List[str],
            "done": bool,
            "reasoning_summary": str,
            "scene_analysis": str,
            "candidates": [{"actions": [...], "reason": "...", "delta": [...]}, ...],
        }
    """
    if not text:
        return _empty_result("empty response")

    text = re.sub(r"```(?:json)?\s*", "", text)
    text = re.sub(r"```\s*", "", text)

    match = re.search(r"\{.*\}", text, re.DOTALL)
    if not match:
        # 兜底：正则提取 action
        return _fallback_regex_extract(text)

    try:
        data = json.loads(match.group(), strict=False)
    except json.JSONDecodeError:
        try:
            fixed = match.group().replace("None", "null").replace("'", '"')
            data = json.loads(fixed, strict=False)
        except json.JSONDecodeError:
            return _fallback_regex_extract(text)

    if not isinstance(data, dict):
        return _empty_result(f"response not dict: {text[:200]}")

    # ── 提取 candidates 列表 ──
    raw_candidates = data.get("candidates", [])
    if not isinstance(raw_candidates, list):
        raw_candidates = []

    # 兼容单条格式：{"actions": [...], "done": false}
    if not raw_candidates and data.get("actions"):
        raw_candidates = [{"actions": data["actions"], "reason": data.get("reasoning", "")}]

    # ── 解析每条候选 ──
    all_candidates = []
    for c in raw_candidates:
        if not isinstance(c, dict):
            continue
        acts = c.get("actions", [])
        if not acts:
            continue
        # 确保 actions 是字符串列表
        acts = [str(a) for a in acts]
        delta = _compute_delta(acts)
        all_candidates.append({
            "actions": acts,
            "reason": str(c.get("reason", "")),
            "delta": delta,
            "scale": float(c.get("scale", 1.0)),
        })

    # ── 选中的轨迹 ──
    sel_idx = int(data.get("selected_index", 0))
    if 0 <= sel_idx < len(all_candidates):
        selected = all_candidates[sel_idx]
    elif all_candidates:
        selected = all_candidates[0]
        sel_idx = 0
    else:
        return _empty_result("no valid candidates")

    return {
        "selected_index": sel_idx,
        "selected_actions": selected["actions"],
        "done": bool(data.get("done", False)),
        "reasoning_summary": str(data.get("reasoning_summary", data.get("reasoning", "")) or ""),
        "scene_analysis": str(data.get("scene_analysis", "") or ""),
        "candidates": all_candidates,
    }


def _empty_result(reason: str = "") -> dict:
    return {
        "selected_index": 0,
        "selected_actions": [],
        "done": False,
        "reasoning_summary": reason,
        "scene_analysis": "",
        "candidates": [],
    }


def _fallback_regex_extract(text: str) -> dict:
    """兜底：正则从自由文本中提取 action 字符串。"""
    pattern = r'(forward|backward|left|right|up|down)\s+(\d+(?:\.\d+)?)'
    matches = re.findall(pattern, text, re.IGNORECASE)
    if matches:
        actions = [f"{m[0].lower()} {m[1]}" for m in matches]
        delta = _compute_delta(actions)
        return {
            "selected_index": 0,
            "selected_actions": actions,
            "done": False,
            "reasoning_summary": f"regex extracted {len(actions)} actions",
            "scene_analysis": "",
            "candidates": [{"actions": actions, "reason": "regex fallback", "delta": delta}],
        }
    return _empty_result(f"parse failed: {text[:200]}")


# ══════════════════════════════════════════════════════════════
#  原子动作 → Delta / Waypoints 计算
# ══════════════════════════════════════════════════════════════

def _compute_delta(actions: List[str]) -> List[float]:
    """原子动作序列 → Delta(dx, dy, dz, dphi)。

    假设起点 (0,0,0)，初始朝向 0°（正 X 轴）。
    返回 [dx, dy, dz, dphi]，dphi 单位：度（正=右转）。
    """
    x, y, z = 0.0, 0.0, 0.0
    yaw_deg = 0.0

    for action_str in actions:
        name, value = _parse_atomic_action(action_str)
        if name == "forward":
            rad = math.radians(yaw_deg)
            x += value * math.cos(rad)
            y += value * math.sin(rad)
        elif name == "backward":
            rad = math.radians(yaw_deg)
            x -= value * math.cos(rad)
            y -= value * math.sin(rad)
        elif name == "left":
            yaw_deg -= value
        elif name == "right":
            yaw_deg += value
        elif name == "up":
            z -= value
        elif name == "down":
            z += value

    return [round(x, 3), round(y, 3), round(z, 3), round(yaw_deg, 3)]


def _parse_atomic_action(action_str: str) -> Tuple[str, float]:
    """解析 "forward 4.5" → ("forward", 4.5)。"""
    parts = action_str.strip().split()
    if len(parts) >= 2:
        name = parts[0].lower()
        try:
            value = float(parts[1])
        except ValueError:
            value = 0.0
        return name, value
    return action_str.strip().lower(), 0.0


def _actions_to_body_waypoints(actions: List[str]) -> List[List[float]]:
    """原子动作 → 初始机体坐标系下的 waypoints [[dx,dy,dz,dyaw], ...]。

    所有 waypoints 都表达在「初始朝向 0°」的机体坐标系中。
    dyaw 是累积的偏航角变化（度, 相对初始朝向, 左负右正）。

    示例:
        ["forward 5", "left 30", "forward 10"]
        → [[5.0, 0.0, 0.0, 0.0],           ← yaw=0°, 前进5m
           [5.0, 0.0, 0.0, -30.0],           ← 累积位移不变, yaw=-30°
           [13.66, -5.0, 0.0, -30.0]]         ← yaw=-30°, 前进10m
    """
    waypoints = []
    yaw_deg = 0.0
    cum_x, cum_y, cum_z = 0.0, 0.0, 0.0

    for action_str in actions:
        name, value = _parse_atomic_action(action_str)

        if name in ("left", "right"):
            sign = -1 if name == "left" else 1
            yaw_deg += sign * value
            waypoints.append([round(cum_x, 3), round(cum_y, 3), round(cum_z, 3),
                              round(yaw_deg, 1)])

        elif name == "forward":
            rad = math.radians(yaw_deg)
            cum_x += value * math.cos(rad)
            cum_y += value * math.sin(rad)
            waypoints.append([round(cum_x, 3), round(cum_y, 3), round(cum_z, 3),
                              round(yaw_deg, 1)])

        elif name == "backward":
            rad = math.radians(yaw_deg)
            cum_x -= value * math.cos(rad)
            cum_y -= value * math.sin(rad)
            waypoints.append([round(cum_x, 3), round(cum_y, 3), round(cum_z, 3),
                              round(yaw_deg, 1)])

        elif name == "up":
            cum_z -= value
            waypoints.append([round(cum_x, 3), round(cum_y, 3), round(cum_z, 3),
                              round(yaw_deg, 1)])

        elif name == "down":
            cum_z += value
            waypoints.append([round(cum_x, 3), round(cum_y, 3), round(cum_z, 3),
                              round(yaw_deg, 1)])

        else:
            waypoints.append([round(cum_x, 3), round(cum_y, 3), round(cum_z, 3),
                              round(yaw_deg, 1)])

    return waypoints
