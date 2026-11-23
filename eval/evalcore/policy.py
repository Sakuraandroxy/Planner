"""策略接入层。

两种模式：
  - sanity：Phase 0 验证用，直接朝目标物体直线推进（不依赖规划器）。
  - http  ：把观测 POST 到规划器端点，拿回 5×[dx,dy,dz(,dyaw)]，展开成世界点。
  - import：进程内调用规划器（暂留接口，Phase 1 再接）。

约定：plan() 一律返回**世界系 NED** 航点 list[[x,y,z], ...]（长度为 chunk_size），
直接喂给 sim.fly()。
"""
from __future__ import annotations

import json
import math
import urllib.request

import numpy as np


def quat_xyzw_to_matrix(q) -> np.ndarray:
    """四元数 (x,y,z,w) -> 旋转矩阵 body->world（NED 系，手写避免 scipy 依赖）。"""
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def body_offsets_to_world(base_pos, base_quat, offsets) -> list[list[float]]:
    """把机身系相对位移 offsets[N][3]（相对 base_pos，非链式）展开成世界点。

    对齐参考实现 eval_qwenvl_direction.py：
        R_t = R.from_quat(q_t).as_matrix();  p = P_t + R_t @ dp_body
    """
    R = quat_xyzw_to_matrix(base_quat)
    base = np.asarray(base_pos, dtype=float)
    pts = []
    for dp in offsets:
        dp = np.asarray(dp[:3], dtype=float)
        pts.append((base + R @ dp).tolist())
    return pts


class SanityController:
    """Phase 0：不看图，直接朝目标物体直线推进的假策略。"""

    def __init__(self, chunk_size: int = 5, step_m: float = 5.0, reach_tol_m: float = 0.6):
        self.chunk_size = chunk_size
        self.step_m = step_m
        self.reach_tol = reach_tol_m

    def plan(self, obs: dict) -> list[list[float]]:
        cur = np.asarray(obs["position"], dtype=float)
        target = np.asarray(obs["target_object_position"], dtype=float)
        vec = target - cur
        remaining = float(np.linalg.norm(vec))
        if remaining <= self.reach_tol:
            return [cur.tolist() for _ in range(self.chunk_size)]
        direction = vec / remaining
        per = min(self.step_m, remaining / self.chunk_size)
        pts = []
        for k in range(1, self.chunk_size + 1):
            p = cur + direction * per * k
            pts.append(p.tolist())
        return pts


class PlannerBridge:
    """把规划器接到评测器。sanity 走内置假策略，http/import 走真规划器。"""

    def __init__(self, settings, chunk_size: int = 5):
        self.mode = settings.mode
        self.endpoint = settings.endpoint
        self.timeout = settings.timeout_s
        self.chunk_size = chunk_size
        self._sanity = SanityController(chunk_size=chunk_size)

    def plan(self, obs: dict) -> list[list[float]]:
        if self.mode == "sanity":
            return self._sanity.plan(obs)
        if self.mode == "http":
            return self._plan_http(obs)
        if self.mode == "import":
            raise NotImplementedError("import 模式将在接入真实规划器后启用")
        raise ValueError(f"未知 planner mode: {self.mode}")

    def _plan_http(self, obs: dict) -> list[list[float]]:
        """POST 观测到规划器端点，期望返回 [[dx,dy,dz,dyaw?]×N] 机身系增量。"""
        payload = {
            "instruction": obs.get("instruction", ""),
            "position": obs["position"],
            "orientation": obs["quat"],
            "front_b64": _encode_png_b64(obs.get("rgb_front")),
            "down_b64": _encode_png_b64(obs.get("rgb_down")),
        }
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            self.endpoint, data=data,
            headers={"Content-Type": "application/json"}, method="POST",
        )
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            out = json.loads(resp.read().decode("utf-8"))
        offsets = out["waypoints"] if isinstance(out, dict) else out
        return body_offsets_to_world(obs["position"], obs["quat"], offsets)


def _encode_png_b64(img) -> str | None:
    if img is None:
        return None
    import base64

    import cv2
    ok, buf = cv2.imencode(".png", img)
    if not ok:
        return None
    return base64.b64encode(buf.tobytes()).decode("ascii")
