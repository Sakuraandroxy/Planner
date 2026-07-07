"""
agent/world_model/api_world_model.py — HTTP API 世界模型客户端。

通过 Flask HTTP 接口访问远程世界模型服务。
输入：前视图 + 下视图 + 指令 + K 条候选轨迹
输出：最优轨迹下标 + 分数
"""

import base64
import io
import json
import time
from typing import List, Optional

import requests
from PIL import Image

from agent.world_model.base import BaseWorldModel, WorldModelResult
from agent.world_model import register_world_model
from agent.planner.api_atomic_planner import _actions_to_body_waypoints


def _pil_to_b64(img) -> str:
    """PIL Image → JPEG base64 字符串。"""
    if img is None:
        return ""
    if img.mode == "RGBA":
        img = img.convert("RGB")
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return base64.b64encode(buf.getvalue()).decode()


@register_world_model("api_world_model")
class ApiWorldModel(BaseWorldModel):
    """通过 HTTP POST 调用远程世界模型服务。"""

    def __init__(self):
        from config import cfg
        wm = cfg.get("WORLD_MODEL", {})
        self.url = wm.get("URL", "http://127.0.0.1:5001/score")
        self.timeout = int(wm.get("TIMEOUT", 30))
        self.enabled = bool(wm.get("ENABLED", False))
        self._session = requests.Session()

    def score(self, front_img_b64: str, down_img_b64: str,
              instruction: str, candidates: List[dict]) -> WorldModelResult:
        """调用远程世界模型打分。

        对每条候选，如果 waypoints 未预计算则从 actions 转换。
        """
        t0 = time.time()

        # 为每条候选补全 waypoints（如果还没有）
        for c in candidates:
            if "waypoints" not in c or not c["waypoints"]:
                c["waypoints"] = _actions_to_body_waypoints(c.get("actions", []))

        payload = {
            "front_image": front_img_b64,
            "down_image": down_img_b64,
            "instruction": instruction,
            "candidates": [
                {
                    "waypoints": c["waypoints"],
                    "actions": c.get("actions", []),
                    "reason": c.get("reason", ""),
                    "delta": c.get("delta", []),
                    "scale": c.get("scale", 1.0),
                }
                for c in candidates
            ],
        }

        try:
            resp = self._session.post(
                self.url,
                json=payload,
                timeout=self.timeout,
                headers={"Content-Type": "application/json"},
            )
            resp.raise_for_status()
            data = resp.json()
            elapsed = time.time() - t0
            result = WorldModelResult(
                best_index=int(data.get("best_index", 0)),
                scores=data.get("scores", []),
                reasoning=data.get("reasoning", ""),
            )
            print(f"  [WorldModel] best={result.best_index} "
                  f"scores={result.scores} ({elapsed:.2f}s)")
            return result
        except requests.exceptions.Timeout:
            print(f"  [WorldModel] ⚠️ timeout ({self.timeout}s)")
            return WorldModelResult(best_index=0, reasoning="timeout")
        except requests.exceptions.ConnectionError:
            print(f"  [WorldModel] ⚠️ 连接失败: {self.url}")
            return WorldModelResult(best_index=0, reasoning="connection error")
        except Exception as e:
            print(f"  [WorldModel] ⚠️ error: {e}")
            return WorldModelResult(best_index=0, reasoning=str(e))

    def score_from_pil(self, front_img, down_img,
                       instruction: str, candidates: List[dict]) -> WorldModelResult:
        """便捷方法：直接传 PIL Image，内部转 base64。"""
        return self.score(
            front_img_b64=_pil_to_b64(front_img),
            down_img_b64=_pil_to_b64(down_img),
            instruction=instruction,
            candidates=candidates,
        )
