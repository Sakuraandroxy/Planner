"""HTTP API world-model client."""

from __future__ import annotations

import base64
import io
import time
from typing import List

import requests

from agent.functions.candidate.base import candidate_dict_to_world_model
from agent.models.world_model import register_world_model
from agent.models.world_model.base import BaseWorldModel, WorldModelResult


def _pil_to_b64(img) -> str:
    if img is None:
        return ""
    if img.mode == "RGBA":
        img = img.convert("RGB")
    elif img.mode != "RGB":
        img = img.convert("RGB")
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return base64.b64encode(buf.getvalue()).decode()


@register_world_model("api_world_model")
class ApiWorldModel(BaseWorldModel):
    """Call a remote world-model service through HTTP POST."""

    def __init__(self):
        from config import cfg

        wm = cfg.get("WORLD_MODEL", {})
        self.url = wm.get("URL", "http://127.0.0.1:5001/score")
        self.timeout = int(wm.get("TIMEOUT", 30))
        self.enabled = bool(wm.get("ENABLED", False))
        self._session = requests.Session()

    def score(self, front_img_b64: str, down_img_b64: str, instruction: str, candidates: List[dict]) -> WorldModelResult:
        started = time.time()
        wm_candidates = [candidate_dict_to_world_model(c) for c in candidates]
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
                    "source": c.get("source", "planner"),
                    "pre_score": c.get("pre_score", 0.0),
                    "confidence": c.get("confidence", 0.0),
                    "score_breakdown": c.get("score_breakdown", {}),
                }
                for c in wm_candidates
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
            result = WorldModelResult(
                best_index=int(data.get("best_index", 0)),
                scores=data.get("scores", []),
                reasoning=data.get("reasoning", ""),
            )
            print(f"  [WorldModel] best={result.best_index} scores={result.scores} ({time.time() - started:.2f}s)")
            return result
        except requests.exceptions.Timeout:
            print(f"  [WorldModel] timeout ({self.timeout}s)")
            return WorldModelResult(best_index=0, reasoning="timeout")
        except requests.exceptions.ConnectionError:
            print(f"  [WorldModel] connection error: {self.url}")
            return WorldModelResult(best_index=0, reasoning="connection error")
        except Exception as exc:
            print(f"  [WorldModel] error: {exc}")
            return WorldModelResult(best_index=0, reasoning=str(exc))

    def score_from_pil(self, front_img, down_img, instruction: str, candidates: List[dict]) -> WorldModelResult:
        return self.score(
            front_img_b64=_pil_to_b64(front_img),
            down_img_b64=_pil_to_b64(down_img),
            instruction=instruction,
            candidates=candidates,
        )
