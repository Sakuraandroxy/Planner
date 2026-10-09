import logging
import time

import requests

from planner.adapters.image_encoding import image_url
from planner.domain.execution import ExecutionBudget
from planner.domain.observation import Observation
from planner.domain.pose import WorldPose
from planner.domain.visual_goal import GoalReference
from planner.domain.target_surface import GoalSurfaceObservation
from planner.errors import BudgetExceeded, ProtocolError
from .prompt import GOAL_OBSERVER_SYSTEM_PROMPT, goal_user_prompt
from .response_parser import parse_goal_observation

logger = logging.getLogger(__name__)


class VLMGoalObserver:
    def __init__(self, url: str, model: str, api_key: str, timeout_s: float,
                 thinking: str | None = None, clock=time.monotonic, *, maximum_candidates: int = 5):
        if isinstance(maximum_candidates, bool) or not isinstance(maximum_candidates, int) or maximum_candidates < 1:
            raise ValueError("maximum_candidates must be a positive integer")
        self.maximum_candidates = maximum_candidates
        self.url = url.rstrip("/")
        if not self.url.endswith("/chat/completions"):
            self.url += "/chat/completions"
        self.model, self.api_key, self.timeout_s = model, api_key, timeout_s
        self.thinking, self.clock = thinking, clock

    def observe(self, observation: Observation, target_description: str,
                reference: GoalReference | None = None, *, initial_pose: WorldPose | None = None,
                budget: ExecutionBudget | None = None) -> GoalSurfaceObservation:
        timeout = self.timeout_s if budget is None else min(self.timeout_s, budget.remaining(self.clock()))
        if timeout <= 0:
            raise BudgetExceeded("deadline before goal observation request")
        content = [{"type": "image_url", "image_url": {"url": image_url(observation.rgb)}}]
        if reference is not None:
            content.append({"type": "image_url", "image_url": {"url": image_url(reference.crop)}})
            content.append({"type": "image_url", "image_url": {"url": image_url(reference.region_crop)}})
        content.append({"type": "text", "text": goal_user_prompt(
            target_description, observation.vehicle_pose, initial_pose, reference, self.maximum_candidates,
        )})
        headers = {"Content-Type": "application/json"}
        if self.api_key and self.api_key != "no-key":
            headers["Authorization"] = f"Bearer {self.api_key}"
        # Three-image reference encoding also consumes the caller's deadline.
        if budget is not None:
            timeout = min(self.timeout_s, budget.remaining(self.clock()))
            if timeout <= 0:
                raise BudgetExceeded("deadline during goal image encoding")
        response = requests.post(self.url, headers=headers, timeout=timeout, json={
            "model": self.model, "messages": [
                {"role": "system", "content": GOAL_OBSERVER_SYSTEM_PROMPT},
                {"role": "user", "content": content},
            ], "temperature": 0.0, "max_tokens": 512+128*self.maximum_candidates,
            **({"thinking": {"type": self.thinking}} if self.thinking else {}),
        })
        response.raise_for_status()
        if budget is not None and budget.remaining(self.clock()) <= 0:
            raise BudgetExceeded("late goal observation discarded")
        try:
            raw = response.json()["choices"][0]["message"]["content"]
            result = parse_goal_observation(raw, self.maximum_candidates)
            if reference is None and any(item.matches_reference_region for item in result.surface_candidates):
                raise ProtocolError("region correspondence claimed without a reference")
        except (KeyError, IndexError, TypeError) as exc:
            raise ProtocolError("goal observer returned no message") from exc
        logger.info("[GoalObserver] status=%s bbox=%s evidence=%s", result.status.value, result.bbox, result.evidence)
        return result
