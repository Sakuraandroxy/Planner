"""Parses VLM JSON response into structured trajectory data."""
import json, re
from planner.trajectory import Trajectory, Delta, compute_delta, add_noise, parse_action


class ResponseParser:
    """Extracts candidates, scene_analysis, reasoning_summary, done flag
    and selected_index from raw VLM response text."""

    def parse(self, response_text: str, config, candidate_count: int,
              target_visible=None):
        """Return (trajectories, scene_analysis, reasoning_summary,
                   task_done, selected_index, raw_candidates)."""
        max_attempts = 2
        for attempt in range(max_attempts):
            try:
                candidate_dicts, scene_analysis, reasoning_summary, task_done, selected_index = \
                    self._parse_json(response_text)
                break
            except (json.JSONDecodeError, ValueError, KeyError) as e:
                if attempt == max_attempts - 1:
                    print(f"[VLM PARSE ERROR] Failed after {max_attempts} attempts: {e}")
                    print(f"[VLM RAW] First 300 chars: {response_text[:300]}")
                    raise
                print(f"[VLM PARSE] Retry {attempt+1}/{max_attempts}...")
                response_text = self._try_fix(response_text)

        trajectories = []
        for i, cand in enumerate(candidate_dicts if isinstance(candidate_dicts, list) else []):
            actions = self._limit_actions(cand.get("actions", []), config, target_visible)
            actions = actions[:getattr(config, "max_trajectory_length", 5)]
            scale = cand.get("scale", 1.0)
            clean_delta = compute_delta(actions, config, scale=scale)
            noisy_delta = add_noise(clean_delta, config)
            traj = Trajectory(
                actions=actions,
                clean_delta=clean_delta,
                noisy_delta=noisy_delta,
                name=f"candidate_{i+1}",
                scale=scale,
            )
            trajectories.append(traj)

        return trajectories, scene_analysis, reasoning_summary, task_done, selected_index, candidate_dicts

    def _limit_actions(self, actions, config, target_visible=None):
        """Clamp unsafe action strings before execution."""
        limited = []
        clamp_yaw = target_visible is not False
        for action in actions:
            try:
                name, value = parse_action(action, config)
            except (TypeError, ValueError):
                limited.append(action)
                continue
            if name == "forward" and value >= config.max_forward_step:
                limited.append(f"forward {config.max_forward_step:g}")
            elif name in ("left", "right") and clamp_yaw and value > config.max_tracking_yaw_step_deg:
                limited.append(f"{name} {config.max_tracking_yaw_step_deg:g}")
            else:
                limited.append(action)
        return limited

    def _parse_json(self, text: str):
        """Extract and parse JSON from VLM response text."""
        text = re.sub(r"```json\s*", "", text)
        text = re.sub(r"```\s*", "", text)
        json_match = re.search(r"\{.*", text, re.DOTALL)
        if not json_match:
            raise ValueError(f"Cannot extract JSON. First 200 chars:\n{text[:200]}")

        raw = json_match.group()
        open_br = raw.count("{")
        close_br = raw.count("}")
        if open_br > close_br:
            raw += "}" * (open_br - close_br)
        open_arr = raw.count("[")
        close_arr = raw.count("]")
        if open_arr > close_arr:
            raw += "]" * (open_arr - close_arr)
        # Replace escaped unescaped control chars
        raw = re.sub(r"[\x00-\x08\x0b\x0c\x0e-\x1f]", "", raw)

        data = json.loads(raw, strict=False)
        candidates = data.get("candidates", data)
        scene_analysis = ""
        reasoning_summary = ""

        if isinstance(candidates, dict):
            scene_analysis = candidates.get("scene_analysis", "")
            reasoning_summary = candidates.get("reasoning_summary", "")
            candidates = candidates.get("candidates", [])
        elif isinstance(candidates, list):
            scene_analysis = data.get("scene_analysis", "")
            reasoning_summary = data.get("reasoning_summary", "")

        task_done = data.get("done", False) if isinstance(data, dict) else False
        selected_index = data.get("selected_index", 0) if isinstance(data, dict) else 0

        return candidates, scene_analysis, reasoning_summary, task_done, selected_index

    def _try_fix(self, text: str) -> str:
        """Attempt to fix common JSON formatting issues."""
        text = text.replace("\u201c", "'").replace("\u201d", "'")
        text = text.replace("\u2018", "'").replace("\u2019", "'")
        text = re.sub(r":\s*'([^']*?)'", r': "\1"', text)
        return text
