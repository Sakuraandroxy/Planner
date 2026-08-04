"""Candidate trajectory preparation pipeline."""

from __future__ import annotations

import math

from agent.functions.candidate.base import CandidatePreparationResult, CandidateSelectionResult
from agent.functions.candidate.generator import (
    build_seed_candidates,
    generate_perturbed_candidates,
    generate_smooth_bridge_candidates,
)
from agent.functions.candidate.scorer import score_candidates
from config import cfg


def prepare_candidates_for_world_model(
    result,
    detection=None,
    direction: str = "",
    stop_threshold: float = 8.0,
    memory_context: dict | None = None,
) -> CandidatePreparationResult:
    """Normalize, augment, score, and filter candidate trajectories."""
    cand_cfg = cfg.get("CANDIDATE", {})
    if not bool(cand_cfg.get("ENABLED", False)):
        return CandidatePreparationResult(prefilter_reason="candidate pipeline disabled")

    mode = str(cand_cfg.get("MODE", "none")).strip().lower()
    if mode not in {"none", "rigid", "smooth_bridge"}:
        raise ValueError("CANDIDATE.MODE must be one of: none, rigid, smooth_bridge")

    candidate_count = max(int(cand_cfg.get("COUNT", 1)), 1)
    topk = max(int(cand_cfg.get("TOPK_FOR_WORLD_MODEL", 1)), 1)
    random_seed = cand_cfg.get("RANDOM_SEED", None)
    random_seed = None if random_seed in (None, "") else int(random_seed)
    scale_factors = cand_cfg.get("RIGID_SCALE_FACTORS", [0.85, 1.0, 1.15])
    yaw_offsets = cand_cfg.get("RIGID_YAW_OFFSETS_DEG", [-10.0, 0.0, 10.0])
    lateral_offsets = cand_cfg.get("RIGID_LATERAL_OFFSETS_M", [-1.5, 0.0, 1.5])
    bridge_lateral_sigma = float(cand_cfg.get("SMOOTH_BRIDGE_LATERAL_SIGMA_M", 1.0))
    bridge_vertical_sigma = float(cand_cfg.get("SMOOTH_BRIDGE_VERTICAL_SIGMA_M", 0.25))
    bridge_length_scale = float(cand_cfg.get("SMOOTH_BRIDGE_LENGTH_SCALE", 0.35))
    bridge_max_attempts = int(cand_cfg.get("SMOOTH_BRIDGE_MAX_ATTEMPTS", 64))
    bridge_max_turn = float(cand_cfg.get("SMOOTH_BRIDGE_MAX_TURN_DEG", 120.0))
    bridge_max_segment = float(cand_cfg.get("SMOOTH_BRIDGE_MAX_SEGMENT_LENGTH_M", 0.0))
    bridge_max_path_ratio = float(cand_cfg.get("SMOOTH_BRIDGE_MAX_PATH_LENGTH_RATIO", 1.5))
    bridge_progress_tol = float(cand_cfg.get("SMOOTH_BRIDGE_PROGRESS_TOLERANCE_M", 1.0))

    candidates = build_seed_candidates(result)
    planner_selected_index = 0

    if candidates:
        seed = candidates[0]
        target_generated = 0 if mode == "none" else max(candidate_count - 1, 0)
        generated = []

        try:
            if mode == "smooth_bridge" and target_generated > 0:
                generated = generate_smooth_bridge_candidates(
                    seed=seed,
                    max_candidates=target_generated,
                    lateral_sigma_m=bridge_lateral_sigma,
                    vertical_sigma_m=bridge_vertical_sigma,
                    smooth_length_scale=bridge_length_scale,
                    max_attempts=bridge_max_attempts,
                    max_turn_deg=bridge_max_turn,
                    max_segment_length_m=bridge_max_segment,
                    max_path_length_ratio=bridge_max_path_ratio,
                    progress_tolerance_m=bridge_progress_tol,
                    random_seed=random_seed,
                )
            elif mode == "rigid" and target_generated > 0:
                generated = generate_perturbed_candidates(
                    seed=seed,
                    scale_factors=scale_factors,
                    yaw_offsets_deg=yaw_offsets,
                    lateral_offsets_m=lateral_offsets,
                    max_candidates=target_generated,
                    random_seed=random_seed,
                )

            # Smooth bridges need at least one free interior waypoint and can
            # be rejected by feasibility checks. Fill any gap with rigid
            # perturbations so COUNT=5 remains an execution contract whenever
            # the planner returned a non-empty trajectory.
            if mode == "smooth_bridge" and len(generated) < target_generated:
                rigid_fill = generate_perturbed_candidates(
                    seed=seed,
                    scale_factors=scale_factors,
                    yaw_offsets_deg=yaw_offsets,
                    lateral_offsets_m=lateral_offsets,
                    max_candidates=max(candidate_count * 4, target_generated),
                    random_seed=random_seed,
                )
                seen = {
                    tuple(tuple(round(float(v), 3) for v in waypoint) for waypoint in candidate.waypoints)
                    for candidate in [seed] + generated
                }
                for candidate in rigid_fill:
                    key = tuple(
                        tuple(round(float(v), 3) for v in waypoint)
                        for waypoint in candidate.waypoints
                    )
                    if key in seen:
                        continue
                    seen.add(key)
                    generated.append(candidate)
                    if len(generated) >= target_generated:
                        break
        except Exception as exc:
            generated = []
            seed.metadata["candidate_generation_error"] = str(exc)

        candidates = ([seed] + generated)[:candidate_count]

    candidates = score_candidates(
        candidates,
        detection=detection,
        direction=direction,
        stop_threshold=stop_threshold,
        memory_context=memory_context,
    )
    candidates = sorted(candidates, key=lambda c: (c.pre_score, c.confidence), reverse=True)
    wm_candidates = candidates[:topk]

    return CandidatePreparationResult(
        all_candidates=candidates,
        wm_candidates=wm_candidates,
        planner_selected_index=planner_selected_index,
        prefilter_reason=(
            f"prepared={len(candidates)} topk={len(wm_candidates)} "
            f"(mode={mode}, count={candidate_count})"
        ),
    )


def select_best_candidate(
    preparation: CandidatePreparationResult,
    *,
    world_model=None,
    front_image=None,
    down_image=None,
    instruction: str = "",
) -> CandidateSelectionResult:
    """Select maximum pre-score, or maximum valid world-model score.

    Candidate preparation already sorts by descending pre-score.  Any world
    model failure or malformed response falls back to candidate zero instead
    of interrupting the flight loop.
    """
    candidates = list(preparation.all_candidates or [])
    if not candidates:
        return CandidateSelectionResult(reasoning="no candidates")

    fallback = CandidateSelectionResult(
        chosen=candidates[0],
        selected_index=0,
        reasoning="maximum pre-score",
    )
    wm_candidates = list(preparation.wm_candidates or [])
    if world_model is None or not wm_candidates:
        return fallback
    if front_image is None:
        fallback.reasoning = "world model skipped: front image unavailable; maximum pre-score"
        return fallback

    try:
        result = world_model.score_from_pil(
            front_image,
            down_image,
            instruction=instruction,
            candidates=[candidate.to_world_model_dict() for candidate in wm_candidates],
        )
    except Exception as exc:
        fallback.reasoning = f"world model failed: {exc}; maximum pre-score"
        return fallback

    if hasattr(result, "ok") and not bool(result.ok):
        fallback.reasoning = (
            f"world model unavailable: {getattr(result, 'reasoning', '')}; maximum pre-score"
        )
        return fallback

    raw_scores = list(getattr(result, "scores", []) or [])
    scores: list[float] = []
    for value in raw_scores[:len(wm_candidates)]:
        try:
            score = float(value)
        except (TypeError, ValueError):
            score = float("nan")
        scores.append(score)

    valid = [(index, score) for index, score in enumerate(scores) if math.isfinite(score)]
    if valid:
        selected_index = max(valid, key=lambda item: item[1])[0]
    else:
        selected_index = int(getattr(result, "best_index", -1))
        if not (0 <= selected_index < len(wm_candidates)):
            fallback.reasoning = "world model returned no valid selection; maximum pre-score"
            return fallback

    chosen = wm_candidates[selected_index]
    all_index = candidates.index(chosen)
    return CandidateSelectionResult(
        chosen=chosen,
        selected_index=all_index,
        world_model_scores=scores,
        used_world_model=True,
        reasoning=str(getattr(result, "reasoning", "") or "maximum world-model score"),
    )



