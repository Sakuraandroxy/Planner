"""Candidate trajectory preparation pipeline."""

from __future__ import annotations

from agent.functions.candidate.base import CandidatePreparationResult
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
        except Exception as exc:
            generated = []
            seed.metadata["candidate_generation_error"] = str(exc)

        candidates = ([seed] + generated)[:candidate_count]

    candidates = score_candidates(
        candidates,
        detection=detection,
        direction=direction,
        stop_threshold=stop_threshold,
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



