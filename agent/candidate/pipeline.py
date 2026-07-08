"""候选轨迹准备流水线。"""

from __future__ import annotations

from agent.candidate.base import CandidatePreparationResult
from agent.candidate.generator import build_seed_candidates, generate_perturbed_candidates
from agent.candidate.scorer import score_candidates
from config import cfg


def prepare_candidates_for_world_model(
    result,
    detection=None,
    direction: str = "",
    stop_threshold: float = 8.0,
) -> CandidatePreparationResult:
    """标准化候选、补生成、预打分，并筛到世界模型需要的数量。"""
    cand_cfg = cfg.get("CANDIDATE", {})
    min_count = int(cand_cfg.get("MIN_COUNT", 5))
    topk = int(cand_cfg.get("TOPK_FOR_WORLD_MODEL", 3))
    max_generated = int(cand_cfg.get("MAX_GENERATED", 8))
    scale_factors = cand_cfg.get("SCALE_FACTORS", [0.85, 1.0, 1.15])
    yaw_offsets = cand_cfg.get("YAW_OFFSETS_DEG", [-10.0, 0.0, 10.0])
    lateral_offsets = cand_cfg.get("LATERAL_OFFSETS_M", [-1.5, 0.0, 1.5])

    candidates = build_seed_candidates(result)
    planner_selected_index = 0

    if len(candidates) < min_count and candidates:
        generated = generate_perturbed_candidates(
            seed=candidates[0],
            scale_factors=scale_factors,
            yaw_offsets_deg=yaw_offsets,
            lateral_offsets_m=lateral_offsets,
            max_candidates=max_generated,
        )
        candidates.extend(generated)

    candidates = score_candidates(
        candidates,
        detection=detection,
        direction=direction,
        stop_threshold=stop_threshold,
    )
    candidates = sorted(candidates, key=lambda c: (c.pre_score, c.confidence), reverse=True)
    wm_candidates = candidates[:max(topk, 1)]

    return CandidatePreparationResult(
        all_candidates=candidates,
        wm_candidates=wm_candidates,
        planner_selected_index=planner_selected_index,
        prefilter_reason=(
            f"prepared={len(candidates)} topk={len(wm_candidates)} "
            f"(min_count={min_count})"
        ),
    )
