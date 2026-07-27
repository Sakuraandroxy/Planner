"""Contract tests for the candidate flow shared by online and eval loops."""

from types import SimpleNamespace

from agent.functions.candidate import prepare_candidates_for_world_model, select_best_candidate
from agent.models.world_model.base import WorldModelResult
from config import cfg


class _ScoringWorldModel:
    def __init__(self, scores):
        self.scores = scores

    def score_from_pil(self, front_image, down_image, instruction, candidates):
        return WorldModelResult(best_index=0, scores=self.scores, reasoning="test scores", ok=True)


class _UnavailableWorldModel:
    def score_from_pil(self, front_image, down_image, instruction, candidates):
        return WorldModelResult(best_index=0, reasoning="connection error", ok=False)


def _prepare(monkeypatch):
    monkeypatch.setitem(cfg, "CANDIDATE", {
        "ENABLED": True,
        "MODE": "smooth_bridge",
        "COUNT": 5,
        "TOPK_FOR_WORLD_MODEL": 5,
        "RANDOM_SEED": 7,
        "SMOOTH_BRIDGE_LATERAL_SIGMA_M": 0.3,
        "SMOOTH_BRIDGE_VERTICAL_SIGMA_M": 0.1,
    })
    result = SimpleNamespace(
        waypoints=[[2.0, 0.0, 0.0], [4.0, 0.5, 0.0], [6.0, 1.0, 0.0],
                   [8.0, 1.0, 0.0], [10.0, 1.0, 0.0]],
        candidates=[],
        reasoning="test",
    )
    return prepare_candidates_for_world_model(result)


def test_candidate_flow_generates_five_and_selects_max_pre_score(monkeypatch):
    preparation = _prepare(monkeypatch)
    selection = select_best_candidate(preparation)

    assert len(preparation.all_candidates) == 5
    assert selection.selected_index == 0
    assert selection.chosen.pre_score == max(c.pre_score for c in preparation.all_candidates)


def test_short_qwen_trajectory_still_fills_five_candidates(monkeypatch):
    monkeypatch.setitem(cfg, "CANDIDATE", {
        "ENABLED": True,
        "MODE": "smooth_bridge",
        "COUNT": 5,
        "TOPK_FOR_WORLD_MODEL": 5,
        "RANDOM_SEED": 3,
    })
    result = SimpleNamespace(waypoints=[[3.0, 0.0, 0.0]], candidates=[], reasoning="short")

    preparation = prepare_candidates_for_world_model(result)

    assert len(preparation.all_candidates) == 5


def test_world_model_selects_maximum_returned_score(monkeypatch):
    preparation = _prepare(monkeypatch)
    selection = select_best_candidate(
        preparation,
        world_model=_ScoringWorldModel([0.1, 0.2, 0.9, 0.3, 0.4]),
        front_image=object(),
        instruction="Fly to the car",
    )

    assert selection.used_world_model
    assert selection.selected_index == 2


def test_unavailable_world_model_falls_back_to_pre_score(monkeypatch):
    preparation = _prepare(monkeypatch)
    selection = select_best_candidate(
        preparation,
        world_model=_UnavailableWorldModel(),
        front_image=object(),
    )

    assert selection.selected_index == 0
    assert not selection.used_world_model
    assert "maximum pre-score" in selection.reasoning
