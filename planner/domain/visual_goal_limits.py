"""Task policy values, separate from simulator or model configuration."""
from dataclasses import dataclass, fields
import math
from .approach import ApproachSettings


@dataclass(frozen=True)
class VisualGoalLimits:
    distance_uncertainty_margin_m: float = 0.3
    target_center_margin: float = 0.1
    confirmation_count: int = 2
    confirmation_interval_s: float = 0.5
    maximum_pitch_abs_deg: float = 5.0
    maximum_vertical_offset_m: float | None = None
    search_step_deg: float = 30.0
    search_half_span_deg: float = 90.0
    maximum_moves_per_search: int = 9
    maximum_search_episodes: int = 2
    maximum_search_moves: int = 18
    maximum_unreliable_observations: int = 2
    maximum_planning_rounds: int = 30
    maximum_task_duration_s: float = 600.0
    model_request_timeout_s: float = 20.0
    model_retry_count: int = 1
    progress_window_rounds: int = 4
    minimum_progress_m: float = 0.5
    maximum_observation_age_s: float = 45.0
    maximum_anchor_translation_m: float = 0.3
    maximum_anchor_yaw_deg: float = 2.0
    maximum_anchor_refresh_failures: int = 2
    search_enabled: bool = True
    target_refresh_interval_s: float = 5.0
    maximum_target_position_jump_m: float = 6.0

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name == "maximum_vertical_offset_m" and value is None:
                continue
            if isinstance(field.default, bool):
                if not isinstance(value, bool):
                    raise ValueError(f"{field.name} must be boolean")
                continue
            try:
                finite_numeric = (not isinstance(value, bool) and isinstance(value, (int, float))
                                  and math.isfinite(value))
            except OverflowError:
                finite_numeric = False
            if not finite_numeric:
                raise ValueError(f"{field.name} must be finite numeric")
            if isinstance(field.default, int) and not isinstance(value, int):
                raise ValueError(f"{field.name} must be an integer")
            if value < 0 or (value == 0 and field.name not in {"model_retry_count"}):
                raise ValueError(f"{field.name} must be positive")
        if not 0 < self.target_center_margin < 0.5:
            raise ValueError("target center margin must be between 0 and 0.5")
        if self.confirmation_count < 2 or self.progress_window_rounds < 2:
            raise ValueError("confirmation/progress require at least two observations")
        if not 0 < self.search_step_deg <= self.search_half_span_deg <= 90:
            raise ValueError("search range must be within +/-90 degrees")
        if self.search_half_span_deg / self.search_step_deg > 12:
            raise ValueError("search scan has too many steps")

    def validate_arrival_radius(self, radius_m: float, maximum_depth_m: float):
        self.approach_settings(radius_m)
        if radius_m + self.distance_uncertainty_margin_m >= maximum_depth_m:
            raise ValueError("arrival radius exceeds valid depth range")
        return radius_m

    def approach_settings(self, radius_m: float) -> ApproachSettings:
        return ApproachSettings(radius_m, self.distance_uncertainty_margin_m)
