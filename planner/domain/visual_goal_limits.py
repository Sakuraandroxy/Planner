"""Task policy values, separate from simulator or model configuration."""
from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class VisualGoalLimits:
    standoff_m: float = 4.0
    distance_tolerance_m: float = 1.0
    minimum_standoff_m: float = 2.0
    distance_uncertainty_margin_m: float = 0.3
    target_center_margin: float = 0.1
    minimum_valid_depth_ratio: float = 0.7
    maximum_depth_mad_m: float = 0.5
    depth_window_fraction: float = 0.02
    confirmation_count: int = 2
    confirmation_interval_s: float = 0.5
    maximum_pitch_abs_deg: float = 5.0
    maximum_vertical_offset_m: float = 1.0
    maximum_round_distance_m: float = 6.0
    maximum_round_motion_s: float = 8.0
    reobserve_heading_change_deg: float = 30.0
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
    safety_replan_count: int = 1
    progress_window_rounds: int = 4
    minimum_progress_m: float = 0.5
    maximum_observation_age_s: float = 45.0
    maximum_anchor_translation_m: float = 0.3
    maximum_anchor_yaw_deg: float = 2.0
    maximum_anchor_refresh_failures: int = 2
    nominal_obstacle_clearance_m: float = 1.0
    vehicle_radius_m: float = 0.5
    tracking_margin_m: float = 0.3
    initial_clearance_radius_m: float = 4.0
    association_gate_m: float = 1.5
    safety_sample_spacing_m: float = 0.2
    execution_overhead_s: float = 0.2
    search_enabled: bool = True
    search_clearance_confirmed: bool = False

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(field.default, bool):
                if not isinstance(value, bool):
                    raise ValueError(f"{field.name} must be boolean")
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{field.name} must be finite numeric")
            if isinstance(field.default, int) and not isinstance(value, int):
                raise ValueError(f"{field.name} must be an integer")
            if value < 0 or (value == 0 and field.name not in {"model_retry_count", "safety_replan_count"}):
                raise ValueError(f"{field.name} must be positive")
        if not 0 < self.target_center_margin < 0.5:
            raise ValueError("target center margin must be between 0 and 0.5")
        if not 0 < self.minimum_valid_depth_ratio <= 1 or not 0 < self.depth_window_fraction <= 0.2:
            raise ValueError("invalid depth sampling ratios")
        if self.confirmation_count < 2 or self.progress_window_rounds < 2:
            raise ValueError("confirmation/progress require at least two observations")
        if self.initial_clearance_radius_m < math.sqrt(3) * self.clearance_radius_m:
            raise ValueError("initial clearance radius must cover the sampled body-margin cube")
        if self.distance_tolerance_m <= self.distance_uncertainty_margin_m:
            raise ValueError("arrival tolerance must exceed uncertainty margin")
        if not 0 < self.search_step_deg <= self.search_half_span_deg <= 90:
            raise ValueError("search range must be within +/-90 degrees")
        if self.search_half_span_deg / self.search_step_deg > 12:
            raise ValueError("search scan has too many steps")

    @property
    def clearance_radius_m(self):
        # Re-anchoring from an accepted displaced start and tracking error can
        # add rather than cancel: reserve both instead of reusing one margin.
        return (self.vehicle_radius_m + self.nominal_obstacle_clearance_m
                + self.tracking_margin_m + self.maximum_anchor_translation_m)

    def validate_standoff(self, requested: float | None, maximum_depth_m: float):
        value = self.standoff_m if requested is None else requested
        if value - self.distance_tolerance_m < max(self.minimum_standoff_m, self.clearance_radius_m):
            raise ValueError("arrival band is below minimum safe standoff")
        if value + self.distance_tolerance_m + self.distance_uncertainty_margin_m >= maximum_depth_m:
            raise ValueError("arrival band exceeds valid depth range")
        return value
