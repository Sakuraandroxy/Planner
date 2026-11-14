"""Stage-local contracts for observing and reaching a target's top surface."""
from dataclasses import dataclass
from enum import Enum
import math

from .observation import Observation
from .pose import WorldPose
from .target_detection import NormalizedBoundingBox
from .execution import ExecutionRecord
from .above_polygon import in_ring


def positive(value):
    return type(value) in (int, float) and math.isfinite(value) and value > 0


@dataclass(frozen=True)
class VisualGoalAboveParameters:
    instruction: str
    target_description: str
    height_above_m: float | None = None

    def __post_init__(self):
        if any(not isinstance(s, str) or not s.strip() for s in (self.instruction, self.target_description)):
            raise ValueError("above task requires instruction and target description")
        if self.height_above_m is not None and not positive(self.height_above_m):
            raise ValueError("height_above_m must be positive or null")


class AbovePhase(str, Enum):
    FINDING_TARGET = "finding_target"
    REVEALING_TOP = "revealing_top"
    APPROACHING_ABOVE = "approaching_above"


class AboveDetectionMode(str, Enum):
    TARGET_ONLY = "target_only"
    TARGET_AND_TOP = "target_and_top"


Point2 = tuple[float, float]
Point3 = tuple[float, float, float]


@dataclass(frozen=True)
class TopImageRegion:
    outline: tuple[Point2, ...]
    holes: tuple[tuple[Point2, ...], ...]
    support_points: tuple[Point2, ...]

    def __post_init__(self):
        for ring in (self.outline, *self.holes):
            if len(ring) < 3 or any(len(p) != 2 or any(
                type(v) not in (int, float) or not math.isfinite(v) or not 0 <= v <= 1 for v in p
            ) for p in ring):
                raise ValueError("top outlines require at least three normalized coordinate pairs")
        if not 3 <= len(self.support_points) <= 5 or any(len(p) != 2 or any(
            type(v) not in (int, float) or not math.isfinite(v) or not 0 <= v <= 1 for v in p
        ) for p in self.support_points):
            raise ValueError("each top region requires 3 to 5 normalized roof support points")


@dataclass(frozen=True)
class AboveDetection:
    bbox: NormalizedBoundingBox | None
    surface_points: tuple[Point2, ...]
    top_regions: tuple[TopImageRegion, ...]


@dataclass(frozen=True)
class AboveProjectedPoint:
    world: Point3
    image: Point2


@dataclass(frozen=True)
class AboveDetectionInput:
    current: Observation
    instruction: str
    target_description: str
    reference_rgb: object = None
    projected_target_points: tuple[AboveProjectedPoint, ...] = ()
    bound_target_points_world: tuple[Point3, ...] = ()
    mode: AboveDetectionMode = AboveDetectionMode.TARGET_ONLY


@dataclass(frozen=True)
class TopWorldRegion:
    outline_world: tuple[Point3, ...]
    holes_world: tuple[tuple[Point3, ...], ...]
    plane: tuple[float, float, float, float]
    capture_id: str
    support_points: tuple[Point3, ...] = ()

    def height_z(self, x: float, y: float) -> float | None:
        """Use the local plane only inside the observed outline, excluding holes."""
        if not in_ring(x,y,self.outline_world) or any(in_ring(x,y,h) for h in self.holes_world):
            return None
        nx,ny,nz,d = self.plane
        return -(nx*x+ny*y+d)/nz if abs(nz) > 1e-12 else None


@dataclass(frozen=True)
class AboveTargetBinding:
    capture_id: str
    surface_points: tuple[Point3, ...]


@dataclass(frozen=True)
class AboveAssociationResult:
    detection: AboveDetection | None
    surfaces: tuple[Point3, ...]
    regions: tuple[TopWorldRegion, ...]
    target_visible: bool
    reason: str


@dataclass(frozen=True)
class AboveTopObservation:
    capture_id: str
    pose: WorldPose
    bbox: NormalizedBoundingBox
    edge_elevations_deg: tuple[float, ...]
    clipped_top: bool


@dataclass(frozen=True)
class AbovePlanningContext:
    phase: AbovePhase
    target_description: str
    surface_points_world: tuple[Point3, ...]
    top_regions: tuple[TopWorldRegion, ...]
    destination: WorldPose | None
    height_above_m: float
    last_executed_before_replan: ExecutionRecord | None = None
    height_is_minimum: bool = False
    target_visible: bool = False
    top_detection_enabled: bool = False
    reported_top_visible: bool = False
    top_observation: AboveTopObservation | None = None
    bound_surface_points_world: tuple[Point3, ...] = ()


@dataclass(frozen=True)
class AboveCompletionRequest:
    parameters: VisualGoalAboveParameters
    observation: Observation
    top_regions: tuple[TopWorldRegion, ...]
    height_above_m: float
    measured_height_m: float | None
    projection_inside: bool
    height_condition_met: bool
    hold_confirmed: bool
    reference_rgb: object = None
    body_projection_world: Point3 | None = None
    body_projection_image: Point2 | None = None


@dataclass(frozen=True)
class CameraMountPose:
    position: Point3
    rotation: tuple[Point3, Point3, Point3]
