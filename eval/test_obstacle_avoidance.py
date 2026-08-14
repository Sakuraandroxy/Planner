"""Focused regression tests for the local depth safety layer."""

import numpy as np

from agent.functions.obstacle_avoidance.local_depth_avoider import DepthObstacleAvoider


def _avoider(**overrides):
    config = {
        "ENABLED": True,
        "FRONT_DEPTH_STRIDE": 2,
        "FRONT_MAX_DEPTH_M": 20.0,
        "GRID_RESOLUTION_M": 0.6,
        "SAFETY_RADIUS_M": 1.5,
        "VERTICAL_CLEARANCE_M": 1.3,
        "BYPASS_ENABLED": True,
        "CONSERVATIVE_STRUCTURE_MODE": True,
        "STRUCTURE_CLUSTER_MIN_CELLS": 6,
        "STOP_BUFFER_M": 2.2,
        "DYNAMIC_STOP_ENABLED": True,
        "REACTION_TIME_S": 0.45,
        "BRAKING_DECEL_MPS2": 2.5,
        "EXTRA_STOP_MARGIN_M": 0.8,
    }
    config.update(overrides)
    return DepthObstacleAvoider(config, {"AIRSIM_VELOCITY": 2.0})


def test_dynamic_stop_buffer_grows_with_speed():
    avoider = _avoider()
    assert avoider.dynamic_stop_buffer_m(4.0) > avoider.dynamic_stop_buffer_m(1.0)
    assert avoider.dynamic_stop_buffer_m(2.0) >= 2.2


def test_raw_depth_corridor_detects_near_tree_returns():
    avoider = _avoider(
        RAW_DEPTH_EMERGENCY_ENABLED=True,
        RAW_DEPTH_CORRIDOR_WIDTH_RATIO=0.4,
        RAW_DEPTH_CORRIDOR_TOP_RATIO=0.2,
        RAW_DEPTH_CORRIDOR_BOTTOM_RATIO=0.8,
        RAW_DEPTH_CORRIDOR_QUANTILE=0.08,
        RAW_DEPTH_CORRIDOR_MIN_SAMPLES=8,
    )
    depth = np.full((64, 64), 20.0, dtype=np.float32)
    depth[22:42, 27:37] = 3.1

    clearance = avoider.front_corridor_clearance_m(depth)

    assert clearance is not None
    assert 3.0 <= clearance <= 3.2


def test_dense_wall_stops_instead_of_fixed_lateral_bypass():
    avoider = _avoider()
    updated = avoider.update_from_depth(
        front_depth_meters=np.full((32, 32), 5.0, dtype=np.float32),
        observer_world=[0.0, 0.0, 0.0],
        observer_yaw_deg=0.0,
    )
    result = avoider.filter_cumulative_waypoints(
        [[10.0, 0.0, 0.0]],
        current_world=[0.0, 0.0, 0.0],
        yaw_deg=0.0,
        speed_mps=2.0,
        allow_bypass=False,
    )
    assert updated > 0
    assert result.changed
    assert result.reason == "stop_before_continuous_structure"
    assert result.waypoints and result.waypoints[0][0] < 5.0


def test_bounded_facade_uses_validated_contour_when_edge_is_visible():
    avoider = _avoider(
        STRUCTURE_CONTOUR_BYPASS_ENABLED=True,
        STRUCTURE_BYPASS_MAX_CLUSTER_WIDTH_M=6.0,
        STRUCTURE_BYPASS_MAX_LATERAL_M=8.0,
        STRUCTURE_BYPASS_LATERAL_MARGIN_M=1.0,
    )
    depth = np.full((32, 32), 30.0, dtype=np.float32)
    depth[:, 14:18] = 6.0
    updated = avoider.update_from_depth(
        front_depth_meters=depth,
        observer_world=[0.0, 0.0, 0.0],
        observer_yaw_deg=0.0,
    )

    result = avoider.filter_cumulative_waypoints(
        [[12.0, 0.0, 0.0]],
        current_world=[0.0, 0.0, 0.0],
        yaw_deg=0.0,
        speed_mps=2.0,
        allow_bypass=True,
    )

    assert updated > 0
    assert result.changed
    assert result.reason == "depth_structure_contour_bypass"
    assert len(result.waypoints) >= 3
    assert result.waypoints[0][1] != 0.0
    assert result.waypoints[1][0] > result.waypoints[0][0]
    assert result.waypoints[-1] == [12.0, 0.0, 0.0]


def test_collision_record_blocks_reusing_the_contact_direction():
    avoider = _avoider(
        BYPASS_ENABLED=False,
        DYNAMIC_STOP_ENABLED=False,
    )
    avoider.record_collision(
        collision_world=[0.0, 0.0, 0.0],
        collision_yaw_deg=0.0,
        forward_distance_m=1.0,
    )
    result = avoider.filter_cumulative_waypoints(
        [[4.0, 0.0, 0.0]],
        current_world=[-2.0, 0.0, 0.0],
        yaw_deg=0.0,
        allow_bypass=False,
    )
    assert result.changed
    assert result.waypoints == []


def test_realtime_mode_can_force_stop_without_bypass():
    avoider = _avoider(BYPASS_ENABLED=True)
    avoider.update_from_depth(
        front_depth_meters=np.full((16, 16), 4.0, dtype=np.float32),
        observer_world=[0.0, 0.0, 0.0],
        observer_yaw_deg=0.0,
    )
    result = avoider.filter_cumulative_waypoints(
        [[8.0, 0.0, 0.0]],
        current_world=[0.0, 0.0, 0.0],
        yaw_deg=0.0,
        allow_bypass=False,
        include_target_keepout=False,
        speed_mps=2.0,
    )
    assert result.changed
    assert "stop_before" in result.reason


def test_lateral_bypass_uses_clearance_corridor_before_rejoining_route():
    avoider = _avoider(
        CONSERVATIVE_STRUCTURE_MODE=False,
        BYPASS_ENABLED=True,
        BYPASS_LATERAL_M=3.0,
        BYPASS_FORWARD_M=2.5,
    )
    avoider.update_from_depth(
        front_depth_meters=np.full((16, 16), 4.0, dtype=np.float32),
        observer_world=[0.0, 0.0, 0.0],
        observer_yaw_deg=0.0,
    )
    # Keep one clear lateral side available so the candidate is validated.
    avoider.obstacle_cells = {
        key: cell for key, cell in avoider.obstacle_cells.items()
        if float(cell.center_world[1]) <= 0.7
    }
    result = avoider.filter_cumulative_waypoints(
        [[10.0, 0.0, 0.0], [18.0, 0.0, 0.0]],
        current_world=[0.0, 0.0, 0.0],
        yaw_deg=0.0,
        allow_bypass=True,
        include_target_keepout=False,
    )
    assert result.changed
    assert result.reason == "depth_lateral_bypass"
    assert len(result.waypoints) >= 3
    assert result.waypoints[0][1] != 0.0
    assert result.waypoints[1][1] == result.waypoints[0][1]
    assert result.waypoints[-1] == [18.0, 0.0, 0.0]
    rejoin = result.details["rejoin_body"]
    align = result.details["align_body"]
    assert rejoin[1] == 0.0
    assert align[1] == 0.0
    assert align[0] > rejoin[0]


def test_ground_strip_below_vehicle_is_not_treated_as_a_wall():
    avoider = _avoider(VERTICAL_CLEARANCE_DOWN_M=0.75)
    path = [[10.0, 0.0, 0.0]]

    ground_hit = avoider._first_collision(path, [[5.0, 0.0, 1.30]])
    wall_hit = avoider._first_collision(path, [[5.0, 0.0, 0.10]])
    target_keepout_hit = avoider._first_collision(
        path,
        [[5.0, 0.0, 1.30]],
        vertical_clearance=2.2,
        source="target_keepout",
    )

    assert ground_hit is None
    assert wall_hit is not None
    # Explicit target clearance remains symmetric and is unaffected by the
    # raw-depth ground filter.
    assert target_keepout_hit is not None


def test_enclosed_sparse_gap_is_not_treated_as_free_corridor():
    avoider = _avoider(
        CONSERVATIVE_STRUCTURE_MODE=False,
        ENCLOSED_GAP_DETECTION_ENABLED=True,
        ENCLOSED_GAP_MAX_WIDTH_M=5.0,
        SAFETY_RADIUS_M=1.0,
    )
    # Two near leaf/branch bands leave a deceptively far center ray.  The
    # center path has insufficient clearance to be a safe corridor.
    obstacles = [
        [4.5, -2.2, 0.0],
        [5.0, -2.0, 0.2],
        [4.8, 2.1, 0.0],
        [5.2, 2.0, 0.2],
    ]
    hit = avoider._first_collision(
        [[12.0, 0.0, 0.0]],
        obstacles,
        max_check_distance_m=12.0,
    )
    assert hit is not None
    assert hit.source == "enclosed_gap"


def test_volumetric_cluster_disables_contour_bypass():
    avoider = _avoider(
        CONSERVATIVE_STRUCTURE_MODE=True,
        STRUCTURE_CONTOUR_BYPASS_ENABLED=True,
        VOLUMETRIC_OBSTACLE_CONSERVATIVE=True,
        VOLUMETRIC_CLUSTER_MIN_CELLS=4,
    )
    obstacles = [
        [3.5, -2.0, -0.8], [5.0, -1.7, 0.0], [6.0, -1.3, 0.8],
        [3.5, 2.0, -0.8], [5.0, 1.7, 0.0], [6.0, 1.3, 0.8],
    ]
    hit = avoider._first_collision([[12.0, 0.0, 0.0]], obstacles, max_check_distance_m=12.0)
    assert hit is not None
    assert avoider._looks_like_volumetric_obstacle(hit, obstacles)
