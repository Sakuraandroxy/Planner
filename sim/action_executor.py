"""Action execution: convert VLM trajectory into AirSim API calls."""
import math, time
from .airsim_client import AirSimClient
from planner.trajectory import parse_action


def actions_to_waypoints(actions, start_pos, start_yaw_deg, config, scale=1.0):
    """Convert atomic actions into a list of absolute waypoints [x,y,z]."""
    x, y, z = start_pos
    yaw = start_yaw_deg
    wps = []
    for a in actions:
        name, val = parse_action(a, config, scale)
        if name == "forward":
            r = math.radians(yaw)
            x += val * math.cos(r)
            y += val * math.sin(r)
        elif name == "backward":
            r = math.radians(yaw)
            x -= val * math.cos(r)
            y -= val * math.sin(r)
        elif name == "left":
            yaw -= val
        elif name == "right":
            yaw += val
        elif name == "up":
            z -= val
        elif name == "down":
            z += val
        wps.append([round(x, 3), round(y, 3), round(z, 3)])
    return wps


def group_consecutive_actions(actions):
    """Group consecutive same actions e.g. [f,f,l,f] -> [("f",2), ("l",1), ("f",1)]."""
    if not actions:
        return []
    groups = []
    cur = actions[0]
    cnt = 1
    for a in actions[1:]:
        if a == cur:
            cnt += 1
        else:
            groups.append((cur, cnt))
            cur = a
            cnt = 1
    groups.append((cur, cnt))
    return groups


class ActionExecutor:
    """Executes grouped VLM trajectories via AirSim API."""

    def __init__(self, client: AirSimClient, config):
        self.client = client
        self.config = config

    def execute(self, actions, scale=1.0):
        """Execute a sequence of atomic actions in place in AirSim.
        Returns (final_pos, final_yaw, collided).
        """
        pos, yaw = self.client.get_pose()
        waypoints = actions_to_waypoints(actions, pos, yaw, self.config, scale=scale)
        groups = group_consecutive_actions(actions)

        wp_idx = 0
        pos_cur = list(pos)
        yaw_cur = yaw

        for action, count in groups:
            action_name, _ = parse_action(action, self.config, scale)
            if action_name in ("left", "right"):
                sign = -1 if action_name == "left" else 1
                action_name, action_val = parse_action(action, self.config, scale)
                yaw_cur += sign * action_val * count
                self.client.rotate_to_yaw(yaw_cur)
                time.sleep(0.5)
            else:
                target_wp = waypoints[wp_idx + count - 1]
                d = math.sqrt(
                    (target_wp[0] - pos_cur[0])**2 +
                    (target_wp[1] - pos_cur[1])**2 +
                    (target_wp[2] - pos_cur[2])**2
                )
                if d > 0.001:
                    timeout = max(5.0, d * 3)
                    self.client.move_to_position(
                        target_wp[0], target_wp[1], target_wp[2],
                        velocity=self.config.velocity, timeout=timeout
                    )
                    pos_cur = list(target_wp)
                time.sleep(self.config.capture_interval)
            wp_idx += count

        return self.client.get_pose() + (self.client.check_collision(),)

    def back_up(self, pos, yaw):
        """Collision recovery: ascend 2m, then back up 5m, then stay at safe z."""
        safe_z = min(pos[2], -2.0)  # ascend to at least z=-2
        if pos[2] > -1.5:
            print(f"  [COLLISION] Ascending from z={pos[2]:.2f} to z={safe_z:.2f}")
            self.client.move_to_position(pos[0], pos[1], safe_z, timeout=5.0)
            time.sleep(0.5)
        bx = pos[0] - self.config.horizontal_step * math.cos(math.radians(yaw))
        by = pos[1] - self.config.horizontal_step * math.sin(math.radians(yaw))
        self.client.move_to_position(bx, by, safe_z, timeout=5.0)
        time.sleep(0.5)
