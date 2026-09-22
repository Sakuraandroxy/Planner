import json

from planner.domain.observation import Observation


class NavigationTrajectoryPrompt:
    def build(self, instruction: str, observation: Observation, max_points: int) -> str:
        camera = {
            "camera_id": observation.camera_id,
            "horizontal_fov_deg": round(observation.intrinsics.horizontal_fov_deg, 3),
            "camera_position_world": [round(value, 4) for value in observation.camera_position_world],
            "rotation_camera_to_world": [
                [round(value, 6) for value in row]
                for row in observation.rotation_camera_to_world
            ],
        }
        return "\n".join([
            "Visual inputs: image 1 is RGB. Image 2 is aligned depth; brighter means nearer, darker gray means farther, and black means invalid depth.",
            f"Instruction: {instruction.strip()}",
            "Camera geometry (JSON): " + json.dumps(camera, separators=(",", ":")),
            f"Output 1 to {max_points} incremental pose waypoints as a JSON list. Use fewer points for simple motions.",
            "Each waypoint must be [dx, dy, dz, dyaw_deg] relative to the previous predicted pose; the first is relative to the current vehicle pose.",
            "+dx is forward and +dy is right in the previous pose horizontal body frame. AirSim NED +dz is down. Positive dyaw_deg turns right.",
            "Z is not altitude: positive dz DESCENDS, negative dz ASCENDS, and dz=0 maintains altitude. Camera pitch does not change these signs.",
            "Translation and viewing direction are independent. Do not infer yaw from translation.",
            "For explicit motion distances, preserve the requested total displacement. Ascending 12 meters with no rotation is [[0,0,-12,0]].",
            "Descending 12 meters with no rotation is [[0,0,12,0]].",
            "Do not output any other text.",
        ])


class NavigationProgressPrompt:
    def build(self, instruction: str, initial: Observation, current: Observation,
              round_index: int) -> str:
        return (
            "Review navigation progress. Images 1/2 are INITIAL RGB/depth; "
            "images 3/4 are CURRENT RGB/depth. Depth is brighter nearer. "
            f"Original task: {instruction}\nInitial vehicle pose: {initial.vehicle_pose}\n"
            f"Current vehicle pose: {current.vehicle_pose}\nExecuted batches: {round_index}\n"
            "Coordinates: world NED, positive Z DOWN, negative Z UP; yaw degrees, positive right. "
            "Use measured pose differences for explicit move/rotation commands (position tolerance 1m, yaw 2 degrees). "
            "Do not repeat an already completed relative command. For visual targets, completion requires "
            "evidence of reaching the requested location, not merely finishing a trajectory. "
            "Above a house requires horizontal overlap AND altitude above its roof; climbing alone is insufficient. "
            "Unless a height offset is specified, do not require a particular roof clearance. "
            "Do not claim success when the target is out of view without sufficient geometric evidence. "
            "If target identity or position cannot be established, report blocked rather than inventing a location. "
            "Return JSON ONLY: {\"status\":\"complete|continue|blocked\",\"reason\":\"evidence\","
            "\"next_instruction\":\"remaining action from CURRENT pose, not original repeated motion\"}."
        )
