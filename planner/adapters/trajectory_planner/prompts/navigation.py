from planner.domain.observation import Observation
from planner.domain.pose import WorldPose
from planner.domain.trajectory import RelativeTrajectory

from .base import BASE_TRAJECTORY_SYSTEM_PROMPT, build_base_user_prompt


# Navigation currently adopts the complete base prompt. Keep a task-level
# constant so future navigation experiments can extend or replace it without
# changing the shared baseline used by other task prompt builders.
NAVIGATION_TRAJECTORY_SYSTEM_PROMPT = BASE_TRAJECTORY_SYSTEM_PROMPT


class NavigationTrajectoryPrompt:
    def system_prompt(self) -> str:
        return NAVIGATION_TRAJECTORY_SYSTEM_PROMPT

    def build_user_prompt(
        self,
        instruction: str,
        observation: Observation,
        max_points: int,
        *,
        current_guidance: str | None = None,
        round_index: int = 1,
        initial_pose: WorldPose | None = None,
        previous_trajectory: RelativeTrajectory | None = None,
    ) -> str:
        return build_base_user_prompt(
            instruction,
            observation,
            max_points,
            current_guidance=current_guidance,
            round_index=round_index,
            initial_pose=initial_pose,
            previous_trajectory=previous_trajectory,
        )

'''
任务阶段完成判断提示词，每5个点执行后会使用，当前也是发送给规划器模型的，待修改
如果后续要不使用大模型判断任务是否完成要删去这
如果后续继续使用大模型判断任务是否完成则需要根据不同的任务设计不同的提示词
'''
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
