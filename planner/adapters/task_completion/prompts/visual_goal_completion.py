"""Completion-only messages; never used by the trajectory planner."""
import json
from planner.domain.depth import depth_display_description
from planner.domain.visual_goal_completion import VisualGoalCompletionInput
from planner.adapters.model_visual_inputs import aligned_depth_image
from planner.adapters.task_completion.prompt import CompletionMessages
from planner.capabilities.completion.visual_goal_request import require_visual_completion_evidence


class VisualGoalCompletionPrompt:
    @staticmethod
    def prepare(request: VisualGoalCompletionInput) -> CompletionMessages:
        require_visual_completion_evidence(request)
        images = ((request.observation.rgb, "JPEG"), (aligned_depth_image(request.observation), "PNG"))
        if request.reference_rgb is not None:
            images += ((request.reference_rgb, "JPEG"),)
        return CompletionMessages(VisualGoalCompletionPrompt.system_prompt(),
            VisualGoalCompletionPrompt.build_user_prompt(request), images)

    @staticmethod
    def system_prompt():
        return ("你是无人机任务完成判断器。只依据当前观测和本地测量判断指定阶段。"
                "全部必需条件满足才能返回true；不满足或无法确认返回false。"
                "不把未执行轨迹当成完成证据。只输出小写true或false，禁止解释或代码块。")

    @staticmethod
    def build_user_prompt(request: VisualGoalCompletionInput):
        parameters, geometry, observation = request.parameters, request.geometry, request.observation
        pose = observation.vehicle_pose
        radius_met = geometry.distance_m+request.uncertainty_margin_m <= request.rules.radius_m
        height_met = (request.height_tolerance_m is None or
                      abs(pose.z-request.initial_pose.z) <= request.height_tolerance_m)
        height_description = ("未额外限制相对起点高度。"
            if request.height_tolerance_m is None else
            f"相对阶段起点的NED z变化不超过{request.height_tolerance_m}米。")
        reference_input = "- 图片3：首次绑定的目标参考图，只用于确认同一目标。" if request.reference_rgb is not None else ""
        required = "\n".join("- " + item for item in request.rules.required_conditions)
        encode = lambda value: json.dumps(value, ensure_ascii=False, allow_nan=False)
        memory = request.target_memory
        source = ('本阶段记忆中的已接受框中心深度估计；不是当前重新测得的目标表面距离。'
                  if memory else '当前检测框中心对应的原始深度反投影估计，不是物体真实三维中心。')
        distance_description = '到记忆目标估计点的距离' if memory else '到当前框中心深度位置估计的距离'
        local_checks = ('停稳、记忆距离和已配置高度条件已通过；原目标身份及当前可见性尚待你判断。'
                        if memory else '当前框中心距离上限、目标关联和稳定条件已通过本地检查。')
        position_evidence = (f'- 首次锁定世界坐标：{encode(memory.anchor_world_point)}\n'
            f'- 位置来源观测ID：{memory.capture_id}\n'
            f'- 位置估计距当前观测的年龄：{observation.captured_monotonic-memory.captured_monotonic:.3f}秒。'
            if memory else f'- 本次关联目标框：{encode(request.goal.bbox)}')
        return f"""请判断当前导航阶段是否已经完成。

视觉输入：
- 图片1：无人机暂停飞行并悬停稳定后采集的当前前视RGB图像。
- 图片2：与当前RGB对应的前视深度可视化图像。
{reference_input}
- 深度显示协议：{depth_display_description(observation.depth_mapping, observation.depth_min_m, observation.depth_max_m)}
- 深度可视化仅用于辅助理解场景。数值距离的来源和局限见下文。

当前阶段：
- 阶段编号：{request.stage_id}
- 任务类型：visual_goal_navigation

当前阶段的原始任务指令：
{parameters.instruction}

本阶段指定目标：
- 目标描述：{parameters.target_description}
- 目标引用：{request.reference_description}
- 目标关联证据来源：{request.goal.association_source}。
{position_evidence}
- 目标世界坐标：{encode(geometry.world_point)}
- 目标位置来源：{source}
- 任务要求保持原目标在前视画面内：{str(parameters.keep_target_in_view).lower()}。
- 阶段起始位姿[world x,y,z,yaw_deg]：{encode([request.initial_pose.x, request.initial_pose.y, request.initial_pose.z, request.initial_pose.yaw_deg])}
- 原始方向描述相对阶段起始视角；已绑定目标不得按当前画面重新排序。

本地触发规则：
- 触发方式：目标半径触发暂停。
- 距离计算方式：当前停稳位姿到上述目标估计点的三维距离，位置来源不可混淆。
- 判定半径：{request.rules.radius_m}米。
- 完成距离上限：{request.rules.radius_m}米；圈内位置均可，不要求停在边界。
- 测距不确定性余量：{request.uncertainty_margin_m}米；距离加余量不得超过半径。
- 高度约束：{height_description}

悬停稳定后的本地测量结果：
- 当前无人机世界坐标：{encode([pose.x, pose.y, pose.z])}
- 当前无人机偏航角：{pose.yaw_deg}度。
- {distance_description}：{geometry.distance_m}米。
- 判定半径条件满足：{str(radius_met).lower()}
- 高度约束满足：{str(height_met).lower()}
- 本地检查：{local_checks}
- 独立新鲜观测确认次数：{request.confirmation_count}/{request.required_confirmation_count}。

本阶段的全部必需完成条件：
{required}

针对当前导航任务的判断说明：
- 确认目标关联符合原始任务，不得把其他相似物体当成指定目标。
- 进入记忆点半径不能证明已到达原目标附近；必须结合当前图像与首次参考图确认。
- 若任务要求保持目标在视野内，确认当前图像中原目标可见；不额外要求目标居中。
- 以本地测量结果判断明确配置的距离和高度条件。
- 如果任务要求观察某个部位或完成某种观察，必须确认相应要求已经满足。
- 如果任务没有要求额外观察，不得自行添加观察要求。
- 圈内位置无需再靠近半径边界；仍须确认目标及全部必需条件。
- 不得把尚未执行的轨迹或预测动作作为完成依据。
- 任意必需条件未满足或无法确认，返回false。

只输出true或false。"""
