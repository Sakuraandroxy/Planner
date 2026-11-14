"""Completion-only messages; never used by the trajectory planner."""
import json
from planner.domain.depth import depth_display_description
from planner.domain.visual_goal_completion import VisualGoalCompletionInput
from planner.adapters.model_visual_inputs import aligned_depth_image
from planner.adapters.task_completion.prompt import CompletionMessages
from planner.capabilities.completion.visual_goal_request import require_visual_completion_evidence


def _binding_evidence(request: VisualGoalCompletionInput) -> str:
    """Stage binding and detector claims are labelled; neither is ground truth."""
    target_state = request.target_state
    lines = ["- 本阶段已建立目标绑定；当前画面不必独立重新识别完整目标。" if target_state else
             "- 本次目标关联来自当前检测观测，未提供本阶段的位置绑定。"]
    evidence = request.target_observation
    observation = request.observation
    if (evidence is None or evidence.stage_id != request.stage_id
            or evidence.timestamp_ns > observation.timestamp_ns
            or evidence.captured_monotonic > observation.captured_monotonic):
        lines.append("- 未提供可对应本阶段的近期检测结果；当前停稳图像仍可结合已有绑定判断。")
        return '\n'.join(lines)
    encode = lambda value: json.dumps(value, ensure_ascii=False, allow_nan=False)
    goal = evidence.goal
    lines.extend((f"- 最近检测来源观测ID：{evidence.capture_id}；"
                  f"距当前观测{observation.captured_monotonic-evidence.captured_monotonic:.3f}秒。",
        f"- 最近检测与本次停稳图像是否同一观测：{str(evidence.capture_id == observation.capture_id).lower()}。",
        f"- 检测器报告状态：{goal.status.value}；声称同一目标：{str(goal.same_target).lower()}。",
        f"- 检测器报告描述：{goal.description}；关联来源：{goal.association_source}。",
        f"- 检测器报告依据：{goal.evidence}",
        f"- 该检测观测中的归一化目标框：{encode(goal.bbox)}；仅属于该观测，不能直接套用到其他图像。"))
    if goal.surface_points is not None:
        lines.append(f"- 该检测观测中模型选择的表面像素：{encode(goal.surface_points)}；不是完整目标边界。")
    if evidence.position_update_rejection is not None:
        lines.append(f"- 该次坐标更新未接受：{evidence.position_update_rejection}；继续使用原绑定位置。"
                     "该检测仅为待核实的辅助证据，不能当作已确认同一目标；坐标偏移也不直接证明目标已改变。")
    else:
        lines.append("- 该次位置估计已被本地接受；这不等于任务已完成。")
    return '\n'.join(lines)


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
        return ("你是视觉目标导航任务完成判断器。综合本阶段目标绑定、当前观测和本地测量判断指定阶段。"
                "接近目标时允许局部、背面及近距离表面视图，不要求当前画面独立重新识别完整目标。"
                "全部必需条件满足才能返回true；不满足或无法确认返回false。"
                "不把未执行轨迹当成完成证据。只输出小写true或false，禁止解释或代码块。")

    @staticmethod
    def build_user_prompt(request: VisualGoalCompletionInput):
        parameters, geometry, observation = request.parameters, request.geometry, request.observation
        pose = observation.pose
        radius_met = geometry.distance_m+request.uncertainty_margin_m <= request.rules.radius_m
        height_met = (request.height_tolerance_m is None or
                      abs(pose.z-request.initial_pose.z) <= request.height_tolerance_m)
        height_description = ("未额外限制相对起点高度。"
            if request.height_tolerance_m is None else
            f"相对阶段起点的世界z变化不超过{request.height_tolerance_m}米。")
        reference_input = "- 图片3：首次绑定的目标参考图，只用于确认同一目标。" if request.reference_rgb is not None else ""
        required = "\n".join("- " + item for item in request.rules.required_conditions)
        encode = lambda value: json.dumps(value, ensure_ascii=False, allow_nan=False)
        target_state = request.target_state
        source = ('本阶段已绑定的表面点深度估计；不是当前重新测得的目标表面距离。'
                  if target_state else '当前提供表面像素对应的原始深度反投影估计，不是物体真实三维中心。')
        source += f' 采样来源：{geometry.position_source}。'
        direction_reference = '原始方向描述相对阶段起始视角；已绑定目标不得按当前画面重新排序。'
        reference_rule = ('首次参考图是已有绑定的辅助依据，允许与当前局部或背面视图不同。' if request.reference_rgb is not None else
                          '未提供本阶段参考图；结合已提供的目标关联、描述、位置及当前观测判断，不得假设存在参考图。')
        distance_description = '到本阶段已绑定表面点集合的最小水平距离' if target_state else '到当前所选表面点的水平距离'
        local_checks = ('停稳、到本阶段表面点集合的水平距离和已配置高度条件已通过；综合已有绑定及当前观测判断任务要求，不必重新证明完整外形。'
                        if target_state else '当前表面点距离上限、目标关联和稳定条件已通过本地检查。')
        position_evidence = (f'- 首次锁定世界坐标：{encode(target_state.anchor_world_point)}\n'
            f'- 已绑定目标表面点集合：{encode(target_state.known_surfaces)}\n'
            f'- 位置来源观测ID：{target_state.capture_id}\n'
            f'- 位置估计距当前观测的年龄：{observation.captured_monotonic-target_state.captured_monotonic:.3f}秒。'
            if target_state else f'- 本次关联目标框：{encode(request.goal.bbox)}')
        return f"""请判断当前导航阶段是否已经完成。

视觉输入：
- 图片1：车辆暂停轨迹并定点保持后采集的当前前视RGB图像，允许观测容差内的小幅残余漂移。
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
- {direction_reference}

本阶段目标绑定与当前观测：
{_binding_evidence(request)}

本地触发规则：
- 触发方式：目标半径触发暂停。
- 距离计算方式：当前保持位姿到已绑定表面点集合的最小水平距离，只使用世界x、y；高度差不计入半径。目标坐标是已知可见表面估计，不代表完整物体边界。
- 急停触发半径：{request.rules.effective_pause_radius_m}米；用于触发暂停，与停稳后的完成距离上限分开。
- 完成距离上限：{request.rules.radius_m}米；圈内位置均可，不要求停在边界。
- 测距不确定性余量：{request.uncertainty_margin_m}米；距离加余量不得超过半径。
- 高度约束：{height_description}

停稳后的本地测量结果：
- 当前车辆世界坐标：{encode([pose.x, pose.y, pose.z])}
- 当前车辆偏航角：{pose.yaw_deg}度。
- {distance_description}：{geometry.distance_m}米。
- 完成距离上限条件满足：{str(radius_met).lower()}
- 高度约束满足：{str(height_met).lower()}
- 本地检查：{local_checks}
- 独立新鲜观测确认次数：{request.confirmation_count}/{request.required_confirmation_count}。

本阶段的全部必需完成条件：
{required}

针对当前导航任务的判断说明：
- 对接近、到附近类任务，可延续本阶段已建立的目标身份，不必每次从当前画面重新识别完整目标。不得忽略当前证据与原目标的明确矛盾。
- 进入本阶段绑定点半径是距离依据，不是单独的完成证明；{reference_rule}
- 允许目标只露出局部、背面、车尾或近距离墙面、窗户；缺少完整轮廓本身不得作为返回false的理由。
- 建筑等大型目标靠近后画面被局部表面占据，可结合已有绑定及位置记录判断；不得仅凭任意墙面或相似物体就认定是指定目标。
- 若任务要求保持目标在视野内，当前图像中有可结合绑定证据关联的目标部分即可，不要求完整外形或居中；目标完全不可见则不满足该要求。
- 以本地测量结果判断明确配置的距离和高度条件。
- 如果原始任务明确要求看清某个部位、门牌、入口或目标全景，必须确认该项观察要求已经满足，不能用其他局部视图代替。
- 如果任务没有要求额外观察，不得自行添加观察要求。
- 圈内位置无需再靠近半径边界；仍须确认目标及全部必需条件。
- 不得把尚未执行的轨迹或预测动作作为完成依据。
- 任意必需条件未满足或无法确认，返回false。

只输出true或false。"""