# AVLN 论文方法与运行时间调研

> 调研范围：阅读当前项目 `AVLN/` 目录下全部 25 篇论文。  
> 说明：表中“运行时间”只记录论文中明确报告的 wall-clock latency、FPS、Hz、每步耗时、离线生成耗时或硬件计算成本；未给出具体秒级指标的论文标注为“未报告”。

## 一、核心结论

1. **你当前项目慢是符合文献现象的**：Uni-LaViRA 明确报告单次 Language Action API 调用健康时为 `30-60s`，拥塞时为 `80-150s`，本地 planner/controller 反而都在 `10ms` 内。因此瓶颈不是 AirSim 或前端，而是在线大模型 API 的网络、prefill、reasoning token 和解码。
2. **演示系统不适合每步都等大型云端 VLM 深度思考**：OnFly、AirDreamer、OA-VAT 等实际可飞系统都把高频控制放到本地小模型/策略/控制器中，大模型只做低频语义、监控或离线训练。
3. **最直接可借鉴路线**：保留 VLM 的语义能力，但把闭环拆成“低频 VLM 决策 + 高频本地执行”。具体包括关闭/减少 reasoning 输出、限制 JSON、合并感知 RPC、增大单次轨迹长度、缓存视觉特征/历史、用本地安全约束或轨迹执行器代替频繁重新询问 VLM。
4. **论文中的快方法通常不是“更会想”，而是“少调用大模型”**：AirDreamer 训练后在 Jetson 上最高 `88Hz` 推理；OnFly 在 Jetson Orin NX 上 decision 分支 `0.81s/step`；OA-VAT 在 RTX 3090 上 `35FPS`。

## 二、运行时间对比总表

| 论文 | 类型 | 核心方法 | 文中报告的运行时间/速度 | 对本项目的启发 |
|---|---|---|---|---|
| AeroScene | 场景生成 | 层级扩散模型生成物理可用 3D 场景，支持下游无人机导航训练 | 约 `2min/scene`，NVIDIA A100；推理用 `100` 个 reverse diffusion steps | 适合离线造场景，不适合在线控制 |
| AirDreamer | 学习式导航 | 世界模型 + 强化学习策略，用深度相机和状态估计导航 | 策略 `20Hz` 执行；Jetson Orin NX 最高 `88Hz` 推理；训练 `38h43min`，2×RTX 4090D | 训练后本地高速执行，是演示流畅性的理想形态 |
| CARLA-AIR | 评测平台 | 单进程融合 CARLA + AirSim，实现空地协同闭环评测 | 平台同步偏移 `0ms`；AerialVLA `6.2Hz/160ms`，OpenFly `3.1Hz/330ms`，OpenUAV `1.6Hz/620ms`，SPF `1.1Hz/850ms` | 低延迟协同需要统一时钟和低频模型调用 |
| CICDWOA | 传统路径规划 | 改进 Whale Optimization，加入认知共享、Cauchy 变异等 | CPU/Matlab 环境；论文称比传统 WOA 多 `15%-25%` 计算时间 | 适合离线/全局规划，不适合每帧实时 VLM 替代 |
| CosFly-Track | 数据集/跟踪规划 | MuCO 多约束连续优化生成 UAV 跟踪轨迹 | MuCO `247ms/trajectory`，A* `5.5s`；BVH 从 `~12s` 降到 `0.1-0.5s`；梯度 `2-5ms/iter` | 可借鉴“优化器快速生成多步轨迹”，减少 VLM 频繁规划 |
| Dynamic-TD3 | 强化学习路径规划 | CMDP + ATREM + PAG-KF，面向动态/对抗环境 | 报告飞行时间约 `136.77-171.46s`，未报告单步推理延迟 | 强化学习策略可减少在线推理，但论文缺少实时延迟数据 |
| Evolutionary Biparty MO UAV Path Planning | 多目标优化 | 将效率部门和安全部门建模为 biparty multiobjective planning | 未报告秒级运行时间；给出复杂度和实际时间比例图 | 偏理论和离线规划，对演示提速帮助有限 |
| FineCog-Nav | 零样本 AVLN | 语言、感知、注意、记忆、想象、推理、决策等细粒度认知模块 | AerialVLN-Fine 每步：BaseModel `18.8s`，NavGPT `116.4s`，DiscussNav `80.5s`，FineCog-Nav `23.5s` | 模块化能降成本，但多 API 调用仍然慢 |
| FlyMirage | 数据生成 | LLM 设计场景 + generative world model + 自动标注 + 可飞轨迹规划 | 约 `$2/scene`；RTX 4070 批量渲染；完整流程约 `1h/scene` | 适合离线扩充训练/测试数据，不解决在线等待 |
| HTNav | 训练式 AVLN | IL + RL 混合训练，分层决策，宏观路径规划 + 细粒度控制 | 实验使用 RTX A5000，未报告单步运行时间 | 分层控制思想有用，但需要训练 |
| HUGE-Bench | 高层 UAV VLA benchmark | 3DGS-Mesh 数字孪生，评估高层指令、多阶段、安全轨迹 | 未报告模型运行时间；采样约 `1m` 间隔轨迹帧 | 可用于设计更像真实演示的评测任务 |
| ImagineUAV | 世界模型 AVLN | latent video diffusion 想象未来观测，action extractor 解码 6-DoF，再经 kinodynamic planner 修正 | 蒸馏前 `14.7s`，10-step 蒸馏后 `6.2s`；PRO 6000 上 imagination `3.2s` + extraction `3.0s`；AGX Thor `11.7s` + `9.0s` | 蒸馏/短步世界模型可把一次规划压到几秒级 |
| OA-VAT | 视觉主动跟踪 | 离线 instance prototype + 在线目标匹配 + 遮挡恢复规划 | RTX 3090 上 `35FPS`；比 TrackVLA `10FPS@RTX4090` 快 | 目标跟踪不要每步问 VLM，可用跟踪器维持目标 |
| LookasideVLN | 方向感知 AVLN | Egocentric Lookaside Graph + 地标方向关系 + 轻量记忆检索 + MLLM agent | 未报告秒级运行时间；强调比 lookahead 更省计算 | 任务指令中“左右/前后/经过”可显式结构化，减少模型搜索 |
| OnFly | onboard 零样本 AVLN | 双 Agent：高频目标生成 + 低频进度监控，共享 ViT 特征和 KV-cache，接安全局部规划器 | Jetson Orin NX：Baseline decision `3.83s`、monitoring `7.47s`；Edge-Deploy decision `0.81s`；Hybrid Memory monitoring `1.15s` | 最适合借鉴：解耦高低频、缓存、低频监控、本地执行 |
| Optimal Path Planning in Hostile Environments | 理论路径规划 | 图上多智能体穿越可恢复 hazard，分析 NP-hard 和特殊图多项式算法 | 未报告实际运行时间 | 偏理论，不直接解决演示延迟 |
| RL-STPA | 安全分析 | 将 STPA hazard analysis 改造到 RL 系统，做覆盖引导扰动测试 | 未报告运行时间 | 可用于安全汇报，不用于提速 |
| Wi2SAR | 无线搜救无人机 | 伪装 Wi-Fi 网络发现设备，RSS-only 3D AoA，Luneburg Lens 扩展范围 | Raspberry Pi CM4：单 RSS snapshot `48ms`，7 目标 `7.8Hz`，`635MB` 内存，`48%-52%` CPU | 说明非视觉传感任务可以做到嵌入式实时 |
| Uni-LaViRA | 统一零样本导航 | Language Action + Vision Action + Robot Action 三层转换，TDM 记忆和 SCB 回退 | 单次 LA API 健康 `30-60s`，拥塞 `80-150s`；本地 planner/controller `<10ms` | 与当前项目最像，直接证明慢点来自在线大模型 API |
| ViSA-Enhanced Aerial VLN | 视觉空间推理 AVLN | VPG 视觉标注 + 三阶段 Verification + 语义/运动解耦执行器 | RTX 4090 + Xeon；Qwen3-VL-PLUS online API；未报告单步延迟 | 结构化视觉提示可提升准确率，但在线 API 仍可能慢 |
| UAV-VLN Survey / Roadmap | 综述 | 总结 UAV-VLN 从模块化、深度学习到 foundation-model agent 的演化 | 综述提到早期 DRL 可 `60Hz`、轻量 onboard VLA 可 `25Hz`，但非该文实测 | 可作为汇报背景，说明“实时化”是领域关键问题 |
| Aerial Robots LLM Survey | 综述 | 总结 Aerial VLN 的 AVIN/AVDN、LLM/VLM 方法、数据集、平台和开放问题 | 未给单一方法运行时间；强调机载计算受限 | 支撑“云端大模型难以机载实时部署”的论点 |
| Weather-Robust CVGL / SKYPART | 地理定位 | DINOv2 ViT-S/14 + prototype semantic part discovery，单次 forward 检索 | 单次 `448×448` forward：`26.95M` 参数，`22.14 GFLOPs`；未报告秒级延迟 | 可作为 GPS-denied 定位模块，速度取决于本地模型 |
| WESPR | 风场感知规划 | 快速 CFD/FluidX3D 预测风场 + 风感知轨迹规划 | RTX 3080 总流程约 `10s`：采集 `1s`、仿真 `4s`、规划 `5s`；GTX 1650 `<15s` | 可借鉴“重计算低频运行，轨迹执行高频运行” |
| WorldVLN | 世界动作模型 AVLN | 自回归 latent world-action model，预测短时世界状态并解码 waypoint，Action-aware GRPO | 训练 8×A800；仿真 rollout RTX 4090；真实部署高层推理在地面站，未报告单步延迟 | 世界模型方向强，但目前仍需地面站推理，未达到完全 onboard |

## 三、与“VLM 思考太慢”的直接关系

### 1. 最相似案例：Uni-LaViRA

Uni-LaViRA 的架构和当前项目非常像：在线大模型负责高层语言/视觉决策，本地控制器负责执行。它的耗时分布说明：

- 单次 Language Action 调用：`30-60s`，API 拥塞时 `80-150s`。
- 本地 planner 和 controller：均 `<10ms`。
- 主要计算集中在 LA 的长上下文 prefill 和 reasoning token；VA 与本地控制几乎不是瓶颈。

这和你现在“第一次 1 分多、之后每步 30 秒多”的现象高度一致。换句话说，项目慢不是因为 AirSim 飞得慢，而是因为每步都在等大模型“想完”。

### 2. 最高价值案例：OnFly

OnFly 专门解决零样本 AVLN 的实时部署问题，它的设计很适合改造当前项目：

- **高频 decision agent**：负责实时输出目标点。
- **低频 monitoring agent**：负责判断任务进度、是否停止、是否恢复。
- **共享 ViT 特征 + 独立 KV-cache**：避免重复视觉编码，减少上下文变化导致的 cache 失效。
- **Hybrid Memory**：固定前缀、保留关键帧，只把最新帧作为短上下文更新。
- **安全局部规划器**：把 VLM 目标点变成可执行轨迹，不让 VLM 每次直接管低层动作。

它在 Jetson Orin NX 上把 decision 从 `3.83s` 降到 `0.81s`，monitoring 从 `7.47s` 降到 `1.15s`。这说明真正有效的提速不是“让大模型更努力”，而是让大模型更少参与高频闭环。

### 3. 当前项目可直接采用的方向

| 优先级 | 优化方向 | 原因 | 对演示效果 |
|---|---|---|---|
| 最高 | 关闭/减少 reasoning 输出，只保留短 JSON | Uni-LaViRA 显示 reasoning token 是明显成本 | 立刻减少等待 |
| 最高 | 单次 VLM 输出更长轨迹，本地连续执行 | 避免每前进一步就重新问模型 | 观众不会频繁看到“思考中” |
| 高 | bbox/深度/轨迹合并或缓存 | 当前每步多次视觉调用会放大 API 延迟 | 降低每步总耗时 |
| 高 | 分离“目标跟踪”和“任务重规划” | OA-VAT/OnFly 都说明跟踪可本地实时做 | 到达目标附近不易因单帧误检跑偏 |
| 中 | 增加低频监控机制 | 每 N 秒确认一次是否偏离/完成，而不是每个动作都问 | 兼顾稳定和速度 |
| 中 | 对演示场景预热 API | 避免第一次 cold start 被观众看到 | 演示观感明显改善 |
| 长期 | 训练/蒸馏本地小模型 | AirDreamer/ImagineUAV/OnFly 都走这条路 | 从几十秒降到秒级/Hz 级 |

## 四、按论文的简要方法说明

### AeroScene

提出面向空中机器人的 progressive scene synthesis。核心是层级扩散模型：用 hierarchy-aware tokenization 表示粗到细的场景结构，用 multi-branch feature extraction 和 cross-scale progressive attention 同时保持全局布局和局部物理可行性。它用于离线生成 Isaac Sim 可用场景，不是在线导航器。

### AirDreamer

使用 world model 学习环境潜在动态，再用强化学习策略在潜空间中决策。训练完成后部署为本地 ONNX policy，仅依赖深度相机和状态估计。它证明了：如果能把“理解环境”压进本地策略，飞行控制可以达到 `20Hz`，Jetson 推理可到 `88Hz`。

### CARLA-AIR

不是导航算法，而是空地协同评测平台。它把 CARLA 与 AirSim 放入同一个 Unreal Engine runtime，共享物理 tick 和传感器管线，避免 ROS bridge 的时钟抖动。对本项目的启发是：系统级延迟需要拆开量，不要把仿真同步、模型推理、控制执行混在一起看。

### CICDWOA

传统启发式优化路径规划。通过 collective cognitive sharing、Cauchy inverse cumulative distribution、DE mutation 等增强 WOA 的全局搜索和收敛稳定性。适合全局路径规划或离线算轨迹，不适合替代每步 VLM 语义决策。

### CosFly-Track

面向 UAV 视觉跟踪的数据集和轨迹生成方法。核心 MuCO 用连续优化同时约束目标可见性、距离、碰撞、平滑性和动力学可行性，BVH 加速可见性查询。其 `247ms/trajectory` 的规划速度说明：一旦目标已知，轨迹生成可以由优化器快速完成，不必每步问 VLM。

### Dynamic-TD3

面向动态/对抗环境的强化学习路径规划框架。它把导航建模为 CMDP，引入 ATREM 预测动态障碍意图，PAG-KF 降噪并融合状态。论文报告的是飞行任务时间而非推理耗时，说明它更偏策略性能评估。

### Evolutionary Biparty Multiobjective UAV Path Planning

把 UAV 路径规划拆成效率决策者和安全决策者，提出 biparty multiobjective optimization。重点是多目标建模和算法比较，不是在线 VLM 导航。

### FineCog-Nav

零样本 AVLN 的模块化认知框架，把导航拆成语言解析、感知、注意、记忆、想象、推理和决策。它比 NavGPT/DiscussNav 快很多，但每步仍 `23.5s`，说明“多模块 prompting”能省 token，但只要仍依赖在线 LLM/VLM，就很难达到真正实时。

### FlyMirage

自动生成 UAV VLN 数据：LLM 生成场景描述，generative world model 生成 3DGS 场景，自动标注目标，最后用可行轨迹规划器收集轨迹。适合构造训练集和演示场景，不解决在线推理慢。

### HTNav

训练式分层导航框架。用 IL 保持基础导航稳定性，用 RL 增强探索，再用 tiered decision-making 连接宏观路径规划和细粒度动作控制。对本项目的启发是“高层语义”和“低层控制”应分层，而不是全部交给 VLM。

### HUGE-Bench

高层 UAV VLA benchmark。它强调真实指令往往是短、高层、多阶段的，不是逐步航点说明。适合作为项目汇报中的评测背景：你的项目也属于从高层语言目标到可执行轨迹的闭环 VLA。

### ImagineUAV

用 latent video diffusion “想象”未来第一视角画面，再用 action extractor 从未来观测中提取 6-DoF 动作，最后通过 kinodynamic planner 修正为安全轨迹。10-step distillation 把规划时间从 `14.7s` 降到 `6.2s`，说明蒸馏世界模型是中长期提速方向。

### OA-VAT

视觉主动跟踪方法。先用 DINO 等特征离线构建目标 prototype，在线通过 prototype matching 跟踪目标，遮挡时用 occlusion-aware planner 主动恢复。它说明：一旦 VLM 找到目标，后续不必每步再问 VLM，可用跟踪器保持目标。

### LookasideVLN

提出方向感知的 lookaside path planning。它不沿用长序列 landmark lookahead，而是构建 Egocentric Lookaside Graph，把“左、右、前方、经过”等方向关系显式编码，再用 SLKB 记忆检索辅助 MLLM。对本项目来说，可以把中文任务里的方向词结构化，降低模型搜索成本。

### OnFly

最值得借鉴的在线系统。它明确针对“单流 VLM 决策不稳定、进度监控低频但控制高频、长上下文破坏 KV-cache、短视动作导致 stop-and-go”这些问题，提出双 agent、共享感知、Hybrid Memory 和局部安全规划。它是当前项目优化的主要参考对象。

### Optimal Path Planning in Hostile Environments

理论路径规划论文，研究可恢复 hazard 图中的多 agent 路由问题。偏复杂度分析，不直接给实时系统方案。

### RL-STPA

安全分析框架，把系统理论过程分析 STPA 迁移到强化学习控制系统，用子任务分解和扰动测试发现危险控制行为。可用于项目安全性汇报，但不是提速方法。

### Wi2SAR

无人机无线搜救系统。它通过模拟已知 Wi-Fi 网络诱导手机重连，再用 RSS-only 3D AoA 定位，Luneburg Lens 扩展发现范围。它证明嵌入式实时感知系统可以在 Raspberry Pi 上跑到 `7.8Hz`，但任务类型不是视觉语言导航。

### Uni-LaViRA

三层 agentic navigation：Language Action 负责语言到高层动作，Vision Action 负责视觉 grounding，Robot Action 负责机器人动作执行。TDM 维护待办子目标，SCB 在出错后回退。它的延迟数据非常关键：LA API `30-60s/80-150s`，本地 planner/controller `<10ms`，说明当前项目应优先减少 LA/VLM 调用和 reasoning token。

### ViSA-Enhanced Aerial VLN

用视觉空间推理增强 VLM：VPG 先给图像加结构化视觉标注，Verification Module 做三阶段空间验证，Executor 把语义结果映射到 UAV pose。它提高了 CityNav 成功率，但依赖 Qwen3-VL-PLUS online API，论文未报告延迟，因此演示时仍可能受 API 影响。

### Vision-and-Language Navigation for UAVs: Progress, Challenges, and Roadmap

综述 UAV-VLN 的方法演化：早期模块化、深度学习、VLM/VLA/世界模型、multi-agent/swarm。对汇报最有用的观点是：领域正在从“每步大模型推理”转向“foundation model 语义 + 地图/记忆/世界模型/本地控制”的混合系统。

### Vision-Language Navigation for Aerial Robots: Towards the Era of Large Language Models

综述 Aerial VLN 在 LLM/VLM 时代的任务定义、AVIN/AVDN 交互范式、数据集、仿真平台和开放问题。它强调 onboard computation 是关键瓶颈，支持你汇报中“云端大模型直接闭环不适合演示实时性”的判断。

### Weather-Robust CVGL / SKYPART

GPS-denied 交叉视角定位方法。SKYPART 用 DINOv2 ViT-S/14 和 prototype-based semantic part discovery 做单次 forward 的 drone-satellite matching。它不是 AVLN 规划器，但可作为未来定位模块补充。

### WESPR

风场自适应规划。通过 GPU 加速 LBM/FluidX3D 快速计算局部风场，再做能耗与安全感知路径规划。其 `10-15s` 的低频规划模式说明：耗时较重的物理计算可以低频执行，然后由控制器连续执行轨迹。

### WorldVLN

自回归 world-action model。它不直接从观察回归动作，而是预测短时 latent world transition，再解码 waypoint，并用 Action-aware GRPO 对真实 rollout 后果优化。论文效果强，但真实部署仍依赖地面站高层推理，未给单步延迟，说明该方向还没有完全解决 onboard 实时性。

## 五、对当前项目的建议结论

如果目标是“项目交差演示不要让人等”，建议按如下顺序做：

1. **先做演示级优化**：关闭深度思考/长 reasoning，要求只输出 JSON + 短 summary；启动后先做一次 API 预热；一次规划输出多步动作。
2. **再做结构级优化**：把 VLM 频率降下来，本地连续执行轨迹；只有目标丢失、偏航过大、接近终点、碰撞恢复时才重新调用 VLM。
3. **然后做感知级优化**：第一次 VLM 找目标框，后续用目标跟踪/深度连续估计维持目标，不要每帧重新让 VLM 找车。
4. **最后做长期优化**：对固定演示场景训练或蒸馏一个小模型/策略，让高频控制本地化，VLM 只做低频语义监控。

一句话总结：**AVLN 最新方法不是靠每一步让大模型“想更久”，而是靠结构设计让大模型“少想、低频想、只想关键问题”，把连续飞行交给本地策略、跟踪器和规划器。**
