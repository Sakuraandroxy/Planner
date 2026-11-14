# 仓库指南

## 项目结构与模块组织

Uni-LaViRA 按运行目标组织代码。`sim-code/habitat/` 包含 VLN、ObjectNav 和 EQA 的室内 Habitat 评测，任务配置位于 `habitat_extensions/config/`，评测脚本位于 `eval_scripts/`。`sim-code/airsim/` 包含基于 TravelUAV/AirSim 的空中 VLN 评测，核心文件是 `unilavira_evaluator.py` 和 `scripts/unilavira_eval.sh`。真实机器人部署代码位于 `real-world-code/`：`cobot_magic/`、`unitree_go1/` 和 `unitree_g1/` 共享 `main.py`、`config.py`、`ai_client/`、`robot/`、`tasks/`、`iplanner/`、`web/` 和 `tests/` 等 Python 模块；`self_built_uav/` 采用 ROS/catkin 工作空间结构，包含 `model_set_node/` 和 `vln_node/`。共享图片和论文资源位于 `assets/`。

## 构建、测试与开发命令

每个目标都有独立环境。对于地面机器人，请在对应机器人目录中运行：

```bash
conda create -n unitree_g1 python=3.10 && conda activate unitree_g1
pip install -r requirements.txt
python main.py --task object_nav --instruction "chair"
pytest tests
```

对于 Habitat，安装 `sim-code/habitat/requirements.txt`，然后运行任务脚本，例如 `bash eval_scripts/vlnce_r2r.sh`。对于 AirSim，先配置 `.env.example` 中的变量，检查 `scripts/unilavira_eval.sh` 里的本地路径，再运行 `bash scripts/unilavira_eval.sh`。对于 `self_built_uav`，将包放入 catkin 工作空间，使用 `git submodule update --init --recursive` 拉取子模块，构建 ROS 依赖，然后按文档启动 ROS 节点。

## 编码风格与命名约定

使用 Python 3.10 风格，缩进为四个空格。模块和函数使用 `snake_case`，类使用 `PascalCase`，环境常量使用大写命名。平台相关逻辑应放在对应的 `robot/` 或 ROS 包中，任务行为应放在 `tasks/` 中。优先通过 `config.py`、YAML 文件或环境变量配置，不要硬编码本地路径。

## 测试指南

机器人目录中的轻量级测试位于 `tests/`。新增测试文件命名为 `test_*.py`，并在对应目标目录中使用 `pytest tests` 运行。建议为 API 客户端、几何变换、任务选择和配置解析添加聚焦测试。硬件、ROS、仿真器和模型服务器相关测试应说明所需设备、topic、checkpoint 和凭据。

## 提交与 Pull Request 指南

当前工作区没有可用的 Git 历史，因此请使用简洁的祈使式提交信息，例如 `Add Unitree G1 camera validation`。Pull Request 应说明受影响的目标，列出环境配置或 checkpoint 假设，包含测试输出；如果修改了 Web、仿真器或机器人行为，还应附上截图或日志。如有相关 issue 或实验记录，请一并链接。

## 安全与配置提示

不要提交 API key、`.env` 文件、模型 checkpoint、证书或机器人网络密钥。请将 `LA_*`、`VA_*`、`FLASK_SECRET_KEY`、设备序列号和机器人 IP 保存在本地环境文件中。启动 `model_set_node` 时务必谨慎：UAV README 中说明该节点可能导致飞行器自动起飞。
