# Uni-LaViRA UAV VLN — 从零到全流程跑通指南

> 本文档面向完全不了解本项目的开发者。**从头到尾执行所有命令后，你将能：**
> - 在 Windows 笔记本上启动 AirSim 仿真 + Web 闭环
> - 在 Linux 服务器上部署 GroundingDINO / Qwen 全系列 / vLLM API 服务
> - 笔记本 ↔ 服务器联合运行完整闭环
> - 运行开环 / 闭环离线评测，输出 SR / NE / SPL 论文指标
> - 完整的 Qwen2.5-VL LoRA 微调 → 合并 → 部署

**每节只给具体命令和必要说明。所有可迁移配置集中在 `config/default.yaml`：换机器只改这一个文件。**

---

## 零、Python 环境矩阵（必读）

本项目需要 **4 个独立 conda 环境**，各自装不同的包，**严禁混用**。

### 环境总览

```
┌── 笔记本 (Windows) ────────────────────────────────────────┐
│  env: airsim                                               │
│  作用: AirSim 客户端 + Flask Web + 调用远端 API             │
│  依赖: airsim, openai, Pillow, numpy, scipy, PyYAML, Flask │
│  GPU:  不需要（推理在远端）                                  │
└────────────────────────────────────────────────────────────┘

┌── 服务器 (Linux, 4× RTX 3080 Ti) ─────────────────────────┐
│                                                            │
│  env: groundingdino              GPU 2                     │
│  作用: GroundingDINO Flask 目标检测服务 (:8003)             │
│  依赖: torch, groundingdino, Flask, opencv, supervision    │
│                                                            │
│  env: vlm                        GPU 0,1 + 3               │
│  作用: vLLM 托管所有 Qwen 模型                              │
│        :8000 → Qwen2.5-7B-AWQ (任务解析)                   │
│        :8001 → Qwen3-VL-4B (VLM检测, 备选)                 │
│        :8004 → Qwen2.5-VL-7B (轨迹规划)                    │
│  依赖: vllm==0.24.0                                        │
│                                                            │
│  env: llama-factory               GPU 0,1,2,3 (训练时)    │
│  作用: LLaMA-Factory LoRA 微调 Qwen2.5-VL-7B               │
│  依赖: llamafactory, deepspeed≤0.16.9, torchvision         │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

### 环境隔离原因

`vlm` 和 `llama-factory` 对核心包的版本要求互相冲突：

| 包 | vLLM 0.24 要求 | LLaMA-Factory 0.9.3 要求 |
|---|---|---|
| transformers | ≥5.5.3 | ≤4.52.4 |
| pydantic | ≥2.12.0 | ≤2.10.6 |
| accelerate | ≥1.14.0 | ≤1.7.0 |
| numpy | ≥2.0.0 | <2.0.0 |

`groundingdino` 需要特定版本的 torch + GroundingDINO 库，与 vLLM 的 torch 版本可能冲突，同样独立隔离。

**混装后果**：某个包的依赖被覆盖 → 推理报错或训练报错，查半天才发现是环境污染。每个环境保持最小依赖集合。

---

# 第一部分：笔记本端 — airsim 环境

## 1.1 创建环境

```bash
# Windows 上用 Anaconda Prompt 或 git-bash
conda create -n airsim python=3.10 -y
conda activate airsim
cd D:\Codex_code\world_model\Planner
pip install -r requirements.txt
```

`requirements.txt` 内容：

```
airsim>=1.8.1
msgpack-rpc-python>=0.4.1
openai>=1.0.0
Pillow>=10.0.0
numpy>=1.24.0
scipy>=1.10.0
PyYAML>=6.0
Flask>=3.0.0
```

> 如 `requirements.txt` 缺少 `scipy` 或 `PyYAML`：`pip install scipy PyYAML`

#### 已知坑：下视图 RGBA→JPEG 崩溃

如果运行时遇到 `OSError: cannot write mode RGBA as JPEG`，说明 `agent/common/image_encoder.py` 的 `encode_down()` 方法缺少 RGBA→RGB 转换。修复方式：在 `encode_down()` 的第 52 行 `buf = BytesIO()` 之前加两行：

```python
if down_frame.mode == "RGBA":
    down_frame = down_frame.convert("RGB")
```

> 此 bug 已在最新代码中修复。如 clone 后仍遇到，按上述手动添加即可。

## 1.2 AirSim settings.json

路径：`C:\Users\<用户名>\Documents\AirSim\settings.json`

```json
{
  "SettingsVersion": 1.2,
  "ClockSpeed": 1,
  "SimMode": "Multirotor",
  "CameraDefaults": {
    "CaptureSettings": [
      {
        "ImageType": 0,
        "FOV_Degrees": 90,
        "Width": 1920,
        "Height": 1080
      },
      {
        "ImageType": 2,
        "FOV_Degrees": 90,
        "Width": 640,
        "Height": 360
      }
    ]
  },
  "TargetFPS": 60
}
```

> **关键坑**：
> - 编码必须是 **UTF-8 with BOM + CRLF**（Unix LF 会导致 AirSim 解析失败）
> - 用 `CameraDefaults` 平铺格式，**不要**用 `Vehicles.Drone_1.Cameras` 嵌套格式
> - `ImageType: 0` = RGB，`ImageType: 2` = 深度矩阵。改后重启 AirSim

## 1.3 启动 AirSim

双击运行 `AirSimNH.exe`（或项目提供的 UE4 可执行文件），确认出现无人机画面。

---

# 第二部分：服务器端 — groundingdino 环境

> 在 Linux 服务器上执行。GroundingDINO 是一个 Flask 服务，接收图片 + 英文目标名，返回 bbox。

## 2.1 安装

```bash
conda create -n groundingdino python=3.10 -y && conda activate groundingdino

# PyTorch (CUDA 12.1)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121

# GroundingDINO 库
git clone https://github.com/IDEA-Research/GroundingDINO.git /data/sakura/models/GroundingDINO
cd /data/sakura/models/GroundingDINO
pip install -e .

# Flask 服务依赖
pip install Flask opencv-python supervision pycocotools timm addict yapf
```

## 2.2 GroundingDINO 权重

```bash
mkdir -p /data/zhusai/zhusai-tmp/codes/GroundingDINO/weights
cd /data/zhusai/zhusai-tmp/codes/GroundingDINO/weights
# 从 HuggingFace 下载（推荐）
wget https://huggingface.co/ShilongLiu/GroundingDINO/resolve/main/groundingdino_swint_ogc.pth
```

## 2.3 启动服务

```bash
conda activate groundingdino
PYTHONPATH=/data/sakura/models/GroundingDINO/GroundingDINO:$PYTHONPATH CUDA_VISIBLE_DEVICES=2 python /data/sakura/models/GroundingDINO/groundingdino_server.py
```

> `groundingdino_server.py` 不在本仓库中——它是一个独立的 Flask wrapper，位于 GroundingDINO 仓库根目录。接口为 `POST :8003/detect`，非标准 OpenAI 格式。

---

# 第三部分：服务器端 — vlm 环境（vLLM 推理）

> 托管所有 Qwen 模型的推理服务。需要在服务器上开 2~3 个终端。

## 3.1 安装

```bash
conda create -n vlm python=3.10 -y && conda activate vlm
pip install vllm==0.24.0
```

## 3.2 下载模型

```bash
pip install huggingface_hub
export HF_ENDPOINT=https://hf-mirror.com

# 任务解析（纯文本，AWQ 量化，仅 4.5GB 显存）
hf download Qwen/Qwen2.5-7B-Instruct-AWQ --local-dir /data/sakura/models/Qwen2.5-7B-Instruct-AWQ

# 轨迹规划（视觉+文本，7B 全量，需双卡并行）
hf download Qwen/Qwen2.5-VL-7B-Instruct --local-dir /data/sakura/models/3DG-VLN

# VLM 检测备选（视觉+文本，4B，单卡可跑）
hf download Qwen/Qwen3-VL-4B-Instruct --local-dir /data/sakura/models/Qwen3-VL-4B-Instruct
```

## 3.3 启动服务（3 个终端）

```bash
conda activate vlm

# 终端1: 任务解析 (:8000) —— GPU 3
CUDA_VISIBLE_DEVICES=3 vllm serve /data/sakura/models/Qwen2.5-7B-Instruct-AWQ \
  --host 0.0.0.0 --port 8000 --max-model-len 4096

# 终端2: 轨迹规划 (:8004) —— GPU 0,1 双卡张量并行
CUDA_VISIBLE_DEVICES=0,1 vllm serve /data/sakura/models/3DG-VLN-finetuned-sliding_window_2 --host 0.0.0.0 --port 8004 --max-model-len 8192 --tensor-parallel-size 2

# 终端3 (可选): VLM 检测备选 (:8001) —— 如果不用 GroundingDINO
# 注意：和终端1 共用 GPU3，显存够但要错开端口
#CUDA_VISIBLE_DEVICES=3 vllm serve /data/sakura/models/Qwen3-VL-4B-Instruct \
#  --host 0.0.0.0 --port 8001 --max-model-len 4096
```

> **备选方案**：`qwen_server_updated.py`（Flask 直连模式，开发调试用），其 `model_path` 在第 12 行硬编码，换模型需手动修改。

## 3.4 验证

笔记本端：

```bash
# 替换为服务器 IP
curl http://172.27.143.102:8000/v1/models   # 任务解析
curl http://172.27.143.102:8004/v1/models   # 轨迹规划
curl -X POST http://172.27.143.102:8003/detect -d '{}'  # GroundingDINO
```

---

# 第四部分：服务器端 — llama-factory 环境（微调）

> 用于 Qwen2.5-VL-7B LoRA 微调。**与 vlm 环境绝对不能共用 conda env。**

## 4.1 安装

```bash
conda create -n llama-factory python=3.10 -y && conda activate llama-factory

pip install llamafactory
pip install torchvision --index-url https://download.pytorch.org/whl/cu121

# DeepSpeed ZeRO-3（版本锁 ≤0.16.9）
conda install -c conda-forge cuda-nvcc=12.1 -y
export CUDA_HOME=$CONDA_PREFIX
pip install 'deepspeed>=0.10.0,<=0.16.9'

# 验证
nvcc --version
python -c "import torch; assert torch.version.cuda=='12.1'; print('torch OK')"
python -c "import deepspeed; print(f'deepspeed {deepspeed.__version__} OK')"
```

## 4.2 数据准备

> 数据集通过百度网盘下载：`https://pan.baidu.com/s/1slWa79ZdNIHid_fwqyhdxA` 提取码 `ymav`
> 详见第七部分 7.2 节。

```bash
conda activate llama-factory
cd /path/to/uni-lavira-code-main

# 从原始数据集提取训练样本
python tools/extract_training_samples.py \
    --dataset /data/sakura/data/UAV-VLN-FOV/train \
    --meta /data/sakura/data/UAV-VLN-FOV/meta \
    --output /data/sakura/data/UAV-VLN-FOV/finetune_data \
    --waypoints 5 \
    --step_interval 5
```

输出：`finetune_data/images/` + `finetune_data/dataset.json`

### 注册数据集

```bash
cat > /data/sakura/data/UAV-VLN-FOV/finetune_data/dataset_info.json << 'EOF'
{
  "3dgvln_waypoints": {
    "file_name": "dataset_v2.jsonl",
    "formatting": "sharegpt",
    "columns": { "messages": "messages", "images": "images" },
    "tags": { "role_tag": "role", "content_tag": "content", "user_tag": "user", "assistant_tag": "assistant" }
  }
}
EOF
```

### 格式转换（content 统一字符串化）

```bash
cd /data/sakura/data/UAV-VLN-FOV/finetune_data
python -c "
import json
with open('dataset.json') as f: data = json.load(f)
out = []
for item in data:
    msgs = []
    for m in item['messages']:
        if isinstance(m['content'], list):
            parts = []
            for b in m['content']:
                if b['type'] == 'image': parts.append('<image>')
                elif b['type'] == 'text': parts.append(b['text'])
            msgs.append({'role': m['role'], 'content': ''.join(parts)})
        else:
            msgs.append({'role': m['role'], 'content': m['content']})
    out.append({'messages': msgs, 'images': item['images']})
with open('dataset_v2.jsonl', 'w') as f:
    for o in out: f.write(json.dumps(o, ensure_ascii=False) + '\n')
print(f'{len(out)} samples')
"
```

## 4.3 DeepSpeed ZeRO-3 配置

```bash
cat > /data/sakura/data/UAV-VLN-FOV/tools/ds_zero3_offload.json << 'EOF'
{
  "bf16": {"enabled": true},
  "zero_optimization": {
    "stage": 3,
    "offload_optimizer": {"device": "none"},
    "offload_param": {"device": "cpu", "pin_memory": true},
    "overlap_comm": true,
    "contiguous_gradients": true,
    "sub_group_size": 1000000000,
    "reduce_bucket_size": "auto",
    "stage3_prefetch_bucket_size": 50000000,
    "stage3_param_persistence_threshold": 100000,
    "stage3_max_live_parameters": 1000000000,
    "stage3_max_reuse_distance": 1000000000,
    "stage3_gather_16bit_weights_on_model_save": true
  },
  "gradient_accumulation_steps": 8,
  "gradient_clipping": 1.0,
  "train_batch_size": "auto",
  "train_micro_batch_size_per_gpu": "auto"
}
EOF
```

## 4.4 微调 YAML 配置

编辑 `tools/qwen2_5vl_7b_3dgvln_lora.yaml`，确保以下三项正确：

```yaml
model_name_or_path: /data/sakura/models/3DG-VLN
deepspeed: /data/sakura/data/UAV-VLN-FOV/tools/ds_zero3_offload.json
dataset: 3dgvln_waypoints
dataset_dir: /data/sakura/data/UAV-VLN-FOV/finetune_data
template: qwen2_vl
```

## 4.5 启动训练

```bash
conda activate llama-factory
export CUDA_HOME=$CONDA_PREFIX
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True

# 4 卡训练 (~10-15h)
CUDA_VISIBLE_DEVICES=0,1,2,3 llamafactory-cli train tools/qwen2_5vl_7b_3dgvln_lora.yaml

# 2 卡训练 (~20-30h, 用同 PCIe switch 的 GPU 2+3)
# CUDA_VISIBLE_DEVICES=2,3 llamafactory-cli train tools/qwen2_5vl_7b_3dgvln_lora.yaml
```

## 4.6 合并 LoRA 并部署

```bash
# 合并（llama-factory 环境）
conda activate llama-factory
llamafactory-cli export \
    --model_name_or_path /data/sakura/models/3DG-VLN \
    --adapter_name_or_path ./output/qwen2_5vl_7b_3dgvln_lora/checkpoint-1500 \
    --template qwen2_vl \
    --finetuning_type lora \
    --export_dir /data/sakura/models/3DG-VLN-finetuned

# 部署（vlm 环境）
conda activate vlm
CUDA_VISIBLE_DEVICES=0,1 vllm serve /data/sakura/models/3DG-VLN-finetuned \
    --host 0.0.0.0 --port 8004 --max-model-len 8192
```

然后在 `config/default.yaml` 中改：`PLANNER_MODEL: /data/sakura/models/3DG-VLN-finetuned`

---

# 第五部分：config/default.yaml 统一配置

> **项目配置中枢。换机器、换服务器、换模型、切换本地/云 API —— 只改这一个文件。**

```yaml
AGENT:
  # 任务解析器 —— 填 :8000 的 vLLM
  TASK_PARSER: vlm_parser
  TASK_PARSER_URL: http://172.27.143.102:8000/v1
  TASK_PARSER_MODEL: /data/sakura/models/Qwen2.5-7B-Instruct-AWQ
  TASK_API_KEY: no-key

  # 目标检测器 —— groundingdino（:8003）或 vlm_detector（:8001/云端）
  DETECTOR: groundingdino
  GROUNDINGDINO_URL: http://172.27.143.102:8003/detect
  # vlm_detector 模式下填写:
  # DETECTOR_URL: http://172.27.143.102:8001/v1
  # DETECTOR_MODEL: /data/sakura/models/Qwen3-VL-4B-Instruct

  # 轨迹规划器 —— api_atomic_planner（云 API）或 qwen_planner（:8004）
  PLANNER: qwen_planner
  PLANNER_URL: http://172.27.143.102:8004/v1
  PLANNER_MODEL: /data/sakura/models/3DG-VLN
  PLANNER_API_KEY: no-key
  PLANNER_MAX_TOKENS: 2048
  PLANNER_CANDIDATE_COUNT: 5

  DIRECTION: three_dg
  STOP_DEPTH_THRESHOLD: 8.0

SIM:
  AIRSIM_IP: ""        # 本地 AirSimNH.exe 留空；远程 UE4 填服务器 IP
  AIRSIM_PORT: 41451

WEB:
  PORT: 5000
```

### 场景切换速查

| 场景 | 改什么 |
|---|---|
| 本地 AirSim + 服务器 API | `AIRSIM_IP: ""` + URL 填服务器内网 IP |
| 服务器 UE4 闭环评测 + 服务器 API | `AIRSIM_IP: "172.27.143.102"` + URL 不变 |
| 本地 AirSim + 云 API (MiMo/GPT-4o) | `PLANNER: api_atomic_planner` + URL 填云 API + API Key |
| 纯开环评测 | 不改 SIM，只确保 AGENT 中 URL 正确即可 |
| 微调后推理 | `PLANNER_MODEL: /data/sakura/models/3DG-VLN-finetuned` |

---

# 第六部分：运行

## 6.1 本地 AirSim + 服务器 API（最常用）

**前提**：第 2~3 部分服务器 3 个服务已启动。

```bash
conda activate airsim
cd E:\uni-lavira-code-main
python run_airsim_web.py
```

浏览器打开 `http://localhost:5000`，输入任务如：`飞到红色汽车旁`

## 6.2 本地 AirSim + 云 API（无需服务器）

编辑 `config/default.yaml`：

```yaml
AGENT:
  TASK_PARSER_URL: https://api.xiaomimimo.com/v1
  TASK_PARSER_MODEL: mimo-v2.5
  TASK_API_KEY: sk-xxx
  DETECTOR: vlm_detector
  DETECTOR_URL: https://api.xiaomimimo.com/v1
  DETECTOR_MODEL: mimo-v2.5
  DETECTOR_API_KEY: sk-xxx
  PLANNER: api_atomic_planner
  PLANNER_URL: https://api.xiaomimimo.com/v1
  PLANNER_MODEL: mimo-v2.5
  PLANNER_API_KEY: sk-xxx
```

然后 `python run_airsim_web.py`。

---

# 第七部分：离线评测

## 7.1 开环评测（笔记本即可，秒级）

> 不需要 AirSim、不需要 GPU。只调 API，纯数学算指标。

```bash
conda activate airsim
cd E:\uni-lavira-code-main

# 数据集通过百度网盘下载（见 7.2 节），放到 E:\UAV-VLN-FOV
# 链接: https://pan.baidu.com/s/1slWa79ZdNIHid_fwqyhdxA  提取码: ymav

# 跑评测
python eval/openloop_eval.py --dataset E:\UAV-VLN-FOV\test
python eval/openloop_eval.py --dataset E:\UAV-VLN-FOV\unobject
python eval/openloop_eval.py --dataset E:\UAV-VLN-FOV\unscene
```

输出：`SR=45.2%  NE=18.3m  SPL=0.312`

## 7.2 闭环评测（Linux 服务器，需要 UE4）

> 需要 TravelUAV 仿真环境（UE4+AirSim）。每步实时渲染。

### 下载数据集

> **UAV-VLN-FOV 数据集来自 3DG-VLN，仅提供百度网盘下载。**
> 链接: `https://pan.baidu.com/s/1slWa79ZdNIHid_fwqyhdxA`  提取码: `ymav`

下载解压后按以下结构放置：

```bash
# 服务器端（评测 + 微调都需要）
mkdir -p /data/sakura/data/UAV-VLN-FOV
# 将解压后的 train/ test/ unobject/ unscene/ meta/ 移入

# 笔记本端（如果做开环评测）
mkdir -p E:\UAV-VLN-FOV
# 同样移入解压后的所有目录
```

目录结构：

```
UAV-VLN-FOV/
├── train/          # 2228条, 训练用
├── test/           # 152条, Seen 场景评测
├── unobject/       # 164条, Unseen Object 评测
├── unscene/        # 173条, Unseen Map 评测
└── meta/           # instructions.json, map_spawnarea_info.json
```

### 下载 3DG-VLN 预训练权重（可选，微调基座）

> 百度网盘: `https://pan.baidu.com/s/11lLcRczubWA01-33xhbK5A`  提取码: `sasg`
> 或者直接用 HuggingFace 上的 Qwen2.5-VL-7B-Instruct 作为基座模型（见 3.2 节）。

### 下载仿真环境

```bash
# 服务器上（来自 TravelUAV 项目的 HF 仓库）
export HF_ENDPOINT=https://hf-mirror.com
hf download wangxiangyu0814/TravelUAV_env --local-dir /data/sakura/data/TravelUAV_env
```

### 运行

```bash
conda activate airsim   # 评测脚本只需要 airsim 客户端
cd /path/to/uni-lavira-code-main

# 多场景自动切换
python eval/run_eval.py \
    --dataset /data/sakura/data/UAV-VLN-FOV/test \
    --env_root /data/sakura/data/TravelUAV_env \
    --gpu 0

# 单场景手动
python eval/run_eval.py \
    --dataset /data/sakura/data/UAV-VLN-FOV/test \
    --scene ModularEuropean
```

---

# 第八部分：常见问题

| 问题 | 原因 | 解决 |
|---|---|---|
| 深度图一直为空 | settings.json 深度分辨率太高 | ImageType:2 设 640×360，重启 AirSim |
| VLM context length 超限 | max_tokens 太大 | 降 `PLANNER_MAX_TOKENS` 到 1024~2048 |
| vLLM OOM | 单卡显存不够 | 双卡 `--tensor-parallel-size 2` |
| 训练 OOM | 权重全在 GPU | ZeRO-3 + `offload_param: cpu` |
| ArrowInvalid: column changed | content 混用 list/string | 执行 4.2 节格式转换脚本 |
| `deepspeed==0.19.2 is required` | 版本超上限 | `pip install 'deepspeed<=0.16.9'` |
| `nvcc not found` | 环境无 CUDA 编译器 | `conda install cuda-nvcc=12.1` + `export CUDA_HOME=$CONDA_PREFIX` |
| `cannot find -lcurand` | CUDA_HOME 未指向 conda prefix | `export CUDA_HOME=$CONDA_PREFIX` |
| 微调后输出全零 | 基座模型未经过 waypoint 训练 | LoRA rank ≥ 8，lr ≈ 5e-5，确认 loss 下降 |
| settings.json 不生效 | 编码不是 UTF-8 BOM + CRLF | 用 VS Code 右下角改编码和换行符 |
| 开环 vs 闭环 | 开环秒级验证管线；闭环出论文指标 | 先开环跑通，再闭环 |
| vlm 和 llama-factory 混装 | 依赖版本冲突 | 立即重建环境。见环境矩阵 |
| groundingdino_server.py 在哪 | 在 GroundingDINO 仓库，不在本 repo | clone 到 `/data/sakura/models/GroundingDINO/` |

---

# 附录 A：项目目录速查

```
E:\uni-lavira-code-main\
├── run_airsim_web.py          # 主入口
├── config/default.yaml        # ⭐ 全局配置
├── requirements.txt
│
├── agent/                     # VLM 规划逻辑
│   ├── common/                # 预热 / 编码缓存 / 任务管理
│   ├── detector/              # GroundingDINO / VLM 检测器
│   ├── direction/             # 方向估计
│   ├── planner/               # API 原子动作 / Qwen waypoints
│   └── task_parser/           # NL → 结构化任务
│
├── sim/                       # AirSim 交互
│   ├── airsim_client.py
│   ├── frame_capturer.py      # 后台取帧
│   └── capture.py
│
├── eval/
│   ├── openloop_eval.py       # 开环评测（不需 AirSim）
│   ├── run_eval.py            # 闭环评测（需 UE4）
│   ├── scene_manager.py       # UE4 进程管理
│   └── metrics.py             # SR/NE/SPL
│
├── tools/
│   ├── extract_training_samples.py
│   └── qwen2_5vl_7b_3dgvln_lora.yaml
│
├── web/                       # Flask 仪表盘
└── docs/                      # 详细技术文档
```

# 附录 B：服务器 GPU 分配

| GPU | 端口 | 模型 | 用途 | 环境 |
|---|---|---|---|---|
| 0+1 | `:8004` | Qwen2.5-VL-7B (3DG-VLN) | 轨迹规划 | `vlm` |
| 2 | `:8003` | GroundingDINO | 目标检测 | `groundingdino` |
| 3 | `:8000` | Qwen2.5-7B-AWQ | 任务解析 | `vlm` |
| (3) | `:8001` | Qwen3-VL-4B | VLM 检测备选 | `vlm` |
| 0-3 | – | LLaMA-Factory 训练 | 微调 | `llama-factory` |

---

> **迁移清单**：换机器只需改 `config/default.yaml` 中的 `*_URL`、`*_MODEL` 和 `AIRSIM_IP`。数据集路径由各脚本 `--dataset` CLI 参数控制。
