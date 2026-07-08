# Qwen2.5-VL-7B LoRA 微调 —— 3DG-VLN Waypoint 预测

本文档记录从原始数据集到可部署模型的完整微调流程。

---

## 0. 环境要求

| 组件 | 要求 | 备注 |
|------|------|------|
| Python | 3.10 | conda 虚拟环境 |
| PyTorch | 2.5.1+cu121 | 与 CUDA 12.1 匹配 |
| torchvision | `pip install torchvision --index-url …/cu121` | Qwen2.5-VL processor 硬依赖，缺少会报 `AutoVideoProcessor requires Torchvision` |
| DeepSpeed | **≥0.10.0, ≤0.16.9** | LLaMA-Factory 版本锁，装 0.19.x 会报 `deepspeed==0.19.2 is required` 版本错 |
| CUDA Toolkit | **12.1**（nvcc） | DeepSpeed JIT 编译 CPUAdam 需要 nvcc，conda：`conda install -c conda-forge cuda-nvcc=12.1` |
| GPU | 3~4× RTX 3080 Ti (12GB) | ZeRO-3 + CPU offload 必需，DDP 每卡 14GB 权重会 OOM |

### 一键安装命令

```bash
conda create -n llama-factory python=3.10 -y && conda activate llama-factory

# LLaMA-Factory
pip install llamafactory

# Qwen2.5-VL 依赖
pip install torchvision --index-url https://download.pytorch.org/whl/cu121

# DeepSpeed（注意版本锁）
conda install -c conda-forge cuda-nvcc=12.1 -y
export CUDA_HOME=$CONDA_PREFIX
pip install 'deepspeed>=0.10.0,<=0.16.9'

# 验证
nvcc --version && python -c "import torch; assert torch.version.cuda=='12.1'; print('torch OK')"
python -c "import deepspeed; print(f'deepspeed {deepspeed.__version__} OK')"
python -c "from transformers import AutoProcessor; AutoProcessor.from_pretrained('Qwen/Qwen2.5-VL-7B-Instruct'); print('processor OK')"
```

### 常见环境错误速查

| 错误 | 根因 | 修复 |
|------|------|------|
| `AutoVideoProcessor requires Torchvision` | torchvision 未安装 | `pip install torchvision` (cu121) |
| `deepspeed==0.19.2 is required` | 版本超上限 | `pip install 'deepspeed<=0.16.9'` |
| `CUDA_HOME does not exist` | conda 环境无 nvcc | `conda install cuda-nvcc=12.1` |
| `Installed CUDA 13.3 ≠ torch 12.1` | nvcc 版本太高 | `conda install cuda-nvcc=12.1` 降级 |
| `cannot find -lcurand` | CUDA_HOME 没指向 conda prefix | `export CUDA_HOME=$CONDA_PREFIX`，nvidia-curand-cu12 在 pip torch 里自带 |
| `CUDA OOM during model init` | vision encoder 先占满 GPU0 | `stage3_max_live_parameters: 0` + `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True` |
| `Please provide model_name_or_path` | CLI 传了 `--deepspeed` | 写 YAML 里，不要用 CLI flag |
| 每卡 14GB OOM | DDP 全量副本 | 用 ZeRO-3 分片到 4 卡 |

---

## 1. 背景

3DG-VLN 论文提供的"预训练权重"实际上是 Qwen2.5-VL-7B 基座模型，未经过 waypoint 预测微调。直接用基座模型推理会输出空轨迹（全零），因为基座模型没学过输出 `[[dx,dy,dz], ...]` 格式的 waypoint 序列。

本文档使用 LLaMA-Factory 对 Qwen2.5-VL-7B-Instruct 做 LoRA 微调，使其学会从双视图（前视+下视）+ 指令 → 输出 5 个位移 waypoint。

---

## 2. 数据准备

### 2.1 数据集结构

3DG-VLN 的 UAV-VLN-FOV 数据集结构：

```
UAV-VLN-FOV/
├── train/                          # 训练集（场景见过，物体见过）
│   └── Town01/
│       └── traj_001/
│           ├── mark.json           # 目标元信息
│           ├── log/                # 每帧 UAV state (position, orientation, imu)
│           │   ├── 000000.json
│           │   └── ...
│           ├── frontcamera/        # 前视 RGB, 1024×1024
│           │   └── 000000.png
│           ├── downcamera/         # 下视 RGB, 1024×1024
│           ├── frontcamera_depth/  # 深度图
│           └── downcamera_depth/
├── test/                           # 验证集（场景见过，物体见过）
├── unobject/                       # 泛化评测（物体未见）
├── unscene/                        # 泛化评测（场景未见）
└── meta/
    ├── instructions.json           # trajectory_name → instruction_text
    ├── object_description.json     # 物体名 → 物体描述
    ├── map_spawnarea_info.json     # 地图 spawn 区域坐标
    └── object_attribute            # 轨迹 → 目标属性文本
```

**waypoint 标注是隐式的**：相邻 `log/` 帧的 `state.position` 差值即为 GT 位移，不存在独立的 waypoint 标签文件。

### 2.2 各 split 用途

| Split | 场景 | 物体 | 用途 |
|-------|------|------|------|
| `train/` | 见过 | 见过 | **微调训练** |
| `test/` | 见过 | 见过 | 训练时验证 |
| `unobject/` | 见过 | 未见过 | 最终泛化评测 |
| `unscene/` | 未见过 | 见过 | 最终泛化评测 |

### 2.3 从原始轨迹提取训练样本

使用 `tools/extract_training_samples.py` 将原始轨迹转为 LLaMA-Factory 兼容的对话格式。

**原理**：遍历每条轨迹的每一帧，取该帧的前视图+下视图作为输入，从后续帧的位置差中采样 5 个 body-frame waypoint 作为输出标签。

**操作步骤**：

```bash
# 1. 解压（在服务器上）
cd /data/sakura/datasets/UAV-VLN-FOV
unrar x train.rar      # → train/
unrar x test.rar       # → test/（可选）

# 2. 提取样本
cd /data/sakura
python tools/extract_training_samples.py \
    --dataset /data/sakura/datasets/UAV-VLN-FOV/train \
    --meta    /data/sakura/datasets/UAV-VLN-FOV/meta \
    --output  ./3dgvln_finetune_data \
    --waypoints 5 \
    --step_interval 5
```

**参数说明**：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `--waypoints` | 5 | 每个样本输出的 waypoint 数量 |
| `--step_interval` | 5 | 每隔多少帧取一个样本（避免相邻帧太相似） |

**输出目录结构**：

```
3dgvln_finetune_data/
├── images/
│   ├── traj_001_step0000_front.png
│   ├── traj_001_step0000_down.png
│   ├── traj_001_step0005_front.png
│   └── ...
└── dataset.json       # Qwen2.5-VL 对话格式
```

**dataset.json 样本格式**：

```json
[
  {
    "messages": [
      {
        "role": "user",
        "content": [
          {"type": "image", "image": "images/traj_001_step0000_front.png"},
          {"type": "image", "image": "images/traj_001_step0000_down.png"},
          {"type": "text", "text": "Stage: cruise\nPrevious displacement: 0.0,0.0,-4.5\nCurrent position: 0.0,0.0,0.0\nInstruction: Fly towards the red car and approach it from the front."}
        ]
      },
      {
        "role": "assistant",
        "content": "[[12.5, 3.2, -2.0], [8.0, -1.5, 0.0], [5.0, 0.0, -1.0], [2.0, 0.5, 0.0], [0.0, 0.0, 0.0]]"
      }
    ],
    "images": ["images/traj_001_step0000_front.png", "images/traj_001_step0000_down.png"]
  }
]
```

---

## 3. 环境安装

```bash
# 推荐方式：conda 环境
conda create -n llama-factory python=3.10 -y
conda activate llama-factory
pip install llamafactory

# ⚠️ 必须安装 torchvision，否则 Qwen2.5-VL processor 加载失败
# （错误: AutoVideoProcessor requires the Torchvision library）
pip install torchvision --index-url https://download.pytorch.org/whl/cu121

# ⚠️ DeepSpeed：4 卡训练必需，注意版本约束 ≤0.16.9
# 先装 CUDA 编译器（conda 环境内，无需 sudo）
conda install -c conda-forge cuda-nvcc cuda-cudart-dev -y
export CUDA_HOME=$CONDA_PREFIX
pip install 'deepspeed>=0.10.0,<=0.16.9'

# 验证
python -c "from transformers import AutoProcessor; AutoProcessor.from_pretrained('Qwen/Qwen2.5-VL-7B-Instruct'); print('OK')"
python -c "import deepspeed; print(f'DeepSpeed {deepspeed.__version__} OK')"
```

---

## 4. 注册数据集

### 4.1 创建 dataset_info.json

LLaMA-Factory 通过 `dataset_info.json` 注册数据集——它告诉框架你的数据集叫什么、存在哪个文件、用什么格式解析。**`dataset_info.json` 和 `dataset.json` 是两个不同的文件**：前者是注册表，后者是训练数据本体。

在微调数据目录下创建 `dataset_info.json`：

```bash
cat > /data/sakura/data/UAV-VLN-FOV/finetune_data/dataset_info.json << 'EOF'
{
  "3dgvln_waypoints": {
    "file_name": "dataset.jsonl",
    "formatting": "sharegpt",
    "columns": {
      "messages": "messages",
      "images": "images"
    },
    "tags": {
      "role_tag": "role",
      "content_tag": "content",
      "user_tag": "user",
      "assistant_tag": "assistant"
    }
  }
}
EOF
```

> **为什么需要 `dataset_info.json`？** YAML 中 `dataset: 3dgvln_waypoints` 只是一个逻辑名称。LLaMA-Factory 启动时从 `dataset_info.json` 查找这个名字，才知道去读哪个文件（`file_name`）、用什么格式解析（`sharegpt`）、字段如何映射（`columns`/`tags`）。没有这个注册表→直接报 `Cannot open data/dataset_info.json`。

### 4.2 配置 dataset_dir

在 YAML 配置中通过 `dataset_dir` 指向微调数据目录，让 `file_name` 的相对路径正确解析：

```yaml
dataset: 3dgvln_waypoints
dataset_dir: /data/sakura/data/UAV-VLN-FOV/finetune_data
```

> **注意**：不再需要修改 LLaMA-Factory 自身的 `data/dataset_info.json`，也不需要用软链接。`dataset_dir` + 本地 `dataset_info.json` 是自包含的方案。

### 4.3 统一 content 为字符串格式（必须）

`datasets` 库通过 Arrow 加载数据，要求同一列的每个元素类型一致。ShareGPT 多模态原始格式中 `content` 混用了 `list`（user：图片+文本）和 `string`（assistant：回复），Arrow **在任何格式（JSON 数组或 JSONL）下都会报错**：

```
ArrowInvalid: Column(/messages/[]/content) changed from array to string in row 0
```

**根本原因**：Arrow 要求 `/messages/[]/content` 这条列路径上的所有值统一类型——要么全是 string，要么全是 list，不能混用。JSONL 只是绕过了 pandas 的第一步，Arrow 自身的 JSON reader 同样会卡在这个 schema 冲突上。

**解决方案**：LLaMA-Factory 的 `qwen2_vl` 模板通过 `<image>` 占位符处理图片——模板在 tokenization 时自动将 `<image>` 替换为 `images[]` 中对应路径的图像 tensor。因此 `content` 必须全为字符串：

- user content: `"<image><image>Stage: cruise\nInstruction: ..."`
- assistant content: `"[[1.0, 2.0, 3.0], ...]"`

将 `[{type:"image",...}, {type:"text",...}]` 列表格式转为上述字符串格式：

```bash
cd /data/sakura/data/UAV-VLN-FOV/finetune_data

python -c "
import json

with open('dataset.json', 'r') as f:
    data = json.load(f)

output = []
for item in data:
    new_msgs = []
    for msg in item['messages']:
        if isinstance(msg['content'], list):
            parts = []
            for block in msg['content']:
                if block['type'] == 'image':
                    parts.append('<image>')
                elif block['type'] == 'text':
                    parts.append(block['text'])
            new_content = ''.join(parts)
        else:
            new_content = msg['content']
        new_msgs.append({'role': msg['role'], 'content': new_content})

    output.append({
        'messages': new_msgs,
        'images': item['images']
    })

with open('dataset_v2.jsonl', 'w') as f:
    for item in output:
        f.write(json.dumps(item, ensure_ascii=False) + '\n')

print(f'{len(output)} samples → dataset_v2.jsonl')
"
```

最终数据目录结构：

```
finetune_data/
├── dataset_info.json    # 数据集注册表（LLaMA-Factory 读取）
├── dataset.json         # 训练数据本体（原始 JSON 数组，保留备份）
├── dataset_v2.jsonl     # 训练数据（content 字符串化，LLaMA-Factory 实际读取）
└── images/              # 图片文件
    ├── xxx_front.png
    └── xxx_down.png
```

---

## 5. 微调配置

配置文件位于 `tools/qwen2_5vl_7b_3dgvln_lora.yaml`：

```yaml
### model
model_name_or_path: /data/sakura/models/3DG-VLN

### method
stage: sft
do_train: true
finetuning_type: lora
lora_target: all
lora_rank: 8
lora_alpha: 16
deepspeed: /data/sakura/data/UAV-VLN-FOV/tools/ds_zero3_offload.json

### dataset
dataset: 3dgvln_waypoints
dataset_dir: /data/sakura/data/UAV-VLN-FOV/finetune_data
template: qwen2_vl
cutoff_len: 4096
image_max_pixels: 1048576          # 1024×1024, 保留高分辨率细节
overwrite_cache: true
preprocessing_num_workers: 4

### output
output_dir: ./output/qwen2_5vl_7b_3dgvln_lora
logging_steps: 10
save_steps: 500
plot_loss: true
overwrite_output_dir: true

### train
per_device_train_batch_size: 1     # 双图输入显存占用大
gradient_accumulation_steps: 8     # 等效 batch=8
learning_rate: 5.0e-5
num_train_epochs: 3.0
lr_scheduler_type: cosine
warmup_ratio: 0.1
bf16: true
gradient_checkpointing: true
ddp_timeout: 180000000
```

**关键参数说明**：

| 参数 | 值 | 原因 |
|------|-----|------|
| `finetuning_type: lora` | 必须 LoRA | RTX 3080 Ti (12GB) 全量微调 7B 显存不足 |
| `lora_target: all` | 全模块 | vision + language 都需要适配 waypoint 输出 |
| `per_device_train_batch_size: 1` | 1 | 每样本含 2 张 1024×1024 图，显存占用约 10GB |
| `gradient_accumulation_steps: 8` | 8 | 累积后有效 batch=8 |
|| `image_max_pixels: 1048576` | 1024² | 保留高分辨率，低于此值会被 resize |
|| `deepspeed: ...` | ZeRO-3 + CPU offload | 模型+优化器分片到 4 卡，12GB 卡也能跑 7B |
|| `gradient_checkpointing: true` | 激活性折半 | 用计算换显存，约节省 30-40% 激活内存 |

### 5.1 DeepSpeed ZeRO-3 配置

ZeRO-3 将模型参数、梯度、优化器状态分片到所有 GPU + offload 到 CPU。12GB 3080 Ti 跑 7B 模型，每卡仅 ~4-6 GB，3~4 卡均可安全运行。

> **关键**：`stage3_max_live_parameters: 0` 确保初始化阶段没有参数常驻 GPU，避免 vision encoder 先占满 GPU0 导致 OOM。

创建 `ds_zero3_offload.json`（放在 `tools/` 下）：

```json
{
  "bf16": {"enabled": true},
  "zero_optimization": {
    "stage": 3,
    "offload_optimizer": {"device": "cpu", "pin_memory": true},
    "offload_param": {"device": "cpu", "pin_memory": true},
    "overlap_comm": true,
    "contiguous_gradients": true,
    "sub_group_size": 1e9,
    "reduce_bucket_size": "auto",
    "stage3_prefetch_bucket_size": 0,
    "stage3_param_persistence_threshold": 0,
    "stage3_max_live_parameters": 0,
    "stage3_max_reuse_distance": 0,
    "stage3_gather_16bit_weights_on_model_save": true
  },
  "gradient_accumulation_steps": 8,
  "gradient_clipping": 1.0,
  "train_batch_size": "auto",
  "train_micro_batch_size_per_gpu": "auto"
}
```

> **注意**：`deepspeed` 路径写在 YAML 里，**不要用 `--deepspeed` CLI 参数**——LLaMA-Factory 不认这个 CLI flag，会导致参数解析失败。

---

## 6. 启动训练

### 6.1 环境变量

启动前**必须**设置：

| 变量 | 作用 | 不设的后果 |
|------|------|-----------|
| `CUDA_HOME=$CONDA_PREFIX` | DeepSpeed 查找 CUDA 工具链（nvcc、libcurand） | CPUAdam JIT 编译失败 |
| `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True` | 允许 CUDA 缓存块动态扩展，防止碎片假 OOM | ZeRO-3 频繁借用层权重导致碎片化后分配失败 |

### 6.2 GPU 数量 → 完整配置对照表

**不同卡数需要改的配置文件**：YAML（1 个参数 + 可选改 1 个分辨率）+ DeepSpeed JSON（3 个参数）。所有参数联动，缺一不可。

| GPU 数 | `gradient_accumulation_steps` | `offload_param` | `offload_optimizer` | `image_max_pixels` | 有效 batch | 预计时间 |
|--------|-------------------------------|------|------|-------------------|-----------|---------|
| **4** | 8 | `cpu` | `none` | `1048576` (1024²) | 32 | **10-15h** |
| 3 | 11 | `cpu` | `none` | `1048576` (1024²) | 33 | 15-20h |
| 2 | 16 | `cpu` | `none` | `524288` (724²) | 32 | 20-30h |

**参数联动原因**：

| 参数 | 变化逻辑 |
|------|---------|
| `gradient_accumulation_steps` | 有效 batch = `1 × steps × GPU数`。保持 ≈32 以维持梯度稳定性。**YAML 和 DS JSON 必须同时改** |
| `offload_param` | **所有卡数都必须 `cpu`**。cross_entropy(logits) 的 1.65 GB 分配无法压缩，权重卸载到 CPU 是唯一不降分辨率的方法 |
| `offload_optimizer` | LoRA 仅 20M 参数，优化器状态分片后每卡忽略不计，放 GPU |
| `image_max_pixels` | 2 卡显存特别紧张（权重卸载后仍 8+GB），必须降分辨率。3~4 卡可以保持 1024² |
>
> **训练分辨率 ≠ 推理分辨率**：`image_max_pixels` 是模型内部处理的上限，不是 AirSim 采集分辨率。推理时 Qwen2.5-VL 的 processor 会自动把 1920×1080 原图等比缩放到训练时的分辨率，无需调低 AirSim 采集参数。原图分辨率越高越好（下采样比直接低分辨率采集细节更丰富）。

### 6.3 配置文件生成脚本

根据 GPU 数一键生成对应配置：

```bash
# 用法: N=4（或 3/2）然后执行
N=4
DS=/data/sakura/data/UAV-VLN-FOV/tools/ds_zero3_offload.json
YML=/data/sakura/data/UAV-VLN-FOV/tools/qwen2_5vl_7b_3dgvln_lora.yaml

case $N in
  4)
    GAS=8;  PARAM=cpu; PIX=1048576 ;;
  3)
    GAS=11; PARAM=cpu;  PIX=1048576 ;;
  2)
    GAS=16; PARAM=cpu;  PIX=524288  ;;
  *) echo "N must be 2/3/4"; exit 1 ;;
esac

# DS JSON
cat > $DS << EOF
{
  "bf16": {"enabled": true},
  "zero_optimization": {
    "stage": 3,
    "offload_optimizer": {"device": "none"},
    "offload_param": {"device": "$PARAM", "pin_memory": true},
    "overlap_comm": true, "contiguous_gradients": true,
    "sub_group_size": 1e9, "reduce_bucket_size": "auto",
    "stage3_prefetch_bucket_size": 50000000,
    "stage3_param_persistence_threshold": 100000,
    "stage3_max_live_parameters": 1000000000,
    "stage3_max_reuse_distance": 1000000000,
    "stage3_gather_16bit_weights_on_model_save": true
  },
  "gradient_accumulation_steps": $GAS,
  "gradient_clipping": 1.0,
  "train_batch_size": "auto",
  "train_micro_batch_size_per_gpu": "auto"
}
EOF

# YAML
sed -i "s/gradient_accumulation_steps: .*/gradient_accumulation_steps: $GAS/" $YML
sed -i "s/image_max_pixels: .*/image_max_pixels: $PIX/" $YML

# 训练
export CUDA_HOME=$CONDA_PREFIX
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True

case $N in
  4) CUDA_VISIBLE_DEVICES=0,1,2,3 ;;
  3) CUDA_VISIBLE_DEVICES=1,2,3   ;;  # 避开 0,1 跨 socket
  2) CUDA_VISIBLE_DEVICES=2,3     ;;  # 同 PCIe switch
esac

llamafactory-cli train $YML
```

---

## 7. 合并部署

训练完成后，将 LoRA adapter 合并到基座模型：

```bash
llamafactory-cli export \
    --model_name_or_path /data/sakura/models/3DG-VLN \
    --adapter_name_or_path ./output/qwen2_5vl_7b_3dgvln_lora/checkpoint-2268 \
    --template qwen2_vl \
    --finetuning_type lora \
    --export_dir /data/sakura/models/3DG-VLN-finetuned
```

然后将 `qwen_server_updated.py` 的 `model_path` 指向新模型：

```python
model_path = "/data/sakura/models/3DG-VLN-finetuned"
```

---

## 8. 验证

启动服务器后用前文的测试指令验证：

```python
# 预期输出示例（不再全零）
waypoints = [[12.5, 3.2, -2.0], [8.0, -1.5, 0.0], [5.0, 0.0, -1.0], [2.0, 0.5, 0.0], [0.0, 0.0, 0.0]]
```

每个 waypoint 格式：`[dx, dy, dz]` — 机体坐标系下的增量位移（米）。

---

## 9. 常见问题

### Q1: 显存不够 (OOM)

- **根本方案**：改用 4 卡 DeepSpeed ZeRO-3（第 6.1 节），模型权重分片到 4 卡 + CPU offload，每卡仅 ~6-8GB
- 如果硬件限制只能单卡/双卡：降低 `image_max_pixels` 到 `524288`，增大 `gradient_accumulation_steps`
- 双卡 DDP 不推荐——每卡持有完整 14GB 权重副本，12GB 3080 Ti 大概率 OOM

### Q2: 输出仍为空/全零

- 检查 LoRA rank 是否太小（建议 ≥ 8）
- 检查学习率是否合适（5e-5 对 LoRA 是合理起点）
- 确认训练 loss 在下降（`plot_loss: true` 可查看）

### Q3: 推理时 waypoint 格式错误

- 基座 Qwen2.5-VL 有时会在 `[[...]]` 外输出额外文本，已在 `prompt_planner.py` 的 `_parse()` 中做正则容错
- 如果仍有问题，在 system prompt 中加强约束："严格只输出一个 Python 列表，不要任何其他文字"

### Q4: 可以用其他基座模型吗

可以。LLaMA-Factory 支持任意 Qwen2-VL / Qwen2.5-VL 系列。只需修改 `model_name_or_path` 和 `template`。注意不同模型支持的 `image_max_pixels` 上限不同。

### Q5: 报 `AutoVideoProcessor requires the Torchvision library`

**根因**：Qwen2.5-VL 的 `Qwen2_5_VLProcessor` 内部硬依赖 `torchvision`（用于视频处理分支），即使你的训练数据只有图片也会触发。日志中的 `Falling back to the slow image processor` warning 是 image processor 级别的降级，但 processor 对象构造时仍会加载 video processor 并检查后端。

**修复**：
```bash
conda activate llama-factory
pip install torchvision --index-url https://download.pytorch.org/whl/cu121
```

安装后重启训练即可。安装前是 `Qwen2VLImageProcessor`（慢速），安装后自动升级为 `Qwen2VLImageProcessorFast`（torchvision 加速）。

### Q6: 报 `Cannot open data/dataset_info.json`

**根因**：LLaMA-Factory 默认 `dataset_dir = "data"`（相对路径），在当前工作目录下找不到 `data/dataset_info.json`。即使已有 `dataset.json`（训练数据本体），仍然需要 `dataset_info.json` 作为**数据集注册表**告诉框架如何解析。

**修复**：
1. 在微调数据目录创建 `dataset_info.json`（内容见第 4.1 节）
2. 在 YAML 中添加 `dataset_dir: /data/sakura/data/UAV-VLN-FOV/finetune_data`

### Q7: 报 `ArrowInvalid: Column(/messages/[]/content) changed from array to string`

**根因**：Arrow 要求 `/messages/[]/content` 列路径上的所有值类型一致。你原始数据中 user `content` 是 `[{type:"image",...}, {type:"text",...}]`（list），assistant `content` 是 `"[[...]]"`（string）——混用导致 Arrow schema 冲突。换成 JSONL 也无效，因为 Arrow 自身的 JSON reader **同样**会拒绝这个 schema 冲突。

**修复**：将 user content 从列表格式转为 `<image>` 占位符字符串格式（第 4.3 节）。`qwen2_vl` 模板在 tokenization 时自动把 `<image>` 替换为 `images[]` 中的图像 tensor。

### Q8: 报 `deepspeed==0.19.2` 不在允许范围

**根因**：LLaMA-Factory 锁定 deepspeed 版本范围为 `>=0.10.0,<=0.16.9`，pip 默认安装最新版 0.19.2 会触发版本检查。

**修复**：
```bash
pip install 'deepspeed>=0.10.0,<=0.16.9'
```

### Q9: 报 `CUDA_HOME does not exist / nvcc not found`

**根因**：pip 安装的 nvidia-* 包不含 nvcc 编译器，deepspeed 初始化需要它来编译自定义 CUDA op。

**修复**：
```bash
conda install -c conda-forge cuda-nvcc cuda-cudart-dev -y
export CUDA_HOME=$CONDA_PREFIX
```

如果没有 sudo 权限装系统级 CUDA toolkit，conda 方案是最干净的。

### Q10: 报 `Please provide model_name_or_path`（YAML 明明有）

**根因**：不小心用了 `--deepspeed /path/to/ds.json` 作为 CLI 参数。LLaMA-Factory 不认 CLI `--deepspeed`，会干扰 argument parser 导致 YAML 文件路径被吞掉。

**修复**：把 `deepspeed` 路径写在 YAML 里（`deepspeed: /path/to/ds.json`），不要用 CLI 传。

### Q11: 报 `cannot find -lcurand`（CPUAdam 链接失败）

**根因**：`CUDA_HOME` 没指向 conda 环境的 prefix，DeepSpeed 找不到 `nvidia-curand-cu12`（pip 装 torch 时自带）。

**修复**：
```bash
export CUDA_HOME=$CONDA_PREFIX
```

### Q12: 模型初始化阶段 OOM（vision encoder 占满 GPU0）

**根因**：ZeRO-3 的 `zero.init()` 顺序初始化模型——vision encoder 先在 GPU0 分配参数（~2.7 GB float32），再 partition 到各卡。如果 `stage3_max_live_parameters` 没设 0，partition 后碎片残留导致后续 `embed_tokens` 没有连续空间。

**修复**：
1. DeepSpeed 配置中设 `"stage3_max_live_parameters": 0`、`"stage3_max_reuse_distance": 0`
2. 启动前 `export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True`

这确保初始化阶段没有任何参数常驻 GPU，全部立即 offload 到 CPU。
