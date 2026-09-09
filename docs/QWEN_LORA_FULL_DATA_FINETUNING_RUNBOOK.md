# Qwen UAV 全量数据 LoRA 微调与模型合并流程

本文档用于在服务器上复现已经跑通的完整流程：

```text
原始 clip
  -> 数据转换与 instruction 改写
  -> 数据格式自检
  -> 4 卡 DeepSpeed ZeRO-3 LoRA 微调
  -> LoRA adapter 检查
  -> 合并为完整模型
  -> 合并模型完整性检查
```

本文中的正式训练仍以现有滑窗模型作为基座：

```text
/data/sakura/models/3DG-VLN-finetuned-sliding_window_2
```

这是因为当前新增数据以道路/天气数据为主，从已经具备 `fly_to` 和滑窗能力的模型继续训练，比直接从干净的 `3DG-VLN` 开始更稳妥。以后收齐 `fly_to`、`above`、道路跟随等完整任务数据后，可以再评估是否从干净基座统一重训。

> 注意：LoRA 训练不能保证完全避免旧能力遗忘。正式训练后必须同时回归旧的 `fly_to/above` 任务和新增道路任务。

## 1. 目录规划

LoRA 项目的代码、转换结果、配置、日志、adapter 和合并模型统一放在：

```text
/data/zhusai/zhusai-tmp/codes/qwen_lora/
```

该目录现在不仅是代码目录，也作为完整的 LoRA 实验工作区。只有两类内容保留在外部：不可修改的原始数据，以及只读的基座模型。这样一次实验产生的文件不会散落到顶层 `datasets/outputs/models` 中。

推荐结构：

```text
/data/zhusai/zhusai-tmp/
├── codes/
│   └── qwen_lora/
│       ├── build_qwen_lora_dataset.py
│       ├── qwen_lora_data/
│       ├── cache/
│       │   └── deepseek_instruction_rewrites.json
│       ├── data/
│       │   └── processed/
│       │       └── qwen_uav_full_stride8_v1/
│       ├── experiments/
│       │   └── qwen_uav_full_v1/
│       │       ├── configs/
│       │       │   ├── ds_zero3_param_offload.json
│       │       │   ├── train_lora.yaml
│       │       │   └── export_merged.yaml
│       │       ├── logs/
│       │       │   ├── data_*.log
│       │       │   ├── train_*.log
│       │       │   └── export_*.log
│       │       ├── adapter/
│       │       ├── merged_model/
│       │       └── metadata/
│       └── docs/
│           └── FULL_DATA_FINETUNING_RUNBOOK.md
├── datasets/
│   └── 世界模型数据/                         # 原始数据，只读
└── （基座模型位于 /data/sakura/models，外部只读）
```

目录职责：

```text
qwen_lora_data/       数据转换程序模块
cache/                可复用的 instruction 改写缓存
data/processed/       转换后的 JSONL、图片和审计报告
experiments/<版本>/   某次训练的配置、日志、adapter 和合并模型
docs/                 操作文档
```

不要把不同版本直接混在同一个实验目录中。参数、数据或基座发生变化时新建 `qwen_uav_full_v2`，不要覆盖 `v1`。

创建目录：

```bash
PROJECT_DIR='/data/zhusai/zhusai-tmp/codes/qwen_lora'
RUN_DIR="$PROJECT_DIR/experiments/qwen_uav_full_v1"

mkdir -p \
  "$PROJECT_DIR/cache" \
  "$PROJECT_DIR/data/processed" \
  "$PROJECT_DIR/docs" \
  "$RUN_DIR/configs" \
  "$RUN_DIR/logs" \
  "$RUN_DIR/metadata"
```

`adapter/` 和 `merged_model/` 由训练、导出命令创建，不必预先建立。正式全量流程不要移动或覆盖已经跑通的 smoke3 目录；它们作为流程验证记录保留即可。

不要把 DeepSeek API key 写进 YAML、脚本、日志或 Markdown。

## 2. 固定本次正式运行的名称和路径

每次正式实验使用新版本号，避免覆盖旧实验：

```bash
PROJECT_DIR='/data/zhusai/zhusai-tmp/codes/qwen_lora'
RUN_ID='qwen_uav_full_v1'
RUN_DIR="$PROJECT_DIR/experiments/$RUN_ID"
RAW_DATA_DIR='/data/zhusai/zhusai-tmp/datasets/世界模型数据'
DATASET_DIR="$PROJECT_DIR/data/processed/qwen_uav_full_stride8_v1"
BASE_MODEL='/data/sakura/models/3DG-VLN-finetuned-sliding_window_2'
ADAPTER_DIR="$RUN_DIR/adapter"
MERGED_DIR="$RUN_DIR/merged_model"
CONFIG_DIR="$RUN_DIR/configs"
LOG_DIR="$RUN_DIR/logs"

mkdir -p "$CONFIG_DIR" "$LOG_DIR" "$RUN_DIR/metadata"
```

把本次实验路径固化下来，方便数据处理、训练和导出跨终端继续执行：

```bash
cat > "$RUN_DIR/metadata/paths.env" <<EOF
export PROJECT_DIR='$PROJECT_DIR'
export RUN_ID='$RUN_ID'
export RUN_DIR='$RUN_DIR'
export RAW_DATA_DIR='$RAW_DATA_DIR'
export DATASET_DIR='$DATASET_DIR'
export BASE_MODEL='$BASE_MODEL'
export ADAPTER_DIR='$ADAPTER_DIR'
export MERGED_DIR='$MERGED_DIR'
export CONFIG_DIR='$CONFIG_DIR'
export LOG_DIR='$LOG_DIR'
EOF
```

以后每次打开新终端，先执行：

```bash
source /data/zhusai/zhusai-tmp/codes/qwen_lora/experiments/qwen_uav_full_v1/metadata/paths.env
```

本文后续命令均假定已经加载了该文件。它只保存路径，不保存 API key。

先检查空间和基础文件：

```bash
df -h /data/zhusai/zhusai-tmp
test -f "$PROJECT_DIR/build_qwen_lora_dataset.py" && echo '转换脚本存在'
test -d "$PROJECT_DIR/qwen_lora_data" && echo '转换模块存在'
test -f "$BASE_MODEL/config.json" && echo '基座模型存在'
test -f "$RAW_DATA_DIR/CarlaAir/follow_car/clip_00000000/poses.json" && echo '原始数据可读'
```

## 3. 全量数据转换

### 3.1 参数解释

正式转换采用：

```text
step_interval=4   每4帧选一张训练画面；8 FPS数据中约每0.5秒一张
future_stride=8   相邻监督航点相隔8帧；5个输出点约覆盖5秒
num_waypoints=5   模型每次固定续写5点
pending=0..5      覆盖队列为空到仍有5个待执行点
one_per_image     每个“帧×天气”只保留一个pending版本
weathers=all      同步展开所有可用天气
```

`step_interval` 只控制训练图片密度；真正决定5点轨迹覆盖时长的是 `future_stride`。当前数据约8 FPS，`future_stride=8` 比冒烟数据使用的3更适合约5秒的Qwen推理延迟。

### 3.2 设置 DeepSeek key

```bash
conda activate llama-factory
cd /data/zhusai/zhusai-tmp/codes/qwen_lora

read -s -p 'DEEPSEEK_API_KEY: ' DEEPSEEK_API_KEY
export DEEPSEEK_API_KEY
echo
```

### 3.3 开始全量处理

第一次运行不要加 `--overwrite`。如果目标目录已经存在，换一个新的版本号；只有明确确认要丢弃旧转换结果时才使用 `--overwrite`。

```bash
set -o pipefail
DATA_LOG="$LOG_DIR/data_$(date +%Y%m%d_%H%M%S).log"

python "$PROJECT_DIR/build_qwen_lora_dataset.py" \
  --data-root "$RAW_DATA_DIR" \
  --output "$DATASET_DIR" \
  --weathers all \
  --step-interval 4 \
  --future-stride 8 \
  --num-waypoints 5 \
  --pending-counts 0,1,2,3,4,5 \
  --pending-policy one_per_image \
  --pending-seed 20260907 \
  --rewrite-instructions \
  --rewrite-model deepseek-v4-flash \
  --rewrite-reasoning-effort high \
  --rewrite-thinking enabled \
  --rewrite-cache "$PROJECT_DIR/cache/deepseek_instruction_rewrites.json" \
  2>&1 | tee "$DATA_LOG"
```

记录转换程序版本，便于以后复现：

```bash
sha256sum \
  "$PROJECT_DIR/build_qwen_lora_dataset.py" \
  "$PROJECT_DIR"/qwen_lora_data/*.py \
  > "$RUN_DIR/metadata/converter_sha256.txt"
```

全量运行时不要再传：

```text
--max-clips 3
```

当前转换器不会预先打乱 `train/val/test.jsonl`，行序保持为 clip、帧、天气，便于人工检查。训练阶段的样本随机化交给 LLaMA-Factory。

## 4. 数据转换后的强制检查

### 4.1 自动验证报告

```bash
python -m json.tool "$DATASET_DIR/audit/validation_report.json"
python -m json.tool "$DATASET_DIR/dataset_info.json"
python -m json.tool "$DATASET_DIR/conversion_report.json" | less
```

必须满足：

```text
valid = true
error_count = 0
pending_policy = one_per_image
checked_samples = unique_image_pairs
pending_distribution中的0、1、2、3、4、5均有样本
```

### 4.2 检查三个 split

```bash
wc -l \
  "$DATASET_DIR/train.jsonl" \
  "$DATASET_DIR/val.jsonl" \
  "$DATASET_DIR/test.jsonl"
```

正式全量训练前，`train` 和 `val` 都必须非空。split 按整个 clip 划分，同一个 clip 的不同帧和不同天气不会泄漏到不同 split。

### 4.3 人工抽查

```bash
head -n 3 "$DATASET_DIR/train.jsonl"
head -n 3 "$DATASET_DIR/audit/samples.jsonl"
```

每条训练数据应满足：

```text
messages[0] = user
messages[1] = assistant
user content以<image><image>开头
images[0] = front RGB
images[1] = 同帧front depth PNG
assistant content = 恰好5个[dx,dy,dz]
```

`pending` 是当前队列中已经规划但尚未执行的点。训练阶段由 `poses.json` 的专家未来轨迹模拟，在线阶段由 Planner 的真实剩余队列提供。

## 5. 创建4卡 DeepSpeed配置

写入：

```bash
cat > "$CONFIG_DIR/ds_zero3_param_offload.json" <<'EOF'
{
  "bf16": {
    "enabled": "auto"
  },
  "zero_optimization": {
    "stage": 3,
    "offload_param": {
      "device": "cpu",
      "pin_memory": true
    },
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
  "gradient_accumulation_steps": "auto",
  "gradient_clipping": "auto",
  "train_batch_size": "auto",
  "train_micro_batch_size_per_gpu": "auto",
  "steps_per_print": 20,
  "wall_clock_breakdown": false
}
EOF
```

这里必须只有 `offload_param`，不要添加 `offload_optimizer: cpu`。服务器此前已经验证：CPU optimizer offload 会触发 DeepSpeed 编译 `cpu_adam`，并因缺少 `libcurand.so` 失败。

检查：

```bash
grep -n 'offload' "$CONFIG_DIR/ds_zero3_param_offload.json"
```

应该只看到 `offload_param`。

## 6. 创建正式LoRA训练配置

写入：

```bash
cat > "$CONFIG_DIR/train_lora.yaml" <<EOF
model_name_or_path: /data/sakura/models/3DG-VLN-finetuned-sliding_window_2
trust_remote_code: true

stage: sft
do_train: true
do_eval: true
finetuning_type: lora

lora_target: all
lora_rank: 8
lora_alpha: 16
lora_dropout: 0.05

freeze_vision_tower: true
freeze_multi_modal_projector: true
freeze_language_model: false

deepspeed: $CONFIG_DIR/ds_zero3_param_offload.json

dataset: qwen_uav_train
eval_dataset: qwen_uav_val
dataset_dir: $DATASET_DIR
template: qwen2_vl

cutoff_len: 2048
image_max_pixels: 131072
packing: false
overwrite_cache: true
preprocessing_num_workers: 8
dataloader_num_workers: 4

output_dir: $ADAPTER_DIR
overwrite_output_dir: false
save_only_model: false

per_device_train_batch_size: 1
per_device_eval_batch_size: 1
gradient_accumulation_steps: 4
learning_rate: 5.0e-5
num_train_epochs: 3.0
lr_scheduler_type: cosine
warmup_ratio: 0.05

bf16: true
gradient_checkpointing: true
logging_steps: 10
eval_strategy: steps
eval_steps: 100
save_strategy: steps
save_steps: 100
save_total_limit: 3
load_best_model_at_end: true
metric_for_best_model: eval_loss
greater_is_better: false
plot_loss: true
report_to: none

ddp_timeout: 180000000
EOF
```

当前有效 batch size 为：

```text
每卡batch 1 × 4张GPU × 梯度累积4 = 16
```

`freeze_vision_tower=true` 是第一版正式训练的保守选择：保留Qwen既有视觉能力，用LoRA重点学习指令、深度提示和轨迹输出协议。后续若要验证“对极端天气做视觉域适配”，应另建实验版本，不要直接覆盖本版本。

如果全量转换后 `val.jsonl` 仍为空，不能使用上面的验证配置。应先检查 clip 数量与 split 报告；仅在确认这是预期行为后，才把 `do_eval` 改为 `false`，并删除 `eval_dataset`、`eval_strategy`、`eval_steps`、`load_best_model_at_end`、`metric_for_best_model` 和 `greater_is_better`。

## 7. 启动4卡正式训练

训练前停止占用4张GPU的 vLLM、GroundingDINO 或其他进程：

```bash
nvidia-smi
```

确认环境：

```bash
conda activate llama-factory
python -c "import torch, deepspeed; print('torch', torch.__version__, 'deepspeed', deepspeed.__version__)"
```

开始训练并保存日志：

```bash
cd "$PROJECT_DIR"
export CUDA_HOME="$CONDA_PREFIX"
export TOKENIZERS_PARALLELISM=false
set -o pipefail

TRAIN_LOG="$LOG_DIR/train_$(date +%Y%m%d_%H%M%S).log"

FORCE_TORCHRUN=1 \
CUDA_VISIBLE_DEVICES=0,1,2,3 \
llamafactory-cli train \
  "$CONFIG_DIR/train_lora.yaml" \
  2>&1 | tee "$TRAIN_LOG"
```

不要设置 `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True`；当前服务器日志已显示该平台不支持它。

训练成功后检查：

```bash
find "$ADAPTER_DIR" -maxdepth 2 -type f | sort | less
ls -lh \
  "$ADAPTER_DIR/adapter_config.json" \
  "$ADAPTER_DIR/adapter_model.safetensors"
```

根目录中的 `adapter_model.safetensors` 是最终LoRA；`checkpoint-*` 用于断点续训。

如训练中断，不要删除现有目录。找到最新checkpoint：

```bash
find "$ADAPTER_DIR" -maxdepth 1 -type d -name 'checkpoint-*' | sort -V | tail -n 1
```

然后将该路径写入训练YAML的 `resume_from_checkpoint` 后继续。不要同时使用 `overwrite_output_dir: true`。

## 8. 创建合并配置

确认LoRA训练成功后写入：

```bash
cat > "$CONFIG_DIR/export_merged.yaml" <<EOF
model_name_or_path: /data/sakura/models/3DG-VLN-finetuned-sliding_window_2
adapter_name_or_path: $ADAPTER_DIR
trust_remote_code: true

template: qwen2_vl
finetuning_type: lora

export_dir: $MERGED_DIR
export_size: 4
export_device: cpu
export_legacy_format: false
EOF
```

不要给导出配置添加 `quantization_bit: 4`。这里需要将LoRA合并成完整BF16模型。

## 9. 合并为完整模型

先检查CPU内存：

```bash
free -h
```

建议至少有约25～30 GB可用内存。开始合并：

```bash
conda activate llama-factory
cd "$PROJECT_DIR"
set -o pipefail

EXPORT_LOG="$LOG_DIR/export_$(date +%Y%m%d_%H%M%S).log"

llamafactory-cli export \
  "$CONFIG_DIR/export_merged.yaml" \
  2>&1 | tee "$EXPORT_LOG"
```

## 10. 合并模型检查

```bash
ls -lh "$MERGED_DIR"
du -sh "$MERGED_DIR"
test -f "$MERGED_DIR/config.json" && echo 'config OK'
test -f "$MERGED_DIR/model.safetensors.index.json" && echo 'weights index OK'
test -f "$MERGED_DIR/preprocessor_config.json" && echo 'vision processor OK'
test -f "$MERGED_DIR/tokenizer.json" && echo 'tokenizer OK'
```

轻量读取配置与视觉处理器：

```bash
python - <<'PY'
from transformers import AutoConfig, AutoProcessor

path = "/data/zhusai/zhusai-tmp/codes/qwen_lora/experiments/qwen_uav_full_v1/merged_model"
config = AutoConfig.from_pretrained(path, trust_remote_code=True)
processor = AutoProcessor.from_pretrained(path, trust_remote_code=True)

print("model_type:", config.model_type)
print("architectures:", config.architectures)
print("processor:", type(processor).__name__)
print("merged model metadata OK")
PY
```

最终可独立部署模型位于：

```text
/data/zhusai/zhusai-tmp/codes/qwen_lora/experiments/qwen_uav_full_v1/merged_model
```

建议再为合并结果保存校验和：

```bash
find "$MERGED_DIR" -maxdepth 1 -type f -print0 \
  | sort -z \
  | xargs -0 sha256sum \
  > "$RUN_DIR/metadata/merged_model_sha256.txt"
```

## 11. 可选：vLLM加载冒烟测试

进入vLLM环境，并使用未占用端口：

```bash
conda deactivate
conda activate vlm

CUDA_VISIBLE_DEVICES=0,1 \
vllm serve \
  "$MERGED_DIR" \
  --host 0.0.0.0 \
  --port 8005 \
  --tensor-parallel-size 2 \
  --max-model-len 4096 \
  --gpu-memory-utilization 0.95 \
  --limit-mm-per-prompt '{"image": 2}'
```

另一个终端检查：

```bash
curl http://127.0.0.1:8005/v1/models
```

加载成功只说明模型文件与接口正常。正式效果判断还必须进行：

```text
1. 道路直行与转弯任务
2. 不同天气任务
3. pending=0与pending>0续写
4. 旧fly_to任务
5. 旧above任务
6. 长时间运行时队列是否连续补点
```

## 12. 每次正式实验必须保留的内容

不要只保留最终模型。每个版本至少保留：

```text
转换脚本的Git版本或文件哈希
数据转换命令与data log
conversion_report.json
validation_report.json
dataset_info.json
DeepSpeed JSON
训练YAML与train log
LoRA adapter
trainer_state.json与train_results.json
导出YAML与export log
合并后的完整模型
离线和AirSim回归测试结果
```

建议每次只增加版本号：

```text
data/processed/qwen_uav_full_stride8_v1
experiments/qwen_uav_full_v1
```

参数、数据或基座模型发生变化时创建 `v2`，不要覆盖 `v1`。

## 13. 正式运行顺序速查

```text
1. 创建目录并写入 metadata/paths.env
2. source paths.env，设置 DEEPSEEK_API_KEY
3. 转换全量数据，保存 data 日志和转换器哈希
4. 检查 validation_report、dataset_info、split 和抽样内容
5. 创建并检查 DeepSpeed JSON
6. 创建训练 YAML
7. 4卡启动LoRA训练，检查最终adapter
8. 创建导出 YAML
9. 在CPU上合并模型
10. 检查合并模型文件、处理器和校验和
11. 可选执行vLLM加载测试
12. 回归道路、天气、pending、fly_to和above任务
```

## 14. 已验证的故障结论

### 单卡加载OOM

11.63 GiB单卡不足以加载当前完整模型训练。正式训练使用：

```text
4卡 + FORCE_TORCHRUN + DeepSpeed ZeRO-3
```

### DeepSpeed `cpu_adam` 编译失败

错误：

```text
ld: cannot find -lcurand
Error building extension 'cpu_adam'
```

处理：只保留模型参数CPU卸载，不使用optimizer CPU卸载。本文件中的 `ds_zero3_param_offload.json` 已采用验证通过的配置。

### 没有 `eval_loss` 图

冒烟训练没有验证集并设置 `do_eval: false` 时，这是正常现象。正式全量数据应确保 `val.jsonl` 非空，并启用本文件中的验证配置。

### 5步冒烟模型不能代表效果

5步训练只证明数据加载、两图预处理、LoRA、DeepSpeed、保存和合并流程跑通，不代表导航能力已经提升。
