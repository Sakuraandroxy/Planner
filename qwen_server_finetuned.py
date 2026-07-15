import os, sys, json, time, re, ast
import base64, io
from flask import Flask, request, jsonify
from PIL import Image
import torch
import numpy as np
from transformers import AutoProcessor, AutoModelForImageTextToText

app = Flask(__name__)

print("[Qwen Server] Loading model...")
model_path = os.environ.get("QWEN_MODEL_PATH", "/data/sakura/models/3DG-VLN-finetuned-1")
processor = AutoProcessor.from_pretrained(model_path, trust_remote_code=True)
model = AutoModelForImageTextToText.from_pretrained(
    model_path, device_map="auto", torch_dtype=torch.float16, trust_remote_code=True
)
model.eval()
device = model.device
print(f"[Qwen Server] Loaded on {device}")


def build_waypoint_prompt(instruction: str, waypoint_count: int = 5) -> str:
    """Build the same user prompt style used during waypoint fine-tuning."""
    return (
        f"Instruction: {instruction}\n"
        f"Output exactly {waypoint_count} cumulative body-frame waypoints as a JSON list.\n"
        "Each waypoint must be [dx, dy, dz].\n"
        "Do not output any other text."
    )


# ═══════════════════════════════════════════════════════
#  /v1/chat/completions — OpenAI 兼容（文本 + 图片）
# ═══════════════════════════════════════════════════════

@app.route("/v1/chat/completions", methods=["POST"])
@app.route("/v1/chat/completions/", methods=["POST"])
def chat_completions():
    data = request.json
    messages = data.get("messages", [])

    pil_images = []
    text_parts = []

    for msg in messages:
        content = msg.get("content", "")
        if isinstance(content, str):
            text_parts.append(content)
        elif isinstance(content, list):
            for item in content:
                t = item.get("type", "")
                if t == "text":
                    text_parts.append(item.get("text", ""))
                elif t == "image_url":
                    url = item.get("image_url", {}).get("url", "")
                    if url.startswith("data:image/"):
                        b64 = url.split(",", 1)[1]
                        pil_images.append(
                            Image.open(io.BytesIO(base64.b64decode(b64))).convert("RGB")
                        )

    combined_text = "\n".join(text_parts)
    has_images = len(pil_images) > 0

    if has_images:
        content_list = [{"type": "image"} for _ in pil_images]
        content_list.append({"type": "text", "text": combined_text})
        qwen_messages = [{"role": "user", "content": content_list}]
        prompt = processor.apply_chat_template(
            qwen_messages, tokenize=False, add_generation_prompt=True
        )
        inputs = processor(
            text=[prompt], images=[pil_images],
            return_tensors="pt", padding=True,
        )
    else:
        qwen_messages = [{"role": "user", "content": combined_text}]
        prompt = processor.apply_chat_template(
            qwen_messages, tokenize=False, add_generation_prompt=True
        )
        inputs = processor(text=[prompt], return_tensors="pt", padding=True)

    for k, v in inputs.items():
        if isinstance(v, torch.Tensor):
            inputs[k] = v.to(device)

    t0 = time.time()
    max_tokens = int(data.get("max_tokens") or data.get("max_completion_tokens", 2048))
    temperature = float(data.get("temperature", 0.0))

    with torch.no_grad():
        outputs = model.generate(
            **inputs,
            max_new_tokens=max_tokens,
            do_sample=(temperature > 0.0),
            temperature=temperature if temperature > 0.0 else None,
        )
    elapsed = time.time() - t0

    input_len = inputs["input_ids"].shape[1]
    new_token_ids = outputs[0][input_len:]
    generated = processor.decode(new_token_ids, skip_special_tokens=True).strip()
    output_len = len(new_token_ids)

    print(f"  [chat] {len(messages)} msgs, {len(pil_images)} imgs, "
          f"{output_len} tokens in {elapsed:.2f}s")

    return jsonify({
        "id": f"chatcmpl-{int(time.time())}",
        "object": "chat.completion",
        "created": int(time.time()),
        "model": "qwen-vl",
        "choices": [{
            "index": 0,
            "message": {"role": "assistant", "content": generated},
            "finish_reason": "stop",
        }],
        "usage": {
            "prompt_tokens": input_len,
            "completion_tokens": output_len,
            "total_tokens": input_len + output_len,
        }
    })


# ═══════════════════════════════════════════════════════
#  /plan — 保留兼容（qwen_planner 仍用此端点）
# ═══════════════════════════════════════════════════════

@app.route("/plan", methods=["POST"])
def plan():
    data = request.json
    front_b64 = data["front_image"]
    down_b64 = data.get("down_image", front_b64)
    instruction = data.get("instruction", "Fly to the target")
    waypoint_count = int(data.get("waypoint_count", 5))

    def decode_b64(b64):
        return Image.open(io.BytesIO(base64.b64decode(b64))).convert("RGB")

    front_img = decode_b64(front_b64)
    down_img = decode_b64(down_b64)

    desc = build_waypoint_prompt(instruction, waypoint_count)

    messages = [{
        "role": "user",
        "content": [
            {"type": "image"},
            {"type": "image"},
            {"type": "text", "text": desc}
        ]
    }]
    prompt = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)

    inputs = processor(text=[prompt], images=[[front_img, down_img]],
                       return_tensors="pt", padding=True)
    for k, v in inputs.items():
        if isinstance(v, torch.Tensor):
            inputs[k] = v.to(device)

    t0 = time.time()
    with torch.no_grad():
        outputs = model.generate(**inputs, max_new_tokens=256, do_sample=False)
    elapsed = time.time() - t0

    result = processor.batch_decode(outputs, skip_special_tokens=True)[0]
    match = re.search(r'\[\[.*?\]\]', result, re.DOTALL)
    waypoints = []
    if match:
        try:
            waypoints = ast.literal_eval(match.group())
        except Exception:
            pass

    print(f"  [plan] '{instruction}' -> {len(waypoints)} waypoints in {elapsed:.2f}s")

    return jsonify({
        "success": True,
        "waypoints": waypoints,
        "raw_output": result,
        "time_s": round(elapsed, 2),
        "device": str(device)
    })


# ═══════════════════════════════════════════════════════
#  健康检查
# ═══════════════════════════════════════════════════════

@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok", "device": str(device)})


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 8004))
    print(f"[Qwen Server] Starting on port {port}...")
    app.run(host="0.0.0.0", port=port, threaded=False)
