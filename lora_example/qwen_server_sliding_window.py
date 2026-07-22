import ast
import base64
import io
import os
import re
import time

from flask import Flask, jsonify, request
from PIL import Image
import torch
from transformers import AutoModelForImageTextToText, AutoProcessor


app = Flask(__name__)

MODEL_PATH = os.environ.get("MODEL_PATH", "/data/sakura/models/3DG-VLN-sliding-window")
DEFAULT_MAX_ADDITIONAL = int(os.environ.get("MAX_ADDITIONAL", "5"))
MAX_PENDING = int(os.environ.get("MAX_PENDING", "3"))

print(f"[Qwen Sliding Server] Loading model: {MODEL_PATH}")
processor = AutoProcessor.from_pretrained(MODEL_PATH, trust_remote_code=True)
model = AutoModelForImageTextToText.from_pretrained(
    MODEL_PATH,
    device_map="auto",
    torch_dtype=torch.float16,
    trust_remote_code=True,
)
model.eval()
device = model.device
print(f"[Qwen Sliding Server] Loaded on {device}")


def normalize_waypoints(value, limit=None):
    waypoints = []
    if value is None:
        return waypoints
    if isinstance(value, str):
        try:
            value = ast.literal_eval(value)
        except Exception:
            return waypoints
    if not isinstance(value, (list, tuple)):
        return waypoints
    for item in value:
        if not isinstance(item, (list, tuple)) or len(item) < 3:
            continue
        try:
            wp = [round(float(item[0]), 3), round(float(item[1]), 3), round(float(item[2]), 3)]
        except Exception:
            continue
        if any(abs(v) >= 1e-6 for v in wp):
            waypoints.append(wp)
        if limit is not None and len(waypoints) >= int(limit):
            break
    return waypoints


def format_waypoints_json(waypoints):
    rows = []
    for wp in normalize_waypoints(waypoints):
        rows.append("[" + ", ".join(f"{v:.2f}" for v in wp) + "]")
    return "[" + ", ".join(rows) + "]"


def build_sliding_prompt(instruction, pending_waypoints=None, max_additional=DEFAULT_MAX_ADDITIONAL):
    pending = normalize_waypoints(pending_waypoints, limit=MAX_PENDING)
    lines = [f"Instruction: {(instruction or '').strip()}"]
    if pending:
        lines.append(
            "Pending incremental body-frame waypoints: "
            f"{format_waypoints_json(pending)}"
        )
    lines.extend([
        (
            f"Output up to {int(max_additional)} additional incremental "
            "body-frame waypoints as a JSON list."
        ),
        (
            "Each waypoint must be [dx, dy, dz], where the first pending "
            "or output waypoint is relative to the current drone position "
            "and each following waypoint is relative to the previous waypoint."
        ),
        "Do not output any other text.",
    ])
    return "\n".join(lines)


def decode_image(value):
    if not value:
        return None
    if isinstance(value, dict):
        value = value.get("url", "")
    if isinstance(value, str) and value.startswith("data:image/"):
        value = value.split(",", 1)[1]
    if isinstance(value, str):
        return Image.open(io.BytesIO(base64.b64decode(value))).convert("RGB")
    return None


def parse_waypoints(text, max_count=DEFAULT_MAX_ADDITIONAL):
    match = re.search(r"\[\s*\[.*?\]\s*\]", text or "", re.DOTALL)
    if not match:
        return []
    try:
        parsed = ast.literal_eval(match.group(0))
    except Exception:
        return []
    return normalize_waypoints(parsed, limit=max_count)


def generate_from_images_and_prompt(front_img, down_img, prompt, max_tokens=256, temperature=0.0):
    images = [img for img in [front_img, down_img] if img is not None]
    if not images:
        qwen_messages = [{"role": "user", "content": prompt}]
        rendered_prompt = processor.apply_chat_template(
            qwen_messages,
            tokenize=False,
            add_generation_prompt=True,
        )
        inputs = processor(text=[rendered_prompt], return_tensors="pt", padding=True)
    else:
        content = [{"type": "image"} for _ in images]
        content.append({"type": "text", "text": prompt})
        qwen_messages = [{"role": "user", "content": content}]
        rendered_prompt = processor.apply_chat_template(
            qwen_messages,
            tokenize=False,
            add_generation_prompt=True,
        )
        inputs = processor(
            text=[rendered_prompt],
            images=[images],
            return_tensors="pt",
            padding=True,
        )

    for key, value in inputs.items():
        if isinstance(value, torch.Tensor):
            inputs[key] = value.to(device)

    t0 = time.time()
    with torch.no_grad():
        outputs = model.generate(
            **inputs,
            max_new_tokens=int(max_tokens),
            do_sample=(float(temperature) > 0.0),
            temperature=float(temperature) if float(temperature) > 0.0 else None,
        )
    elapsed = time.time() - t0

    input_len = inputs["input_ids"].shape[1]
    new_token_ids = outputs[0][input_len:]
    generated = processor.decode(new_token_ids, skip_special_tokens=True).strip()
    return generated, elapsed, input_len, len(new_token_ids)


@app.route("/v1/chat/completions", methods=["POST"])
@app.route("/v1/chat/completions/", methods=["POST"])
def chat_completions():
    data = request.json or {}
    messages = data.get("messages", [])

    pil_images = []
    text_parts = []
    for msg in messages:
        content = msg.get("content", "")
        if isinstance(content, str):
            text_parts.append(content)
            continue
        if not isinstance(content, list):
            continue
        for item in content:
            item_type = item.get("type", "")
            if item_type == "text":
                text_parts.append(item.get("text", ""))
            elif item_type == "image_url":
                img = decode_image(item.get("image_url", {}))
                if img is not None:
                    pil_images.append(img)

    prompt = "\n".join(text_parts)
    front_img = pil_images[0] if len(pil_images) >= 1 else None
    down_img = pil_images[1] if len(pil_images) >= 2 else front_img
    max_tokens = int(data.get("max_tokens") or data.get("max_completion_tokens") or 256)
    temperature = float(data.get("temperature", 0.0))

    generated, elapsed, input_len, output_len = generate_from_images_and_prompt(
        front_img,
        down_img,
        prompt,
        max_tokens=max_tokens,
        temperature=temperature,
    )
    print(
        f"  [chat-sliding] {len(messages)} msgs, {len(pil_images)} imgs, "
        f"{output_len} tokens in {elapsed:.2f}s"
    )

    return jsonify({
        "id": f"chatcmpl-{int(time.time())}",
        "object": "chat.completion",
        "created": int(time.time()),
        "model": data.get("model", os.path.basename(MODEL_PATH) or "qwen-sliding-window"),
        "choices": [{
            "index": 0,
            "message": {"role": "assistant", "content": generated},
            "finish_reason": "stop",
        }],
        "usage": {
            "prompt_tokens": input_len,
            "completion_tokens": output_len,
            "total_tokens": input_len + output_len,
        },
        "time_s": round(elapsed, 2),
    })


@app.route("/plan", methods=["POST"])
def plan():
    data = request.json or {}
    front_img = decode_image(data.get("front_image"))
    down_img = decode_image(data.get("down_image")) or front_img
    instruction = data.get("instruction", "Fly to the target")
    pending = normalize_waypoints(
        data.get("pending_waypoints", data.get("pending", [])),
        limit=MAX_PENDING,
    )
    max_additional = int(data.get("max_additional", data.get("waypoint_count", DEFAULT_MAX_ADDITIONAL)))
    max_tokens = int(data.get("max_tokens", 256))
    temperature = float(data.get("temperature", 0.0))

    prompt = build_sliding_prompt(
        instruction=instruction,
        pending_waypoints=pending,
        max_additional=max_additional,
    )
    generated, elapsed, _input_len, _output_len = generate_from_images_and_prompt(
        front_img,
        down_img,
        prompt,
        max_tokens=max_tokens,
        temperature=temperature,
    )
    waypoints = parse_waypoints(generated, max_count=max_additional)
    print(
        f"  [plan-sliding] pending={len(pending)} -> {len(waypoints)} "
        f"waypoints in {elapsed:.2f}s"
    )

    return jsonify({
        "success": True,
        "waypoints": waypoints,
        "waypoint_format": "incremental_body",
        "raw_output": generated,
        "time_s": round(elapsed, 2),
        "device": str(device),
    })


@app.route("/health", methods=["GET"])
def health():
    return jsonify({
        "status": "ok",
        "model_path": MODEL_PATH,
        "device": str(device),
        "waypoint_format": "incremental_body",
        "max_pending": MAX_PENDING,
        "max_additional": DEFAULT_MAX_ADDITIONAL,
    })


if __name__ == "__main__":
    port = int(os.environ.get("PORT", "8004"))
    print(f"[Qwen Sliding Server] Starting on port {port}...")
    app.run(host="0.0.0.0", port=port, threaded=False)
