"""Compact appearance signatures for MissionMemory.

The signature is deliberately small and illumination-aware:
- Lab a/b histograms keep chromatic information while ignoring most brightness.
- HSV hue uses saturation gating, so gray/low-saturation pixels do not invent
  unstable hue values.
- RGB ratios and relative object-background ratios survive simple shadows.
"""

from __future__ import annotations

import math
from typing import Iterable, Optional

import numpy as np

from agent.functions.memory.schemas import AppearancePrototype


def build_appearance_signature(
    image,
    bbox,
    *,
    view: str = "unknown",
    max_size: int = 64,
) -> Optional[AppearancePrototype]:
    """Build a compact appearance prototype from a bbox crop."""
    if image is None or bbox is None or not hasattr(image, "size"):
        return None
    crop = _safe_crop(image, bbox)
    if crop is None:
        return None
    if max(crop.size) > int(max_size):
        crop.thumbnail((int(max_size), int(max_size)))

    rgb = np.asarray(crop.convert("RGB"), dtype=np.float32) / 255.0
    if rgb.size == 0:
        return None

    lab_hist = _lab_ab_hist(crop)
    hue_hist, mean_sat, mean_val = _hue_hist_saturation_gated(rgb)
    rgb_ratio = _rgb_ratio(rgb)
    texture = _texture_density(rgb)
    background_ratio = _background_rgb_ratio(image, bbox)
    relative_ratio = [
        round(float(rgb_ratio[i] - background_ratio[i]), 4)
        for i in range(3)
    ]
    reliability = _appearance_reliability(rgb, mean_sat=mean_sat, mean_val=mean_val)
    return AppearancePrototype(
        lab_ab_hist=lab_hist,
        hue_hist=hue_hist,
        rgb_ratio=[round(float(v), 4) for v in rgb_ratio],
        relative_rgb_ratio=relative_ratio,
        texture=round(float(texture), 4),
        reliability=round(float(reliability), 4),
        view=str(view or "unknown"),
    )


def appearance_similarity(query: Optional[AppearancePrototype], prototypes: Iterable[AppearancePrototype]) -> float:
    """Return the best similarity against a prototype set."""
    if query is None:
        return 0.0
    best = 0.0
    for proto in prototypes or []:
        best = max(best, _similarity_one(query, proto))
    return max(0.0, min(1.0, best))


def merge_prototype_set(
    prototypes: list[AppearancePrototype],
    query: Optional[AppearancePrototype],
    *,
    max_prototypes: int = 5,
    merge_threshold: float = 0.78,
) -> None:
    """Merge query into the closest prototype, or append it as a new view state."""
    if query is None:
        return
    if not prototypes:
        prototypes.append(query)
        return
    best_index = -1
    best_similarity = -1.0
    for idx, proto in enumerate(prototypes):
        sim = _similarity_one(query, proto)
        if sim > best_similarity:
            best_similarity = sim
            best_index = idx
    if best_index >= 0 and best_similarity >= float(merge_threshold):
        _merge_into(prototypes[best_index], query)
        return
    prototypes.append(query)
    prototypes.sort(key=lambda p: (float(p.reliability), int(p.observation_count)), reverse=True)
    del prototypes[int(max_prototypes):]


def _safe_crop(image, bbox):
    width, height = image.size
    if width <= 1 or height <= 1:
        return None
    x1, y1, x2, y2 = [int(round(float(v))) for v in bbox[:4]]
    x1 = max(0, min(width - 1, x1))
    x2 = max(0, min(width, x2))
    y1 = max(0, min(height - 1, y1))
    y2 = max(0, min(height, y2))
    if x2 <= x1 + 1 or y2 <= y1 + 1:
        return None
    return image.crop((x1, y1, x2, y2))


def _lab_ab_hist(crop) -> list[float]:
    try:
        lab = np.asarray(crop.convert("LAB"), dtype=np.uint8)
    except Exception:
        return [0.0] * 16
    if lab.size == 0:
        return [0.0] * 16
    a = lab[:, :, 1].reshape(-1)
    b = lab[:, :, 2].reshape(-1)
    hist_a, _ = np.histogram(a, bins=8, range=(0, 256))
    hist_b, _ = np.histogram(b, bins=8, range=(0, 256))
    hist = np.concatenate([hist_a.astype(np.float32), hist_b.astype(np.float32)])
    total = float(hist.sum())
    if total <= 0.0:
        return [0.0] * 16
    return [round(float(v / total), 5) for v in hist]


def _hue_hist_saturation_gated(rgb: np.ndarray) -> tuple[list[float], float, float]:
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    maxc = np.max(rgb, axis=2)
    minc = np.min(rgb, axis=2)
    delta = maxc - minc
    sat = np.where(maxc > 1e-6, delta / np.maximum(maxc, 1e-6), 0.0)
    val = maxc
    hue = np.zeros_like(maxc)

    mask = delta > 1e-6
    r_mask = mask & (maxc == r)
    g_mask = mask & (maxc == g)
    b_mask = mask & (maxc == b)
    hue[r_mask] = ((g[r_mask] - b[r_mask]) / delta[r_mask]) % 6.0
    hue[g_mask] = ((b[g_mask] - r[g_mask]) / delta[g_mask]) + 2.0
    hue[b_mask] = ((r[b_mask] - g[b_mask]) / delta[b_mask]) + 4.0
    hue = hue / 6.0

    weights = np.clip((sat - 0.15) / 0.85, 0.0, 1.0)
    if float(weights.sum()) <= 1e-6:
        return [0.0] * 12, float(np.mean(sat)), float(np.mean(val))
    hist, _ = np.histogram(hue.reshape(-1), bins=12, range=(0.0, 1.0), weights=weights.reshape(-1))
    hist = hist.astype(np.float32)
    hist = hist / max(float(hist.sum()), 1e-6)
    return [round(float(v), 5) for v in hist], float(np.mean(sat)), float(np.mean(val))


def _rgb_ratio(rgb: np.ndarray) -> list[float]:
    mean = np.mean(rgb.reshape(-1, 3), axis=0)
    total = float(np.sum(mean))
    if total <= 1e-6:
        return [0.3333, 0.3333, 0.3333]
    return [float(v / total) for v in mean]


def _background_rgb_ratio(image, bbox) -> list[float]:
    width, height = image.size
    x1, y1, x2, y2 = [int(round(float(v))) for v in bbox[:4]]
    bw = max(2, x2 - x1)
    bh = max(2, y2 - y1)
    pad_x = max(4, int(0.4 * bw))
    pad_y = max(4, int(0.4 * bh))
    ox1 = max(0, x1 - pad_x)
    oy1 = max(0, y1 - pad_y)
    ox2 = min(width, x2 + pad_x)
    oy2 = min(height, y2 + pad_y)
    if ox2 <= ox1 + 1 or oy2 <= oy1 + 1:
        return [0.3333, 0.3333, 0.3333]
    region = np.asarray(image.crop((ox1, oy1, ox2, oy2)).convert("RGB"), dtype=np.float32) / 255.0
    mask = np.ones(region.shape[:2], dtype=bool)
    ix1 = max(0, x1 - ox1)
    iy1 = max(0, y1 - oy1)
    ix2 = min(region.shape[1], x2 - ox1)
    iy2 = min(region.shape[0], y2 - oy1)
    mask[iy1:iy2, ix1:ix2] = False
    pixels = region[mask]
    if pixels.size == 0:
        return [0.3333, 0.3333, 0.3333]
    mean = np.mean(pixels.reshape(-1, 3), axis=0)
    total = float(np.sum(mean))
    if total <= 1e-6:
        return [0.3333, 0.3333, 0.3333]
    return [float(v / total) for v in mean]


def _texture_density(rgb: np.ndarray) -> float:
    gray = 0.299 * rgb[:, :, 0] + 0.587 * rgb[:, :, 1] + 0.114 * rgb[:, :, 2]
    if gray.shape[0] <= 1 or gray.shape[1] <= 1:
        return 0.0
    dx = np.abs(gray[:, 1:] - gray[:, :-1])
    dy = np.abs(gray[1:, :] - gray[:-1, :])
    return max(0.0, min(1.0, float((np.mean(dx) + np.mean(dy)) * 4.0)))


def _appearance_reliability(rgb: np.ndarray, *, mean_sat: float, mean_val: float) -> float:
    exposure = 1.0
    if mean_val < 0.12:
        exposure *= max(0.15, mean_val / 0.12)
    if mean_val > 0.92:
        exposure *= max(0.15, (1.0 - mean_val) / 0.08)
    sat_score = max(0.0, min(1.0, (float(mean_sat) - 0.05) / 0.45))
    contrast = float(np.std(rgb))
    contrast_score = max(0.0, min(1.0, contrast / 0.18))
    # 低饱和或强曝光时颜色不可靠，但纹理仍保留少量作用。
    return max(0.05, min(1.0, exposure * (0.65 * sat_score + 0.35 * contrast_score)))


def _similarity_one(query: AppearancePrototype, proto: AppearancePrototype) -> float:
    color_weight = min(float(query.reliability), float(proto.reliability))
    lab = _hist_intersection(query.lab_ab_hist, proto.lab_ab_hist)
    hue = _hist_intersection(query.hue_hist, proto.hue_hist)
    ratio = 1.0 - min(1.0, _l1(query.rgb_ratio, proto.rgb_ratio) / 1.2)
    rel = 1.0 - min(1.0, _l1(query.relative_rgb_ratio, proto.relative_rgb_ratio) / 1.2)
    texture = 1.0 - min(1.0, abs(float(query.texture) - float(proto.texture)) / 0.8)
    appearance = 0.35 * lab + 0.20 * hue + 0.20 * ratio + 0.15 * rel + 0.10 * texture
    return max(0.0, min(1.0, 0.35 * texture + 0.65 * color_weight * appearance))


def _hist_intersection(a: list[float], b: list[float]) -> float:
    if not a or not b:
        return 0.0
    n = min(len(a), len(b))
    return max(0.0, min(1.0, sum(min(float(a[i]), float(b[i])) for i in range(n))))


def _l1(a: list[float], b: list[float]) -> float:
    n = min(len(a or []), len(b or []))
    if n <= 0:
        return 1.0
    return sum(abs(float(a[i]) - float(b[i])) for i in range(n))


def _merge_into(base: AppearancePrototype, query: AppearancePrototype) -> None:
    n0 = max(1, int(base.observation_count))
    n1 = max(1, int(query.observation_count))
    denom = float(n0 + n1)

    def merge_list(a, b):
        n = min(len(a or []), len(b or []))
        if n <= 0:
            return list(a or b or [])
        return [round(float((a[i] * n0 + b[i] * n1) / denom), 5) for i in range(n)]

    base.lab_ab_hist = merge_list(base.lab_ab_hist, query.lab_ab_hist)
    base.hue_hist = merge_list(base.hue_hist, query.hue_hist)
    base.rgb_ratio = merge_list(base.rgb_ratio, query.rgb_ratio)
    base.relative_rgb_ratio = merge_list(base.relative_rgb_ratio, query.relative_rgb_ratio)
    base.texture = round(float((base.texture * n0 + query.texture * n1) / denom), 4)
    base.reliability = round(float(max(base.reliability, query.reliability)), 4)
    base.observation_count = n0 + n1
    if query.view and query.view != "unknown":
        base.view = query.view
