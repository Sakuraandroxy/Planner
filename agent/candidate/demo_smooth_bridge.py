"""Smooth-bridge trajectory perturbation demo.

This script is intentionally standalone. It mirrors the planner waypoint format:
all waypoints are cumulative offsets in the UAV body frame, and the current UAV
position is the implicit start point [0, 0, 0].
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import List, Optional

import numpy as np


# Planner-style input: cumulative body-frame waypoints, excluding implicit start.
# x: forward, y: right, z: down/up depending on your convention in the planner.
BASE_WAYPOINTS = [
    [4.0, 0.0, 0.0],
    [8.0, 1.4, 0.5],
    [12.0, 4.0, 2.0],
    [16.0, 7.1, 2.0],
    [20.0, 7.7, 2.0],
    [24.0, 7.2, 2.5],
    [28.0, 8.0, 2.5],
]


def generate_smooth_bridge_paths(
    waypoints: List[List[float]],
    num_paths: int,
    lateral_sigma_m: float,
    vertical_sigma_m: float,
    smooth_length_scale: float,
    seed: Optional[int],
    max_attempts: int,
    max_turn_deg: float,
    max_path_length_ratio: float,
    progress_tolerance_m: float,
) -> List[List[List[float]]]:
    """Generate fixed-endpoint smooth perturbations.

    Returns paths in planner format: each generated path excludes [0, 0, 0].
    """
    base_waypoints = normalize_waypoints(waypoints)
    if len(base_waypoints) < 2 or num_paths <= 0:
        return []

    base_path = [[0.0, 0.0, 0.0]] + base_waypoints
    s = normalized_arclength(base_path)
    free_indices = list(range(1, len(base_path) - 1))
    s_free = np.array([s[i] for i in free_indices], dtype=float)
    covariance = smooth_covariance(s_free, max(smooth_length_scale, 1e-3))
    normals = path_normals(base_path)
    base_length = path_length(base_path)
    rng = np.random.default_rng(seed)

    generated: List[List[List[float]]] = []
    seen = set()
    attempts = max(max_attempts, num_paths * 8)

    for _ in range(attempts):
        lateral_noise = sample_bridge_noise(rng, covariance, lateral_sigma_m, s_free)
        vertical_noise = sample_bridge_noise(rng, covariance, vertical_sigma_m, s_free)

        new_path = [list(p) for p in base_path]
        for noise_idx, path_idx in enumerate(free_indices):
            normal = normals[path_idx]
            new_path[path_idx] = [
                round(base_path[path_idx][0] + lateral_noise[noise_idx] * normal[0], 3),
                round(base_path[path_idx][1] + lateral_noise[noise_idx] * normal[1], 3),
                round(base_path[path_idx][2] + vertical_noise[noise_idx], 3),
            ]

        new_path[0] = list(base_path[0])
        new_path[-1] = list(base_path[-1])

        if not is_feasible_bridge(
            new_path,
            base_path=base_path,
            base_length=base_length,
            max_turn_deg=max_turn_deg,
            max_path_length_ratio=max_path_length_ratio,
            progress_tolerance_m=progress_tolerance_m,
        ):
            continue

        planner_waypoints = normalize_waypoints(new_path[1:])
        key = tuple(tuple(round(v, 2) for v in p) for p in planner_waypoints)
        if key in seen:
            continue
        seen.add(key)
        generated.append(planner_waypoints)
        if len(generated) >= num_paths:
            break

    return generated


def sample_bridge_noise(
    rng: np.random.Generator,
    covariance: np.ndarray,
    sigma: float,
    s_free: np.ndarray,
) -> np.ndarray:
    if sigma <= 0.0 or len(s_free) == 0:
        return np.zeros(len(s_free), dtype=float)
    noise = rng.multivariate_normal(
        mean=np.zeros(len(s_free), dtype=float),
        cov=(sigma**2) * covariance,
    )
    return noise * np.sin(np.pi * s_free)


def smooth_covariance(s_free: np.ndarray, smooth_length_scale: float) -> np.ndarray:
    if len(s_free) == 0:
        return np.zeros((0, 0), dtype=float)
    d = s_free[:, None] - s_free[None, :]
    cov = np.exp(-0.5 * (d / smooth_length_scale) ** 2)
    cov += np.eye(len(s_free)) * 1e-6
    return cov


def normalize_waypoints(waypoints) -> List[List[float]]:
    out = []
    for p in waypoints or []:
        if not isinstance(p, (list, tuple)) or len(p) < 3:
            continue
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        if abs(x) < 1e-9 and abs(y) < 1e-9 and abs(z) < 1e-9:
            continue
        out.append([round(x, 3), round(y, 3), round(z, 3)])
    return out


def normalized_arclength(path: List[List[float]]) -> List[float]:
    total = 0.0
    cumulative = [0.0]
    for prev, cur in zip(path, path[1:]):
        total += distance(prev, cur)
        cumulative.append(total)
    if total <= 1e-9:
        denom = max(len(path) - 1, 1)
        return [i / denom for i in range(len(path))]
    return [v / total for v in cumulative]


def path_normals(path: List[List[float]]) -> List[List[float]]:
    fallback = xy_unit([path[-1][0] - path[0][0], path[-1][1] - path[0][1]])
    if fallback is None:
        fallback = [1.0, 0.0]

    normals = []
    for i, _ in enumerate(path):
        prev = path[max(0, i - 1)]
        nxt = path[min(len(path) - 1, i + 1)]
        tangent = xy_unit([nxt[0] - prev[0], nxt[1] - prev[1]])
        if tangent is None:
            tangent = fallback
        normals.append([-tangent[1], tangent[0], 0.0])
    return normals


def is_feasible_bridge(
    path: List[List[float]],
    base_path: List[List[float]],
    base_length: float,
    max_turn_deg: float,
    max_path_length_ratio: float,
    progress_tolerance_m: float,
) -> bool:
    if path[0] != base_path[0] or path[-1] != base_path[-1]:
        return False
    if base_length > 1e-9 and path_length(path) > base_length * max_path_length_ratio:
        return False
    if max_turn_deg > 0.0 and max_turn_angle_deg(path) > max_turn_deg:
        return False
    return has_monotonic_goal_progress(path, progress_tolerance_m)


def max_turn_angle_deg(path: List[List[float]]) -> float:
    segments = []
    for prev, cur in zip(path, path[1:]):
        v = [cur[0] - prev[0], cur[1] - prev[1], cur[2] - prev[2]]
        if norm(v) > 1e-9:
            segments.append(v)

    max_angle = 0.0
    for a, b in zip(segments, segments[1:]):
        denom = norm(a) * norm(b)
        if denom <= 1e-9:
            continue
        cos_v = max(-1.0, min(1.0, dot(a, b) / denom))
        max_angle = max(max_angle, math.degrees(math.acos(cos_v)))
    return max_angle


def has_monotonic_goal_progress(path: List[List[float]], tolerance: float) -> bool:
    goal = [path[-1][0] - path[0][0], path[-1][1] - path[0][1], path[-1][2] - path[0][2]]
    goal_norm = norm(goal)
    if goal_norm <= 1e-9:
        return True
    axis = [v / goal_norm for v in goal]
    prev_progress = -float("inf")
    for point in path:
        rel = [point[0] - path[0][0], point[1] - path[0][1], point[2] - path[0][2]]
        progress = dot(rel, axis)
        if progress + tolerance < prev_progress:
            return False
        prev_progress = max(prev_progress, progress)
    return True


def with_start(path_without_start: List[List[float]]) -> List[List[float]]:
    return [[0.0, 0.0, 0.0]] + normalize_waypoints(path_without_start)


def path_length(path: List[List[float]]) -> float:
    return sum(distance(a, b) for a, b in zip(path, path[1:]))


def distance(a: List[float], b: List[float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def xy_unit(v: List[float]) -> Optional[List[float]]:
    n = math.hypot(float(v[0]), float(v[1]))
    if n <= 1e-9:
        return None
    return [float(v[0]) / n, float(v[1]) / n]


def dot(a: List[float], b: List[float]) -> float:
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1]) + float(a[2]) * float(b[2])


def norm(v: List[float]) -> float:
    return math.sqrt(dot(v, v))


def save_json(
    output_path: Path,
    base_waypoints: List[List[float]],
    generated: List[List[List[float]]],
    args: argparse.Namespace,
) -> None:
    config = {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}
    payload = {
        "note": "Planner-style cumulative body-frame waypoints. Visualization adds implicit start [0,0,0].",
        "config": config,
        "base_waypoints": base_waypoints,
        "paths": generated,
    }
    output_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def save_matplotlib_png(output_path: Path, base_waypoints, generated) -> bool:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"[WARN] matplotlib unavailable, skip PNG: {exc}")
        return False

    fig = plt.figure(figsize=(11, 7))
    ax = fig.add_subplot(111, projection="3d")

    base = np.array(with_start(base_waypoints), dtype=float)
    ax.plot(base[:, 0], base[:, 1], base[:, 2], color="black", linewidth=3.0, label="base")
    ax.scatter(base[:, 0], base[:, 1], base[:, 2], color="black", s=22)

    colors = plt.cm.viridis(np.linspace(0.08, 0.92, max(len(generated), 1)))
    for idx, path in enumerate(generated):
        arr = np.array(with_start(path), dtype=float)
        ax.plot(arr[:, 0], arr[:, 1], arr[:, 2], color=colors[idx], alpha=0.8, linewidth=1.5)
        ax.scatter(arr[-1, 0], arr[-1, 1], arr[-1, 2], color=colors[idx], s=16)

    ax.set_title("Smooth-Bridge Perturbed Planner Waypoints")
    ax.set_xlabel("x forward (m)")
    ax.set_ylabel("y right (m)")
    ax.set_zlabel("z (m)")
    ax.legend(loc="upper left")
    ax.view_init(elev=24, azim=-62)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)
    return True


def save_html_viewer(output_path: Path, base_waypoints, generated) -> None:
    paths = [
        {"name": "base", "color": "#111111", "points": with_start(base_waypoints)},
    ]
    palette = [
        "#2563eb",
        "#dc2626",
        "#16a34a",
        "#9333ea",
        "#ea580c",
        "#0891b2",
        "#be123c",
        "#4f46e5",
        "#65a30d",
        "#c026d3",
    ]
    for i, path in enumerate(generated):
        paths.append(
            {
                "name": f"smooth_bridge_{i + 1}",
                "color": palette[i % len(palette)],
                "points": with_start(path),
            }
        )

    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Smooth Bridge Trajectory Viewer</title>
  <style>
    body {{ margin: 0; font-family: Arial, sans-serif; background: #f7f7f5; color: #161616; }}
    header {{ padding: 14px 18px; border-bottom: 1px solid #ddd; background: white; }}
    canvas {{ display: block; width: 100vw; height: calc(100vh - 68px); cursor: grab; }}
    .hint {{ font-size: 13px; color: #555; margin-top: 4px; }}
  </style>
</head>
<body>
  <header>
    <strong>Smooth Bridge Trajectory Viewer</strong>
    <div class="hint">Drag to rotate, wheel to zoom. Coordinates are planner-style body-frame offsets; [0,0,0] is the implicit UAV start.</div>
  </header>
  <canvas id="view"></canvas>
  <script>
const paths = {json.dumps(paths)};
const canvas = document.getElementById("view");
const ctx = canvas.getContext("2d");
let rotZ = -0.85;
let rotX = 0.52;
let zoom = 24;
let dragging = false;
let lastX = 0;
let lastY = 0;

function resize() {{
  canvas.width = Math.floor(canvas.clientWidth * devicePixelRatio);
  canvas.height = Math.floor(canvas.clientHeight * devicePixelRatio);
  draw();
}}

function rotate(p) {{
  let [x, y, z] = p;
  let cz = Math.cos(rotZ), sz = Math.sin(rotZ);
  let x1 = x * cz - y * sz;
  let y1 = x * sz + y * cz;
  let cx = Math.cos(rotX), sx = Math.sin(rotX);
  let y2 = y1 * cx - z * sx;
  let z2 = y1 * sx + z * cx;
  return [x1, y2, z2];
}}

function project(p) {{
  const [x, y, z] = rotate(p);
  const scale = zoom * devicePixelRatio;
  return [
    canvas.width * 0.5 + x * scale,
    canvas.height * 0.62 - y * scale,
    z
  ];
}}

function drawLine(points, color, width) {{
  ctx.beginPath();
  points.forEach((p, i) => {{
    const [x, y] = project(p);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }});
  ctx.strokeStyle = color;
  ctx.lineWidth = width * devicePixelRatio;
  ctx.stroke();
}}

function drawPoint(p, color, radius) {{
  const [x, y] = project(p);
  ctx.beginPath();
  ctx.arc(x, y, radius * devicePixelRatio, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
}}

function drawAxes() {{
  const origin = [0, 0, 0];
  const axes = [
    {{end: [8, 0, 0], color: "#444", label: "x forward"}},
    {{end: [0, 5, 0], color: "#777", label: "y right"}},
    {{end: [0, 0, 3], color: "#999", label: "z"}},
  ];
  axes.forEach(a => {{
    drawLine([origin, a.end], a.color, 1);
    const [x, y] = project(a.end);
    ctx.fillStyle = a.color;
    ctx.font = `${{12 * devicePixelRatio}}px Arial`;
    ctx.fillText(a.label, x + 6 * devicePixelRatio, y);
  }});
}}

function drawLegend() {{
  let x = 18 * devicePixelRatio;
  let y = 22 * devicePixelRatio;
  ctx.font = `${{12 * devicePixelRatio}}px Arial`;
  paths.forEach((path, i) => {{
    ctx.fillStyle = path.color;
    ctx.fillRect(x, y + i * 18 * devicePixelRatio, 18 * devicePixelRatio, 3 * devicePixelRatio);
    ctx.fillStyle = "#222";
    ctx.fillText(path.name, x + 26 * devicePixelRatio, y + 5 * devicePixelRatio + i * 18 * devicePixelRatio);
  }});
}}

function draw() {{
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#f7f7f5";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  drawAxes();
  paths.slice(1).forEach(path => drawLine(path.points, path.color, 1.5));
  drawLine(paths[0].points, paths[0].color, 3.0);
  paths.forEach(path => {{
    path.points.forEach((p, i) => drawPoint(p, i === path.points.length - 1 ? path.color : "#ffffff", i === 0 ? 4 : 3));
  }});
  drawLegend();
}}

canvas.addEventListener("mousedown", e => {{
  dragging = true;
  lastX = e.clientX;
  lastY = e.clientY;
}});
window.addEventListener("mouseup", () => dragging = false);
window.addEventListener("mousemove", e => {{
  if (!dragging) return;
  rotZ += (e.clientX - lastX) * 0.008;
  rotX += (e.clientY - lastY) * 0.008;
  rotX = Math.max(-1.35, Math.min(1.35, rotX));
  lastX = e.clientX;
  lastY = e.clientY;
  draw();
}});
canvas.addEventListener("wheel", e => {{
  e.preventDefault();
  zoom *= e.deltaY > 0 ? 0.9 : 1.1;
  zoom = Math.max(8, Math.min(90, zoom));
  draw();
}}, {{passive: false}});
window.addEventListener("resize", resize);
resize();
  </script>
</body>
</html>
"""
    output_path.write_text(html, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Simulate smooth-bridge trajectory perturbations.")
    parser.add_argument("--num-paths", type=int, default=8, help="Number of perturbed paths to generate.")
    parser.add_argument("--seed", type=int, default=7, help="Random seed.")
    parser.add_argument("--lateral-sigma", type=float, default=1.0, help="Mid-path lateral noise sigma in meters.")
    parser.add_argument("--vertical-sigma", type=float, default=0.25, help="Mid-path vertical noise sigma in meters.")
    parser.add_argument("--length-scale", type=float, default=0.35, help="Smoothness scale on normalized path length.")
    parser.add_argument("--max-attempts", type=int, default=128, help="Maximum sampling attempts.")
    parser.add_argument("--max-turn-deg", type=float, default=120.0, help="Reject paths sharper than this angle.")
    parser.add_argument("--max-path-length-ratio", type=float, default=1.5, help="Reject paths longer than base * ratio.")
    parser.add_argument("--progress-tolerance", type=float, default=1.0, help="Allowed backward progress in meters.")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "outputs")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    generated = generate_smooth_bridge_paths(
        BASE_WAYPOINTS,
        num_paths=args.num_paths,
        lateral_sigma_m=args.lateral_sigma,
        vertical_sigma_m=args.vertical_sigma,
        smooth_length_scale=args.length_scale,
        seed=args.seed,
        max_attempts=args.max_attempts,
        max_turn_deg=args.max_turn_deg,
        max_path_length_ratio=args.max_path_length_ratio,
        progress_tolerance_m=args.progress_tolerance,
    )

    json_path = args.output_dir / "smooth_bridge_paths.json"
    html_path = args.output_dir / "smooth_bridge_paths.html"
    png_path = args.output_dir / "smooth_bridge_paths.png"

    save_json(json_path, BASE_WAYPOINTS, generated, args)
    save_html_viewer(html_path, BASE_WAYPOINTS, generated)
    png_ok = save_matplotlib_png(png_path, BASE_WAYPOINTS, generated)

    print(f"Generated {len(generated)} perturbed paths from {len(BASE_WAYPOINTS)} planner waypoints.")
    print(f"JSON: {json_path}")
    print(f"HTML viewer: {html_path}")
    if png_ok:
        print(f"PNG: {png_path}")


if __name__ == "__main__":
    main()
