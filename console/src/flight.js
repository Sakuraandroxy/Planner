"use strict";

// 执行侧的显示：剩余航点列表、3D 轨迹视图的喂数据、画面通道（xpra 与 MJPEG）与机位。

function wrapYawDelta(deg) {
  const value = Number(deg);
  if (!Number.isFinite(value)) return 0;
  return ((((value + 180) % 360) + 360) % 360) - 180;
}

function relativeIncrement(previous, point) {
  const worldDx = Number(point.x) - Number(previous.x);
  const worldDy = Number(point.y) - Number(previous.y);
  const worldDz = Number(point.z) - Number(previous.z);
  const angle = (Number(previous.yaw_deg) * Math.PI) / 180;
  return {
    dx: Math.cos(angle) * worldDx + Math.sin(angle) * worldDy,
    dy: -Math.sin(angle) * worldDx + Math.cos(angle) * worldDy,
    dz: worldDz,
    dyaw: wrapYawDelta(Number(point.yaw_deg) - Number(previous.yaw_deg)),
  };
}

const INCREMENTS = [
  // 每一分量一对箭头：正方向在前，负方向在后。
  { key: "dx", arrows: ["▲", "▼"], digits: 2, suffix: "" },
  { key: "dy", arrows: ["▶", "◀"], digits: 2, suffix: "" },
  { key: "dz", arrows: ["↑", "↓"], digits: 2, suffix: "" },
  { key: "dyaw", arrows: ["↻", "↺"], digits: 2, suffix: "°" },
];

function formatIncrement(value, arrows, digits, suffix) {
  const number = Number(value);
  if (!Number.isFinite(number)) return "—";
  if (Math.abs(number) < 0.005) return `· 0.00${suffix}`;
  return `${number > 0 ? arrows[0] : arrows[1]} ${Math.abs(number).toFixed(digits)}${suffix}`;
}

function incrementParts(delta, showDz) {
  if (!delta) return ["—"];
  // dz 按引擎 NED 向下为正，这里取反成人的直觉：向上为正。
  const values = { ...delta, dz: -delta.dz };
  return INCREMENTS.filter((axis) => showDz || axis.key !== "dz").map((axis) =>
    formatIncrement(values[axis.key], axis.arrows, axis.digits, axis.suffix),
  );
}

function renderTrajectory(snapshot) {
  lastTrajectory = snapshot;
  const active = Boolean(snapshot && snapshot.active);
  const points =
    snapshot && Array.isArray(snapshot.points) ? snapshot.points : [];
  const queue = (snapshot && snapshot.queue) || { count: 0, capacity: 0 };
  const expanded = trajectoryExpanded && points.length > 1;
  const visible = expanded ? points : points.slice(0, 1);
  let previous =
    snapshot && snapshot.vehicle && snapshot.vehicle.yaw_deg !== undefined
      ? snapshot.vehicle
      : null;

  $("trajectory-status").textContent = active
    ? `队列 ${queue.count || 0} / ${queue.capacity || 0}`
    : "已同步";

  $("trajectory-card").classList.toggle("expanded", expanded);
  const toggle = $("trajectory-toggle");
  toggle.hidden = points.length <= 1;
  toggle.textContent = expanded ? "收起 ▴" : "展开 ▾";
  toggle.setAttribute("aria-expanded", String(expanded));

  const list = $("trajectory-list");
  const scrollTop = list.scrollTop;
  list.hidden = visible.length === 0;
  list.replaceChildren();
  const showDz = !snapshot || snapshot.vertical_motion !== false;
  for (const point of visible) {
    const item = document.createElement("li");
    item.className = "trajectory-point";

    const marker = document.createElement("span");
    marker.className = "point-marker";
    marker.textContent = String(point.index ?? "");

    const main = document.createElement("div");
    main.className = "point-main";
    const delta = previous ? relativeIncrement(previous, point) : null;
    const coords = document.createElement("span");
    coords.className = "point-coords";
    const parts = incrementParts(delta, showDz);
    coords.textContent = parts.join(" · ");
    coords.title = showDz
      ? "▲前进 ▼后退 · ▶右移 ◀左移 · ↑上升 ↓下降 · ↻右转 ↺左转"
      : "▲前进 ▼后退 · ▶右移 ◀左移 · ↻右转 ↺左转";
    const meta = document.createElement("span");
    meta.className = "point-meta";
    meta.textContent = `#${point.waypoint_id}`;
    main.append(coords, meta);

    item.append(marker, main);
    list.appendChild(item);
    previous = point;
  }
  list.scrollTop = scrollTop;

  const empty = $("trajectory-empty");
  empty.hidden = points.length > 0;
  if (points.length === 0) {
    let title = "等待任务";
    let hint = "提交任务后这里会显示剩余航点";
    if (active) {
      title = "轨迹队列为空";
      hint = "规划请求返回后，排队的航点会显示在这里";
    }
    $("trajectory-empty-title").textContent = title;
    $("trajectory-empty-hint").textContent = hint;
  }

  // The 3D panel is a separate module and may never have loaded: no WebGL, or the
  // library did not arrive. The list above is the source of truth either way.
  if (window.Trajectory3D) window.Trajectory3D.update(snapshot);
}

async function pollTrajectory() {
  if (trajectoryPending) {
    setTimeout(pollTrajectory, POLL_MS);
    return;
  }
  trajectoryPending = true;
  try {
    const body = await request("/api/trajectory");
    if (!body || typeof body !== "object") {
      throw new Error("轨迹响应格式无效");
    }
    renderTrajectory(body);
  } catch (_) {
    $("trajectory-status").textContent = "轨迹连接异常";
  } finally {
    trajectoryPending = false;
    setTimeout(pollTrajectory, POLL_MS);
  }
}

// 三维模块就绪后调这里，用本文件那份快照补画一次。
function replayTrajectory() {
  if (lastTrajectory) renderTrajectory(lastTrajectory);
}

// ---- picture channels --------------------------------------------------

function markVideoLoaded() {
  clearInterval(videoWatch);
  videoWatch = null;
  videoLoaded = true;
  $("video-overlay").hidden = true;
  updateStreamStatus();
}

function stopVideo() {
  clearTimeout(videoRetry);
  clearInterval(videoWatch);
  videoRetry = null;
  videoWatch = null;
  videoLoaded = false;
  const video = $("video");
  video.removeAttribute("src");
  $("video-overlay").hidden = true;
}

function startVideo() {
  if (viewMode !== "camera") return;
  clearTimeout(videoRetry);
  clearInterval(videoWatch);
  videoLoaded = false;
  $("video-overlay").hidden = false;
  $("stream-status").textContent = "连接画面…";
  const video = $("video");
  video.src = `/api/video?t=${Date.now()}`;
  // An endless MJPEG stream never fires `load`, so watch the decoded size.
  videoWatch = setInterval(() => {
    if (video.naturalWidth > 0) markVideoLoaded();
  }, 300);
}

function scheduleVideoRetry() {
  clearTimeout(videoRetry);
  videoRetry = setTimeout(startVideo, 3000);
}

function xpraUrl() {
  const override = new URLSearchParams(location.search).get("xpra");
  let url;
  if (override) {
    url = new URL(override.includes("://") ? override : `http://${override}`);
  } else {
    url = new URL(`http://${location.hostname}:${XPRA_PORT}/`);
  }
  // Mirror the reference console: grab keyboard focus on connect, and ask the
  // server for a shared session (xpra rejects steal=false without sharing).
  const params = {
    floating_menu: "false",
    border: "no",
    clipboard: "false",
    printing: "false",
    file_transfer: "false",
    sound: "false",
    steal: "true",
    sharing: "true",
  };
  for (const [key, value] of Object.entries(params)) {
    url.searchParams.set(key, value);
  }
  return url.href;
}

function focusXpra() {
  if (viewMode !== "xpra") return;
  const frame = $("xpra-frame");
  frame.focus({ preventScroll: true });
  try {
    frame.contentWindow.focus();
  } catch (_) {
    // Cross-origin access to contentWindow.focus is allowed; ignore refusals.
  }
}

async function probeXpra(url) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), 4000);
  try {
    // xpra answers cross-origin requests opaquely; resolving is enough here.
    await fetch(url, {
      mode: "no-cors",
      cache: "no-store",
      signal: controller.signal,
    });
    return true;
  } catch (_) {
    return false;
  } finally {
    clearTimeout(timer);
  }
}

async function startXpra() {
  const url = xpraUrl();
  const frame = $("xpra-frame");
  xpraState = "connecting";
  frame.hidden = false;
  $("xpra-overlay").hidden = true;
  $("open-xpra").href = url;
  frame.src = url;
  updateStreamStatus();
  const reachable = await probeXpra(url);
  if (viewMode !== "xpra") return;
  xpraState = reachable ? "ready" : "down";
  $("xpra-overlay").hidden = reachable;
  updateStreamStatus();
}

function stopXpra() {
  const frame = $("xpra-frame");
  frame.hidden = true;
  frame.src = "about:blank";
  $("xpra-overlay").hidden = true;
  xpraState = "connecting";
}

function setViewMode(mode) {
  if (mode === viewMode) return;
  viewMode = mode;
  const xpra = mode === "xpra";
  $("video").hidden = xpra;
  $("focus-view").hidden = !xpra;
  $("open-xpra").hidden = !xpra;
  $("view-toggle").textContent = xpra ? "相机画面" : "交互画面";
  if (xpra) {
    stopVideo();
    startXpra();
  } else {
    stopXpra();
    startVideo();
  }
  renderViewButtons();
  updateStreamStatus();
}

function renderViewButtons() {
  const selectable =
    currentState === "idle" ||
    currentState === "running" ||
    currentState === "stopping";
  const shown = perspectiveOf(taskView);
  document.querySelectorAll("#view-modes button").forEach((button) => {
    button.classList.toggle("active", button.dataset.perspective === shown);
    button.disabled = !selectable;
  });
  if (viewMode === "xpra") {
    $("view-hint").textContent = shown
      ? `当前视角：${PERSPECTIVE_LABELS[shown]}`
      : "当前视角未确认";
  } else {
    $("view-hint").textContent = "相机最新帧 · 空闲时停留最后一帧";
  }
}

async function requestView(perspective) {
  const label = PERSPECTIVE_LABELS[perspective];
  if (!label) return;
  const vehicle = runningVehicle || selectedVehicle;
  const mode = (VIEW_BY_VEHICLE[vehicle] || {})[perspective];
  if (!mode || mode === taskView) return;
  if (caps && Array.isArray(caps.view_modes) && !caps.view_modes.includes(mode)) {
    return;
  }
  const previous = taskView;
  taskView = mode;
  viewPending = true;
  renderViewButtons();
  renderViewShield();
  try {
    const body = await request("/api/view", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mode }),
    });
    if (body && body.view_mode) taskView = body.view_mode;
    if (body && body.deferred) {
      viewPending = false;
      setFeedback(`视角已记录为${label}，任务结束后生效。`, "ok");
    } else {
      setFeedback(`正在切换到${label}，完成后自动显示画面。`, "ok");
    }
  } catch (error) {
    taskView = previous;
    viewPending = false;
    setFeedback("视角切换失败：" + error.message, "error");
  }
  renderViewButtons();
  renderViewShield();
}

function resetXpra() {
  const frame = $("xpra-frame");
  frame.src = "about:blank";
  startXpra();
}

function retryView() {
  if (viewMode === "xpra") startXpra();
  else startVideo();
}
