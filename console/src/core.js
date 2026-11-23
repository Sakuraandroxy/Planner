"use strict";

// 骨架层：常量、共享状态、DOM 工具、渲染入口、能力门控、tab、HTTP 请求。
// 面板实现拆在 world.js / settings.js / flight.js，轮询与启动绑定在 app.js。
// 这些文件都是 classic script，顶层 let 是同一 realm 的全局词法绑定，跨文件可读写。

const $ = (id) => document.getElementById(id);

const STATE_LABELS = {
  disconnected: "未连接场景",
  connecting: "连接仿真器中",
  idle: "可接受新任务",
  running: "任务执行中",
  stopping: "正在停止",
  unknown: "状态未确认",
};
const POLL_MS = 1000;
const XPRA_PORT = 10010;
const WORLD_MODEL_PORT = 8765;
const VEHICLE_LABELS = {
  rover: "无人车",
  drone: "无人机",
};
const DEFAULT_VEHICLES = ["rover", "drone"];
// 界面只区分两种视角：第一人称用当前载具的前视相机，第三人称用它的追尾相机。
const VIEW_BY_VEHICLE = {
  rover: { first: "rover_front", third: "rover_chase" },
  drone: { first: "drone_front", third: "drone_chase" },
};
const PERSPECTIVE_LABELS = { first: "第一人称", third: "第三人称" };

function perspectiveOf(mode) {
  for (const views of Object.values(VIEW_BY_VEHICLE)) {
    if (mode === views.first) return "first";
    if (mode === views.third) return "third";
  }
  return "";
}

let currentState = "unknown";
let statusPending = false;
let trajectoryPending = false;
let submitPending = false;
let viewMode = "xpra";
let viewPending = false;
let videoRetry = null;
let videoWatch = null;
let videoLoaded = false;
let xpraState = "connecting";
let vehicles = DEFAULT_VEHICLES.slice();
let selectedVehicle = "drone";
let runningVehicle = null;
let vehicleTouched = false;
let trajectoryExpanded = false;
let lastTrajectory = null;
let taskView = "drone_chase";
let worldCatalog = null;
// Capability flags for the simulator driving the active map. The console renders
// optional controls from this and never tests a family name, so a new
// simulator needs no change here. Null until the backend reports it.
let caps = null;
let currentWorld = { map: "" };
let mapSwitching = false;
let worldPending = false;
let worldRebuilding = false;
// Set while /api/world/stop is in flight, so the status poll can tell a clean
// stop apart from a failed launch.
let worldStopping = false;
let worldOptionsPending = false;
let weatherCatalog = null;
let weatherStatus = null;
let weatherFieldSpecs = [];
let weatherValues = {};
let weatherTouched = false;
let weatherPending = false;
let weatherOptionsPending = false;
let modelsCatalog = null;
let modelSelection = {};
let modelsTouched = false;
let modelsPending = false;
let modelsRebuilding = false;
let modelsSwitching = false;
let modelsOptionsPending = false;
let profileCatalog = null;
let profileCurrent = "";
let profileSelection = "";
let profilePending = false;
let profileRebuilding = false;
let profileSwitching = false;
let profileOptionsPending = false;

function setModelFeedback(text, kind = "") {
  const node = $("models-feedback");
  node.textContent = text;
  if (kind) node.dataset.kind = kind;
  else delete node.dataset.kind;
}

function setWorldFeedback(text, kind = "") {
  const node = $("world-feedback");
  node.textContent = text;
  if (kind) node.dataset.kind = kind;
  else delete node.dataset.kind;
}

function setWeatherFeedback(text, kind = "") {
  const node = $("weather-feedback");
  node.textContent = text;
  if (kind) node.dataset.kind = kind;
  else delete node.dataset.kind;
}

function setFeedback(text, kind = "") {
  const node = $("feedback");
  node.textContent = text;
  if (kind) node.dataset.kind = kind;
  else delete node.dataset.kind;
}

function setState(next) {
  currentState = next;
  const chip = $("state-chip");
  chip.dataset.state = next;
  $("state-text").textContent = STATE_LABELS[next] || STATE_LABELS.unknown;
  updateStreamStatus();
  renderViewShield();
  renderControls();
}

function renderViewShield() {
  const shield = $("view-shield");
  if (!shield) return;
  // The simulator cycles through every streaming camera while a view is being
  // applied, and calibration does the same before the worker is ready. Cover
  // the picture with an opaque black shield so those intermediate frames are
  // never shown; the cover carries one line saying what is happening.
  const covered =
    viewPending ||
    currentState === "connecting" ||
    currentState === "unknown";
  shield.hidden = !covered;
  if (covered) {
    $("shield-hint").textContent = viewPending
      ? "正在切换机位…"
      : currentState === "connecting"
      ? "场景加载中…"
      : "正在确认服务状态…";
  }
}

function updateStreamStatus() {
  if (currentState === "disconnected") {
    $("stream-status").textContent = "未连接场景";
    return;
  }
  if (viewMode === "xpra") {
    $("stream-status").textContent =
      xpraState === "ready"
        ? "交互画面"
        : xpraState === "down"
        ? "交互画面连不上"
        : "连接交互画面…";
    return;
  }
  if (videoLoaded) {
    $("stream-status").textContent =
      currentState === "running" ? "实时画面" : "最后一帧（待机）";
  }
}

function renderControls() {
  const instruction = $("instruction").value.trim();
  const idle = currentState === "idle";
  $("start").disabled =
    submitPending || !idle || instruction.length === 0;
  $("stop").disabled = submitPending || currentState !== "running";
  $("home").disabled = submitPending || !idle;
  applyCapabilities();
  renderVehicles();
  renderWorld();
  renderModels();
  renderProfiles();
  renderWeather();
  renderViewButtons();
}

// ---- capability gating -------------------------------------------------
// ``caps`` 为 null 表示服务端没有能力报告：`/api/status` 与 `/api/world/options`
// 在这种情况下显式给 `"capabilities": null`，所有可选控件按不可用处理。
function hasAction(action) {
  return Boolean(caps && Array.isArray(caps.actions) && caps.actions.includes(action));
}

function allowsVehicle(name) {
  return Boolean(
    caps && Array.isArray(caps.vehicles) && caps.vehicles.includes(name)
  );
}

function show(node, visible) {
  if (node) node.hidden = !visible;
}

function applyCapabilities() {
  show($("home"), hasAction("home"));
  // Gate the tab, never the card. Hiding the models block used to hide the whole
  // world card with it, because the two shared markup: a second after load the
  // console had no map selector left. Each panel now owns its controls, so only the
  // tab that leads to a missing capability goes away.
  show($("tab-models"), hasAction("models"));
  selectSettingsTab(settingsTab);
  // The remaining-trajectory card is not gated. It shows what the shared
  // planning layer recorded for this service, so every simulator has it; a
  // numeric basic-motion task simply reports no pending waypoints, which is
  // true on both families and not a missing feature.
  const views =
    caps && Array.isArray(caps.view_modes) ? caps.view_modes.length > 0 : false;
  show($("view-modes"), views);
  show($("view-toggle"), views);
  const vehicleGroup = $("vehicle-modes");
  show(vehicleGroup, vehicles.length > 1);
  if (vehicleGroup && vehicleGroup.previousElementSibling) {
    show(vehicleGroup.previousElementSibling, vehicles.length > 1);
  }
}

// ---- settings tabs -----------------------------------------------------
// 世界、天气、模型、规划器共用一张卡和一条 tab 栏。选中项记在这里，能力刷新后重新
// 应用，以免当前 tab 被隐藏时留下空白面板。
let settingsTab = "world";

function settingsTabs() {
  return Array.from(document.querySelectorAll("#settings-card .tab"));
}

function selectSettingsTab(name) {
  const tabs = settingsTabs();
  const target =
    tabs.find((tab) => tab.dataset.tab === name && !tab.hidden) ||
    tabs.find((tab) => !tab.hidden) ||
    tabs[0];
  for (const tab of tabs) {
    const on = tab === target;
    tab.setAttribute("aria-selected", String(on));
    tab.classList.toggle("active", on);
    const panel = $(`panel-${tab.dataset.tab}`);
    if (panel) panel.hidden = !on;
  }
  settingsTab = target.dataset.tab;
}

// ---- HTTP --------------------------------------------------------------
async function request(path, options = {}) {
  const response = await fetch(path, {
    ...options,
    credentials: "omit",
    cache: "no-store",
  });
  let body = null;
  try {
    body = await response.json();
  } catch (_) {
    body = null;
  }
  if (!response.ok) {
    const detail = body && body.detail;
    let message;
    if (Array.isArray(detail)) {
      message = detail
        .map((item) => `${(item.loc || []).join(".")}: ${item.msg}`)
        .join("; ");
    } else if (detail && typeof detail === "object") {
      message = detail.message || `HTTP ${response.status}`;
    } else {
      message = detail || (body && body.message) || `HTTP ${response.status}`;
    }
    const error = new Error(message);
    error.status = response.status;
    throw error;
  }
  return body;
}
