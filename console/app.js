"use strict";

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
const VEHICLE_LABELS = {
  rover: "无人车",
  drone: "无人机",
};
const DEFAULT_VEHICLES = ["rover", "drone"];
const VIEW_LABELS = {
  rover_front: "无人车前视",
  rover_chase: "无人车追尾",
  drone_front: "无人机前视",
  drone_chase: "无人机追尾",
};

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
let selectedVehicle = "rover";
let runningVehicle = null;
let vehicleTouched = false;
let trajectoryExpanded = false;
let lastTrajectory = null;
let taskView = "rover_chase";
let worldCatalog = null;
let currentWorld = { map: "" };
let mapSwitching = false;
let worldPending = false;
let worldRebuilding = false;
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
  // never shown; the active camera appears only once the view is confirmed.
  // The shield stays plain black -- no text or status is rendered on it.
  shield.hidden = !(
    viewPending ||
    currentState === "connecting" ||
    currentState === "unknown"
  );
}

function updateStreamStatus() {
  if (currentState === "disconnected") {
    $("stream-status").textContent = "未连接场景";
    return;
  }
  if (viewMode === "xpra") {
    $("stream-status").textContent =
      xpraState === "ready"
        ? "交互画面（xpra）"
        : xpraState === "down"
        ? "xpra 未连接"
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
  renderVehicles();
  renderWorld();
  renderModels();
  renderWeather();
  renderViewButtons();
}

function renderVehicles() {
  const idle = currentState === "idle" && !submitPending;
  document.querySelectorAll("#vehicle-modes button").forEach((button) => {
    const name = button.dataset.vehicle;
    button.classList.toggle("active", name === selectedVehicle);
    button.disabled = !idle;
  });
  const active =
    currentState === "running" || currentState === "stopping";
  const name = active ? runningVehicle || selectedVehicle : selectedVehicle;
  $("current-vehicle").textContent = VEHICLE_LABELS[name] || name || "—";
}

async function activateVehicle(name) {
  if (!vehicles.includes(name) || submitPending || currentState !== "idle") {
    return;
  }
  const previousVehicle = selectedVehicle;
  const previousView = taskView;
  const label = VEHICLE_LABELS[name] || name;
  selectedVehicle = name;
  vehicleTouched = true;
  taskView = `${name}_chase`;
  submitPending = true;
  viewPending = true;
  renderControls();
  renderViewShield();
  setFeedback(`正在切换到${label}：停放另一台并回到起点…`);
  try {
    const body = await request("/api/vehicle", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ vehicle: name }),
    });
    if (body && body.view_mode) taskView = body.view_mode;
    setFeedback(
      `${label}切换中：正在停放另一台、回到起点并切换视角，完成后自动显示画面。`,
      "ok"
    );
    pollStatus();
  } catch (error) {
    selectedVehicle = previousVehicle;
    taskView = previousView;
    viewPending = false;
    setFeedback("切换载具失败：" + error.message, "error");
  } finally {
    submitPending = false;
    renderControls();
    renderViewButtons();
    renderViewShield();
  }
}

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

async function pollStatus() {
  if (statusPending) {
    setTimeout(pollStatus, POLL_MS);
    return;
  }
  statusPending = true;
  try {
    const body = await request("/api/status");
    const state = [
      "disconnected",
      "connecting",
      "idle",
      "running",
      "stopping",
    ].includes(body && body.state)
      ? body.state
      : "unknown";
    // A worker restart (map switch or reconnect) always comes back on the
    // default vehicle, so re-sync the vehicle buttons with the server.
    const previousState = currentState;
    if (currentState === "connecting" && state === "idle") vehicleTouched = false;
    if (body && Array.isArray(body.vehicles) && body.vehicles.length) {
      vehicles = body.vehicles.filter(
        (name) => typeof name === "string" && name.length > 0
      );
      if (!vehicles.includes(selectedVehicle)) {
        selectedVehicle = vehicles[0];
      }
    }
    if (body && body.vehicle) {
      runningVehicle = body.vehicle;
      if (
        !vehicleTouched &&
        state === "idle" &&
        vehicles.includes(body.vehicle)
      ) {
        selectedVehicle = body.vehicle;
      }
    }
    viewPending = Boolean(body && body.view_pending);
    setState(state);
    currentWorld = { map: (body && body.map) || "" };
    mapSwitching = Boolean(body && body.map_switching);
    if (body && body.models) {
      modelSelection = { ...body.models };
      if (!modelsTouched) syncModelSelects();
    }
    modelsSwitching = Boolean(body && body.models_switching);
    if (body && body.view_mode) taskView = body.view_mode;
    if (body && body.weather) {
      weatherStatus = body.weather;
      if (!weatherTouched) {
        setWeatherFields(weatherStatus);
        $("weather-current").textContent = weatherSummary(weatherStatus);
      }
    }
    const active = state === "running" || state === "stopping";
    $("current-instruction").textContent = active
      ? body.instruction || "—"
      : "—";
    $("last-result").textContent = body.last_result || "—";
    if (state === "idle" && modelsRebuilding) {
      modelsRebuilding = false;
      modelsTouched = false;
      setModelFeedback("载具模型已切换，载具已回到出生点。", "ok");
      loadModelOptions();
      if (viewMode === "xpra") resetXpra();
      else {
        stopVideo();
        startVideo();
      }
    } else if (state === "idle" && worldRebuilding) {
      worldRebuilding = false;
      setWorldFeedback(`已切换到 ${currentWorld.map}。`, "ok");
      loadWorldOptions();
      loadWeatherOptions();
      if (viewMode === "xpra") resetXpra();
      else {
        stopVideo();
        startVideo();
      }
    } else if (
      state === "idle" &&
      (worldCatalog === null || weatherCatalog === null || modelsCatalog === null)
    ) {
      if (worldCatalog === null) loadWorldOptions();
      if (weatherCatalog === null) loadWeatherOptions();
      if (modelsCatalog === null) loadModelOptions();
    }
    if (body && body.sim_error && worldRebuilding) {
      setWorldFeedback("切换失败：" + body.sim_error, "error");
    }
    // The simulator may finish launching after this page loaded; retry the
    // embedded xpra view once the worker becomes ready.
    if (
      previousState === "connecting" &&
      state === "idle" &&
      viewMode === "xpra" &&
      xpraState === "down"
    ) {
      startXpra();
    }
    renderControls();
  } catch (error) {
    if (currentState !== "unknown") setState("unknown");
    if ($("feedback").dataset.kind !== "ok") {
      setFeedback("状态连接异常：" + error.message, "error");
    }
  } finally {
    statusPending = false;
    setTimeout(pollStatus, POLL_MS);
  }
}

async function submitTask(event) {
  event.preventDefault();
  const instruction = $("instruction").value.trim();
  if (!instruction || submitPending) return;
  const payload = { instruction, vehicle: selectedVehicle };

  submitPending = true;
  renderControls();
  setFeedback("正在提交…");
  try {
    const body = await request("/api/tasks", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    setFeedback(
      `已接受 ${body.task_id}（${
        VEHICLE_LABELS[body.vehicle] || body.vehicle || selectedVehicle
      }），等待状态更新。`,
      "ok"
    );
  } catch (error) {
    setFeedback("提交失败：" + error.message, "error");
  } finally {
    submitPending = false;
    renderControls();
  }
}

async function stopTask() {
  if (submitPending) return;
  submitPending = true;
  renderControls();
  try {
    const body = await request("/api/stop", { method: "POST" });
    setFeedback(
      body.state === "stopping"
        ? "停止请求已发送，等待当前航点结束。"
        : "当前没有运行中的任务。",
      body.state === "stopping" ? "ok" : ""
    );
  } catch (error) {
    setFeedback("停止失败：" + error.message, "error");
  } finally {
    submitPending = false;
    renderControls();
  }
}

async function returnHome() {
  if (submitPending || currentState !== "idle") return;
  const label = VEHICLE_LABELS[selectedVehicle] || selectedVehicle;
  submitPending = true;
  renderControls();
  setFeedback(`正在把${label}传回起点…`);
  try {
    const body = await request("/api/home", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ vehicle: selectedVehicle }),
    });
    const chosen = VEHICLE_LABELS[body.vehicle] || body.vehicle || label;
    setFeedback(`回位请求已发送，${chosen}已瞬时传送回出生点。`, "ok");
  } catch (error) {
    setFeedback("回位失败：" + error.message, "error");
  } finally {
    submitPending = false;
    renderControls();
  }
}

function fillSelect(node, values, selected, labels = null) {
  const chosen = selected || node.value;
  node.replaceChildren();
  for (const value of values || []) {
    const option = document.createElement("option");
    option.value = value;
    option.textContent = (labels && labels[value]) || value;
    node.appendChild(option);
  }
  if (
    chosen &&
    Array.from(node.options).some((option) => option.value === chosen)
  ) {
    node.value = chosen;
  }
}

function renderWorld() {
  const selectable =
    (currentState === "idle" || currentState === "disconnected") &&
    !worldPending &&
    !worldRebuilding &&
    !mapSwitching &&
    worldCatalog !== null;
  for (const id of ["world-map", "world-apply", "world-reload"]) {
    $(id).disabled = !selectable;
  }
  $("world-apply").textContent =
    currentState === "disconnected" ? "启动场景" : "应用世界设置";
  const selection = (worldCatalog && worldCatalog.selection) || currentWorld;
  const switching = worldRebuilding || mapSwitching;
  if (currentState === "disconnected" && !switching) {
    $("world-current").textContent =
      selection && selection.map
        ? `未启动 · ${selection.map}`
        : "请选择地图启动";
  } else {
    $("world-current").textContent =
      selection && selection.map
        ? `${selection.map}${switching ? " · 切换中…" : ""}`
        : "选项未加载";
  }
}

async function loadWorldOptions() {
  if (worldOptionsPending) return;
  worldOptionsPending = true;
  try {
    const body = await request("/api/world/options");
    worldCatalog = body;
    fillSelect(
      $("world-map"),
      body.maps,
      body.selection && body.selection.map,
      body.map_labels
    );
  } catch (_) {
    worldCatalog = null;
  } finally {
    worldOptionsPending = false;
    renderWorld();
  }
}

async function applyWorld() {
  const ready =
    currentState === "idle" || currentState === "disconnected";
  if (worldPending || !ready || !worldCatalog) return;
  const map = $("world-map").value;
  if (!map) return;
  const starting = currentState === "disconnected";
  worldPending = true;
  renderControls();
  setWorldFeedback(
    starting
      ? `正在启动 ${map}；后台会拉起整套仿真器…`
      : `正在切换到 ${map}；后台会重启整套仿真器…`
  );
  try {
    const body = await request("/api/world", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ map }),
    });
    if (body && body.state === "connecting") {
      worldRebuilding = true;
      setWorldFeedback(
        starting
          ? `正在启动 ${map}；等待仿真器就绪并连接…`
          : `正在切换到 ${map}；等待仿真器重启并重新连接…`
      );
    } else {
      setWorldFeedback("选择未变化。", "ok");
    }
  } catch (error) {
    setWorldFeedback(
      (starting ? "启动失败：" : "切换失败：") + error.message,
      "error"
    );
  } finally {
    worldPending = false;
    renderControls();
  }
}

function modelsSummary() {
  if (!modelsCatalog) return "选项未加载";
  const parts = [];
  const options = modelsCatalog.options || {};
  for (const vehicle of Object.keys(options)) {
    const id = modelSelection[vehicle];
    const option = (options[vehicle] || []).find((item) => item.id === id);
    if (option) {
      parts.push(`${VEHICLE_LABELS[vehicle] || vehicle}：${option.label}`);
    }
  }
  if (!parts.length) return "—";
  const switching = modelsRebuilding || modelsSwitching;
  return parts.join(" · ") + (switching ? " · 切换中…" : "");
}

function renderModels() {
  const enabled =
    currentState === "idle" &&
    !modelsPending &&
    !modelsRebuilding &&
    !modelsSwitching &&
    !worldRebuilding &&
    !mapSwitching &&
    modelsCatalog !== null;
  for (const id of ["model-rover", "model-drone", "models-apply"]) {
    const node = $(id);
    if (node) node.disabled = !enabled;
  }
  $("models-current").textContent = modelsSummary();
}

function syncModelSelects() {
  const options = (modelsCatalog && modelsCatalog.options) || {};
  for (const vehicle of Object.keys(options)) {
    const select = $(`model-${vehicle}`);
    if (!select) continue;
    const wanted = modelSelection[vehicle];
    if (
      wanted &&
      Array.from(select.options).some((option) => option.value === wanted)
    ) {
      select.value = wanted;
    }
  }
}

function fillModelSelect(id, vehicle) {
  const select = $(id);
  if (!select) return;
  const options =
    (modelsCatalog && modelsCatalog.options && modelsCatalog.options[vehicle]) ||
    [];
  select.replaceChildren();
  for (const option of options) {
    const node = document.createElement("option");
    node.value = option.id;
    node.textContent = option.label || option.id;
    select.appendChild(node);
  }
  if (options.length) {
    const wanted = modelSelection[vehicle];
    select.value =
      wanted && options.some((option) => option.id === wanted)
        ? wanted
        : options[0].id;
  }
}

async function loadModelOptions() {
  if (modelsOptionsPending) return;
  modelsOptionsPending = true;
  try {
    const body = await request("/api/models");
    modelsCatalog = body;
    modelSelection = { ...((body && body.models) || {}) };
    modelsTouched = false;
    fillModelSelect("model-rover", "rover");
    fillModelSelect("model-drone", "drone");
  } catch (_) {
    modelsCatalog = null;
  } finally {
    modelsOptionsPending = false;
    renderModels();
  }
}

async function applyModels() {
  if (modelsPending || currentState !== "idle" || !modelsCatalog) return;
  const payload = {};
  for (const vehicle of Object.keys(modelsCatalog.options || {})) {
    const select = $(`model-${vehicle}`);
    if (select && select.value) payload[vehicle] = select.value;
  }
  if (Object.keys(payload).length === 0) return;
  modelsPending = true;
  renderControls();
  setModelFeedback("正在重载场景以切换载具模型…");
  try {
    const body = await request("/api/models", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ models: payload }),
    });
    modelSelection = { ...((body && body.models) || payload) };
    modelsTouched = false;
    if (body && body.state === "connecting") {
      modelsRebuilding = true;
      setModelFeedback("正在重载场景；等待 worker 重新连接…");
    } else {
      setModelFeedback("选择未变化。", "ok");
    }
  } catch (error) {
    modelsTouched = false;
    syncModelSelects();
    setModelFeedback("切换失败：" + error.message, "error");
  } finally {
    modelsPending = false;
    renderControls();
  }
}

function weatherSummary(fields) {
  if (!fields) return "—";
  const pct = (value) => `${Math.round((Number(value) || 0) * 100)}%`;
  return [
    `雨 ${pct(fields.rain)}`,
    `雪 ${pct(fields.snow)}`,
    `雾 ${pct(fields.fog)}`,
    `沙尘 ${pct(fields.dust)}`,
  ].join(" · ");
}

function formatWeatherValue(value) {
  const number = Number(value);
  if (!Number.isFinite(number)) return "—";
  return String(Number(number.toFixed(2)));
}

function updateWeatherOutput(spec) {
  const output = $(`weather-${spec.key}-value`);
  if (output) output.textContent = formatWeatherValue(weatherValues[spec.key]);
}

function setWeatherFields(fields) {
  for (const spec of weatherFieldSpecs) {
    const value = Number(fields && fields[spec.key]);
    if (Number.isFinite(value)) {
      weatherValues[spec.key] = value;
      const input = $(`weather-${spec.key}`);
      if (input) input.value = String(value);
    }
    updateWeatherOutput(spec);
  }
}

function initWeatherFields() {
  const container = $("weather-fields");
  container.replaceChildren();
  for (const spec of weatherFieldSpecs) {
    const row = document.createElement("div");
    row.className = "weather-row";

    const label = document.createElement("label");
    label.htmlFor = `weather-${spec.key}`;
    label.textContent = spec.label;

    const input = document.createElement("input");
    input.type = "range";
    input.id = `weather-${spec.key}`;
    input.min = "0";
    input.max = "1";
    input.step = "0.01";
    input.disabled = true;
    input.addEventListener("input", () => {
      weatherValues[spec.key] = Number(input.value);
      weatherTouched = true;
      updateWeatherOutput(spec);
      $("weather-preset").value = "";
      setWeatherFeedback("");
    });

    const output = document.createElement("output");
    output.id = `weather-${spec.key}-value`;
    output.textContent = "—";

    row.append(label, input, output);
    container.appendChild(row);
  }
}

function fillWeatherPresets(presets) {
  const select = $("weather-preset");
  const chosen = select.value;
  select.replaceChildren();
  const custom = document.createElement("option");
  custom.value = "";
  custom.textContent = "自定义";
  select.appendChild(custom);
  for (const preset of presets || []) {
    if (!preset || !preset.id) continue;
    const option = document.createElement("option");
    option.value = preset.id;
    option.textContent = preset.label || preset.id;
    select.appendChild(option);
  }
  if (
    chosen &&
    Array.from(select.options).some((option) => option.value === chosen)
  ) {
    select.value = chosen;
  }
}

function renderWeather() {
  const enabled =
    currentState === "idle" &&
    !weatherPending &&
    !worldRebuilding &&
    weatherCatalog !== null;
  $("weather-preset").disabled = !enabled;
  $("weather-apply").disabled = !enabled;
  for (const spec of weatherFieldSpecs) {
    const input = $(`weather-${spec.key}`);
    if (input) input.disabled = !enabled;
  }
}

async function loadWeatherOptions() {
  if (weatherOptionsPending) return;
  weatherOptionsPending = true;
  try {
    const body = await request("/api/weather");
    weatherCatalog = body;
    const fields = Array.isArray(body && body.fields) ? body.fields : [];
    if (weatherFieldSpecs.length === 0 && fields.length) {
      weatherFieldSpecs = fields.map((field) => ({
        key: field.id,
        label: field.label,
      }));
      weatherValues = {};
      for (const spec of weatherFieldSpecs) weatherValues[spec.key] = 0;
      initWeatherFields();
    }
    fillWeatherPresets((body && body.presets) || []);
    const current = weatherStatus;
    setWeatherFields(
      current || Object.fromEntries(weatherFieldSpecs.map((s) => [s.key, 0]))
    );
    $("weather-current").textContent = weatherSummary(current);
  } catch (_) {
    weatherCatalog = null;
    $("weather-current").textContent = "天气未连接";
  } finally {
    weatherOptionsPending = false;
    renderWeather();
  }
}

async function applyWeather() {
  if (weatherPending || currentState !== "idle" || !weatherCatalog) return;
  const payload = {};
  for (const spec of weatherFieldSpecs) {
    const value = Number(weatherValues[spec.key]);
    if (Number.isFinite(value)) payload[spec.key] = value;
  }
  weatherPending = true;
  renderControls();
  setWeatherFeedback("正在应用天气…");
  try {
    const body = await request("/api/weather", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const applied = (body && body.weather) || payload;
    weatherStatus = applied;
    weatherTouched = false;
    setWeatherFields(applied);
    $("weather-current").textContent = weatherSummary(applied);
    setWeatherFeedback("天气已应用。", "ok");
  } catch (error) {
    setWeatherFeedback("应用失败：" + error.message, "error");
  } finally {
    weatherPending = false;
    renderControls();
  }
}


function formatAngle(value) {
  const number = Number(value);
  return Number.isFinite(number) ? `${number.toFixed(1)}°` : "—";
}

function wrapYawDelta(deg) {
  const value = Number(deg);
  if (!Number.isFinite(value)) return 0;
  return ((((value + 180) % 360) + 360) % 360) - 180;
}

function relativeIncrement(previous, point) {
  const worldDx = Number(point.x) - Number(previous.x);
  const worldDy = Number(point.y) - Number(previous.y);
  const angle = (Number(previous.yaw_deg) * Math.PI) / 180;
  return {
    dx: Math.cos(angle) * worldDx + Math.sin(angle) * worldDy,
    dy: -Math.sin(angle) * worldDx + Math.cos(angle) * worldDy,
    dyaw: wrapYawDelta(Number(point.yaw_deg) - Number(previous.yaw_deg)),
  };
}

function formatSigned(value, digits) {
  const number = Number(value);
  if (!Number.isFinite(number)) return "—";
  return `${number >= 0 ? "+" : ""}${number.toFixed(digits)}`;
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
    coords.textContent = delta
      ? `dx ${formatSigned(delta.dx, 2)} · dy ${formatSigned(delta.dy, 2)}`
      : "dx — · dy —";
    const meta = document.createElement("span");
    meta.className = "point-meta";
    meta.textContent = delta
      ? `#${point.waypoint_id} · dyaw ${formatSigned(delta.dyaw, 1)}°`
      : `#${point.waypoint_id} · 航向 ${formatAngle(point.yaw_deg)}`;
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
  document.querySelectorAll("#view-modes button").forEach((button) => {
    button.classList.toggle("active", button.dataset.mode === taskView);
    button.disabled = !selectable;
  });
  if (viewMode === "xpra") {
    $("view-hint").textContent =
      `xpra 交互画面 · 当前视角：${VIEW_LABELS[taskView] || taskView}`;
  } else {
    $("view-hint").textContent = "相机最新帧 · 空闲时停留最后一帧";
  }
}

async function requestView(mode) {
  if (mode === taskView || !VIEW_LABELS[mode]) return;
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
      setFeedback(
        `视角已记录为${VIEW_LABELS[mode]}，任务结束后生效。`,
        "ok"
      );
    } else {
      setFeedback(`正在切换到${VIEW_LABELS[mode]}，完成后自动显示画面。`, "ok");
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

function init() {
  $("task-form").addEventListener("submit", submitTask);
  $("stop").addEventListener("click", stopTask);
  $("clear-instruction").addEventListener("click", () => {
    $("instruction").value = "";
    setFeedback("");
    renderControls();
  });
  $("instruction").addEventListener("input", renderControls);
  $("instruction").addEventListener("keydown", (event) => {
    if (event.key === "Enter" && !event.shiftKey) {
      event.preventDefault();
      $("task-form").requestSubmit();
    }
  });
  $("home").addEventListener("click", returnHome);
  $("retry-video").addEventListener("click", retryView);
  $("focus-view").addEventListener("click", focusXpra);
  $("xpra-frame").addEventListener("load", () => {
    if (viewMode === "xpra") focusXpra();
  });
  $("view-toggle").addEventListener("click", () => {
    setViewMode(viewMode === "xpra" ? "camera" : "xpra");
  });
  document.querySelectorAll("#vehicle-modes button").forEach((button) => {
    button.addEventListener("click", () =>
      activateVehicle(button.dataset.vehicle)
    );
  });
  document.querySelectorAll("#view-modes button").forEach((button) => {
    button.addEventListener("click", () => requestView(button.dataset.mode));
  });
  $("world-apply").addEventListener("click", applyWorld);
  $("world-reload").addEventListener("click", () => {
    loadWorldOptions();
    loadWeatherOptions();
    loadModelOptions();
  });
  $("models-apply").addEventListener("click", applyModels);
  for (const id of ["model-rover", "model-drone"]) {
    $(id).addEventListener("change", () => {
      modelsTouched = true;
      setModelFeedback("已选择新的模型，点击「应用载具模型」生效。");
      renderModels();
    });
  }
  $("weather-apply").addEventListener("click", applyWeather);
  $("weather-preset").addEventListener("change", () => {
    const select = $("weather-preset");
    const preset =
      weatherCatalog && Array.isArray(weatherCatalog.presets)
        ? weatherCatalog.presets.find((item) => item.id === select.value)
        : null;
    if (!preset || !preset.values) return;
    weatherTouched = true;
    setWeatherFields(preset.values);
    setWeatherFeedback("已载入预设，点击「应用天气」生效。");
  });
  $("trajectory-toggle").addEventListener("click", () => {
    trajectoryExpanded = !trajectoryExpanded;
    if (lastTrajectory) renderTrajectory(lastTrajectory);
  });
  $("video").addEventListener("load", () => {
    if (viewMode === "camera" && $("video").naturalWidth > 0) {
      markVideoLoaded();
    }
  });
  $("video").addEventListener("error", () => {
    if (viewMode !== "camera") return;
    clearInterval(videoWatch);
    videoWatch = null;
    videoLoaded = false;
    $("video-overlay").hidden = false;
    $("stream-status").textContent = "画面中断，3 秒后重试";
    scheduleVideoRetry();
  });

  setState("unknown");
  renderControls();
  loadWorldOptions();
  loadWeatherOptions();
  loadModelOptions();
  startXpra();
  pollStatus();
  pollTrajectory();
}

init();