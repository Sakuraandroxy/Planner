"use strict";

// 入口：状态轮询、任务提交与停止、载具切换、事件绑定。
// 加载顺序 core → world → settings → flight → 本文件，最后的 init 才动手。

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
  taskView = (VIEW_BY_VEHICLE[name] || {}).third || previousView;
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
    // Take the family flags first: the vehicle buttons below must be filtered
    // by them, because server.vehicles is global while the active map is not.
    if (body && body.capabilities) caps = body.capabilities;
    if (body && Array.isArray(body.vehicles) && body.vehicles.length) {
      vehicles = body.vehicles.filter(
        (name) =>
          typeof name === "string" &&
          name.length > 0 &&
          allowsVehicle(name)
      );
      if (!vehicles.includes(selectedVehicle)) {
        // 界面默认无人机。
        selectedVehicle = vehicles.includes("drone") ? "drone" : vehicles[0];
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
    profileCurrent = (body && body.profile) || "";
    profileSwitching = Boolean(body && body.profile_switching);
    $("current-profile").textContent = profileName(profileCurrent);
    if (body && body.view_mode) taskView = body.view_mode;
    if (body && body.weather) {
      weatherStatus = body.weather;
      if (!weatherTouched) setWeatherFields(weatherStatus);
    }
    const active = state === "running" || state === "stopping";
    $("current-instruction").textContent = active
      ? body.instruction || "—"
      : "—";
    $("last-result").textContent = body.last_result || "—";
    if (state === "idle" && profileRebuilding) {
      profileRebuilding = false;
      setProfileFeedback("已切换", "ok");
      loadProfileOptions();
      if (viewMode === "xpra") resetXpra();
      else {
        stopVideo();
        startVideo();
      }
    } else if (state === "idle" && modelsRebuilding) {
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
      (worldCatalog === null ||
        weatherCatalog === null ||
        modelsCatalog === null ||
        profileCatalog === null)
    ) {
      if (worldCatalog === null) loadWorldOptions();
      if (weatherCatalog === null) loadWeatherOptions();
      if (modelsCatalog === null) loadModelOptions();
      if (profileCatalog === null) loadProfileOptions();
    }
    if (body && body.sim_error && worldRebuilding) {
      setWorldFeedback("切换失败：" + body.sim_error, "error");
    }
    if (worldStopping && (state === "disconnected" || (body && body.sim_error))) {
      worldStopping = false;
      if (body && body.sim_error) {
        setWorldFeedback("停止失败：" + body.sim_error, "error");
      } else {
        setWorldFeedback("场景已停止，可以重新选择地图启动。", "ok");
        loadWorldOptions();
        stopVideo();
        resetXpra();
      }
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

async function saveEvalResult() {
  if ($("eval-save").disabled || submitPending) return;
  setFeedback("正在保存评测结果…");
  try {
    const body = await request("/api/eval/save", { method: "POST" });
    const names = (body.saved || []).map((item) => item.name).join("、");
    setFeedback(
      `已保存 ${body.count || 0} 条评测记录：${names || "—"}（点「评测」查看）`,
      "ok"
    );
  } catch (error) {
    setFeedback("保存评测结果失败：" + error.message, "error");
  }
}

// 只有「最近一次任务已结束且含带目标子任务」时，「保存评测结果」才可点。
async function pollEvalPending() {
  const button = $("eval-save");
  if (!button) return;
  try {
    const body = await request("/api/eval/pending");
    button.disabled = !(body && body.savable);
  } catch (error) {
    button.disabled = true;
  }
}

// 预设指令读 console/presets.json：一个字符串数组，一条一行。后端把 console/ 目录
// 当静态文件伺服，加指令只改这个文件再刷新页面。
let presetCommands = [];

function renderPresetCommands() {
  const select = $("instruction-preset");
  select.replaceChildren();
  const placeholder = document.createElement("option");
  placeholder.value = "";
  placeholder.textContent = "";
  select.appendChild(placeholder);
  for (const text of presetCommands) {
    const node = document.createElement("option");
    node.value = text;
    node.textContent = text;
    select.appendChild(node);
  }
  select.value = "";
  $("preset-strip").hidden = presetCommands.length === 0;
}

async function loadPresetCommands() {
  try {
    const body = await request("/console/presets.json");
    presetCommands = Array.isArray(body)
      ? body
          .map((text) => (typeof text === "string" ? text.trim() : ""))
          .filter((text) => text.length > 0)
      : [];
  } catch (error) {
    // 预设文件缺失或写坏：下拉框隐藏，指令输入框照常可用。
    presetCommands = [];
  }
  renderPresetCommands();
}

function applyPresetCommand() {
  const select = $("instruction-preset");
  const text = select.value;
  // 选完回到「预设」，同一条可以连续选两次。
  select.value = "";
  if (!text) return;
  $("instruction").value = text;
  renderControls();
  setFeedback("已填入预设指令，点击「开始执行」提交。");
}

function init() {
  // Browser-level jump to the world-model service on this same host; the
  // console does not call it. Scheme follows the page, host follows the
  // address bar.
  $("world-model").href =
    `${location.protocol}//${location.hostname}:${WORLD_MODEL_PORT}/`;
  $("eval-dash").href =
    `${location.protocol}//${location.hostname}:${EVAL_PORT}/`;
  // 点「评测」→ 先让规划器后端把评测仪表盘拉起来（没跑的话），再打开它。
  $("eval-dash").addEventListener("click", (event) => {
    event.preventDefault();
    const url = $("eval-dash").href;
    // 同步开窗，避免 await 之后被浏览器当成非用户手势而拦截弹窗。
    const win = window.open("", "_blank");
    if (win) {
      win.document.write(
        "<!doctype html><title>评测</title>" +
          "<p style='font-family:sans-serif;padding:2rem'>正在启动评测服务…</p>"
      );
      win.document.close();
    }
    request("/api/eval/start", { method: "POST" })
      .then(() => {
        if (win) win.location = url;
        else window.open(url, "_blank", "noopener");
      })
      .catch((error) => {
        if (win) win.close();
        setFeedback("启动评测服务失败：" + error.message, "error");
      });
  });
  $("task-form").addEventListener("submit", submitTask);
  $("stop").addEventListener("click", stopTask);
  $("clear-instruction").addEventListener("click", () => {
    $("instruction").value = "";
    setFeedback("");
    renderControls();
  });
  $("instruction").addEventListener("input", renderControls);
  $("instruction-preset").addEventListener("change", applyPresetCommand);
  $("instruction").addEventListener("keydown", (event) => {
    if (event.key === "Enter" && !event.shiftKey) {
      event.preventDefault();
      $("task-form").requestSubmit();
    }
  });
  $("home").addEventListener("click", returnHome);
  $("eval-save").addEventListener("click", saveEvalResult);
  pollEvalPending();
  setInterval(pollEvalPending, 1500);
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
    button.addEventListener("click", () =>
      requestView(button.dataset.perspective)
    );
  });
  $("world-apply").addEventListener("click", applyWorld);
  $("world-stop").addEventListener("click", stopWorld);
  settingsTabs().forEach((tab) => {
    tab.addEventListener("click", () => selectSettingsTab(tab.dataset.tab));
  });
  $("world-reload").addEventListener("click", () => {
    loadWorldOptions();
    loadWeatherOptions();
    loadModelOptions();
    loadProfileOptions();
  });
  $("models-apply").addEventListener("click", applyModels);
  $("profile-apply").addEventListener("click", applyProfile);
  $("profile-choice").addEventListener("change", () => {
    profileSelection = $("profile-choice").value;
    renderProfiles();
  });
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
  loadPresetCommands();
  loadWorldOptions();
  loadWeatherOptions();
  loadModelOptions();
  loadProfileOptions();
  startXpra();
  pollStatus();
  pollTrajectory();
}

init();
