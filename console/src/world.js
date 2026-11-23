"use strict";

// 场景侧设置：地图（启动、切换、停止）与天气。两者都作用于仿真器世界，
// 所以同在一个文件；规划器与载具外观在 settings.js。

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
  // Stopping only makes sense while a scene is up, and only when this service
  // launched it: /api/world/options reports that as simulator_owned.
  const owned = worldCatalog === null || worldCatalog.simulator_owned !== false;
  $("world-stop").disabled = !(
    selectable &&
    owned &&
    currentState === "idle"
  );
  $("world-stop").hidden = !owned;
  $("world-apply").textContent =
    currentState === "disconnected" ? "启动场景" : "应用世界设置";
}

async function stopWorld() {
  if (worldPending || currentState !== "idle" || !worldCatalog) return;
  worldPending = true;
  renderControls();
  setWorldFeedback("正在停止场景；先关闭会话，再停掉仿真器…");
  try {
    const body = await request("/api/world/stop", { method: "POST" });
    if (body && body.state === "disconnected") {
      setWorldFeedback("场景已停止。", "ok");
    } else {
      worldStopping = true;
    }
  } catch (error) {
    setWorldFeedback("停止失败：" + error.message, "error");
  } finally {
    worldPending = false;
    renderControls();
  }
}

async function loadWorldOptions() {
  if (worldOptionsPending) return;
  worldOptionsPending = true;
  try {
    const body = await request("/api/world/options");
    worldCatalog = body;
    // 显式 null 表示没有能力报告，按不可用处理；不能用 `if (caps)` 判值。
    if (body && "capabilities" in body) caps = body.capabilities;
    if (Array.isArray(body.maps) && body.selection && body.selection.map) {
      // The simulator follows the selected map, so switching scenes re-gates
      // the controls without waiting for the next status poll.
      if (body.map_simulators && body.map_simulators[body.selection.map]) {
        renderControls();
      }
    }
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

// ---- weather -----------------------------------------------------------

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
  } catch (_) {
    weatherCatalog = null;
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
    setWeatherFeedback("天气已应用。", "ok");
  } catch (error) {
    setWeatherFeedback("应用失败：" + error.message, "error");
  } finally {
    weatherPending = false;
    renderControls();
  }
}
