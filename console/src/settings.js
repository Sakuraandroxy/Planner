"use strict";

// 这一趟用什么跑：载具外观模型与导航配置档。两者都会重建会话、把载具送回出生点，
// 所以门禁条件一致，放在一起看最清楚。

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

// ---- navigation profile ------------------------------------------------

function setProfileFeedback(text, kind = "") {
  const node = $("profile-feedback");
  node.textContent = text;
  if (kind) node.dataset.kind = kind;
  else delete node.dataset.kind;
}

function profileName(id) {
  const list = (profileCatalog && profileCatalog.profiles) || [];
  const info = list.find((item) => item.id === id) || null;
  return (info && info.label) || id || "—";
}

function fillProfileOptions() {
  const select = $("profile-choice");
  select.replaceChildren();
  for (const info of (profileCatalog && profileCatalog.profiles) || []) {
    const node = document.createElement("option");
    node.value = info.id;
    node.textContent = info.label || info.id;
    select.appendChild(node);
  }
}

function renderProfiles() {
  const enabled =
    (currentState === "idle" || currentState === "disconnected") &&
    !profilePending &&
    !profileRebuilding &&
    !profileSwitching &&
    !modelsRebuilding &&
    !modelsSwitching &&
    !worldRebuilding &&
    !mapSwitching &&
    profileCatalog !== null;
  const select = $("profile-choice");
  select.disabled = !enabled;
  if (profileSelection) select.value = profileSelection;
  $("profile-apply").disabled =
    !enabled || !profileSelection || profileSelection === profileCurrent;
}

async function loadProfileOptions() {
  if (profileOptionsPending) return;
  profileOptionsPending = true;
  try {
    const body = await request("/api/profile");
    profileCatalog = body;
    profileCurrent = (body && body.current) || "";
    profileSelection = profileCurrent;
    fillProfileOptions();
  } catch (_) {
    profileCatalog = null;
  } finally {
    profileOptionsPending = false;
    renderProfiles();
  }
}

async function applyProfile() {
  if (profilePending) return;
  if (currentState !== "idle" && currentState !== "disconnected") return;
  if (!profileCatalog) return;
  if (!profileSelection || profileSelection === profileCurrent) return;
  const wanted = profileSelection;
  profilePending = true;
  renderControls();
  setProfileFeedback("切换中…");
  try {
    const body = await request("/api/profile", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ profile: wanted }),
    });
    if (body && body.state === "connecting") {
      profileRebuilding = true;
      profileSwitching = true;
    } else {
      // Nothing was rebuilt, so this choice is already the current one. Waiting for a
      // rebuild that never comes would leave the panel stuck on 切换中….
      profileCurrent = (body && body.profile) || profileCurrent;
      profileSelection = profileCurrent;
      setProfileFeedback("已切换", "ok");
    }
  } catch (error) {
    profileSelection = profileCurrent;
    // The reason goes to the browser console; the panel shows one word.
    console.warn("profile switch failed", error);
    setProfileFeedback("切换失败", "error");
  } finally {
    profilePending = false;
    renderControls();
  }
}
