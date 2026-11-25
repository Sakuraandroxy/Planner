"use strict";

const state = {
  run: null,           // 当前所选 episode 所属的 run（看轨迹/画面用）
  latestRun: null,     // 最新一批 run 名（顶部指标来源）
  history: [],         // 平铺时间线：所有 run 的每集结果
  selectedUuid: null,
  traj: null,          // {positions, target, start, radius}
  cam: "front",
  viewSaved: false,    // 正在看已保存记录（无相机帧）
  viewDir: null,       // 正在看的那条已保存记录目录名
  savedTabVisible: false,
  profileTouched: false, // 用户手动选过 profile 后，不再自动跟随主界面当前值
  az: -Math.PI / 4,    // 方位角
  el: Math.PI / 6,     // 俯仰角
  dragging: false,
  lastMouse: null,
};

const $ = (id) => document.getElementById(id);
const plannerPort = new URLSearchParams(location.search).get("planner-port");
const theme = getComputedStyle(document.documentElement);
const chartColors = {
  panel: theme.getPropertyValue("--panel").trim(),
  background: theme.getPropertyValue("--bg").trim(),
  text: theme.getPropertyValue("--muted").trim(),
  start: theme.getPropertyValue("--accent").trim(),
  current: theme.getPropertyValue("--warning").trim(),
  target: theme.getPropertyValue("--danger").trim(),
};
let selectionRequest = 0;

function setPlannerLink(configuredPort = 8000) {
  const port = plannerPort ?? String(configuredPort);
  const url = new URL("/", location.href);
  url.port = port === "" || (/^[0-9]{1,5}$/.test(port) && Number(port) > 0 && Number(port) <= 65535)
    ? port : String(configuredPort);
  $("plannerLink").href = url.href;
}

function escapeHTML(value) {
  return String(value).replace(/[&<>]/g, (char) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[char]));
}

function fmt(v, suffix = "") {
  if (v === null || v === undefined || Number.isNaN(v)) return "—";
  return (typeof v === "number" ? v.toFixed(2) : v) + suffix;
}

async function getJSON(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(r.status);
  return r.json();
}

// run_YYYYmmdd_HHMMSS -> "MM-DD HH:MM"（历史卡片上显示跑批时间）
function runTimeLabel(run) {
  const m = /^run_(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})$/.exec(run || "");
  return m ? `${m[2]}-${m[3]} ${m[4]}:${m[5]}` : (run || "");
}

async function poll() {
  let hist = null, s = null;
  try { hist = await getJSON("/api/history"); } catch (e) {}
  try { s = await getJSON("/api/state"); } catch (e) {}   // 不带 run = 最新一批
  if (hist) {
    state.history = hist.episodes || [];
    renderCards(state.history);
  }
  if (!s) return;
  const m = s.metrics || {};
  const prog = s.progress || {};
  state.latestRun = s.run || null;
  // 看已保存记录时，顶部指标保留该记录自身的值，不被 run 覆盖。
  if (!state.savedTabVisible) {
    $("m-sr").textContent = m.sr != null ? m.sr.toFixed(1) + "%" : "—";
    $("m-osr").textContent = m.osr != null ? m.osr.toFixed(1) + "%" : "—";
    $("m-ne").textContent = m.ne_mean != null ? m.ne_mean.toFixed(1) + "m" : "—";
    $("m-spl").textContent = m.spl != null ? m.spl.toFixed(1) + "%" : "—";
    $("m-prog").textContent = `${prog.done ?? 0}/${prog.total ?? "?"}`;
  }
  // 若当前有运行中 episode 且没手选，自动跟到时间线上（看已保存记录时不抢焦点）
  if (!state.savedTabVisible && !state.selectedUuid && prog.current) {
    const hit = state.history.find(e => e.uuid === prog.current && e.run === s.run);
    if (hit) selectEpisode(hit.uuid, hit.run);
  }
}

function revealViewer() {
  if (matchMedia("(max-width: 760px)").matches) {
    document.querySelector(".viewer").scrollIntoView({ block: "start" });
  }
}

let historySig = null;
function renderCards(entries) {
  const sig = JSON.stringify([entries, state.selectedUuid, state.run, profileLabels]);
  if (sig === historySig) return;
  historySig = sig;
  const box = $("cards");
  const scrollTop = box.scrollTop;
  box.innerHTML = "";
  if (!entries.length) {
    box.innerHTML = '<div class="stats">暂无历史结果。</div>';
    return;
  }
  for (const e of entries) {
    const div = document.createElement("div");
    const active = e.uuid === state.selectedUuid && e.run === state.run;
    div.className = "card" + (active ? " active" : "");
    const badge = e.success
      ? `<span class="pill ok">成功</span>`
      : (e.collided ? `<span class="pill coll">碰撞</span>` : `<span class="pill fail">未成</span>`);
    const ptxt = profileText(e.planner);
    const ptag = ptxt ? `<span class="pill model">${escapeHTML(ptxt)}</span>` : "";
    const tlabel = runTimeLabel(e.run);
    div.innerHTML =
      `<div class="row1"><span class="uuid">${escapeHTML((e.uuid || "").slice(0, 13))}</span>
         <span class="row1-right">${ptag}${badge}<button class="card-del" aria-label="删除该集结果" title="删除该集结果">×</button></span></div>
       <div class="scene">${escapeHTML([e.scene, tlabel].filter(Boolean).join(" · "))}</div>
       <div class="meta"><span>NE ${fmt(e.final_ne, "m")}</span><span>步 ${e.steps ?? "—"}</span></div>`;
    div.querySelector(".card-del").onclick = (ev) => deleteEpisode(e.run, e.uuid, div, ev);
    div.onclick = () => {
      selectEpisode(e.uuid, e.run);
      revealViewer();
    };
    box.appendChild(div);
  }
  box.scrollTop = scrollTop;
}

function setViewerTitle(title) {
  const heading = $("epTitle");
  heading.textContent = title;
  heading.title = title;
}

function clearSelection() {
  selectionRequest++;
  state.selectedUuid = null;
  state.run = null;
  state.viewSaved = false;
  state.viewDir = null;
  state.traj = null;
  setViewerTitle("轨迹与相机画面");
  renderStats();
  refreshCamView();
  draw();
}

async function deleteEpisode(run, uuid, card, ev) {
  ev.stopPropagation();
  if (!confirm(`删除该集结果？\n${uuid}`)) return;
  card.remove();
  historySig = null;
  if (state.selectedUuid === uuid && state.run === run) clearSelection();
  try { await postJSON("/api/episode/delete", { run, id: uuid }); }
  catch (e) { alert("删除失败：" + e.message); }
  await poll();                                   // 拉最新列表与重算后的指标
}

async function selectEpisode(uuid, run) {
  const requestId = ++selectionRequest;
  state.viewSaved = false;
  state.viewDir = null;
  state.selectedUuid = uuid;
  state.run = run || state.run;
  state.traj = null;
  renderCards(state.history);
  setViewerTitle("评测样本 " + uuid.slice(0, 13));
  updateFrame();
  renderStats();
  draw();
  let traj = null;
  try {
    traj = await getJSON(`/api/traj?run=${encodeURIComponent(state.run)}&id=${encodeURIComponent(uuid)}`);
  } catch (e) {}
  if (requestId !== selectionRequest) return;
  state.traj = traj;
  renderStats();
  draw();
}

// 相机缩略图的显隐与「无画面」占位（不再出现碎图图标）。
function refreshCamView() {
  const front = $("camFront"), down = $("camDown"), empty = $("camEmpty");
  const showEmpty = (text) => { empty.textContent = text; empty.hidden = false; };
  front.hidden = true;
  down.hidden = true;
  if (state.viewSaved) {
    showEmpty("该记录仅保存轨迹与指标，无画面");
    return;
  }
  if (!state.selectedUuid) {
    showEmpty("选择一条评测结果查看画面");
    return;
  }
  const active = state.cam === "front" ? front : down;
  if (active.dataset.ok === "1") {
    active.hidden = false;
    empty.hidden = true;
  } else {
    showEmpty(active.dataset.ok === "0" ? "未采集到此视角的画面" : "正在读取画面…");
  }
}

function bindCamImages() {
  for (const img of [$("camFront"), $("camDown")]) {
    img.addEventListener("load", () => {
      if (img.naturalWidth > 0) { img.dataset.ok = "1"; refreshCamView(); }
    });
    img.addEventListener("error", () => { img.dataset.ok = "0"; refreshCamView(); });
  }
}

function updateFrame() {
  if (!state.selectedUuid) return;
  const img = state.cam === "front" ? $("camFront") : $("camDown");
  img.dataset.ok = "";
  refreshCamView();
  const url = `/api/frame?run=${encodeURIComponent(state.run)}&id=${encodeURIComponent(state.selectedUuid)}&cam=${state.cam}&t=${Date.now()}`;
  img.src = url;
}

function renderStats() {
  const t = state.traj;
  const box = $("epStats");
  if (!t || !t.positions || !t.positions.length) { box.innerHTML = ""; return; }
  const target = t.target;
  const last = t.positions[t.positions.length - 1];
  const ne = target ? dist(last, target) : null;
  box.innerHTML =
    `<div>目标: <b>[${target ? target.map(v => v.toFixed(1)).join(", ") : "—"}]</b></div>
     <div>成功半径: <b>${t.radius} m</b></div>
     <div>末点 NE: <b>${ne != null ? ne.toFixed(2) + " m" : "—"}</b></div>
     <div>轨迹点: <b>${t.positions.length}</b></div>`;
}

function dist(a, b) {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

// ---------- 3D 画布 ----------
function project(p, center, scale, W, H, az, el) {
  const x = (p[0] - center[0]) * scale;
  const y = (p[1] - center[1]) * scale;
  const z = (p[2] - center[2]) * scale;
  const x1 = x * Math.cos(az) - y * Math.sin(az);
  const y1 = x * Math.sin(az) + y * Math.cos(az);
  // z 朝下（NED），屏幕上取 -z 为向上
  const up = -z * Math.cos(el) - y1 * Math.sin(el);
  return [W / 2 + x1, H / 2 - up];
}

function draw() {
  const cv = $("traj3d");
  const ctx = cv.getContext("2d");
  // 按显示尺寸 × devicePixelRatio 渲染，高分屏不糊、拉伸不变形。
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const cssW = cv.clientWidth || 640;
  const cssH = cv.clientHeight || 440;
  const bw = Math.round(cssW * dpr), bh = Math.round(cssH * dpr);
  if (cv.width !== bw || cv.height !== bh) { cv.width = bw; cv.height = bh; }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const W = cssW, H = cssH;

  // 背景径向渐变（看着有纵深）
  ctx.clearRect(0, 0, W, H);
  const bg = ctx.createRadialGradient(W * 0.5, H * 0.40, 30, W * 0.5, H * 0.5, Math.max(W, H) * 0.8);
  bg.addColorStop(0, chartColors.panel); bg.addColorStop(1, chartColors.background);
  ctx.fillStyle = bg; ctx.fillRect(0, 0, W, H);

  const t = state.traj;
  const hasTrail = Boolean(t?.positions?.length);
  $("trajLegend").hidden = !hasTrail;
  $("trajTarget").hidden = !t?.target;
  if (!hasTrail) {
    ctx.fillStyle = chartColors.text;
    ctx.font = "13px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("暂无轨迹", W / 2, H / 2);
    ctx.textAlign = "start";
    return;
  }
  const pts = t.positions;
  const all = pts.slice();
  if (t.target) all.push(t.target);
  const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  for (const p of all) for (let i = 0; i < 3; i++) {
    mn[i] = Math.min(mn[i], p[i]); mx[i] = Math.max(mx[i], p[i]);
  }
  const center = [(mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2, (mn[2] + mx[2]) / 2];
  const unitProjection = (p) => project(p, center, 1, 0, 0, state.az, state.el);
  const targetRing = t.target && t.radius != null
    ? Array.from({ length: 64 }, (_, i) => {
      const angle = i / 64 * Math.PI * 2;
      return [t.target[0] + t.radius * Math.cos(angle), t.target[1] + t.radius * Math.sin(angle), t.target[2]];
    })
    : [];
  let left = Infinity, right = -Infinity, top = Infinity, bottom = -Infinity;
  for (const p of all.concat(targetRing)) {
    const [x, y] = unitProjection(p);
    left = Math.min(left, x); right = Math.max(right, x);
    top = Math.min(top, y); bottom = Math.max(bottom, y);
  }
  const scale = Math.min(
    Math.max(1, W - 84) / Math.max(1, right - left),
    Math.max(1, H - 96) / Math.max(1, bottom - top),
  );
  const proj = (p) => {
    const [x, y] = unitProjection(p);
    return [W / 2 + (x - (left + right) / 2) * scale, H / 2 + 8 + (y - (top + bottom) / 2) * scale];
  };

  // 地面参考面：取轨迹最低处 z（NED 里 z 越大越靠下）
  const zG = mx[2];
  const padX = (mx[0] - mn[0]) * 0.25 + 1, padY = (mx[1] - mn[1]) * 0.25 + 1;
  const X0 = mn[0] - padX, X1 = mx[0] + padX, Y0 = mn[1] - padY, Y1 = mx[1] + padY;
  const corners = [proj([X0, Y0, zG]), proj([X1, Y0, zG]), proj([X1, Y1, zG]), proj([X0, Y1, zG])];

  // 地面菱形填充
  ctx.save();
  ctx.beginPath();
  ctx.moveTo(corners[0][0], corners[0][1]);
  for (let i = 1; i < 4; i++) ctx.lineTo(corners[i][0], corners[i][1]);
  ctx.closePath();
  const gg = ctx.createLinearGradient(corners[0][0], corners[0][1], corners[2][0], corners[2][1]);
  gg.addColorStop(0, "rgba(76,194,165,0.10)");
  gg.addColorStop(1, "rgba(76,194,165,0.02)");
  ctx.fillStyle = gg; ctx.fill();
  ctx.restore();

  // 地面网格
  const N = 8;
  ctx.strokeStyle = "rgba(141,160,168,0.22)"; ctx.lineWidth = 1;
  for (let i = 0; i <= N; i++) {
    const fx = X0 + (X1 - X0) * i / N;
    const a = proj([fx, Y0, zG]), b = proj([fx, Y1, zG]);
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
    const fy = Y0 + (Y1 - Y0) * i / N;
    const c = proj([X0, fy, zG]), d = proj([X1, fy, zG]);
    ctx.beginPath(); ctx.moveTo(c[0], c[1]); ctx.lineTo(d[0], d[1]); ctx.stroke();
  }

  // 高度垂线（虚线，从轨迹点落到地面，体现高度）
  ctx.save();
  ctx.strokeStyle = "rgba(76,194,165,0.18)"; ctx.setLineDash([3, 4]); ctx.lineWidth = 1;
  const stepN = Math.max(1, Math.floor(pts.length / 24));
  for (let i = 0; i < pts.length; i += stepN) {
    const p = pts[i]; const a = proj(p), b = proj([p[0], p[1], zG]);
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
  }
  ctx.restore();

  // 成功半径球（水平圆）
  if (targetRing.length) {
    ctx.strokeStyle = chartColors.target; ctx.lineWidth = 1.5;
    ctx.beginPath();
    targetRing.forEach((p, i) => {
      const [x, y] = proj(p);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.closePath();
    ctx.stroke();
  }

  // 轨迹：先一道发光底线，再按进度叠彩色细线
  ctx.save();
  ctx.lineJoin = "round"; ctx.lineCap = "round";
  ctx.shadowColor = chartColors.start; ctx.shadowBlur = 10;
  ctx.strokeStyle = "rgba(76,194,165,0.22)"; ctx.lineWidth = 5;
  ctx.beginPath();
  const s0 = proj(pts[0]); ctx.moveTo(s0[0], s0[1]);
  for (let i = 1; i < pts.length; i++) { const s = proj(pts[i]); ctx.lineTo(s[0], s[1]); }
  ctx.stroke();
  ctx.restore();

  ctx.lineJoin = "round"; ctx.lineCap = "round";
  for (let i = 1; i < pts.length; i++) {
    const a = proj(pts[i - 1]), b = proj(pts[i]);
    const f = i / pts.length;
    ctx.strokeStyle = `hsl(${165 - 125 * f}, 52%, 58%)`;
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
  }

  // 标记颜色与图例对应。
  const marker = (p, color, r) => {
    const s = proj(p);
    ctx.save();
    ctx.shadowColor = color; ctx.shadowBlur = 12;
    ctx.fillStyle = color; ctx.beginPath(); ctx.arc(s[0], s[1], r, 0, Math.PI * 2); ctx.fill();
    ctx.restore();
    ctx.strokeStyle = "rgba(255,255,255,0.85)"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(s[0], s[1], r, 0, Math.PI * 2); ctx.stroke();
  };
  marker(pts[0], chartColors.start, 5);
  marker(pts[pts.length - 1], chartColors.current, 4);
  if (t.target) marker(t.target, chartColors.target, 6);
}

// 拖拽旋转
function bindDrag() {
  const cv = $("traj3d");
  cv.addEventListener("mousedown", (e) => { state.dragging = true; state.lastMouse = [e.clientX, e.clientY]; });
  window.addEventListener("mouseup", () => { state.dragging = false; });
  window.addEventListener("mousemove", (e) => {
    if (!state.dragging) return;
    const dx = e.clientX - state.lastMouse[0];
    const dy = e.clientY - state.lastMouse[1];
    state.lastMouse = [e.clientX, e.clientY];
    state.az -= dx * 0.01;
    state.el = Math.max(-1.4, Math.min(1.4, state.el + dy * 0.01));
    draw();
  });
}

function bindCamTabs() {
  document.querySelectorAll(".cam-tabs button").forEach(btn => {
    btn.onclick = () => {
      document.querySelectorAll(".cam-tabs button").forEach((item) => {
        item.classList.toggle("active", item === btn);
        item.setAttribute("aria-pressed", String(item === btn));
      });
      state.cam = btn.dataset.cam;
      if (state.viewSaved || !state.selectedUuid) { refreshCamView(); return; }
      updateFrame();
    };
  });
}

// ---------- 发起评测 ----------
async function postJSON(url, body) {
  const r = await fetch(url, {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  let data = null;
  try { data = await r.json(); } catch (_) {}
  if (!r.ok) throw new Error((data && data.error) || ("HTTP " + r.status));
  return data;
}

let lastActive = false;

function setRunState(text, on) {
  const el = $("runState");
  el.textContent = text;
  el.classList.toggle("on", !!on);
}

function setLaunchDisabled(disabled) {
  $("batchBtn").disabled = disabled || !$("sceneSelect").value;
  document.querySelectorAll(".ep-test").forEach(b => b.disabled = disabled);
}

let lastProfileSig = null;
const profileLabels = {};   // profile id -> 中文名（default→世界模型，vlm→普通VLM）
// 只返回已知的中文名；未知/缺失返回空串（老数据没有 profile 就不显示，不显示裸 id）。
function profileText(id) {
  if (!id) return "";
  return profileLabels[id] || "";
}
// 规划器下拉：选项来自主服务（世界模型/普通VLM）。默认为主界面当前 profile，可手动改；
// 选了就用所选 profile 跑评测（仅影响评测，不动主界面实时会话）。
async function loadPlannerProfile() {
  const sel = $("modeSelect");
  let p;
  try { p = await getJSON("/api/eval/profile"); }
  catch (e) { p = { online: false }; }
  setPlannerLink(p.planner_port ?? 8000);
  if (!p.online) {
    if (lastProfileSig !== "offline") {
      sel.innerHTML = "";
      const opt = document.createElement("option");
      opt.value = ""; opt.textContent = "主界面未连接";
      sel.appendChild(opt);
      lastProfileSig = "offline";
    }
    return;
  }
  const profiles = p.profiles || [];
  for (const info of profiles) {
    if (info.id) profileLabels[info.id] = info.label || info.id;
  }
  const sig = profiles.map(x => x.id + ":" + x.label).join(",");
  if (sig !== lastProfileSig) {
    lastProfileSig = sig;
    sel.innerHTML = "";
    for (const info of profiles) {
      const opt = document.createElement("option");
      opt.value = info.id;
      opt.textContent = info.label || info.id;
      sel.appendChild(opt);
    }
    state.profileTouched = false;  // 选项变了，重置为跟随主界面
  }
  // 默认跟随主界面当前值；用户手动选过就不再覆盖。
  if (!state.profileTouched && p.current) sel.value = p.current;
}

async function loadDataset() {
  let data;
  try { data = await getJSON("/api/dataset"); } catch (e) { return; }
  const sel = $("sceneSelect");
  sel.innerHTML = "";
  for (const s of data.scenes || []) {
    const opt = document.createElement("option");
    opt.value = s.id;
    opt.textContent = `${s.id} (${s.num_episodes})`;
    sel.appendChild(opt);
  }
  sel.onchange = () => loadEpisodes(sel.value);
  setLaunchDisabled(lastActive);
  if (sel.value) await loadEpisodes(sel.value);
  else $("epList").innerHTML = '<div class="stats">暂无可评测的场景。</div>';
}

async function loadEpisodes(scene) {
  const box = $("epList");
  box.innerHTML = "";
  if (!scene) return;
  let data;
  try { data = await getJSON(`/api/episodes?scene=${encodeURIComponent(scene)}`); }
  catch (e) { return; }
  if (scene !== $("sceneSelect").value) return;
  for (const ep of data.episodes || []) {
    const row = document.createElement("div");
    row.className = "ep";
    const info = document.createElement("div");
    info.className = "info";
    const ins = document.createElement("div");
    ins.className = "ins";
    ins.textContent = ep.instruction || "(无指令)";
    ins.title = ep.instruction || "";
    const uid = document.createElement("div");
    uid.className = "uid";
    uid.textContent = (ep.uuid || "").slice(0, 18);
    info.appendChild(ins); info.appendChild(uid);
    const btn = document.createElement("button");
    btn.className = "ep-test"; btn.textContent = "测试";
    btn.disabled = lastActive;
    btn.onclick = () => startRun(scene, [ep.uuid]);
    row.appendChild(info); row.appendChild(btn);
    box.appendChild(row);
  }
}

async function startRun(scene, uuids) {
  if (!scene) { setRunState("先选一个场景", false); return; }
  const profile = $("modeSelect").value || null;
  const label = ($("modeSelect").selectedOptions[0] || {}).textContent || "";
  setRunState(`提交中: ${scene}${uuids ? " / 1集" : " / 全场景"}${label ? " (" + label + ")" : ""}…`, false);
  try {
    await postJSON("/api/start", { scene, uuids: uuids || null, profile });
    setRunState(`已提交，场景 ${scene} 启动中…`, true);
    await pollRunStatusOnce();
  } catch (e) {
    setRunState("启动失败: " + e.message, false);
  }
}

async function pollRunStatusOnce() {
  let s;
  try { s = await getJSON("/api/runstatus"); } catch (e) { return; }
  const active = !!s.active;
  setLaunchDisabled(active);
  if (active) {
    const pn = profileText(s.profile) || s.profile;
    setRunState(`运行中: ${s.scene}${pn ? " (" + pn + ")" : ""}…`, true);
    // 跟随最新 run（刚起的那个）
    await poll();
  } else if (lastActive) {
    const code = s.returncode;
    setRunState(`已结束${code != null ? " (exit " + code + ")" : ""}`, false);
    await poll();
  } else if (s.scene) {
    setRunState(`上次: ${s.scene}${s.returncode != null ? " 已结束" : ""}`, false);
  }
  lastActive = active;
}

async function pollRunStatus() {
  await pollRunStatusOnce();
  setTimeout(pollRunStatus, 2000);
}

// ---------- 已保存评测记录（规划器「保存评测结果」写入） ----------
let savedSig = null;   // 记录目录集合签名；null=尚未渲染，避免每 2s 重建闪烁
async function loadSaved() {
  let data;
  try { data = await getJSON("/api/saved"); } catch (e) { return; }
  const records = data.records || [];
  const sig = JSON.stringify([records, state.viewDir, profileLabels]);
  if (sig === savedSig) return;   // 没有增删，不重建
  savedSig = sig;
  const box = $("savedList");
  const scrollTop = box.scrollTop;
  box.innerHTML = "";
  if (!records.length) {
    const empty = document.createElement("div");
    empty.className = "stats";
    empty.textContent = "还没有保存的评测记录（在规划器里点「保存评测结果」）。";
    box.appendChild(empty);
    return;
  }
  for (const r of records) {
    const div = document.createElement("div");
    div.className = "card" + (r.dir === state.viewDir ? " active" : "");
    const sr = r.sr != null ? (r.sr * 100).toFixed(0) + "%" : "—";
    const pill = r.success
      ? '<span class="pill ok">成功</span>'
      : '<span class="pill fail">未成</span>';
    const ptxt = profileText(r.planner_profile);
    const ptag = ptxt ? `<span class="pill model">${escapeHTML(ptxt)}</span>` : "";
    div.innerHTML =
      `<div class="row1"><span class="uuid">${escapeHTML(r.name || r.dir)}</span>
         <span class="row1-right">${ptag}${pill}<button class="card-del" aria-label="删除该记录" title="删除该记录">×</button></span></div>
       <div class="scene">${escapeHTML(r.map || "")}</div>
       <div class="meta"><span>SR ${sr}</span><span>末距 ${r.final_distance_m != null ? r.final_distance_m.toFixed(2) + "m" : "—"}</span></div>`;
    div.querySelector(".uuid").title = r.name || r.dir;
    div.querySelector(".card-del").onclick = (ev) => deleteSaved(r.dir, div, ev);
    div.onclick = () => {
      viewSaved(r.dir);
      revealViewer();
    };
    box.appendChild(div);
  }
  box.scrollTop = scrollTop;
}

async function deleteSaved(dir, card, ev) {
  ev.stopPropagation();
  if (!confirm(`删除该评测记录？\n${dir}`)) return;
  card.remove();          // 先从界面移除，立刻见效（不等网络、不刷新）
  savedSig = null;        // 允许随后用服务器实际结果对齐重建
  if (state.viewSaved && state.viewDir === dir) {
    clearSelection();
    setSavedMetrics(null);
  }
  try { await postJSON("/api/saved/delete", { dir }); }
  catch (e) { alert("删除失败：" + e.message); }
  await loadSaved();      // 对齐服务器（删除失败则该卡片会重新出现）
}

async function viewSaved(dir) {
  const requestId = ++selectionRequest;
  let rec;
  try { rec = await getJSON(`/api/saved/traj?dir=${encodeURIComponent(dir)}`); }
  catch (e) { return; }
  if (requestId !== selectionRequest) return;
  state.selectedUuid = null;
  state.run = null;
  state.viewSaved = true;
  state.viewDir = dir;
  loadSaved();
  setViewerTitle("已保存: " + (rec.name || dir));
  state.traj = { positions: rec.positions || [], target: null, start: rec.start, radius: null };
  // 已保存记录只有轨迹+指标，没有相机帧
  refreshCamView();
  const pct = rec.sr != null ? (rec.sr * 100).toFixed(0) + "%" : "—";
  $("epStats").innerHTML =
    `<div>目标: <b>${escapeHTML(rec.target_description || "—")}</b></div>
     <div>成功: <b>${rec.success ? "是" : "否"}</b> (SR ${pct})</div>
     <div>末距离: <b>${rec.final_distance_m != null ? rec.final_distance_m.toFixed(2) + " m" : "—"}</b></div>
     <div>轨迹点: <b>${(rec.positions || []).length}</b></div>`;
  setSavedMetrics(rec);
  draw();
}

// 顶部指标改为显示所选「已保存记录」自身的值（切回历史 run 时 poll 会恢复）。
function setSavedMetrics(rec) {
  $("m-sr").textContent = rec?.sr != null ? (rec.sr * 100).toFixed(1) + "%" : "—";
  $("m-osr").textContent = "—";
  $("m-ne").textContent = rec?.final_distance_m != null ? rec.final_distance_m.toFixed(1) + "m" : "—";
  $("m-spl").textContent = "—";
  $("m-prog").textContent = rec ? "记录 1/1" : "—";
}

function bindResTabs() {
  document.querySelectorAll(".res-tab").forEach((btn) => {
    btn.onclick = () => {
      const showSaved = btn.dataset.tab === "saved";
      if (state.savedTabVisible === showSaved) return;
      document.querySelectorAll(".res-tab").forEach((item) => {
        item.classList.toggle("active", item === btn);
        item.setAttribute("aria-pressed", String(item === btn));
      });
      state.savedTabVisible = showSaved;
      clearSelection();
      $("savedList").hidden = !showSaved;
      $("cards").hidden = showSaved;
      if (showSaved) {
        setSavedMetrics(null);
        loadSaved();
      } else {
        poll();
      }
    };
  });
}

async function main() {
  setPlannerLink();
  new ResizeObserver(draw).observe($("traj3d"));
  bindDrag();
  bindCamTabs();
  bindCamImages();
  bindResTabs();
  $("batchBtn").onclick = () => startRun($("sceneSelect").value, null);
  // 用户手动选 profile 后停止自动跟随主界面当前值。
  $("modeSelect").onchange = () => { state.profileTouched = true; };
  await loadDataset();
  await loadPlannerProfile();
  await poll();
  setInterval(poll, 2000);
  // 每 3s 同步一次主界面 profile（只读镜像），主界面切了这里跟着变。
  setInterval(loadPlannerProfile, 3000);
  // 「已保存评测」标签可见时，每 2s 拉一次，新增/删除都自动反映（无增删则跳过多余重建）。
  setInterval(() => { if (state.savedTabVisible) loadSaved(); }, 2000);
  pollRunStatus();
}

main();
