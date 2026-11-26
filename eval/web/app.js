"use strict";

const DEMO_RUN = "demo-city-core";
const DEMO_REPORT = {
  demo: true,
  run: "演示批次 · 城市核心区",
  time: "2025-06-18 14:30",
  planner: "default",
  num_episodes: 6,
  num_success: 5,
  num_collided: 1,
  dataset: {
    split: "test",
    scenes: ["城市核心区（示例）"],
    success_radius_m: 10,
  },
  metrics: { sr: 83.3, osr: 100.0, ne_mean: 6.42, spl: 71.8 },
  episodes: [
    {
      uuid: "city-core-001",
      scene: "城市核心区",
      success: true,
      final_ne: 3.24,
      steps: 42,
      path_length: 118.6,
    },
    {
      uuid: "city-core-002",
      scene: "城市核心区",
      success: true,
      final_ne: 4.81,
      steps: 38,
      path_length: 102.3,
    },
    {
      uuid: "city-core-003",
      scene: "城市核心区",
      success: true,
      final_ne: 5.16,
      steps: 45,
      path_length: 126.8,
    },
    {
      uuid: "city-core-004",
      scene: "城市核心区",
      success: true,
      final_ne: 6.03,
      steps: 51,
      path_length: 142.1,
    },
    {
      uuid: "city-core-005",
      scene: "城市核心区",
      success: true,
      final_ne: 7.86,
      steps: 47,
      path_length: 131.4,
    },
    {
      uuid: "city-core-006",
      scene: "城市核心区",
      success: false,
      collided: true,
      final_ne: 11.42,
      steps: 34,
      path_length: 89.7,
      error: "碰撞终止（示例）",
    },
  ],
};

const DEMO_VLM_RUN = "demo-city-core-vlm";
const DEMO_VLM_REPORT = {
  ...DEMO_REPORT,
  run: "演示批次 · 城市核心区 · 普通 VLM",
  time: "2025-06-18 14:45",
  planner: "vlm",
  num_success: 4,
  num_collided: 1,
  num_failed: 1,
  metrics: { sr: 66.7, osr: 83.3, ne_mean: 9.18, spl: 54.6 },
  episodes: DEMO_REPORT.episodes.map((episode, index) => ({
    ...episode,
    uuid: `vlm-${episode.uuid}`,
    success: index < 4,
    collided: index === 4,
    final_ne: [5.4, 7.2, 8.6, 9.8, 12.1, 11.4][index],
    steps: [47, 53, 58, 62, 39, 44][index],
    path_length: [132.5, 145.2, 151.8, 166.4, 98.7, 121.3][index],
    error: index === 4 ? "碰撞终止（演示数据）" : undefined,
  })),
};
const DEMO_BATCHES = [
  { name: DEMO_RUN, label: "世界模型", report: DEMO_REPORT },
  { name: DEMO_VLM_RUN, label: "普通 VLM", report: DEMO_VLM_REPORT },
];

const state = {
  demoActive: false,
  demoSuppressed: false,
  run: null, // 当前所选 episode 所属的 run（看轨迹/画面用）
  latestRun: null, // 最新一批 run 名（顶部指标来源）
  history: [], // 平铺时间线：所有 run 的每集结果
  runSummaries: {}, // run 名 -> 批次汇总（/api/runs，历史列表的批次卡片用）
  summaryRun: null, // 当前选中的批次（点了批次汇总卡片时；与 episode 选择互斥）
  selectedUuid: null,
  traj: null, // {positions, target, start, radius}
  cam: "front",
  viewSaved: false, // 正在看已保存记录（无相机帧）
  viewDir: null, // 正在看的那条已保存记录目录名
  savedTabVisible: false,
  profileTouched: false, // 用户手动选过 profile 后，不再自动跟随主界面当前值
  az: -Math.PI / 4, // 方位角
  el: Math.PI / 6, // 俯仰角
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
let comparisonCharts = [];
window.addEventListener("resize", () =>
  comparisonCharts.forEach((chart) => chart.resize()),
);

function setPlannerLink(configuredPort = 8000) {
  const port = plannerPort ?? String(configuredPort);
  const url = new URL("/", location.href);
  url.port =
    port === "" ||
    (/^[0-9]{1,5}$/.test(port) && Number(port) > 0 && Number(port) <= 65535)
      ? port
      : String(configuredPort);
  $("plannerLink").href = url.href;
}

function escapeHTML(value) {
  return String(value).replace(
    /[&<>]/g,
    (char) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" })[char],
  );
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
  return m ? `${m[2]}-${m[3]} ${m[4]}:${m[5]}` : run || "";
}

async function poll() {
  let hist = null,
    s = null,
    runs = null;
  try {
    hist = await getJSON("/api/history");
  } catch (e) {}
  try {
    s = await getJSON("/api/state");
  } catch (e) {} // 不带 run = 最新一批
  try {
    runs = await getJSON("/api/runs");
  } catch (e) {} // 批次汇总（SR 后端现算）
  if (runs && Array.isArray(runs.runs)) {
    state.runSummaries = {};
    for (const r of runs.runs) state.runSummaries[r.name] = r;
  }
  if (hist) {
    state.history = hist.episodes || [];
    renderCards(state.history);
    if (state.summaryRun && !state.demoActive)
      renderRunSummaryStats(state.summaryRun);
  }
  if (!s) return;
  const m = s.metrics || {};
  const progress = s.progress || {};
  const done = Number(progress.done || 0),
    total = Number(progress.total || 0);
  const running = progress.state === "running";
  if (running) {
    state.demoSuppressed = true;
    if (state.demoActive) {
      state.demoActive = false;
      clearSelection();
      renderCards(state.history);
    }
  }
  const progressBox = $("runProgress");
  progressBox.hidden = !running && !total;
  $("stopBtn").hidden = !running;
  if (!progressBox.hidden) {
    const percent = total ? Math.min(100, Math.round((done * 100) / total)) : 0;
    $("runProgressLabel").textContent =
      running && progress.current
        ? `正在测试 ${progress.current}`
        : running
          ? "批量评测进行中"
          : "最近一次评测进度";
    $("runProgressValue").textContent = `${done}/${total || "—"} · ${percent}%`;
    $("runProgressBar").style.width = `${percent}%`;
  }
  state.latestRun = s.run || null;
  if (!state.savedTabVisible && !state.summaryRun && !state.selectedUuid) {
    $("m-sr").textContent = m.sr != null ? m.sr.toFixed(1) + "%" : "—";
    $("m-osr").textContent = m.osr != null ? m.osr.toFixed(1) + "%" : "—";
    $("m-ne").textContent =
      m.ne_mean != null ? m.ne_mean.toFixed(1) + "m" : "—";
    $("m-spl").textContent = m.spl != null ? m.spl.toFixed(1) + "%" : "—";
  }
  if (
    !state.savedTabVisible &&
    !state.summaryRun &&
    !state.selectedUuid &&
    !progress.current &&
    state.history.length
  ) {
    selectRunSummary(state.history[0].run);
  }
  if (
    !state.savedTabVisible &&
    !state.summaryRun &&
    !state.selectedUuid &&
    progress.current
  ) {
    const hit = state.history.find(
      (e) => e.uuid === progress.current && e.run === s.run,
    );
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
  const box = $("cards");
  if (!entries.length) {
    if (state.savedTabVisible) {
      state.demoActive = false;
      box.innerHTML = '<div class="stats">暂无批量测试结果。</div>';
      return;
    }
    if (state.demoSuppressed) {
      state.demoActive = false;
      box.innerHTML = '<div class="stats">暂无批量测试结果。</div>';
      return;
    }
    renderDemoBoard();
    return;
  }
  if (state.demoActive) {
    state.demoActive = false;
    clearSelection();
  }
  const sig = JSON.stringify([
    entries,
    state.selectedUuid,
    state.run,
    state.summaryRun,
    state.runSummaries,
    profileLabels,
  ]);
  if (sig === historySig) return;
  historySig = sig;
  const scrollTop = box.scrollTop;
  box.innerHTML = "";
  for (const run of [...new Set(entries.map((e) => e.run))]) {
    box.appendChild(buildRunSummaryCard(run));
  }
  box.scrollTop = scrollTop;
}

function renderDemoBoard() {
  if (state.demoActive) return;
  state.demoActive = true;
  state.selectedUuid = null;
  state.run = null;
  state.summaryRun = DEMO_RUN;
  historySig = "demo-preview";
  renderDemoCards();
  renderDemoOverview(DEMO_RUN);
}

function renderDemoCards() {
  $("cards").innerHTML = DEMO_BATCHES.map(({ name, label, report }) => {
    const active = state.summaryRun === name || state.run === name;
    return `<div class="card card-run demo-run-card${active ? " active" : ""}" data-run="${name}">
      <div class="row1"><span class="uuid run-title">城市核心区 · ${label}</span><span class="pill demo-pill">演示数据</span></div>
      <div class="scene">${label}　·　${report.num_episodes} 个样例</div>
      <div class="meta run-metrics"><span>SR <b>${report.metrics.sr.toFixed(1)}%</b></span><span>OSR <b>${report.metrics.osr.toFixed(1)}%</b></span><span>NE <b>${report.metrics.ne_mean.toFixed(2)}m</b></span><span>SPL <b>${report.metrics.spl.toFixed(1)}%</b></span></div>
    </div>`;
  }).join("");
  $("cards")
    .querySelectorAll("[data-run]")
    .forEach((card) => {
      card.onclick = () => {
        selectDemoRun(card.dataset.run);
        revealViewer();
      };
    });
}

function selectDemoRun(run) {
  state.summaryRun = run;
  state.selectedUuid = null;
  state.run = null;
  state.traj = null;
  renderDemoCards();
  renderDemoOverview(run);
}

function renderDemoOverview(run = DEMO_RUN) {
  const batch =
    DEMO_BATCHES.find((item) => item.name === run) || DEMO_BATCHES[0];
  const report = batch.report;
  const episodes = report.episodes;
  const numSuccess = episodes.filter((episode) => episode.success).length;
  const numCollided = episodes.filter((episode) => episode.collided).length;
  const numFailed = episodes.length - numSuccess - numCollided;
  state.demoActive = true;
  state.summaryRun = batch.name;
  $("viewerDetail").hidden = true;
  $("viewer").classList.remove("has-episode");
  $("reportOverview").hidden = false;
  $("exportReportBtn").disabled = false;
  $("reportEyebrow").textContent = "演示数据";
  $("epTitle").textContent = "批次评测报告";
  $("reportSubtitle").textContent =
    `城市核心区（示例）　/　${batch.label}　/　${episodes.length} 个样例`;
  $("m-sr").textContent = `${report.metrics.sr.toFixed(1)}%`;
  $("m-osr").textContent = `${report.metrics.osr.toFixed(1)}%`;
  $("m-ne").textContent = `${report.metrics.ne_mean.toFixed(2)} m`;
  $("m-spl").textContent = `${report.metrics.spl.toFixed(1)}%`;
  const rows = episodes
    .map((episode) => {
      const status = episode.success
        ? '<span class="pill ok">成功</span>'
        : episode.collided
          ? '<span class="pill coll">碰撞</span>'
          : '<span class="pill fail">未完成</span>';
      return `<tr class="report-episode-row" data-uuid="${episode.uuid}"><td class="mono">${episode.uuid}</td><td>${status}</td><td>${episode.final_ne.toFixed(2)} m</td><td>${episode.steps}</td></tr>`;
    })
    .join("");
  $("reportOverview").innerHTML =
    `<div class="overview-banner"><div><span class="section-kicker">演示批次</span><h3>城市核心区（示例）</h3><p>${batch.label}　·　${episodes.length} 个样例</p></div>
      <div class="completion-ring"><b>${numSuccess}<small>/${episodes.length}</small></b><span>成功样例</span></div></div>
    <div class="result-counts"><div><i class="count-success"></i>成功 <b>${numSuccess}</b></div><div><i class="count-collision"></i>碰撞 <b>${numCollided}</b></div><div><i class="count-failed"></i>未完成 <b>${numFailed}</b></div></div>
    <div class="overview-table-head"><div><span class="section-kicker">EPISODE BREAKDOWN</span><h3>样例结果</h3></div></div>
    <div class="report-table-wrap"><table class="report-table"><thead><tr><th>样例 ID</th><th>结果</th><th>末端距离 NE</th><th>步数</th></tr></thead><tbody>${rows}</tbody></table></div>`;
  $("reportOverview")
    .querySelectorAll(".report-episode-row")
    .forEach((row) => {
      row.onclick = () => selectDemoEpisode(row.dataset.uuid, batch.name);
    });
}

function selectDemoEpisode(uuid, run) {
  const batch = DEMO_BATCHES.find((item) => item.name === run);
  const ep = batch?.report.episodes.find((item) => item.uuid === uuid);
  if (!ep) return;
  state.selectedUuid = uuid;
  state.run = run;
  state.summaryRun = null;
  const offset = run === DEMO_VLM_RUN ? 4 : 0;
  state.traj = {
    positions: [
      [0, 0, 0],
      [8 + offset, 3, -2],
      [16 + offset, 7, -5],
      [24 + offset, 12, -8],
      [31 + offset, 18, -10],
    ],
    target: [35 + offset, 21, -12],
    start: [0, 0, 0],
    radius: 10,
  };
  renderDemoCards();
  $("reportOverview").hidden = false;
  $("viewerDetail").hidden = false;
  $("viewer").classList.add("has-episode");
  $("reportEyebrow").textContent = "演示数据";
  $("episodeHeading").textContent = ep.uuid;
  $("camEmpty").textContent = "无相机画面";
  $("epStats").innerHTML =
    `<div>结果: <b>${ep.success ? "成功" : ep.collided ? "碰撞" : "未完成"}</b></div><div>末端距离: <b>${ep.final_ne.toFixed(2)} m</b></div><div>路径长度: <b>${ep.path_length.toFixed(1)} m</b></div><div>执行步数: <b>${ep.steps}</b></div>`;
  draw();
}

function runStatePill(sum) {
  if (!sum) return "";
  if (sum.state === "running") {
    const prog = `${sum.done ?? 0}/${sum.total ?? "?"}`;
    return `<span class="pill run">运行中 ${prog}</span>`;
  }
  if (sum.state === "done") return `<span class="pill done">已完成</span>`;
  if (sum.state === "stopped") return `<span class="pill fail">已停止</span>`;
  return "";
}

function buildRunSummaryCard(run) {
  const sum = (state.runSummaries || {})[run] || null;
  const div = document.createElement("div");
  const active = state.summaryRun === run;
  div.className = "card card-run" + (active ? " active" : "");
  const ptxt = profileText(sum && sum.planner);
  const ptag = ptxt
    ? `<span class="pill model">${escapeHTML(ptxt)}</span>`
    : "";
  const scene =
    sum && sum.scenes && sum.scenes.length
      ? sum.scenes.join(", ")
      : runTimeLabel(run)
        ? ""
        : run;
  const sr = sum && sum.sr != null ? sum.sr.toFixed(1) + "%" : "—";
  const osr = sum && sum.osr != null ? sum.osr.toFixed(1) + "%" : "—";
  const ne = sum && sum.ne_mean != null ? sum.ne_mean.toFixed(1) + "m" : "—";
  const spl = sum && sum.spl != null ? sum.spl.toFixed(1) + "%" : "—";
  const counts = sum
    ? `成功 ${sum.num_success}/${sum.num_episodes} · 碰撞 ${sum.num_collided} · 失败 ${sum.num_failed}`
    : "汇总加载中…";
  div.innerHTML = `<div class="row1"><span class="uuid run-title">批次 ${escapeHTML(runTimeLabel(run))}</span>
       <span class="row1-right">${ptag}${runStatePill(sum)}<button class="card-del" aria-label="删除该批次" title="删除该批次">×</button></span></div>
     <div class="scene">${escapeHTML(scene)}</div>
     <div class="meta run-metrics"><span>SR <b>${sr}</b></span><span>OSR <b>${osr}</b></span>
       <span>NE <b>${ne}</b></span><span>SPL <b>${spl}</b></span></div>
     <div class="meta run-counts">${counts}</div>`;
  div.querySelector(".card-del").onclick = (ev) => deleteRun(run, div, ev);
  div.onclick = () => {
    selectRunSummary(run);
    revealViewer();
  };
  return div;
}

function setViewerTitle(title) {
  const heading = $("epTitle");
  heading.textContent = title;
  heading.title = title;
  $("episodeHeading").textContent = title;
}

function clearSelection() {
  selectionRequest++;
  state.selectedUuid = null;
  state.run = null;
  state.viewSaved = false;
  state.viewDir = null;
  state.summaryRun = null;
  state.traj = null;
  setViewerTitle("批次评测报告");
  $("reportEyebrow").textContent = "EVALUATION REPORT";
  $("reportSubtitle").textContent = "选择左侧批次查看评测摘要与逐样例结果";
  $("reportOverview").hidden = false;
  $("reportOverview").innerHTML =
    '<div class="report-empty"><span class="empty-mark">◎</span><b>等待评测批次</b></div>';
  $("viewerDetail").hidden = true;
  $("viewer").classList.remove("has-episode");
  $("exportReportBtn").disabled = true;
  $("m-sr").textContent =
    $("m-osr").textContent =
    $("m-ne").textContent =
    $("m-spl").textContent =
      "—";
  refreshCamView();
  draw();
}

// 点批次汇总卡片：右侧统计面板显示该批次的指标 + 逐集明细（不选具体 episode）。
function selectRunSummary(run) {
  selectionRequest++; // 取消可能在途的轨迹请求
  state.selectedUuid = null;
  state.run = null;
  state.viewSaved = false;
  state.viewDir = null;
  state.traj = null;
  state.summaryRun = run;
  $("viewerDetail").hidden = true;
  $("viewer").classList.remove("has-episode");
  $("reportOverview").hidden = false;
  setViewerTitle("批次评测报告");
  refreshCamView();
  renderCards(state.history); // 更新卡片高亮
  renderRunSummaryStats(run);
  draw();
}

async function deleteEpisode(run, uuid, card, ev) {
  ev.stopPropagation();
  if (!confirm(`删除该集结果？\n${uuid}`)) return;
  card.remove();
  historySig = null;
  if (state.selectedUuid === uuid && state.run === run) clearSelection();
  try {
    await postJSON("/api/episode/delete", { run, id: uuid });
  } catch (e) {
    alert("删除失败：" + e.message);
  }
  await poll(); // 拉最新列表与重算后的指标
}

async function deleteRun(run, card, ev) {
  ev.stopPropagation();
  if (!confirm(`删除整批结果？\n${run}\n该批次下所有集都会被删除。`)) return;
  card.remove();
  historySig = null;
  if (state.summaryRun === run) clearSelection();
  try {
    await postJSON("/api/runs/delete", { run });
  } catch (e) {
    alert("删除失败：" + e.message);
  }
  await poll();
}

async function selectEpisode(uuid, run) {
  const requestId = ++selectionRequest;
  state.viewSaved = false;
  state.viewDir = null;
  state.summaryRun = null; // 选具体 episode 时退出批次汇总视图
  state.selectedUuid = uuid;
  state.run = run || state.run;
  state.traj = null;
  renderCards(state.history);
  $("episodeHeading").textContent = "评测样例 " + uuid.slice(0, 13);
  // 没有批次表格时收起概览区，不留下只有占位语的空白块
  $("reportOverview").hidden = !$("reportOverview").querySelector(
    ".overview-banner, .report-table-wrap",
  );
  $("viewerDetail").hidden = false;
  $("viewer").classList.add("has-episode");
  $("reportEyebrow").textContent = "EVALUATION REPORT";
  updateFrame();
  renderStats();
  draw();
  let traj = null;
  try {
    traj = await getJSON(
      `/api/traj?run=${encodeURIComponent(state.run)}&id=${encodeURIComponent(uuid)}`,
    );
  } catch (e) {}
  if (requestId !== selectionRequest) return;
  state.traj = traj;
  renderStats();
  draw();
}

// 相机缩略图的显隐与「无画面」占位（不再出现碎图图标）。
function refreshCamView() {
  const front = $("camFront"),
    down = $("camDown"),
    empty = $("camEmpty");
  const showEmpty = (text) => {
    empty.textContent = text;
    empty.hidden = false;
  };
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
  if (state.demoActive) {
    showEmpty("无相机画面");
    return;
  }
  const active = state.cam === "front" ? front : down;
  if (active.dataset.ok === "1") {
    active.hidden = false;
    empty.hidden = true;
  } else {
    showEmpty(
      active.dataset.ok === "0" ? "未采集到此视角的画面" : "正在读取画面…",
    );
  }
}

function bindCamImages() {
  for (const img of [$("camFront"), $("camDown")]) {
    img.addEventListener("load", () => {
      if (img.naturalWidth > 0) {
        img.dataset.ok = "1";
        refreshCamView();
      }
    });
    img.addEventListener("error", () => {
      img.dataset.ok = "0";
      refreshCamView();
    });
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
  if (state.summaryRun && !state.selectedUuid) {
    renderRunSummaryStats(state.summaryRun);
    return;
  }
  const t = state.traj;
  const box = $("epStats");
  if (!t || !t.positions || !t.positions.length) {
    box.innerHTML = "";
    return;
  }
  const target = t.target;
  const last = t.positions[t.positions.length - 1];
  const ne = target ? dist(last, target) : null;
  box.innerHTML = `<div>目标: <b>[${target ? target.map((v) => v.toFixed(1)).join(", ") : "—"}]</b></div>
     <div>成功半径: <b>${t.radius} m</b></div>
     <div>末点 NE: <b>${ne != null ? ne.toFixed(2) + " m" : "—"}</b></div>
     <div>轨迹点: <b>${t.positions.length}</b></div>`;
}

// 批次摘要同时作为首页报告概览；完整逐样例数据按需读取。
function renderRunSummaryStats(run) {
  const sum = (state.runSummaries || {})[run];
  const overview = $("reportOverview");
  overview.hidden = false;
  $("viewer").classList.remove("has-episode");
  $("viewerDetail").hidden = true;
  if (!sum) {
    $("exportReportBtn").disabled = true;
    overview.innerHTML = `<div class="report-empty"><b>批次摘要加载中</b></div>`;
    return;
  }
  $("exportReportBtn").disabled = sum.state === "running";
  const pct = (v) => (v != null ? v.toFixed(1) + "%" : "—");
  const mt = (v) => (v != null ? v.toFixed(2) + " m" : "—");
  const eps = state.history.filter((e) => e.run === run);
  const rows = eps
    .map((e) => {
      const status = e.success
        ? '<span class="pill ok">成功</span>'
        : e.collided
          ? '<span class="pill coll">碰撞</span>'
          : '<span class="pill fail">未完成</span>';
      return `<tr class="report-episode-row" data-uuid="${escapeHTML(e.uuid || "")}">
      <td class="mono">${escapeHTML(e.uuid || "")}</td><td>${status}</td>
      <td>${fmt(e.final_ne, " m")}</td><td>${e.steps ?? "—"}</td></tr>`;
    })
    .join("");
  const scenes = (sum.scenes || []).join(", ") || "—";
  const profile = profileText(sum.planner) || sum.planner || "—";
  $("reportEyebrow").textContent = "EVALUATION REPORT";
  $("epTitle").textContent = "批次评测报告";
  $("reportSubtitle").textContent =
    `${scenes}　/　${profile}　/　${runTimeLabel(run)}　/　${sum.num_episodes} 个样例`;
  $("m-sr").textContent = pct(sum.sr);
  $("m-osr").textContent = pct(sum.osr);
  $("m-ne").textContent = mt(sum.ne_mean);
  $("m-spl").textContent = pct(sum.spl);
  overview.innerHTML = `<div class="overview-banner"><div><span class="section-kicker">BATCH SUMMARY</span>
      <h3>${escapeHTML(scenes)}</h3><p>${escapeHTML(profile)}　·　${escapeHTML(runTimeLabel(run))}</p></div>
      <div class="completion-ring"><b>${sum.num_success ?? 0}<small>/${sum.num_episodes ?? 0}</small></b><span>成功样例</span></div></div>
     <div class="result-counts"><div><i class="count-success"></i>成功 <b>${sum.num_success ?? 0}</b></div>
       <div><i class="count-collision"></i>碰撞 <b>${sum.num_collided ?? 0}</b></div><div><i class="count-failed"></i>未完成 <b>${sum.num_failed ?? 0}</b></div></div>
     <div class="overview-table-head"><div><span class="section-kicker">EPISODE BREAKDOWN</span><h3>样例结果</h3></div></div>
     <div class="report-table-wrap"><table class="report-table"><thead><tr><th>样例 ID</th><th>结果</th><th>末端距离 NE</th><th>步数</th></tr></thead>
       <tbody>${rows || '<tr><td colspan="4" class="table-empty">暂无逐样例结果</td></tr>'}</tbody></table></div>`;
  overview.querySelectorAll(".report-episode-row").forEach((row) => {
    row.onclick = () => selectEpisode(row.dataset.uuid, run);
  });
}

async function exportReport() {
  const run = state.summaryRun || state.run;
  if (!run) return;
  try {
    const report =
      DEMO_BATCHES.find((batch) => batch.name === run)?.report ||
      (await getJSON(`/api/report?run=${encodeURIComponent(run)}`));
    renderFullReport(report);
    $("reportPrint").hidden = false;
    window.addEventListener(
      "afterprint",
      () => {
        $("reportPrint").hidden = true;
      },
      { once: true },
    );
    window.print();
  } catch (error) {
    alert("报告加载失败：" + error.message);
  }
}

function renderFullReport(report) {
  const episodes = report.episodes || [];
  const metrics = report.metrics || {};
  const pct = (value) => (value != null ? `${Number(value).toFixed(1)}%` : "—");
  const num = (value, suffix = "") =>
    value != null && Number.isFinite(Number(value))
      ? `${Number(value).toFixed(2)}${suffix}`
      : "—";
  const rows = episodes
    .map((ep, index) => {
      const status = ep.success ? "成功" : ep.collided ? "碰撞" : "未完成";
      return `<tr><td>${index + 1}</td><td class="mono">${escapeHTML(ep.uuid || "—")}</td>
      <td>${escapeHTML(ep.scene || "—")}</td><td>${status}</td><td>${ep.collided ? "是" : "否"}</td>
      <td>${num(ep.final_ne, " m")}</td><td>${ep.steps ?? "—"}</td><td>${num(ep.path_length, " m")}</td>
      <td>${escapeHTML(ep.error || "—")}</td></tr>`;
    })
    .join("");
  const dataset = report.dataset || {};
  const scenes = Array.isArray(dataset.scenes)
    ? dataset.scenes.join("、")
    : dataset.scenes || "—";
  const demoBatch = DEMO_BATCHES.find(
    (batch) => batch.name === (state.summaryRun || state.run),
  );
  const planner = report.demo
    ? demoBatch?.label || "—"
    : profileText(report.planner) || report.planner || "—";
  $("reportPrint").innerHTML =
    `<article class="print-report ${report.demo ? "is-demo" : ""}">
    <header class="print-report-head"><div><div class="print-brand">UAV-VLN <span>评测报告</span></div><div class="print-kicker">EVALUATION REPORT</div>
      <h1>${escapeHTML(scenes)}</h1><p>${escapeHTML(report.run || "")}　·　${escapeHTML(report.time || "")}</p></div>
      <div class="print-stamp">${report.demo ? "演示数据" : "批次报告"}</div></header>
    <section class="print-meta"><div><span>规划配置</span><b>${escapeHTML(planner)}</b></div><div><span>数据集切分</span><b>${escapeHTML(dataset.split || "—")}</b></div>
      <div><span>样例总数</span><b>${report.num_episodes ?? episodes.length}</b></div><div><span>成功半径</span><b>${num(dataset.success_radius_m, " m")}</b></div></section>
    <section class="print-metrics"><div><span>成功率　SR</span><b>${pct(metrics.sr)}</b></div><div><span>可达率　OSR</span><b>${pct(metrics.osr)}</b></div>
      <div><span>末端距离　NE</span><b>${num(metrics.ne_mean, " m")}</b></div><div><span>路径效率　SPL</span><b>${pct(metrics.spl)}</b></div></section>
    <section class="print-episodes"><div class="print-section-title"><span>逐样例结果</span><small>${episodes.length} 个样例</small></div>
      <table><thead><tr><th>#</th><th>样例 ID</th><th>场景</th><th>结果</th><th>碰撞</th><th>NE</th><th>步数</th><th>路径长度</th><th>错误信息</th></tr></thead><tbody>${rows}</tbody></table></section>
    <footer class="print-report-footer"><span>UAV-VLN · Evaluation System</span><span>${escapeHTML(new Date().toLocaleString("zh-CN"))}</span></footer>
  </article>`;
}

function comparisonRunData() {
  if (state.demoActive) {
    return DEMO_BATCHES.map(({ name, label, report }) => ({
      name,
      scenes: report.dataset.scenes,
      planner: label,
      ...report.metrics,
      num_episodes: report.num_episodes,
      demo: true,
    }));
  }
  const visibleRuns = new Set(state.history.map((episode) => episode.run));
  return Object.values(state.runSummaries).filter(
    (run) =>
      visibleRuns.has(run.name) &&
      run.state !== "running" &&
      (run.scenes || []).length === 1,
  );
}

function openCompare() {
  disposeComparisonCharts();
  const runs = comparisonRunData();
  const scenes = [...new Set(runs.flatMap((run) => run.scenes || []))].sort();
  const select = $("compareScene");
  select.innerHTML = scenes
    .map(
      (scene) =>
        `<option value="${escapeHTML(scene)}">${escapeHTML(scene)}</option>`,
    )
    .join("");
  $("compareDialog").showModal();
  if (!scenes.length) {
    $("compareChoices").innerHTML =
      '<div class="compare-empty">暂无可对比批次</div>';
    $("compareResult").innerHTML = "";
    $("exportCompareBtn").disabled = true;
  } else {
    renderCompareChoices();
  }
}

function renderCompareChoices() {
  const scene = $("compareScene").value;
  const runs = comparisonRunData().filter((run) => run.scenes?.[0] === scene);
  const prior = new Set(
    [...$("compareChoices").querySelectorAll("input:checked")].map(
      (input) => input.value,
    ),
  );
  const validPrior = new Set(
    [...prior].filter((name) => runs.some((run) => run.name === name)),
  );
  const defaults = [];
  const profiles = new Set();
  for (const run of runs) {
    if (!profiles.has(run.planner)) {
      profiles.add(run.planner);
      defaults.push(run.name);
    }
  }
  const selected = validPrior.size ? validPrior : new Set(defaults.slice(0, 3));
  $("compareChoices").innerHTML =
    runs
      .map(
        (run) => `<label class="compare-option">
    <input type="checkbox" value="${escapeHTML(run.name)}" ${selected.has(run.name) ? "checked" : ""}>
    <span class="compare-option-main"><b>${escapeHTML(profileText(run.planner) || run.planner || "未知配置")}</b><small>${escapeHTML(run.name)}　·　${run.num_episodes ?? 0} 样例</small></span>
    ${run.demo ? '<span class="pill demo-pill">演示</span>' : `<span class="compare-sr">SR ${run.sr != null ? run.sr.toFixed(1) + "%" : "—"}</span>`}
    </label>`,
      )
      .join("") || '<div class="compare-empty">该场景暂无评测批次</div>';
  $("compareChoices")
    .querySelectorAll("input")
    .forEach((input) => (input.onchange = renderCompareResult));
  renderCompareResult();
}

function renderCompareResult() {
  const scene = $("compareScene").value;
  const runs = comparisonRunData().filter((run) => run.scenes?.[0] === scene);
  const selectedIds = [
    ...$("compareChoices").querySelectorAll("input:checked"),
  ].map((input) => input.value);
  const selected = runs.filter((run) => selectedIds.includes(run.name));
  const profiles = new Set(selected.map((run) => run.planner));
  const rows = [
    ["成功率 · SR", "sr", "%"],
    ["可达率 · OSR", "osr", "%"],
    ["末端距离 · NE", "ne_mean", " m"],
    ["路径效率 · SPL", "spl", "%"],
  ];
  const valid = selected.length >= 2 && profiles.size >= 2;
  $("compareSelectionCount").textContent =
    `已选 ${selected.length} 个批次 · ${profiles.size} 种配置`;
  $("exportCompareBtn").disabled = !valid;
  if (!valid) {
    disposeComparisonCharts();
    $("compareResult").innerHTML =
      '<div class="compare-empty">选择同一场景下至少两种规划配置</div>';
    return;
  }
  disposeComparisonCharts();
  const chartMarkup = `<div class="compare-charts">${rows.map(([label, key], index) => `<section class="compare-chart-panel"><div class="compare-chart-heading"><b>${label}</b><span>${index === 2 ? "越低越好" : "越高越好"}</span></div><div class="compare-chart" id="compareChart${index}"></div></section>`).join("")}</div>`;
  const tableMarkup = `<div class="compare-table-wrap"><table class="compare-table"><thead><tr><th>评测指标</th>${selected.map((run) => `<th>${escapeHTML(profileText(run.planner) || run.planner || "未知配置")}</th>`).join("")}</tr></thead>
      <tbody>${rows.map(([label, key, suffix]) => `<tr><th>${label}</th>${selected.map((run) => `<td>${run[key] != null ? Number(run[key]).toFixed(2) + suffix : "—"}</td>`).join("")}</tr>`).join("")}</tbody></table></div>`;
  $("compareResult").innerHTML =
    `<div class="compare-result-head"><div><span class="eyebrow">${selected.some((run) => run.demo) ? "演示数据" : "同场景批次对比"}</span><h3>${escapeHTML(scene)}</h3></div></div>${chartMarkup}${tableMarkup}
    <div class="compare-runs">${selected.map((run) => `<span>${escapeHTML(runTimeLabel(run.name) || run.name)}　·　${run.num_episodes ?? 0} 样例</span>`).join("")}</div>`;
  renderComparisonCharts(selected, rows);
  $("exportCompareBtn").dataset.scene = scene;
  $("exportCompareBtn").dataset.runs = JSON.stringify(selectedIds);
}

function disposeComparisonCharts() {
  comparisonCharts.forEach((chart) => chart.dispose());
  comparisonCharts = [];
}

function renderComparisonCharts(selected, rows) {
  if (!window.echarts) return;
  const colors = ["#61d6b2", "#82b8ff", "#ffc86b", "#f18b86", "#b5a0ff"];
  rows.forEach(([label, key, suffix], index) => {
    const values = selected
      .map((run) => Number(run[key]))
      .filter(Number.isFinite);
    const maxValue = key === "ne_mean" ? Math.max(12, ...values) : 100;
    const chart = window.echarts.init($("compareChart" + index));
    chart.setOption({
      animationDuration: 650,
      grid: { left: 78, right: 54, top: 8, bottom: 20 },
      xAxis: {
        type: "value",
        min: 0,
        max: maxValue,
        splitNumber: 4,
        axisLabel: {
          color: "#8e9f9d",
          fontSize: 10,
          formatter: (value) => `${value}${suffix}`,
        },
        axisLine: { lineStyle: { color: "#354443" } },
        splitLine: { lineStyle: { color: "#263433", type: "dashed" } },
      },
      yAxis: {
        type: "category",
        data: selected.map(
          (run) => profileText(run.planner) || run.planner || "未知配置",
        ),
        axisLabel: {
          color: "#cbd6d4",
          fontSize: 11,
          width: 70,
          overflow: "truncate",
        },
        axisLine: { show: false },
        axisTick: { show: false },
      },
      tooltip: {
        trigger: "axis",
        axisPointer: { type: "shadow" },
        valueFormatter: (value) => `${Number(value).toFixed(2)}${suffix}`,
      },
      series: [
        {
          type: "bar",
          barWidth: 19,
          showBackground: true,
          backgroundStyle: { color: "#202c2d", borderRadius: 4 },
          itemStyle: {
            borderRadius: [0, 4, 4, 0],
            color: (params) => colors[params.dataIndex % colors.length],
          },
          label: {
            show: true,
            position: "right",
            color: "#dce6e3",
            fontSize: 10,
            formatter: (params) =>
              `${Number(params.value).toFixed(2)}${suffix}`,
          },
          data: selected.map((run) => Number(run[key])),
        },
      ],
    });
    comparisonCharts.push(chart);
  });
}

function exportComparisonPdf() {
  const scene = $("compareScene").value;
  const selectedIds = JSON.parse($("exportCompareBtn").dataset.runs || "[]");
  const runs = comparisonRunData().filter((run) =>
    selectedIds.includes(run.name),
  );
  if (runs.length < 2) return;
  const metrics = [
    ["成功率 · SR", "sr", "%"],
    ["可达率 · OSR", "osr", "%"],
    ["末端距离 · NE", "ne_mean", " m"],
    ["路径效率 · SPL", "spl", "%"],
  ];
  const demo = runs.some((run) => run.demo);
  comparisonCharts.forEach((chart) => chart.setOption({ animation: false }));
  const chartImages = comparisonCharts.map((chart, index) => ({
    title: metrics[index][0],
    src: chart.getDataURL({ pixelRatio: 2, backgroundColor: "#14201f" }),
  }));
  $("reportPrint").innerHTML =
    `<article class="print-report ${demo ? "is-demo" : ""}">
    <header class="print-report-head"><div><div class="print-brand">UAV-VLN <span>规划配置对比</span></div><div class="print-kicker">SCENE COMPARISON</div><h1>${escapeHTML(scene)}</h1><p>${runs.length} 个批次　·　${new Set(runs.map((run) => run.planner)).size} 种配置</p></div>
      <div class="print-stamp">${demo ? "演示数据" : "对比报告"}</div></header>
    <section class="print-metrics compare-print-metrics">${runs.map((run) => `<div><span>${escapeHTML(profileText(run.planner) || run.planner || "未知配置")} · SR</span><b>${run.sr != null ? run.sr.toFixed(1) + "%" : "—"}</b></div>`).join("")}</section>
    <section class="print-chart-grid">${chartImages.map((chart) => `<figure><figcaption>${escapeHTML(chart.title)}</figcaption><img src="${chart.src}" alt="${escapeHTML(chart.title)}对比图"></figure>`).join("")}</section>
    <section class="print-episodes"><div class="print-section-title"><span>指标对比</span></div><table><thead><tr><th>评测指标</th>${runs.map((run) => `<th>${escapeHTML(profileText(run.planner) || run.planner || "未知配置")}</th>`).join("")}</tr></thead>
      <tbody>${metrics.map(([label, key, suffix]) => `<tr><th>${label}</th>${runs.map((run) => `<td>${run[key] != null ? Number(run[key]).toFixed(2) + suffix : "—"}</td>`).join("")}</tr>`).join("")}</tbody></table></section>
    <footer class="print-report-footer"><span>UAV-VLN · ${demo ? "演示数据" : "Evaluation System"}</span><span>${escapeHTML(new Date().toLocaleString("zh-CN"))}</span></footer></article>`;
  $("compareDialog").close();
  $("reportPrint").hidden = false;
  window.addEventListener(
    "afterprint",
    () => {
      $("reportPrint").hidden = true;
    },
    { once: true },
  );
  window.print();
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
  const bw = Math.round(cssW * dpr),
    bh = Math.round(cssH * dpr);
  if (cv.width !== bw || cv.height !== bh) {
    cv.width = bw;
    cv.height = bh;
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const W = cssW,
    H = cssH;

  // 背景径向渐变（看着有纵深）
  ctx.clearRect(0, 0, W, H);
  const bg = ctx.createRadialGradient(
    W * 0.5,
    H * 0.4,
    30,
    W * 0.5,
    H * 0.5,
    Math.max(W, H) * 0.8,
  );
  bg.addColorStop(0, chartColors.panel);
  bg.addColorStop(1, chartColors.background);
  ctx.fillStyle = bg;
  ctx.fillRect(0, 0, W, H);

  const t = state.traj;
  const hasTrail = Boolean(t?.positions?.length);
  const expert = Array.isArray(t?.expert) ? t.expert : [];
  $("trajLegend").hidden = !hasTrail;
  $("trajTarget").hidden = !t?.target;
  $("trajExpert").hidden = !expert.length;
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
  for (const p of expert) all.push(p);
  if (t.target) all.push(t.target);
  const mn = [Infinity, Infinity, Infinity],
    mx = [-Infinity, -Infinity, -Infinity];
  for (const p of all)
    for (let i = 0; i < 3; i++) {
      mn[i] = Math.min(mn[i], p[i]);
      mx[i] = Math.max(mx[i], p[i]);
    }
  const center = [
    (mn[0] + mx[0]) / 2,
    (mn[1] + mx[1]) / 2,
    (mn[2] + mx[2]) / 2,
  ];
  // 固定取景，与 main 一致：缩放按模型三维跨度一次算定，转轴是包围盒中心。
  // 拖动只改视角，不重算缩放/中心，避免旋转时画面呼吸、转轴漂移。
  const span = Math.max(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2], 1);
  const scale = (Math.min(W, H) * 0.72) / span;
  const proj = (p) => project(p, center, scale, W, H, state.az, state.el);
  const targetRing =
    t.target && t.radius != null
      ? Array.from({ length: 64 }, (_, i) => {
          const angle = (i / 64) * Math.PI * 2;
          return [
            t.target[0] + t.radius * Math.cos(angle),
            t.target[1] + t.radius * Math.sin(angle),
            t.target[2],
          ];
        })
      : [];

  // 地面参考面：取轨迹最低处 z（NED 里 z 越大越靠下）
  const zG = mx[2];
  const padX = (mx[0] - mn[0]) * 0.25 + 1,
    padY = (mx[1] - mn[1]) * 0.25 + 1;
  const X0 = mn[0] - padX,
    X1 = mx[0] + padX,
    Y0 = mn[1] - padY,
    Y1 = mx[1] + padY;
  const corners = [
    proj([X0, Y0, zG]),
    proj([X1, Y0, zG]),
    proj([X1, Y1, zG]),
    proj([X0, Y1, zG]),
  ];

  // 地面菱形填充
  ctx.save();
  ctx.beginPath();
  ctx.moveTo(corners[0][0], corners[0][1]);
  for (let i = 1; i < 4; i++) ctx.lineTo(corners[i][0], corners[i][1]);
  ctx.closePath();
  const gg = ctx.createLinearGradient(
    corners[0][0],
    corners[0][1],
    corners[2][0],
    corners[2][1],
  );
  gg.addColorStop(0, "rgba(76,194,165,0.10)");
  gg.addColorStop(1, "rgba(76,194,165,0.02)");
  ctx.fillStyle = gg;
  ctx.fill();
  ctx.restore();

  // 地面网格
  const N = 8;
  ctx.strokeStyle = "rgba(141,160,168,0.22)";
  ctx.lineWidth = 1;
  for (let i = 0; i <= N; i++) {
    const fx = X0 + ((X1 - X0) * i) / N;
    const a = proj([fx, Y0, zG]),
      b = proj([fx, Y1, zG]);
    ctx.beginPath();
    ctx.moveTo(a[0], a[1]);
    ctx.lineTo(b[0], b[1]);
    ctx.stroke();
    const fy = Y0 + ((Y1 - Y0) * i) / N;
    const c = proj([X0, fy, zG]),
      d = proj([X1, fy, zG]);
    ctx.beginPath();
    ctx.moveTo(c[0], c[1]);
    ctx.lineTo(d[0], d[1]);
    ctx.stroke();
  }

  // 高度垂线（虚线，从轨迹点落到地面，体现高度）
  ctx.save();
  ctx.strokeStyle = "rgba(76,194,165,0.18)";
  ctx.setLineDash([3, 4]);
  ctx.lineWidth = 1;
  const stepN = Math.max(1, Math.floor(pts.length / 24));
  for (let i = 0; i < pts.length; i += stepN) {
    const p = pts[i];
    const a = proj(p),
      b = proj([p[0], p[1], zG]);
    ctx.beginPath();
    ctx.moveTo(a[0], a[1]);
    ctx.lineTo(b[0], b[1]);
    ctx.stroke();
  }
  ctx.restore();

  // 成功半径球（水平圆）
  if (targetRing.length) {
    ctx.strokeStyle = chartColors.target;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    targetRing.forEach((p, i) => {
      const [x, y] = proj(p);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.closePath();
    ctx.stroke();
  }

  // 专家轨迹：对比色虚线，画在智能体轨迹下方
  if (expert.length > 1) {
    ctx.save();
    ctx.strokeStyle = "rgba(110,168,255,0.85)";
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 5]);
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.beginPath();
    const e0 = proj(expert[0]);
    ctx.moveTo(e0[0], e0[1]);
    for (let i = 1; i < expert.length; i++) {
      const e = proj(expert[i]);
      ctx.lineTo(e[0], e[1]);
    }
    ctx.stroke();
    ctx.restore();
  }

  // 轨迹：先一道发光底线，再按进度叠彩色细线
  ctx.save();
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.shadowColor = chartColors.start;
  ctx.shadowBlur = 10;
  ctx.strokeStyle = "rgba(76,194,165,0.22)";
  ctx.lineWidth = 5;
  ctx.beginPath();
  const s0 = proj(pts[0]);
  ctx.moveTo(s0[0], s0[1]);
  for (let i = 1; i < pts.length; i++) {
    const s = proj(pts[i]);
    ctx.lineTo(s[0], s[1]);
  }
  ctx.stroke();
  ctx.restore();

  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  for (let i = 1; i < pts.length; i++) {
    const a = proj(pts[i - 1]),
      b = proj(pts[i]);
    const f = i / pts.length;
    ctx.strokeStyle = `hsl(${165 - 125 * f}, 52%, 58%)`;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(a[0], a[1]);
    ctx.lineTo(b[0], b[1]);
    ctx.stroke();
  }

  // 标记颜色与图例对应。
  const marker = (p, color, r) => {
    const s = proj(p);
    ctx.save();
    ctx.shadowColor = color;
    ctx.shadowBlur = 12;
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(s[0], s[1], r, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
    ctx.strokeStyle = "rgba(255,255,255,0.85)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(s[0], s[1], r, 0, Math.PI * 2);
    ctx.stroke();
  };
  marker(pts[0], chartColors.start, 5);
  marker(pts[pts.length - 1], chartColors.current, 4);
  if (t.target) marker(t.target, chartColors.target, 6);
}

// 拖拽旋转
function bindDrag() {
  const cv = $("traj3d");
  cv.addEventListener("mousedown", (e) => {
    state.dragging = true;
    state.lastMouse = [e.clientX, e.clientY];
  });
  window.addEventListener("mouseup", () => {
    state.dragging = false;
  });
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
  document.querySelectorAll(".cam-tabs button").forEach((btn) => {
    btn.onclick = () => {
      document.querySelectorAll(".cam-tabs button").forEach((item) => {
        item.classList.toggle("active", item === btn);
        item.setAttribute("aria-pressed", String(item === btn));
      });
      state.cam = btn.dataset.cam;
      if (state.demoActive) {
        $("camEmpty").textContent = "无相机画面";
        refreshCamView();
        return;
      }
      if (state.viewSaved || !state.selectedUuid) {
        refreshCamView();
        return;
      }
      updateFrame();
    };
  });
}

// ---------- 发起评测 ----------
async function postJSON(url, body) {
  const r = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  let data = null;
  try {
    data = await r.json();
  } catch (_) {}
  if (!r.ok) throw new Error((data && data.error) || "HTTP " + r.status);
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
  document.querySelectorAll(".ep-test").forEach((b) => (b.disabled = disabled));
}

let lastProfileSig = null;
const profileLabels = {}; // profile id -> 中文名（default→世界模型，vlm→普通VLM）
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
  try {
    p = await getJSON("/api/eval/profile");
  } catch (e) {
    p = { online: false };
  }
  setPlannerLink(p.planner_port ?? 8000);
  if (!p.online) {
    if (lastProfileSig !== "offline") {
      sel.innerHTML = "";
      const opt = document.createElement("option");
      opt.value = "";
      opt.textContent = "主界面未连接";
      sel.appendChild(opt);
      lastProfileSig = "offline";
    }
    return;
  }
  const profiles = p.profiles || [];
  for (const info of profiles) {
    if (info.id) profileLabels[info.id] = info.label || info.id;
  }
  const sig = profiles.map((x) => x.id + ":" + x.label).join(",");
  if (sig !== lastProfileSig) {
    lastProfileSig = sig;
    sel.innerHTML = "";
    for (const info of profiles) {
      const opt = document.createElement("option");
      opt.value = info.id;
      opt.textContent = info.label || info.id;
      sel.appendChild(opt);
    }
    state.profileTouched = false; // 选项变了，重置为跟随主界面
  }
  // 默认跟随主界面当前值；用户手动选过就不再覆盖。
  if (!state.profileTouched && p.current) sel.value = p.current;
}

async function loadDataset() {
  let data;
  try {
    data = await getJSON("/api/dataset");
  } catch (e) {
    return;
  }
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
  try {
    data = await getJSON(`/api/episodes?scene=${encodeURIComponent(scene)}`);
  } catch (e) {
    return;
  }
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
    info.appendChild(ins);
    info.appendChild(uid);
    const btn = document.createElement("button");
    btn.className = "ep-test";
    btn.textContent = "测试";
    btn.disabled = lastActive;
    btn.onclick = () => startRun(scene, [ep.uuid]);
    row.appendChild(info);
    row.appendChild(btn);
    box.appendChild(row);
  }
}

async function startRun(scene, uuids) {
  if (!scene) {
    setRunState("先选一个场景", false);
    return;
  }
  state.demoSuppressed = true;
  if (state.demoActive) {
    state.demoActive = false;
    clearSelection();
  }
  const profile = $("modeSelect").value || null;
  const label = ($("modeSelect").selectedOptions[0] || {}).textContent || "";
  setRunState(
    `提交中: ${scene}${uuids ? " / 1集" : " / 全场景"}${label ? " (" + label + ")" : ""}…`,
    false,
  );
  try {
    await postJSON("/api/start", { scene, uuids: uuids || null, profile });
    setRunState(`已提交，场景 ${scene} 启动中…`, true);
    await pollRunStatusOnce();
  } catch (e) {
    setRunState("启动失败: " + e.message, false);
  }
}

async function stopRun() {
  if (!confirm("停止当前评测批次？已完成的样例结果会保留。")) return;
  const btn = $("stopBtn");
  btn.disabled = true;
  try {
    const res = await postJSON("/api/stop", {
      run: state.latestRun || state.summaryRun || "",
    });
    setRunState(res.stopped ? "已停止评测" : res.reason || "停止失败", false);
  } catch (e) {
    setRunState("停止失败: " + e.message, false);
  } finally {
    btn.disabled = false;
  }
  await pollRunStatusOnce();
  await poll();
}

async function pollRunStatusOnce() {
  let s;
  try {
    s = await getJSON("/api/runstatus");
  } catch (e) {
    return;
  }
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
    setRunState(
      `上次: ${s.scene}${s.returncode != null ? " 已结束" : ""}`,
      false,
    );
  }
  lastActive = active;
}

async function pollRunStatus() {
  await pollRunStatusOnce();
  setTimeout(pollRunStatus, 2000);
}

// ---------- 已保存执行记录（规划器「保存执行记录」写入） ----------
let savedSig = null; // 记录目录集合签名；null=尚未渲染，避免每 2s 重建闪烁
async function loadSaved() {
  let data;
  try {
    data = await getJSON("/api/saved");
  } catch (e) {
    return;
  }
  const records = data.records || [];
  const sig = JSON.stringify([records, state.viewDir, profileLabels]);
  if (sig === savedSig) return; // 没有增删，不重建
  savedSig = sig;
  const box = $("savedList");
  const scrollTop = box.scrollTop;
  box.innerHTML = "";
  if (!records.length) {
    const empty = document.createElement("div");
    empty.className = "stats";
    empty.textContent = "还没有执行记录（在导航系统里点「保存执行记录」）。";
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
    const ptag = ptxt
      ? `<span class="pill model">${escapeHTML(ptxt)}</span>`
      : "";
    div.innerHTML = `<div class="row1"><span class="uuid">${escapeHTML(r.name || r.dir)}</span>
         <span class="row1-right">${ptag}${pill}<button class="card-del" aria-label="删除该记录" title="删除该记录">×</button></span></div>
       <div class="scene">${escapeHTML(r.map || "")}</div>
       <div class="meta"><span>SR ${sr}</span><span>末距 ${r.final_distance_m != null ? r.final_distance_m.toFixed(2) + "m" : "—"}</span></div>`;
    div.querySelector(".uuid").title = r.name || r.dir;
    div.querySelector(".card-del").onclick = (ev) =>
      deleteSaved(r.dir, div, ev);
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
  if (!confirm(`删除该执行记录？\n${dir}`)) return;
  card.remove(); // 先从界面移除，立刻见效（不等网络、不刷新）
  savedSig = null; // 允许随后用服务器实际结果对齐重建
  if (state.viewSaved && state.viewDir === dir) {
    clearSelection();
    setSavedMetrics(null);
  }
  try {
    await postJSON("/api/saved/delete", { dir });
  } catch (e) {
    alert("删除失败：" + e.message);
  }
  await loadSaved(); // 对齐服务器（删除失败则该卡片会重新出现）
}

async function viewSaved(dir) {
  const requestId = ++selectionRequest;
  let rec;
  try {
    rec = await getJSON(`/api/saved/traj?dir=${encodeURIComponent(dir)}`);
  } catch (e) {
    return;
  }
  if (requestId !== selectionRequest) return;
  state.selectedUuid = null;
  state.run = null;
  state.viewSaved = true;
  state.viewDir = dir;
  loadSaved();
  setViewerTitle("已保存记录: " + (rec.name || dir));
  $("reportOverview").hidden = false;
  $("viewerDetail").hidden = false;
  $("viewer").classList.add("has-episode");
  $("exportReportBtn").disabled = true;
  $("reportSubtitle").textContent =
    `${escapeHTML(rec.map || "")}　/　已保存执行记录`;
  state.traj = {
    positions: rec.positions || [],
    target: null,
    start: rec.start,
    radius: null,
  };
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
  $("m-sr").textContent =
    rec?.sr != null ? (rec.sr * 100).toFixed(1) + "%" : "—";
  $("m-osr").textContent = "—";
  $("m-ne").textContent =
    rec?.final_distance_m != null ? rec.final_distance_m.toFixed(1) + "m" : "—";
  $("m-spl").textContent = "—";
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
  $("stopBtn").onclick = stopRun;
  $("exportReportBtn").onclick = exportReport;
  $("compareBtn").onclick = openCompare;
  $("closeCompareBtn").onclick = () => $("compareDialog").close();
  $("compareScene").onchange = renderCompareChoices;
  $("exportCompareBtn").onclick = exportComparisonPdf;
  $("compareDialog").addEventListener("click", (event) => {
    if (event.target === $("compareDialog")) $("compareDialog").close();
  });
  // 用户手动选 profile 后停止自动跟随主界面当前值。
  $("modeSelect").onchange = () => {
    state.profileTouched = true;
  };
  renderDemoBoard();
  await loadDataset();
  await loadPlannerProfile();
  await poll();
  setInterval(poll, 2000);
  // 每 3s 同步一次主界面 profile（只读镜像），主界面切了这里跟着变。
  setInterval(loadPlannerProfile, 3000);
  // 「执行记录」标签可见时，每 2s 拉一次，新增/删除都自动反映（无增删则跳过多余重建）。
  setInterval(() => {
    if (state.savedTabVisible) loadSaved();
  }, 2000);
  pollRunStatus();
}

main();
